#!/usr/bin/env python3
"""
step22_ardupilot_extnav_check.py -- what rate and latency will EKF3 actually accept?

    python3 docs/step22_ardupilot_extnav_check.py
    python3 docs/step22_ardupilot_extnav_check.py --ref Copter-4.5
    python3 docs/step22_ardupilot_extnav_check.py --cache /tmp/ap --offline

THE QUESTION THIS EXISTS TO ANSWER

CLAUDE.md carried an open item: "verify against ArduPilot source the minimum
rate EKF3 accepts on the ExternalNav path. If XFeat is fast enough, no UKF is
needed at all and VINS-Fusion drops out of the plan entirely."

It also carried an assumption, in the section rejecting the two-board compute
pool: "ArduPilot's VISO path compensates delay from the capture timestamp."

Both are answered by constants in the source. The answers are not the same
shape as the question: there is NO minimum-rate check anywhere on the ExternalNav
path. The floor is implied by four timeouts. The ceiling is an explicit 50 Hz.
And the delay compensation the plan leans on is capped at 250 ms, past which
latency is silently absorbed as a timing error rather than rejected.

WHY THIS IS A SCRIPT AND NOT A NOTE

Every number below is a hard-coded constant in ArduPilot master, so every number
below can rot. This fetches the source and asserts them. If upstream changes one,
this prints DRIFT and the finding gets rechecked instead of quietly going stale.

Exit code 0 = all constants as documented. 1 = at least one drifted or the
extraction failed.
"""

import argparse
import os
import re
import sys
import urllib.request

RAW = "https://raw.githubusercontent.com/ArduPilot/ardupilot"

# file -> path under libraries/
FILES = {
    "AP_NavEKF3.h":                "AP_NavEKF3/AP_NavEKF3.h",
    "AP_NavEKF3.cpp":              "AP_NavEKF3/AP_NavEKF3.cpp",
    "AP_NavEKF3_core.h":           "AP_NavEKF3/AP_NavEKF3_core.h",
    "AP_NavEKF3_core.cpp":         "AP_NavEKF3/AP_NavEKF3_core.cpp",
    "AP_NavEKF3_Measurements.cpp": "AP_NavEKF3/AP_NavEKF3_Measurements.cpp",
    "AP_NavEKF3_PosVelFusion.cpp": "AP_NavEKF3/AP_NavEKF3_PosVelFusion.cpp",
    "AP_NavEKF3_Control.cpp":      "AP_NavEKF3/AP_NavEKF3_Control.cpp",
    "EKF_Buffer.cpp":              "AP_NavEKF/EKF_Buffer.cpp",
    "AP_VisualOdom.cpp":           "AP_VisualOdom/AP_VisualOdom.cpp",
}

# name -> (file, regex with one capture group, expected, unit, what it governs)
CHECKS = [
    ("extNavIntervalMin_ms", "AP_NavEKF3.h",
     r"extNavIntervalMin_ms\s*=\s*(\d+)", "20", "ms",
     "CEILING. writeExtNavData() returns early on anything faster. 50 Hz."),

    ("deadReckonDeclare_ms", "AP_NavEKF3.h",
     r"deadReckonDeclare_ms\s*=\s*(\d+)", "1000", "ms",
     "PRACTICAL FLOOR. Longer gap and the filter is flagged dead reckoning."),

    ("posRetryTimeNoVel_ms", "AP_NavEKF3.h",
     r"posRetryTimeNoVel_ms\s*=\s*(\d+)", "7000", "ms",
     "HARD FLOOR, position only. Then posAidLossCritical -> posTimeout."),

    ("posRetryTimeUseVel_ms", "AP_NavEKF3.h",
     r"posRetryTimeUseVel_ms\s*=\s*(\d+)", "10000", "ms",
     "HARD FLOOR when a velocity source is also aiding."),

    ("tiltDriftTimeMax_ms", "AP_NavEKF3.h",
     r"tiltDriftTimeMax_ms\s*=\s*(\d+)", "15000", "ms",
     "Attitude aiding loss. Beyond this EKF3 drops to AID_NONE."),

    ("EKF_TARGET_DT_MS", "AP_NavEKF3_core.h",
     r"define\s+EKF_TARGET_DT_MS\s+(\d+)", "12", "ms",
     "Fusion time horizon step. It sweeps past any buffered sample."),

    ("recall window", "EKF_Buffer.cpp",
     r"dt\s*>=\s*0\s*&&\s*dt\s*<\s*(\d+)", "100", "ms",
     "A sample is fused only if the horizon reaches it within this window."),

    ("visual_odom delay cap", "AP_NavEKF3_core.cpp",
     r"MIN\(visual_odom->get_delay_ms\(\),\s*(\d+)\)", "250", "ms",
     "DELAY COMPENSATION CAP. The plan leaned on this being unbounded."),

    ("VISO_DELAY_MS range max", "AP_VisualOdom.cpp",
     r"_DELAY_MS[\s\S]{0,200}?@Range:\s*0\s+(\d+)", "250", "ms",
     "Parameter range agrees with the EKF3 clamp. 250 ms is the ceiling."),

    ("horiz posErr clamp, in flight", "AP_NavEKF3_PosVelFusion.cpp",
     r"extNavUsedForVel\s*=[\s\S]{0,2500}?R_OBS\[3\]\s*=\s*sq\(constrain_ftype\("
     r"extNavDataDelayed\.posErr,\s*[\d.]+f?,\s*([\d.]+)f\)\)",
     "100.0", "m",
     "THE ONE THAT MATTERS. Sigma we may report while moving. p99 fits under it."),

    ("horiz posErr clamp, zero-vel", "AP_NavEKF3_PosVelFusion.cpp",
     r"fusingStationaryZeroVel\)\s*\{[\s\S]{0,900}?R_OBS\[3\]\s*=\s*sq\(constrain_ftype\("
     r"extNavDataDelayed\.posErr,\s*[\d.]+f?,\s*([\d.]+)f\)\)",
     "10.0", "m",
     "Stationary branch only. Clamps at 10 m, but we are never in it in flight."),

    ("extnav posErr clamp, vertical", "AP_NavEKF3_PosVelFusion.cpp",
     r"posDownObsNoise\s*=\s*sq\(constrain_ftype\(extNavDataDelayed\.posErr,\s*[\d.]+f?,\s*([\d.]+)f\)\)", "10.0", "m",
     "Vertical is clamped at 10 m. Same saturation NGPS Eq. 7 has."),

    ("EK3_POS_I_GATE default", "AP_NavEKF3.cpp",
     r"define\s+POS_I_GATE_DEFAULT\s+(\d+)", "500", "pct",
     "5 sigma. A fix outside it is rejected, not fused."),

    ("EK3_GLITCH_RAD default", "AP_NavEKF3.cpp",
     r"define\s+GLITCH_RADIUS_DEFAULT\s+(\d+)", "25", "m",
     "Position variance above this radius forces a reset to the sensor."),
]


def fetch(ref, cache, offline):
    """Return {name: text}. Cache to disk so --offline can rerun without network."""
    out = {}
    if cache:
        os.makedirs(cache, exist_ok=True)
    for name, path in FILES.items():
        local = os.path.join(cache, name) if cache else None
        if local and os.path.exists(local):
            with open(local, "r", errors="replace") as fh:
                out[name] = fh.read()
            continue
        if offline:
            sys.exit("offline but %s is not cached in %s" % (name, cache))
        url = "%s/%s/libraries/%s" % (RAW, ref, path)
        try:
            with urllib.request.urlopen(url, timeout=45) as resp:
                text = resp.read().decode("utf-8", errors="replace")
        except Exception as exc:
            sys.exit("could not fetch %s\n  %s\n  %s" % (name, url, exc))
        if local:
            with open(local, "w") as fh:
                fh.write(text)
        out[name] = text
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ref", default="master",
                    help="ArduPilot git ref to check (default master)")
    ap.add_argument("--cache", default=None,
                    help="directory to cache fetched sources in")
    ap.add_argument("--offline", action="store_true",
                    help="use only what is already in --cache")
    args = ap.parse_args()

    src = fetch(args.ref, args.cache, args.offline)

    print("ArduPilot ref: %s" % args.ref)
    print()
    print("%-34s %-9s %-9s %s" % ("constant", "found", "expected", "state"))
    print("-" * 78)

    drift = 0
    notes = []
    for name, fname, pattern, expected, unit, note in CHECKS:
        m = re.search(pattern, src[fname])
        if not m:
            found, state = "-", "NOT FOUND"
            drift += 1
        else:
            found = m.group(1)
            if found.rstrip("0").rstrip(".") == expected.rstrip("0").rstrip("."):
                state = "ok"
            else:
                state = "DRIFT"
                drift += 1
        print("%-34s %-9s %-9s %s" % (name, found + unit, expected + unit, state))
        notes.append((name, note))

    print()
    print("what each one governs")
    print("-" * 78)
    for name, note in notes:
        print("  %-34s %s" % (name, note))

    print()
    print("THE RATE ENVELOPE FOR GEOANCHOR")
    print("-" * 78)
    print("""  There is no minimum-rate check on the ExternalNav path. The floor is
  implied by timeouts, so it is a band and not a single number:

      > 50 Hz     silently dropped by writeExtNavData()
      1 - 50 Hz   healthy. no dead-reckoning flag
      < 1 Hz      dead_reckoning flag set, position still fused
      < 1/7 Hz    posTimeout. EKF3 stops trusting absolute position

  XFEAT_LG at 200-3000 ms per fix is 0.33-5 Hz, which clears the 1/7 Hz hard
  floor everywhere and the 1 Hz healthy floor above roughly 5 fps of matcher
  throughput. Rate is not the binding constraint. LATENCY IS.

  Delay compensation stops at 250 ms. writeExtNavData() does

      timeStamp_ms = MAX(timeStamp_ms - delay_ms - dt/2, imuDataDelayed.time_ms)

  so a fix older than the fusion horizon is NOT rejected. It is clamped
  forward and fused as though it were current. Latency past 250 ms turns
  into position error at the aircraft's own speed:

      at 5 m/s, every 100 ms of uncompensated latency = 0.5 m of injected error

      XFEAT_LG   ~200 ms    0 m uncompensated       free
      XFEAT_LG   ~500 ms    250 ms -> 1.25 m        11% of its 11.50 m median
      Roma      ~3000 ms   2750 ms -> 13.75 m       larger than its 6.36 m median

  That last line is a reason to prefer XFeat over Roma that has nothing to do
  with whether the board can run it. Roma's accuracy advantage is consumed by
  its own latency before the fix reaches the filter.""")

    print()
    if drift:
        print("%d constant(s) drifted or were not found. Recheck the finding." % drift)
        return 1
    print("All constants as documented in docs/ardupilot_extnav_limits_2026-09-01.md")
    return 0


if __name__ == "__main__":
    sys.exit(main())
