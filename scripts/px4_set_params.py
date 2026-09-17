#!/usr/bin/env python3
"""Apply a PX4 parameter file over MAVLink, and verify each one took.

    python scripts/px4_set_params.py --endpoint udpout:127.0.0.1:14580 \
        --file configs/px4_extnav.params

PARAM_SET is fire-and-forget: PX4 answers with a PARAM_VALUE, but a parameter
that does not exist, or a value out of range, is simply never acknowledged and
nothing on this side notices. Every setting here is read back and compared, and
a mismatch is an exit code rather than a line of output nobody reads.

Types are inferred from the file: a value with a decimal point is REAL32,
otherwise INT32. PX4 rejects a PARAM_SET whose declared type disagrees with the
parameter's own (`param types mismatch`, mavlink_parameters.cpp:130), so
getting this wrong looks exactly like the parameter being absent.

**AN INT32 GOES ON THE WIRE AS ITS BIT PATTERN, NOT AS A NUMBER.** MAVLink has
one float32 slot for every parameter value and `param_type` says what the bits
mean. PX4 takes that literally:

    mavlink_parameters.cpp:135    param_set(param, &(set.param_value));

-- the ADDRESS of the float field, handed to a function that will read four
bytes as an int32. So a PARAM_SET carrying the float 1.0 stores 1065353216,
which is 0x3F800000, the bit pattern of 1.0f. PX4 acknowledges it. ArduPilot,
which sends and accepts every parameter as a plain REAL32, does not behave this
way at all.

This cost a run here, and it did so by cancelling out: the set stored
1065353216, the read-back was decoded as a raw float and came back as 1.0, and
the check printed `ok`. Meanwhile EKF2_EV_CTRL's bit 0 -- the bit that switches
external vision on -- was CLEAR, because 1065353216 is even. Both directions
have to honour `param_type` or neither does.
"""
from __future__ import annotations

import argparse
import struct
import sys
import time

MAV_PARAM_TYPE_INT32 = 6
MAV_PARAM_TYPE_REAL32 = 9
# Everything below REAL32 is an integer type. PX4 returns an int-typed
# parameter as the integer REINTERPRETED as a float -- 1 arrives as 1.4013e-45,
# the float32 denormal with bit pattern 1 -- so a read-back compared as a float
# fails against a value that was set perfectly.
_INT_PARAM_TYPES = {1, 2, 3, 4, 5, 6, 7, 8}


def param_value(m) -> float:
    """The number a PARAM_VALUE actually carries, given its declared type."""
    v = float(m.param_value)
    if getattr(m, "param_type", MAV_PARAM_TYPE_REAL32) in _INT_PARAM_TYPES:
        return float(struct.unpack("<i", struct.pack("<f", v))[0])
    return v


def to_wire(value: float, ptype: int) -> float:
    """The float32 slot's contents for a value of this type."""
    if ptype in _INT_PARAM_TYPES:
        return struct.unpack("<f", struct.pack("<i", int(round(value))))[0]
    return float(value)


def parse(path):
    """name -> (value, mav_param_type). Comments are '#' to end of line."""
    out = {}
    for line in open(path):
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) < 2:
            continue
        name, raw = parts[0], parts[1]
        out[name] = (float(raw),
                     MAV_PARAM_TYPE_REAL32 if "." in raw else MAV_PARAM_TYPE_INT32)
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--endpoint", default="udpout:127.0.0.1:14580")
    ap.add_argument("--file", required=True)
    ap.add_argument("--timeout", type=float, default=20.0)
    ap.add_argument("--no-save", action="store_true",
                    help="skip the PREFLIGHT_STORAGE save (parameters then last "
                         "only until the autopilot restarts)")
    a = ap.parse_args()

    from pymavlink import mavutil
    want = parse(a.file)
    print(f"applying {len(want)} parameters from {a.file}")

    conn = mavutil.mavlink_connection(a.endpoint, source_system=255)
    conn.mav.heartbeat_send(6, 8, 0, 0, 0)
    if conn.wait_heartbeat(timeout=a.timeout) is None:
        print("no heartbeat")
        return 2

    got = {}
    for attempt in range(3):
        pending = [n for n in want if n not in got]
        if not pending:
            break
        for name in pending:
            value, ptype = want[name]
            conn.mav.param_set_send(conn.target_system, conn.target_component,
                                    name.encode(), to_wire(value, ptype), ptype)
        deadline = time.time() + 5.0
        while time.time() < deadline and len(got) < len(want):
            m = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
            if m and m.param_id in want:
                got[m.param_id] = param_value(m)

    bad = 0
    for name, (value, _t) in want.items():
        if name not in got:
            print(f"  ABSENT  {name:18s} -- three PARAM_SETs unacknowledged")
            bad += 1
            continue
        # Tolerance rather than equality: a REAL32 round-trip is not exact, and
        # an INT32 comes back as a float.
        ok = abs(got[name] - value) <= max(1e-4, abs(value) * 1e-5)
        print(f"  {'ok    ' if ok else 'WRONG '} {name:18s} {got[name]:<10g}"
              + ("" if ok else f" -- asked for {value:g}"))
        bad += 0 if ok else 1

    if bad:
        conn.close()
        print(f"\n{bad} parameter(s) did not take.")
        return 1

    # SAVE, EXPLICITLY. PX4 autosaves a changed parameter a few seconds later,
    # and SITL killed before that window loses every one of them silently: the
    # set is acknowledged, the read-back is correct, and the next boot comes up
    # on defaults. That looked exactly like the reboot itself resetting them.
    #
    # MAV_CMD_PREFLIGHT_STORAGE param1=1 is "write parameters to storage", and
    # waiting for its COMMAND_ACK is what makes this deterministic rather than
    # a sleep long enough to probably work.
    if not a.no_save:
        conn.mav.command_long_send(conn.target_system, conn.target_component,
                                   245,           # MAV_CMD_PREFLIGHT_STORAGE
                                   0, 1, -1, -1, -1, 0, 0, 0)
        deadline, acked = time.time() + 10.0, False
        while time.time() < deadline:
            m = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=1)
            if m and m.command == 245:
                acked = m.result == 0
                print(f"\nPREFLIGHT_STORAGE save: result {m.result}"
                      + ("" if acked else " -- NOT accepted"))
                break
        else:
            print("\nPREFLIGHT_STORAGE save was not acknowledged within 10 s.")
        if not acked:
            conn.close()
            print("Parameters are set but NOT persisted; they will not survive a reboot.")
            return 1

    conn.close()
    print("\nall parameters applied, read back and saved.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
