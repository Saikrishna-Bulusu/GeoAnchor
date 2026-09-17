"""What board is this, how hot is it, and how much power is it drawing.

Energy per fix is the headline measurement of the wider study and no onboard
AVL paper reports it, so the probe is part of the runtime rather than a
separate benchmarking script. Every board reports power differently and some
do not report it at all; the contract here is that a missing reading returns
None and never a zero, because a zero silently becomes a J/fix of zero and
that has already cost one run.
"""
from __future__ import annotations

import glob
import os
import re
import subprocess
from dataclasses import dataclass, field


@dataclass
class Board:
    kind: str                 # "jetson" | "pi" | "generic"
    model: str
    arch: str
    cores: int
    ram_gb: float
    l4t: str = ""
    jetpack_hint: str = ""
    cuda_visible: bool = False
    power_source: str = "none"
    notes: list = field(default_factory=list)


def _read(path: str, default: str = "") -> str:
    try:
        with open(path) as fh:
            return fh.read().strip("\x00").strip()
    except OSError:
        return default


def _l4t_release() -> str:
    txt = _read("/etc/nv_tegra_release")
    m = re.search(r"R(\d+).*?REVISION:\s*([\d.]+)", txt)
    return f"{m.group(1)}.{m.group(2)}" if m else ""


_JETPACK_BY_L4T_MAJOR = {
    "32": "JetPack 4.x  (CUDA 10.2, Python 3.6 -- torch>=1.10 is not available here)",
    "35": "JetPack 5.x  (Ubuntu 20.04, Python 3.8, CUDA 11.4 -- the ceiling for Xavier)",
    "36": "JetPack 6.x  (Ubuntu 22.04, Python 3.10 -- Orin only, not Xavier)",
    "38": "JetPack 7.x  (Ubuntu 24.04 -- Thor only)",
}


def detect() -> Board:
    # device-tree first: that is where every board this project targets -- Pi,
    # Xavier, Orin -- states what it is. x86 machines have no device tree, so
    # `model` came back "unknown" on the laptop, which is what board.json has
    # been recording for it and what forced log_layout to map an ARCHITECTURE
    # to a machine name. DMI is x86's equivalent and product_version carries
    # the friendly string ("Legion Pro 5 16IRX10") where product_name is a
    # sales code ("83NN").
    model = (_read("/proc/device-tree/model")
             or _read("/sys/firmware/devicetree/base/model")
             or _read("/sys/devices/virtual/dmi/id/product_version")
             or _read("/sys/devices/virtual/dmi/id/product_name"))
    arch = os.uname().machine
    cores = os.cpu_count() or 1
    try:
        ram_gb = round(os.sysconf("SC_PAGE_SIZE") * os.sysconf("SC_PHYS_PAGES") / 1e9, 1)
    except (ValueError, OSError):
        ram_gb = 0.0

    l4t = _l4t_release()
    notes: list = []
    if l4t or "NVIDIA" in model or "Jetson" in model or "tegra" in model.lower():
        kind = "jetson"
        major = l4t.split(".")[0] if l4t else ""
        jp = _JETPACK_BY_L4T_MAJOR.get(major, "")
        if "Xavier" in model:
            notes.append(
                "AGX Xavier tops out at JetPack 5.1.x. JetPack 6 is Orin-only, so "
                "do not follow Orin instructions for CUDA, TensorRT or wheels."
            )
        if major == "35":
            notes.append(
                "System Python here is 3.8. torch publishes cp38 aarch64 wheels only "
                "up to 2.4.x -- bootstrap.sh pins accordingly."
            )
    elif "Raspberry Pi" in model:
        kind = "pi"
        jp = ""
    else:
        kind = "generic"
        jp = ""
        notes.append("Not an embedded board. Timing and energy from here are not board results.")

    cuda_visible = bool(glob.glob("/dev/nvidia*")) or os.path.exists("/dev/nvhost-gpu")
    return Board(
        kind=kind, model=model or "unknown", arch=arch, cores=cores, ram_gb=ram_gb,
        l4t=l4t, jetpack_hint=jp, cuda_visible=cuda_visible,
        power_source=_power_source(kind), notes=notes,
    )


# -- power -----------------------------------------------------------------
# Jetson carries INA3221 rails; the sysfs layout moved between L4T releases,
# so every known location is tried and the one that answered is recorded.
#
# hwmon naming is not a detail here. The convention is fixed: inN_input is a
# VOLTAGE in millivolts, currN_input a CURRENT in milliamps, powerN_input a
# POWER in microwatts. This list used to lead with `in*_input` and multiply it
# by 1000 as though it were power, which sums the board's rail voltages and
# calls the total watts -- a number that looks plausible, barely moves under
# load, and would make every joule-per-fix in the study wrong. It stayed hidden
# only because these files are mode 400 on L4T 35, so the read failed and the
# function correctly returned None; a udev rule added to "make power work" would
# have turned a silent absence into a confident fabrication.
#
# Verified on this AGX Xavier, 4 Sept 2026: chips 1-0040 and 1-0041 expose
# in1..in7_input and curr1..curr4_input, and NO powerN_input at all. So the
# third entry is the one that actually applies to JetPack 5, and it needs the
# voltage x current pairing below rather than a plain sum.
_JETSON_POWER_GLOBS = [
    "/sys/bus/i2c/drivers/ina3221/*/hwmon/hwmon*/power*_input",     # microwatts, if exposed
    "/sys/bus/i2c/drivers/ina3221x/*/iio:device*/in_power*_input",  # milliwatts, L4T 32 (JetPack 4)
    "/sys/bus/i2c/drivers/ina3221/*/hwmon/hwmon*/curr*_input",      # mA, paired with inN_input mV
]


def _power_source(kind: str) -> str:
    if kind == "jetson":
        for pat in _JETSON_POWER_GLOBS:
            if glob.glob(pat):
                return f"ina3221:{pat}"
        return "none"
    if kind == "pi":
        return "vcgencmd" if _which("vcgencmd") else "none"
    return "none"


def _which(cmd: str) -> bool:
    return any(os.access(os.path.join(p, cmd), os.X_OK) for p in os.environ.get("PATH", "").split(":") if p)


def read_power_w(board: Board) -> float | None:
    """Total board power in watts, or None. Never returns 0.0 for 'unknown'."""
    if board.kind == "jetson":
        return _jetson_power_w()
    if board.kind == "pi":
        return _pi_power_w()
    return None


def _jetson_power_w() -> float | None:
    for pat in _JETSON_POWER_GLOBS:
        files = sorted(glob.glob(pat))
        if not files:
            continue
        total_uw, found = 0.0, False
        for f in files:
            base = os.path.basename(f)
            raw = _read(f)
            if not raw.lstrip("-").isdigit():
                continue
            val = float(raw)
            if base.startswith("curr"):
                # P = V x I. The matching voltage channel sits beside it as
                # inN_input in millivolts, so mV x mA = microwatts directly.
                # A current with no voltage beside it is not power and is
                # skipped rather than guessed at.
                volts = _read(os.path.join(os.path.dirname(f),
                                           base.replace("curr", "in", 1)))
                if not volts.lstrip("-").isdigit():
                    continue
                total_uw += val * float(volts)
            elif "power" in base:
                # hwmon powerN_input is microwatts; iio in_power*_input is milliwatts.
                total_uw += val if "hwmon" in f else val * 1000.0
            else:
                continue
            found = True
        if found and total_uw > 0:
            return round(total_uw / 1e6, 3)
    # None, never 0.0. A zero silently becomes a joules-per-fix of zero, and
    # that has already cost one run.
    return None


_PI_RAILS = ["VDD_CORE_A", "3V7_WL_SW_A", "3V3_SYS_A", "1V8_SYS_A", "DDR_VDD2_A", "DDR_VDDQ_A"]


def _pi_power_w() -> float | None:
    """vcgencmd uses TWO keywords: _A rails answer `current(n)=`, _V rails
    answer `volt(n)=`. Matching only `current(` leaves the volts dict empty and
    turns J/fix into NaN. That happened once; hence the explicit pairing."""
    try:
        total = 0.0
        got = False
        for rail in _PI_RAILS:
            volt_rail = rail[:-2] + "_V"
            a = _vcgencmd_value(rail)
            v = _vcgencmd_value(volt_rail)
            if a is not None and v is not None:
                total += a * v
                got = True
        return round(total, 3) if got else None
    except Exception:
        return None


def _vcgencmd_value(rail: str) -> float | None:
    try:
        out = subprocess.run(["vcgencmd", "pmic_read_adc", rail], capture_output=True,
                             text=True, timeout=2).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    m = re.search(r"(?:current|volt)\(\d+\)=([\d.eE+-]+)", out)
    return float(m.group(1)) if m else None


def read_temp_c() -> float | None:
    temps = []
    for f in glob.glob("/sys/class/thermal/thermal_zone*/temp"):
        raw = _read(f)
        if raw.lstrip("-").isdigit():
            t = float(raw)
            temps.append(t / 1000.0 if t > 200 else t)
    return round(max(temps), 1) if temps else None


def throttled() -> str | None:
    """Pi reports a bitmask; Jetson has no single equivalent, so None there."""
    try:
        out = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True,
                             text=True, timeout=2).stdout.strip()
        return out.split("=", 1)[1] if "=" in out else None
    except (OSError, subprocess.SubprocessError, IndexError):
        return None


def summary() -> dict:
    b = detect()
    return {
        "kind": b.kind, "model": b.model, "arch": b.arch, "cores": b.cores,
        "ram_gb": b.ram_gb, "l4t": b.l4t, "jetpack": b.jetpack_hint,
        "cuda_devices_present": b.cuda_visible, "power_source": b.power_source,
        "power_w": read_power_w(b), "temp_c": read_temp_c(), "throttled": throttled(),
        "notes": b.notes,
    }


if __name__ == "__main__":
    import json
    print(json.dumps(summary(), indent=2))
