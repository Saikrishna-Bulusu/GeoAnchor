"""Step-code registry for the three layers.

Every meaningful thing a layer does emits a code. Three kinds per layer:

    <L>-nn      a step completed
    <L>E-nn     an error the layer handled (logic, data, protocol)
    <L>DE-nn    a device error (hardware, driver, OS, link)

The distinction matters operationally: an E code means this fix is bad, a DE
code means the board or a peripheral is bad. The dashboard colours them
differently and the output layer refuses closed-loop on any live DE.

Codes are append-only. Never renumber one -- old session JSON files carry
these strings and a renumber silently rewrites history.
"""
from __future__ import annotations

STEP, ERROR, DEVICE = "step", "error", "device"

# --------------------------------------------------------------------------
# Data layer
# --------------------------------------------------------------------------
DL = {
    "DL-01": "layer process started",
    "DL-02": "configuration loaded and validated",
    "DL-03": "publish socket bound",
    "DL-04": "map source opened",
    "DL-05": "map georeference read (CRS, transform, GSD)",
    "DL-06": "map tiled into the pyramid level for this altitude band",
    "DL-07": "reference features extracted from map tiles",
    "DL-08": "feature store written to disk",
    "DL-09": "feature store cache hit -- preprocessing skipped",
    "DL-10": "map packet published",
    "DL-11": "feed opened",
    "DL-12": "first frame captured",
    "DL-13": "frame preprocessed (undistort, grayscale, scale)",
    "DL-14": "frame packet published",
    "DL-15": "GPS link opened",
    "DL-16": "first actual GPS fix received",
    "DL-17": "GPS packet published",
    "DL-18": "all three inputs live -- layer ready",
    "DL-19": "feed reached end of stream",
    "DL-20": "layer stopped cleanly",
    # Appended after DL-20 because this registry is append-only, not because
    # it belongs at the end of the layer's life. See "Step codes" in CLAUDE.md.
    "DL-21": "feed rate re-paced to the processing layer",
}
DLE = {
    "DLE-01": "configuration invalid or missing a required key",
    "DLE-02": "map file not found at the configured path",
    "DLE-03": "map has no CRS or no affine transform -- not georeferenced",
    "DLE-04": "map GSD outside the supported range for 50-100 m AGL",
    "DLE-05": "feature store version mismatch -- rebuilding",
    "DLE-06": "feature store corrupt or unreadable -- rebuilding",
    "DLE-07": "rasterio unavailable -- cannot ingest a new GeoTIFF here",
    "DLE-08": "frame decode failed",
    "DLE-09": "frame dropped, subscriber not keeping up",
    "DLE-10": "GPS message malformed",
    "DLE-11": "GPS stale beyond the configured timeout",
    "DLE-12": "altitude outside the 50-100 m AGL envelope",
    "DLE-13": "no camera intrinsics configured -- undistortion skipped",
    "DLE-14": "fps: auto requested on a feed whose rate this layer does not control",
}
DLDE = {
    "DLDE-01": "camera device node not present",
    "DLDE-02": "camera open failed or device busy",
    "DLDE-03": "camera disconnected mid-run",
    "DLDE-04": "MAVLink endpoint not present (serial port or UDP bind)",
    "DLDE-05": "MAVLink link lost -- no heartbeat within timeout",
    "DLDE-06": "disk full while writing the feature store",
    "DLDE-07": "insufficient memory to preprocess this map",
    "DLDE-08": "publish socket bind failed -- address already in use",
}

# --------------------------------------------------------------------------
# Processing layer
# --------------------------------------------------------------------------
PL = {
    "PL-01": "layer process started",
    "PL-02": "configuration loaded and validated",
    "PL-03": "publish socket bound",
    "PL-04": "method registry loaded",
    "PL-05": "method initialised and weights resident",
    "PL-06": "subscribed to the data layer",
    "PL-07": "map packet received, feature store attached",
    "PL-08": "layer ready",
    "PL-09": "frame received",
    "PL-10": "attitude rectification applied",
    "PL-11": "search region selected from prior and altitude",
    "PL-12": "frame features extracted",
    "PL-13": "correspondences established",
    "PL-14": "geometric verification passed",
    "PL-15": "frame pixel mapped to map pixel and then to geodetic",
    "PL-16": "uncertainty estimated",
    "PL-17": "fix published",
    "PL-18": "layer stopped cleanly",
    "PL-19": "data layer reported end of feed -- link watchdog stood down",
}
PLE = {
    "PLE-01": "no map packet yet -- waiting, not failing",
    "PLE-02": "requested method is not in the registry",
    "PLE-03": "method weights missing on this machine",
    "PLE-04": "too few keypoints in the frame",
    "PLE-05": "too few correspondences to fit a model",
    "PLE-06": "geometric verification failed",
    "PLE-07": "inlier count below the rejection gate -- fix rejected",
    "PLE-08": "solution implausible (outside the map, or a degenerate solve)",
    "PLE-09": "latency budget exceeded -- fix is late for EKF3 delay compensation",
    "PLE-10": "frame skipped, still busy with the previous one",
    "PLE-11": "altitude outside the 50-100 m envelope -- fix marked unreliable",
    "PLE-12": "no attitude available -- rectification skipped, matcher sees rotation",
    "PLE-13": "frame processing raised an unhandled error -- frame skipped, layer alive",
    "PLE-14": "method and feature store disagree -- store rebuilt with the wrong descriptor",
    "PLE-15": "prior lies outside the reference map -- prior dropped, next frame searches cold",
    "PLE-16": "prior is older than the configured max age -- dropped, searching cold",
}
PLDE = {
    "PLDE-01": "cannot connect to the data layer endpoint",
    "PLDE-02": "out of memory during feature extraction",
    "PLDE-03": "thermal throttling detected -- timings are not comparable",
    "PLDE-04": "accelerator requested but unavailable -- running on CPU",
    "PLDE-05": "feature store unreadable from this process",
    "PLDE-06": "publish socket bind failed -- address already in use",
}

# --------------------------------------------------------------------------
# Output layer
# --------------------------------------------------------------------------
OL = {
    "OL-01": "layer process started",
    "OL-02": "configuration loaded and validated",
    "OL-03": "publish socket bound",
    "OL-04": "subscribed to the processing layer",
    "OL-05": "subscribed to the actual GPS stream",
    "OL-06": "session record file opened",
    "OL-07": "layer ready",
    "OL-08": "fix paired with an actual GPS sample",
    "OL-09": "position error computed",
    "OL-10": "calibration loss computed",
    "OL-11": "record appended to the session file",
    "OL-12": "record published to the dashboard",
    "OL-13": "flight controller link opened",
    "OL-14": "predicted position sent to the flight controller (closed loop)",
    "OL-15": "open loop -- predicted position logged, not sent",
    "OL-16": "session file flushed",
    "OL-17": "layer stopped cleanly",
    "OL-18": "QGroundControl link opened",
    "OL-19": "predicted position published to QGroundControl",
}
OLE = {
    "OLE-01": "no actual GPS available to pair against",
    "OLE-02": "pairing window exceeded -- fix and GPS too far apart in time",
    "OLE-03": "fix was rejected upstream -- recorded, not sent",
    "OLE-04": "position error above the alarm threshold",
    "OLE-05": "session file write failed",
    "OLE-06": "closed loop requested but fix quality is below the floor -- refused",
    "OLE-07": "altitude above 100 m -- fix marked unreliable and not sent",
    "OLE-08": "a device error is live in another layer -- closed loop refused",
}
OLDE = {
    "OLDE-01": "flight controller endpoint not present",
    "OLDE-02": "flight controller link lost",
    "OLDE-03": "MAVLink send failed",
    "OLDE-04": "disk full while writing the session file",
    "OLDE-05": "cannot connect to an upstream layer endpoint",
    "OLDE-06": "publish socket bind failed -- address already in use",
    "OLDE-07": "QGroundControl endpoint unreachable -- display only, flight unaffected",
}

_TABLES = (("DL", DL), ("DLE", DLE), ("DLDE", DLDE), ("PL", PL), ("PLE", PLE),
           ("PLDE", PLDE), ("OL", OL), ("OLE", OLE), ("OLDE", OLDE))

REGISTRY: dict[str, str] = {**DL, **DLE, **DLDE, **PL, **PLE, **PLDE, **OL, **OLE, **OLDE}


def _duplicates() -> list[str]:
    """Codes defined more than once, across all nine tables.

    This cannot be found by inspecting REGISTRY: ** merging collapses a repeat
    at construction time and the later definition silently wins, so the merged
    dict is simply one entry shorter and validate() sees nothing wrong. The
    registry is append-only precisely because old session files carry these
    strings, and a code that quietly changed meaning rewrites the history of
    every run that used it -- so the check has to happen before the merge.
    """
    seen, dupes = {}, []
    for table_name, table in _TABLES:
        for code in table:
            if code in seen:
                dupes.append(f"{code}: defined in both {seen[code]} and {table_name}")
            else:
                seen[code] = table_name
    return dupes

_LAYER_OF = {"DL": "data", "PL": "processing", "OL": "output"}


def describe(code: str) -> str:
    return REGISTRY.get(code, "unknown code")


def kind(code: str) -> str:
    """step | error | device, decided by the prefix before the dash."""
    head = code.split("-", 1)[0]
    if head.endswith("DE"):
        return DEVICE
    if head.endswith("E"):
        return ERROR
    return STEP


def layer(code: str) -> str:
    return _LAYER_OF.get(code[:2], "unknown")


def validate() -> list[str]:
    """Return the list of problems with the registry. Empty means healthy."""
    bad = _duplicates()
    for c in REGISTRY:
        head, _, num = c.partition("-")
        if not num.isdigit() or len(num) != 2:
            bad.append(f"{c}: number must be two digits")
        if head[:2] not in _LAYER_OF:
            bad.append(f"{c}: unknown layer prefix")
        if kind(c) not in (STEP, ERROR, DEVICE):
            bad.append(f"{c}: unclassifiable")
    return bad


if __name__ == "__main__":
    problems = validate()
    for layer_prefix in ("DL", "PL", "OL"):
        codes = [c for c in REGISTRY if c.startswith(layer_prefix)]
        steps = [c for c in codes if kind(c) == STEP]
        errs = [c for c in codes if kind(c) == ERROR]
        devs = [c for c in codes if kind(c) == DEVICE]
        print(f"{_LAYER_OF[layer_prefix]:11s} {len(steps):3d} steps  {len(errs):3d} errors  {len(devs):3d} device errors")
    print(f"total        {len(REGISTRY)} codes")
    print("registry OK" if not problems else "\n".join(problems))
