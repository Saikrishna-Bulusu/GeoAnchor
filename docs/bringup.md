# Bring-up, in order

Each step answers one question and produces one number. Do them in this order:
later steps are meaningless if an earlier one has not been done, and step 1 in
particular decides whether the rest of the deployment story holds.

Every command below is a single runnable line.

---

## 0. The board

```bash
bash bootstrap.sh
```

On a Jetson, before **any** timing measurement, once per boot:

```bash
sudo nvpmodel -m 0 && sudo jetson_clocks && sudo nvpmodel -q
```

Record what `nvpmodel -q` printed in the run notes. A timing number without a
power mode next to it is not comparable with anything.

```bash
python scripts/preflight.py
```

---

## 1. OVERHEAD_MS — do this first

**The one number nobody has measured, and everything else is downstream of it.**

The Pi 5 run of 2 Sept 2026 found XFeat sparse fits the 250 ms budget warm with
48 ms of margin, against an assumed overhead of 40 ms that was a guess. Capture,
ISP, copy, encode and the bus hop above 88 ms and it misses. Below, it fits.

```bash
python scripts/measure_overhead.py --device 0 --n 200
```

Use the real camera at the real resolution. N=30 under-samples the tail badly —
on the Pi 5 the cold p95 moved 17% going from N=30 to N=200 with no throttling.

Read the result against your matcher's warm p95. If the overhead eats the
margin, the levers in order of cost are: drop the capture resolution, lower
`top_k`, or move to a faster matcher and state the accuracy price.

---

## 2. fx_px — the rescale depends on it entirely

```bash
python scripts/calibrate_camera.py --device 0 --rows 6 --cols 9 --square 25
```

`GSD = altitude / fx_px` is the whole altitude-adaptive rescale. An fx that is
5% wrong makes every frame 5% the wrong size and hands the matcher a scale gap
it did not need to bridge. **Do not take fx from a datasheet.**

Twenty views, varied angles and distances, flat well-lit board. RMS above 1 px
means re-shoot. Paste the printed block into `configs/system.yaml`, and note
that it is valid only at the resolution you calibrated at.

---

## 3. The reference map

```bash
python -m geoanchor.data_layer --build-map
```

Once per map. The store is keyed by the content hash of the source plus the
method, so re-running is a cache hit and switching descriptors builds a second
store rather than corrupting the first.

Check the store's GSD against your frame GSD at 50 m and at 100 m. If the ratio
sits outside roughly 0.3–3 across that band, the reference is at the wrong zoom
level for this flight envelope and no amount of matcher will fix it.

---

## 4. End to end, on the bench

```bash
bash run.sh
```

Open `http://<board>:8000`. All three layer cards should go live, the map
should draw, and records should accumulate.

With the shipped `configs/system.yaml` the frames are cut from the reference
map, so the error will be centimetres and the dashboard shows a warning banner.
That is a plumbing check. For numbers that mean anything:

```bash
GEOANCHOR_CONFIG=configs/env80.yaml bash run.sh
```

Real UAV frames over a real satellite basemap, with published ground truth.

---

## 5. The flight controller, in open loop

Start with SITL. There is no reason to find a wiring problem and an EKF
problem at the same time.

```bash
sim_vehicle.py -v ArduCopter --out=udp:127.0.0.1:14550
```

then, in another terminal:

```bash
python scripts/check_extnav.py --endpoint udpin:0.0.0.0:14550
```

It reads the parameters ArduPilot needs, then sends ODOMETRY carrying the
vehicle's **own** position back to it. That is the point: the message, the
origin, the covariance and the acceptance logic all get exercised with a
position that cannot mislead the filter, so every failure it finds is a
plumbing failure.

Parameters that must be right, or the fix is ignored in silence:

| parameter | value |
|---|---|
| `AHRS_EKF_TYPE` | 3 |
| `EK3_SRC1_POSXY` | 6 (ExternalNav) |
| `VISO_TYPE` | non-zero |
| `VISO_POS_X/Y/Z` | camera offset from the IMU, metres |
| `VISO_DELAY_MS` | your measured latency, 0–250 |

Then enable the link in `configs/system.yaml`:

```yaml
output_layer:
  fc:
    enabled: true
    loop_mode: open
```

Watch `posTestRatio`, not the feed rate. A fix failing the 5-sigma innovation
gate does not refresh `lastGpsPosPassTime_ms`, so sustained rejection walks the
filter to `posTimeout` on the 7 s clock while data is still arriving on time.

**The most common silent failure is the origin.** An ExternalNav position is
local. If the EKF origin and the origin the output layer used differ, every fix
carries a constant offset and nothing reports it.

---

## 6. Closed loop

Only after open loop has run clean on hardware for a full flight's worth of
data, and only with `min_quality_inliers` set from measured inlier counts on
**your** reference — not from the default, and not from the Sydney demo, where a
self-match returns 200+ inliers while a real satellite basemap returns 12–79.

Set `loop_mode: closed`. Every fix is still gated on the inlier floor, the 100 m
altitude cap and the absence of live device errors in any layer.

---

## What each step leaves behind

| step | artefact |
|---|---|
| 1 | OVERHEAD_MS p95, in the run notes and in `CLAUDE.md` |
| 2 | an `intrinsics` block in `configs/system.yaml` |
| 3 | a store under `stores/`, and its GSD |
| 4 | `runs/<stamp>_<tag>/session.json` |
| 5 | the parameter table above, confirmed, plus a clean open-loop run |
