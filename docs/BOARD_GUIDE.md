# Running GeoAnchor on any board

Written 10 Sept 2026. This is the doc to hand someone — including A/Prof Sharma
— who asks "how do I start the whole thing".

Everything here assumes the runtime repo (`geoanchor-rt/`), not the parent
research repo. Nothing in `geoanchor/` or `configs/` contains an absolute path,
so the same checkout runs from any directory on any board.

---

## The 60-second version

```bash
git clone https://github.com/Saikrishna-Bulusu/geoanchor-rt.git ~/geoanchor-rt
cd ~/geoanchor-rt
bash bootstrap.sh
bash scripts/check_board.sh
```

`check_board.sh` tells you whether this board can do the job, and what it can
do instead if it cannot. Then:

```bash
python -m geoanchor.data_layer --build-map
bash run.sh
```

Open `http://<board-ip>:8000`.

---

## Do not skip `check_board.sh`

It was written on 10 Sept 2026 because board viability had been decided twice
by extrapolation and both times the extrapolation was wrong in a way that
mattered. It measures instead of guessing, and it answers three questions in
order, stopping at the first hard no:

1. **Does the runtime fit in RAM?** The processing layer alone peaks at
   **351 MB** with the model loaded and one 1024 px detect done — measured, not
   estimated. Four layers plus a headless OS lands around 540–600 MB. This is
   the only *hard* wall in the system.
2. **Which matchers are even importable?** torch, the weights, EdgePoint2.
3. **Does a matcher that actually localises fit the 250 ms budget?**

That third question is the one to understand, because it has a trap in it.

### The trap: fitting the budget and working are different things

From the env80 sweep, 326 real frames over a real satellite reference:

| matcher | fits a slow board? | localises? |
|---|---|---|
| ORB | easily | **zero plausible fixes in 192 Scene_10 frames** |
| SIFT | easily | **zero plausible fixes in 192 Scene_10 frames** |
| AKAZE | easily | 6 of 192 |
| xfeat_mnn | no, on most boards | yes |
| edgepoint2_s64 | no, on most boards | yes, and fewer catastrophic failures |

A board that runs ORB in 12 ms has not solved the problem. `check_board.sh`
marks these with `fits*` and a footnote for exactly this reason, and refuses to
call a board viable on the strength of them.

### The other trap: `OVERHEAD_MS` is not a board constant

It is **capture + preprocess + bus + decode**, and capture dominates it. Two
measurements in this project, same camera model:

    Xavier AGX     45.7 ms median    51.3 p95
    Pi 5          132.1 ms median   138.8 p95

A 3× spread, and **every millisecond of it is the camera, not the board** — a
C270 under indoor lighting delivers ~8 fps against a negotiated 30, and
`UvcFeed.read()` cannot return faster than frames arrive. Better lighting, a
fixed exposure, or a different sensor moves this number a long way.

So: measure it on *your* rig before believing any verdict.

```bash
python scripts/measure_overhead.py
bash scripts/check_board.sh --overhead <the median it printed>
```

Without `--overhead`, the script uses 45.7 ms and says loudly that it is a
placeholder.

### Pin the clocks first, or the numbers are fiction

**The governor string is not the test.** `performance` only requests the top of
whatever range `scaling_min_freq`/`scaling_max_freq` allow. This project has
lost two runs to that, once on each board.

```bash
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_min_freq
```

Jetson: `sudo nvpmodel -m 0 && sudo jetson_clocks`.

Neither survives a reboot. `check_board.sh` and `bench_matchers.py` both check
`scaling_min_freq == scaling_max_freq` and warn if it does not hold. They also
warn if `loadavg` is above a quarter of the core count, because contention
looks exactly like a code regression in the torch path and has been misread as
one twice.

---

## Per-board notes

### Raspberry Pi 5 (8 GB) — measured, works, misses the budget on this camera

The reference board. Everything below is measured, `performance` pinned.

    matcher            1 tile / 2048 refkp
    orb                     67.9 ms
    akaze                   81.3 ms
    sift                   183.7 ms
    edgepoint2_s64         280.4 ms
    xfeat_mnn              309.7 ms
    xfeat_lg              3638.0 ms

With the measured 132 ms overhead only ~118 ms is left, so **on the C270 rig,
today, only ORB and AKAZE fit — and neither localises.** With a camera that
delivers its rated frame rate the overhead drops toward the Xavier's 45.7 ms,
leaving ~204 ms, and `edgepoint2_s64` at 280 ms is still over by ~76 ms.

Setup:

```bash
sudo apt install -y python3-venv libgl1 libglib2.0-0 git
bash bootstrap.sh
```

**torch must stay pinned below 2.11.** 2.11.0 dropped the
`platform_machine == "x86_64"` guard on its CUDA deps, so an unpinned install
on any ARM Linux board drags in ~1.1 GB of CUDA a Pi cannot execute.
`bootstrap.sh` pins it and then asserts `torch.version.cuda is None`.

**`opencv-python-headless` must stay pinned below 5.** OpenCV 5 moved the
2D-features constructors and `cv2.AKAZE_create` disappears, killing the AKAZE
baseline mid-run.

### Raspberry Pi 4B (4/8 GB) — untested here, expected to run and miss

**Run `check_board.sh` and believe it over this paragraph.** The estimate: the
A72 at 1.8 GHz against the Pi 5's A76 at 2.4 GHz is roughly 2.5–3× slower on
this workload, plus a memory-bandwidth gap that hits the matmul-heavy match
stage. That puts `edgepoint2_s64` around 700–840 ms and `xfeat_mnn` similar.

RAM is fine on either variant — 4 GB against a ~600 MB need.

So the honest expectation is **it runs, produces fixes, and does not close the
loop** — the same category as the Pi 5, further from the line. That makes it a
legitimate point on the joules-per-fix curve, which is the contribution no
onboard AVL study reports. Measure it, record the energy, report it.

Identical setup to the Pi 5. Use a 64-bit OS — a 32-bit userland has no
usable torch wheel.

### Raspberry Pi 3B (1 GB) — will run, slowly; RAM is tight not fatal

1 GB against a measured ~600 MB need leaves ~300 MB of headroom on a headless
image. It fits, but not comfortably — run headless, and expect the OOM killer
if anything else is resident.

Speed: the A53 is in-order and clocked at 1.2 GHz. Expect roughly 8–10× the
Pi 5, so ORB lands near 550 ms and anything that localises is measured in
seconds per fix.

**Use it as a curve point and a MAVLink bridge, not a matcher host.** Set
`processing_layer.method` to `orb` if you want it to complete at all, and treat
the accuracy as the known-bad number it is.

If it thrashes, add swap before concluding it cannot run:

```bash
sudo dphys-swapfile swapoff
sudo sed -i 's/^CONF_SWAPSIZE=.*/CONF_SWAPSIZE=1024/' /etc/dphys-swapfile
sudo dphys-swapfile setup && sudo dphys-swapfile swapon
```

Swap on an SD card is slow enough to distort every timing number, so do this
only to get it *running*, and never quote a timing taken with swap active.

### Raspberry Pi Zero 2W (512 MB) — will not run the matcher

The one board with a hard, measured no. 512 MB total against **351 MB for the
processing layer alone**, before the data, output and API layers and the OS.
`check_board.sh` exits at step 1 here and does not bother timing anything.

Speed would also rule it out — same A53 cores as the Pi 3B at a lower clock —
but RAM gets there first, and no matcher choice or config change moves it.

It also has no ethernet, which removes it from the HITL setup below.

**What it is genuinely good for:** a MAVLink telemetry bridge or a log shipper.
Both are useful and neither needs torch:

```bash
pip install pymavlink pyzmq
python -m geoanchor.output_layer --loop-mode off
```

### Jetson Orin Nano (7/15 W) — the strongest board you have

Set the power mode and pin the clocks before anything:

```bash
sudo nvpmodel -m 0
sudo jetson_clocks
sudo nvpmodel -q
```

**Check `nproc` before believing any number off a Jetson.** The board boots
into a mode that onlines a subset of cores while nothing errors — on the Xavier
that meant four of eight cores at a reported "4 cores", which reads as the
board's spec rather than as a mode. It roughly halved the solve time when
corrected.

JetPack 6 is Orin-only; do **not** follow Xavier (JetPack 5) instructions for
wheels or flashing. Python 3.10 on JetPack 6 takes modern torch, so the
Xavier's `torch<=2.4` cap does not apply.

**The CUDA question, deliberately left closed.** The Orin Nano has 1024 CUDA
cores and `methods.py` hardcodes `torch.device("cpu")`. That is not an
oversight — it is what makes joules-per-fix compare *boards* rather than
implementations, which is the headline contribution. The decision taken
10 Sept 2026 was to **keep CPU-only everywhere.** If that is ever revisited,
add a device knob and report the CUDA path as a separate labelled row; do not
silently switch one board.

Consequence: on CPU the Orin Nano's A78AE cores are roughly Pi 5-class, so
expect it to *also* miss the 250 ms budget. Measure it.

---

## Once it runs

```bash
bash verify.sh    # 46 checks against the specification, ~3 min
bash sweep.sh     # every env80 frame through the real modules
```

`verify.sh` runs the real processes over the real bus, kills a layer to prove
the other two survive, checks the export, and round-trips real MAVLink through
a transcription of ArduPilot's own handler. Run it after any change to a layer
boundary.

### Things that will bite, in the order they usually do

1. **`data` is a symlink, per board, and gitignored.** Config paths resolve
   against the repo root, so `data/sydney/ref_tile.tif` only resolves through
   it. Make it before `bootstrap.sh`:
   `ln -s /path/to/datasets data`
2. **A feature store holds descriptors from ONE extractor.** Changing
   `processing_layer.method` without rebuilding the store is `PLE-14`, not a
   bad result. The store id hashes the method in so the two cannot silently
   disagree.
3. **EdgePoint2 is not vendored.** `configs/system.yaml` ships
   `edgepoint2_s64`, and without the clone the data layer logs `DLE-01` and
   publishes nothing — the processing layer then sits on `PLDE-01` for the
   whole run and you get a session with zero records. Clone it:
   `git clone --depth 1 https://github.com/HITCSC/EdgePoint2 edgepoint2`
4. **`fps: auto` is the default and paces off measured `stage_ms`.** Do not
   pin a frame rate to "get more fixes" — asking for more frames than the board
   can match makes latency worse and returns no extra fixes.
5. **Never quote the mean, and never RMSE.** Satellite runs contain fixes wrong
   by up to 2.06e93 m. Median, p90, p99, and the fraction inside 5/10/20 m.

---

## Log collection and cross-device sync

See [`docs/LOG_SYNC.md`](LOG_SYNC.md). Every board pushes its session
transcripts to a shared repo; the dashboard on the laptop reads a local clone,
so every device's runs are visible in one place.
