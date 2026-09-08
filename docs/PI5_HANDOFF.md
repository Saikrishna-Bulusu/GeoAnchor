# Handing GeoAnchor to a second board

Written on the AGX Xavier, 8 Sept 2026, for the Raspberry Pi 5. Nothing here is
Pi-specific by accident -- it is the list of things that were learned the hard
way on this board and would otherwise be learned again on the next one.

Read `CLAUDE.md` first; it is the project's authoritative notes and it travels
in the repo. This file is only the part that is about *moving to another board*.

---

## 1. What to copy, and why git is not enough

The code goes over git. Four directories are gitignored and must travel
separately -- `bash scripts/pack_for_board.sh` makes exactly that tarball
(166 MB for the sydney replay; add `--with-env80` for the 2.4 GB AnyVisLoc
scenes):

| directory | size | why it cannot be regenerated on the Pi |
|---|---|---|
| `stores/` | 126 MB | Built feature stores. **Copying these is how the Pi avoids needing rasterio/GDAL at all.** |
| `data/sydney/` | 39 MB | The source GeoTIFF. Required even though the store is prebuilt -- see below. |
| `demo/` | 3.7 MB | `flight.mp4` + `flight_gt.jsonl`. The only feed that exists without a camera. |
| `results/` | 1.4 MB | The Xavier's numbers, so the Pi diffs against something instead of starting blank. |

Not copied on purpose: `xfeat/`, `edgepoint2/` (bootstrap.sh clones both) and
`.venv/` (not portable across architectures, and the Pi will want a different
torch wheel anyway).

**The source image must be present even when the store already exists.**
`store_id()` in `geoanchor/data_layer/store.py` hashes the source file's
*content*, not its path -- deliberately, so a store survives being copied
between machines and renamed. But that means the runtime opens `ref_tile.tif`
to compute the key before it can find the store. It reads it as raw bytes
(`open(source,"rb")`), so no GeoTIFF reader is involved on the cache-hit path:
`read_georeference` -- the only thing that imports rasterio -- sits *after* the
`store.exists()` early return in `build_store`. Ship the .tif, skip GDAL.

**Absolute paths inside `stores/*/manifest.json` are provenance, not
dependency.** `"source": "/data/datasets/sydney/ref_tile.tif"` records where
the store was built. Nothing resolves it at runtime. Do not "fix" it.

---

## 2. The five traps, in the order they will bite

1. **torch will try to install CUDA on the Pi.** PyTorch 2.11.0 dropped the
   `platform_machine == "x86_64"` guard on its CUDA extras and kept only
   `platform_system == "Linux"`. An unpinned `pip install torch` on any ARM
   Linux board now drags in cudnn, cublas, cusparselt and triton -- over a
   gigabyte that can never execute. `bootstrap.sh` step 5 pins `torch<2.11`
   and then *verifies* `torch.version.cuda is None`, failing loudly if not.
   Do not route around that check.

2. **No CUDA. This is a requirement, not an optimisation.** The professor's
   constraint is that the architecture stays reproducible on an Orin Nano, a
   TX1, a TX2 and a Pi 5, so joules-per-fix compares *boards* rather than four
   different implementations. Everything is CPU-only by design. If a Pi
   experiment ever seems to want a GPU path, it is out of scope.

3. **rasterio needs GDAL >= 3.1.** On the Xavier (Ubuntu 20.04, GDAL 3.0.4)
   the fix was `pip install "rasterio<1.3"`, which builds 1.2.10. On Pi OS
   Bookworm GDAL is 3.6, so plain `rasterio` should work -- but if the Pi is
   only running prebuilt stores, skip it entirely. Fastest path is the one
   where GDAL is never installed.

4. **Import order: torch before rasterio, on any aarch64 board.** Loading GDAL
   exhausts the process's static TLS surplus, and a torch imported *afterwards*
   dies with `libgomp-....so: cannot allocate memory in static TLS block`.
   It surfaces as `DLE-01 torch is not installed` on a machine where torch
   imports perfectly well on its own. `mapprep.build_store` already constructs
   the method before reading the georeference for exactly this reason; the
   comment there says so. Keep that ordering.

5. **EdgePoint2 has three non-obvious requirements**, all handled in
   `EdgePoint2Method` in `geoanchor/methods.py` and all easy to break:
   - Upstream does `torch.load('./weights/{cfg}.pth')` **relative to process
     cwd**, so `_load()` chdirs into the EdgePoint2 root inside a `try/finally`.
   - It takes **RGB in 0..1**. XFeat takes 0..255. Feeding it XFeat's range
     produces keypoints and garbage descriptors, which looks like a bad scene
     rather than a bug.
   - The matcher picks its reduction axis by reference size (`WIDE_REF = 20000`).
     Below that, one matmul and two reductions over it; above, two matmuls.
     Always using two regressed env80 Scene_09 match from 72 ms to 123 ms.
     This is a cache-shape effect, so **re-measure the crossover on the Pi** --
     20000 is a Xavier number and there is no reason it transfers.

---

## 3. Clocks: the Pi's equivalent of `jetson_clocks`

Timing numbers are only comparable at a fixed clock. On the Jetson that is
`sudo nvpmodel -m 0 && sudo jetson_clocks` (and note `nvpmodel` persists across
reboot while `jetson_clocks` does **not**). The Pi 5 analogue:

    # pin the governor
    echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
    # confirm it took: min should equal max
    grep . /sys/devices/system/cpu/cpu0/cpufreq/scaling_{min,max,cur}_freq
    # and check you are not throttling mid-run -- 0x0 is clean
    vcgencmd get_throttled

The Pi 5 throttles hard without active cooling, and a throttled run looks
exactly like a slow matcher. Check `get_throttled` **after** every benchmark,
not before. The canonical Pi table in `CLAUDE.md` was taken on `performance`
with no throttling, and anything compared to it must be too.

---

## 4. What the Xavier says, so the Pi has something to beat

Same frame, 646x484, MAXN, pinned, 8 threads. Full tables are in `CLAUDE.md`;
these are the two rows that matter for planning Pi work.

Matched geometry -- 1 tile, 2048 reference keypoints, p95 of 15 reps
(`python scripts/bench_matchers.py --tiles 1 --reps 15`):

    matcher            Pi 5 p95   Xavier p95   Xavier is
    orb                    56.6         96.6   1.71x slower
    sift                   98.9        197.5   2.00x slower
    akaze                 107.5        157.1   1.46x slower
    xfeat_mnn             161.6        291.9   1.81x slower
    xfeat_lg             2108.6       2655.9   1.26x slower
    edgepoint2_s64          272.5      223.9   <-- FILLED 8 Sept, and it REVERSED

That row landed on 8 Sept and did not go as predicted: the Pi's detection is
only 1.1x better (183.5 vs ~203 ms) while its matching is 4.3x WORSE (89.1 vs
20.7 ms), because detection does not thread and matching does -- so the
Xavier's eight cores win the matmul against the Pi's four. See "The Pi 5 wins
every matcher except EdgePoint2" in `CLAUDE.md`, including the `WIDE_REF`
hypothesis and the one-command test for it, which is now the open question.

**The original note, kept because the reasoning is what was wrong:** EdgePoint2 is the
pipeline's current default (`configs/system.yaml`, gate 8, k=2048) and it has
never run on a Pi. If the 1.4-2.0x per-core gap holds, its 203 ms detection
lands near 110 ms there, which is the difference between missing the EKF3
budget and fitting inside it.

The gap is per-core, not throughput: on the Xavier, 8 threads buys only 1.26x
on detection and 1.57x on matching over 1 thread. Carmel is a 2018 core; the
Pi's A76 wins per clock. **The Xavier is a bench board, not a flight step** --
treat its numbers as an upper bound on cost.

Do not compare any number to the Pi table without checking the tile count
behind it. Match cost is linear in reference keypoints, and 1 tile vs 25 tiles
on the same board and frame is 28.6 ms vs 439 ms. An apparent "the Xavier
loses to a Pi by 10x" was entirely this, and was not a board result at all.

---

## 5. First three things to run on the Pi

    bash bootstrap.sh                                  # expect step 5 to be slow
    python scripts/bench_matchers.py --tiles 1 --reps 15   # fills the table above
    bash run.sh                                        # sydney replay, dashboard on :8000

Then, in order of what the project actually needs:

1. `edgepoint2_s64` at 1 tile and at 9 tiles, so the Pi has both the
   like-geometry row and the deployed-geometry row.
1. **Re-measure `feed.fps`. Do not inherit the Xavier's `2`.** It is tuned to
   a ~410 ms fix; if the Pi matches in 250 ms the right value is 4. The curve
   is not monotonic -- staleness falls from 4 to 16 fps because the drain
   conflates to the newest frame, then reverses when the data layer's encoding
   starts stealing cores from the matcher. The sweep table and the reasoning
   are in the `feed:` block of `configs/system.yaml`; run the same five arms
   (`loop: true`, 100 s each) and set the Pi's value from its own numbers.
2. Re-measure `WIDE_REF`: sweep reference size across the one-matmul /
   two-matmul crossover and find where the Pi's cache flips it.
3. `OVERHEAD_MS`, if a camera is available. Capture to first byte at the
   matcher is still a *guess* of 40 ms on both boards, and it is the number
   every deployment conclusion is downstream of. `configs/camera.yaml` exists
   for exactly this and its header explains what the run can and cannot tell
   you. 200 frames is enough.
