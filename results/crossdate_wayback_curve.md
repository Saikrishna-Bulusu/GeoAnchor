# How old can the reference map be?

Measured 10 Sept 2026 on the Legion. **Esri World Imagery Wayback**, 13 distinct
captures over the Sydney CBD tile, all refetched at **zoom 19 = 0.2476 m/px** so
that resolution is held constant and only the capture date varies.

Query: `wayback_z19/ref_tile_2026-03-26.tif`, 24 frames of 512 px, cut from the
newest capture and matched against each older one. `edgepoint2_s64`,
`max_keypoints 2048`, inlier gate 8, seed 0, cold start.

Reproduce:

```bash
python scripts/fetch_wayback_tile.py --match data/sydney/ref_tile.tif --all \
    --zoom 19 --outdir data/sydney/wayback_z19
python scripts/crossdate_probe.py \
    --query data/sydney/wayback_z19/ref_tile_2026-03-26.tif \
    --reference data/sydney/wayback_z19/ref_tile_2023-06-13.tif
```

## The curve

| capture | gap, years | inliers (med) | solved / 24 | median | p90 |
|---|---|---|---|---|---|
| 2026-03-26 | 0.0 *(control)* | 91 | 24 | **0.029 m** | 0.050 |
| 2025-12-18 | 0.3 | 31 | 24 | **3.34 m** | 6.30 |
| 2023-08-10 | 2.6 | 21 | 23 | **6.84 m** | 14.81 |
| 2023-06-29 | 2.7 | 24 | 23 | **7.90 m** | 19.19 |
| 2023-06-13 | 2.8 | 20 | 22 | **7.58 m** | 14.83 |
| 2022-08-10 | 3.6 | 5 | 1 | 81.92 m | — |
| 2021-11-03 | 4.4 | 5 | 0 | — | — |
| 2021-02-24 | 5.1 | 5 | 2 | 85.15 m | — |
| 2020-03-23 | 6.0 | 5 | 1 | 72.31 m | — |
| 2019-03-13 | 7.0 | 6 | 5 | 12.91 m | 116.18 |
| 2017-11-16 | 8.4 | 5 | 5 | 21.46 m | 35.76 |
| 2017-10-25 | 8.4 | 5 | 1 | 54.58 m | — |
| 2014-07-02 | 11.7 | 5 | 1 | 55.88 m | — |

## What it says

**There is a cliff between 2.8 and 3.6 years, and it is sharp.**

- Inside ~3 years: **20–31 inliers, 92–100% solved, 3.3–7.9 m median.** That is
  a working system, and the error is the same order as the 2.5–3.8 m env80
  gets against real satellite reference.
- Past ~3.6 years: **5 inliers, 0–2 of 24 solved, and the few that survive are
  wrong by 55–85 m.** Those are not degraded fixes, they are confident fixes on
  the wrong building — the same failure mode ORB and SIFT show on real
  reference, and the reason this project gates on inliers rather than matches.

The inlier count is the tell, not the error: it drops 20 → 5 across the cliff
and never recovers at any older epoch. A gate of 8 correctly rejects almost
everything past it, which is the gate doing its job.

**So: refresh the reference map at least every 3 years.** That is the
deployment consequence, and it is a number nobody had.

## Two traps this measurement walked into first

**Resolution is a confound and it hides the cliff.** The first pass fetched
each release at its own deepest zoom — 0.124 m/px for the four post-2023
captures, 0.248 for the nine older ones. That conflates "older" with "coarser"
and produced a curve that looked like a smooth decline. Refetching everything
at zoom 19 is what made the cliff visible. **Hold resolution constant or the
result is about resolution.**

**A "distinct" release is not distinct everywhere.** `--list` hashes one zoom-18
tile per release to find the distinct captures. At that footprint 2025-12-18 and
2026-03-26 differ; over the CBD bbox at zoom 20 they are byte-identical, so the
first run's "0.023 m at a 0.3-year gap" was a control mislabelled as a result.
At zoom 19 they genuinely differ (31 inliers, 3.34 m). **Verify distinctness at
the resolution and footprint you are actually going to measure at.**

## The rows that need care

`2019-03-13` solved 5 of 24 at a 12.91 m median but a **116 m p90**, and
`2017-11-16` solved 5 at 21 m. Do not read these as partial recovery at 7–8
years. With 5–6 inliers against a gate of 8 these are a handful of marginal
solves out of 24 attempts, and their p90 says what they are. The honest summary
is that everything past the cliff is noise; the ordering inside the noise is not
a signal.

## Licence

Esri World Imagery is **not** CC BY — see the header of
`scripts/fetch_wayback_tile.py`. The numbers here are ours; the pixels are
working data for measurement and are not redistributable. The reference tile the
pipeline actually flies against is NSW Spatial Services (CC BY), built by
`step17`.
