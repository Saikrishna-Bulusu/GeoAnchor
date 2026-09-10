# Cross-date reference imagery, and what it costs you

Written 10 Sept 2026, answering: *find NSW imagery other than the one we are
using — or an older version — so we can match one against the other and get an
idea of the actual benchmarking stats.*

**The answer: refresh the reference map at least every three years.**

That number comes from **Esri's World Imagery Wayback archive**, which keeps 196
dated versions of the global basemap and gives 13 distinct captures of the
Sydney CBD spanning 2014–2026 — free, no token, at the same 0.124 m/px the
project's reference tile uses. Held at one resolution so only the date varies,
the matcher works to about **3 years** and falls off a cliff after it:

| gap | inliers | solved | median |
|---|---|---|---|
| 0.3 y | 31 | 24/24 | **3.34 m** |
| 2.8 y | 20 | 22/24 | **7.58 m** |
| **3.6 y** | **5** | **1/24** | **81.92 m** |

Full curve, method and the two traps it walked into:
[`../results/crossdate_wayback_curve.md`](../results/crossdate_wayback_curve.md).

```bash
python scripts/fetch_wayback_tile.py --match data/sydney/ref_tile.tif --list
python scripts/fetch_wayback_tile.py --match data/sydney/ref_tile.tif --all \
    --zoom 19 --outdir data/sydney/wayback_z19
```

The rest of this file is the NSW route, which was tried first. It is worth
reading because NSW imagery is **CC BY** and Esri's is not — the pipeline flies
against NSW, and Wayback is measurement-only working data.

---

## The problem this was meant to solve

`demo/flight.mp4` cuts its frames out of `data/sydney/ref_tile.tif`. The
pipeline is therefore matching a picture against a copy of that same picture,
and reports **0.007 m**. That proves the wiring is connected and nothing else.
Its whole job is to be sub-millimetre; if it ever stops, something is broken.

An honest localisation result needs the query and the reference to be
**independent captures of the same ground**.

---

## What NSW actually publishes

The mosaic behind `LPI_Imagery_Best` carries several dated sources over any
given point. Querying its Footprint layer at the Sydney CBD returns four:

| capture | date | resolution |
|---|---|---|
| `sydney_2013_09_50cm` | Sept 2013 | 50 cm |
| `Sydney_CBD_EasternSuburb_Region_2018_04_07` | Apr 2018 | sub-metre |
| `NSW_SPOT_150cm_2020Q1_RGB_Mosaic` | 2020 Q1 | 150 cm (satellite) |
| `AAM_Sydney_2021_01` | Jan 2021 | **7.5 cm** |

Reproduce that list:

```bash
curl -s "https://maps.six.nsw.gov.au/arcgis/rest/services/sixmaps/LPI_Imagery_Best/MapServer/2/query?geometry=151.2080,-33.8677&geometryType=esriGeometryPoint&inSR=4326&outFields=Name,BlockStartDate,LowPS&returnGeometry=false&f=json"
```

**But you cannot render a chosen one.** The mosaic composites by `ZOrder` and
`BestByResDate` and serves whatever it decides is best — currently the 2021 AAM
capture. `layerDefs` is rejected with `Invalid 'layerDefs' is specified`,
because the imagery layer is a raster and a definition query does not apply to
it. So the four epochs are *visible in the index* and not *separately
retrievable* through this service.

### What is separately retrievable

`sixmaps/sydney1943` is its own cached service: a greyscale aerial survey of
inner Sydney and the highways, circa 1943, © Department of Customer Service,
CC BY. It covers the exact ground the project's CBD tile covers.

```bash
python scripts/fetch_historical_tile.py --match data/sydney/ref_tile.tif
```

That fetches the tiles, mosaics them, georeferences them locally, warps to the
same UTM CRS as the tile you matched, and writes `data/sydney/ref_tile_1943.tif`
with the attribution in its tags. 81 tiles, 6.2 MB, a few seconds.

Two things about it that the service metadata gets wrong:

- **The cache stops at zoom 19, not 21.** The service advertises 22 levels of
  detail and 20 and 21 both return 404 over the CBD. Zoom 19 is 0.2986 m/px in
  Web Mercator units, which at Sydney's latitude is **0.2476 m/px** on the
  ground — about half the detail of the current 0.124 m/px tile.
- **`export` is disabled on it**, so the tile endpoint is the only way in. The
  script uses it.

Coarser reference is not automatically worse here. `rho` — reference pixels per
frame pixel — actually *improves*, because it is the ratio of the two
resolutions: **0.142 at 50 m AGL and 0.284 at 100 m**, against 0.104 and 0.208
for the current tile.

---

## The measurement

`scripts/crossdate_probe.py` cuts frames from one tile and matches them against
another, resampling the query to the reference's GSD the same way the
`Preprocessor` does. 512 px frames, 2048 keypoints, inlier gate 8, seed 0.

```bash
python scripts/crossdate_probe.py --query data/sydney/ref_tile.tif            # control
python scripts/crossdate_probe.py --query data/sydney/ref_tile.tif \
                                  --reference data/sydney/ref_tile_1943.tif
```

**Control — 2021 tile against itself, 12 frames.** This is the demo's regime:

| matcher | inliers (med) | plausible | median | p90 | max |
|---|---|---|---|---|---|
| EdgePoint2 Small (64-D) | 22 | 8/12 | 0.018 m | 0.024 | 0.024 |
| XFeat + Nearest-Neighbour | 17 | 12/12 | 0.058 m | 0.088 | 0.133 |
| SIFT | 8 | 7/12 | 17.637 m | 105.592 | 105.592 |

**1943 reference, 2021 query, 24 frames:**

| matcher | inliers (med) | solved | plausible | median |
|---|---|---|---|---|
| EdgePoint2 Small (64-D) | 4 | 0 | 0 | — |
| XFeat + Nearest-Neighbour | 0 | 0 | 0 | — |
| SIFT | 6 | 0 | 0 | — |

**Zero fixes, every matcher, every frame.** Inliers collapse from 22 to 4.

The control is what makes that statement worth anything. Without it a
zero-fix result is indistinguishable from a broken probe — so the script runs
the control on request and prints a reminder to do so whenever a cross-date run
comes back empty.

Results are in `results/crossdate_control.json` and
`results/crossdate_1943.json`.

---

## Why it fails, and why that is not a surprise

78 years. In the Sydney CBD essentially every building in the frame has been
demolished and replaced, most of them more than once. What survives is road
geometry, the coastline, some parks and a few landmarks — and those are
precisely the low-texture, repetitive structures that a local feature detector
has least to say about. The 1943 imagery is also **greyscale**, so every
colour-derived gradient is gone.

This is a **failure boundary, not a benchmark**. It tells you the appearance
change this matcher cannot absorb. That is genuinely on-topic — a covariance
estimator has to know when to say "I do not know" — but it is not the system's
accuracy and must never be quoted as one.

---

## Where the realistic number actually comes from

**AnyVisLoc env80 already is the cross-date benchmark.** Real UAV frames from a
real flight, matched against a satellite basemap captured at a different time,
with published ground truth. That is exactly the independent-capture condition
this whole exercise was chasing, and it is already wired up:

```bash
GEOANCHOR_CONFIG=configs/env80.yaml bash run.sh
```

| scene | matcher | gate | accept | median | worst |
|---|---|---|---|---|---|
| 09 | XFeat + NN | 10 | 35.1% | 2.67 m | 6.65 m |
| 09 | EdgePoint2 S64 | 8 | 32.1% | 2.92 m | 7.50 m |
| 10 | XFeat + NN | 10 | 17.2% | 3.52 m | 6.75 m |
| 10 | EdgePoint2 S64 | 8 | 41.1% | 3.83 m | 7.36 m |

**2.5–3.8 m median is the number to quote.** Not 0.007 m, and not "no fix".

---

## What is worth doing next

The gap between "same image, 0.018 m" and "78 years apart, nothing" is enormous
and completely unsampled. The interesting question is where in between it
breaks, and there are two ways to find out.

1. **Buy the middle epochs.** The 2013 and 2018 captures exist and the mosaic
   will not serve them individually. NSW Spatial Services distributes historical
   imagery on request through the Historical Imagery Viewer; a 2013 vs 2021 pair
   over the same CBD tile would give a **7-year** gap, which is a realistic
   reference-map age for a deployed system and is exactly the number nobody has.

2. **Use the 1943 tile as the hard end of a sweep**, not as a benchmark. It is
   free, it is already fetched, and it anchors one end of a
   degradation curve whose other end is the control. A covariance estimator that
   correctly reports "no confidence" on 1943 frames is doing its job, and that
   is a testable claim.

Also worth knowing: the same fetch script works anywhere in the 1943 survey's
footprint, which is inner Sydney plus the highway corridors — so if the flight
area ends up inside it, this becomes a free second epoch for that area too.

```bash
python scripts/fetch_historical_tile.py --bbox 151.19,-33.89,151.21,-33.87 \
    --out data/sydney/other_area_1943.tif
```

---

## Licence

NSW Spatial Services imagery is **CC BY 4.0**. Attribute as: *NSW Department of
Customer Service, Sydney 1943 imagery.* The fetch script writes that string into
the GeoTIFF's tags so it travels with the file.
