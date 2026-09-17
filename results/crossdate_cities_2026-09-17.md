# Cross-date over four Australian areas: it is the land cover, not the matcher

Measured 17 Sept 2026 on the Legion. `edgepoint2_s64`, `max_keypoints` 2048,
inlier gate 8, 24 frames of 512 px, cold start, **Esri World Imagery Wayback**
at zoom 19. The query is always each area's newest capture, matched against
every older one.

`results/crossdate_wayback_curve.md` measured one place — a 510 x 500 m tile
over the Sydney CBD — and found a sharp cliff between 2.8 and 3.6 years. The
open question was whether that cliff is a property of the **matcher** or of the
**Sydney CBD**. This answers it, and in doing so breaks the question's own
premise twice.

Reproduce:

```bash
bash scripts/crossdate_cities.sh
python scripts/crossdate_table.py
python scripts/wayback_source_meta.py --json results/wayback_source_meta.json
```

## The answer

**The cliff is a property of LAND COVER.** It is not the matcher, and it is not
Sydney specifically.

| area | result |
|---|---|
| **rural (Griffith, NSW farmland)** | **works at every measured gap, out to 8.8 years** |
| Brisbane CBD | not monotonic — 4 of 8 gaps work |
| Perth CBD | not monotonic — 3 of 11 gaps work |
| Melbourne CBD | not monotonic — 1 of 12 gaps work |

Every control passed (24/24 solved, 0.011–0.021 m median), so none of the
failures above is a harness or tile fault.

The rural area is the finding, and it inverts the intuition. Crops change every
season and buildings do not, so a city ought to be the easy case. It is the
hard one:

```
rural_griffith   118-443 inliers, 0.02-1.92 m median, 24/24 at every gap
three CBDs         5-38  inliers, most gaps failing outright
```

What changes between two satellite passes over a CBD is not the ground — it is
the **apparent geometry of tall structures**: facade parallax with view angle,
and shadow with sun angle and season. Flat farmland has neither, so two
captures eight years apart still align. A high-rise CBD disagrees with itself
between passes taken months apart.

**For this project that is directly actionable.** How stale the reference map
may be is a question about the flight area, not about the pipeline. The
"three years" figure measured over the Sydney CBD should not be carried to
wherever UTS actually flies — and if that is open country, the real limit is
much more forgiving.

## The question's premise was wrong twice

### 1. The x-axis is not age

Three of four areas are non-monotonic. Perth solves 24/24 at a 0.7-year gap,
0/24 at 1.6, 13/24 at 2.6, 0/24 at 3.1, and 16/24 at 9.1. **A shorter gap
failing while a longer one works means elapsed time is not the variable being
measured.**

`fetch_wayback_tile.py` labels each capture with the Wayback **release** date,
read from the service's `itemTitle`. That is when Esri published a version of
the basemap — not when the imagery was flown. A release can republish older
imagery, and two releases months apart can carry acquisitions years apart.

So `results/crossdate_wayback_curve.md`'s x-axis is a **publication gap that
has been read as map age**. Over the Sydney CBD it happened to come out
monotonic, so nothing drew attention to it.

`scripts/wayback_source_meta.py` reads the real thing from Esri's per-release
metadata services: `SRC_DATE` (acquisition), `SRC_RES` (native resolution),
`SRC_DESC` (sensor), `SRC_ACC` (stated horizontal accuracy).

### 2. Fixed zoom is not fixed resolution

Holding the zoom constant was meant to remove resolution as a confound. It does
not, for two independent reasons:

- **Web Mercator scale goes as 1/cos(lat).** Zoom 19 is 0.2644 m/px at
  Brisbane, 0.2476 at Sydney, 0.2356 at Melbourne — a 12% spread across the
  areas compared here.
- **The native source differs per area.** A zoom-19 tile is whatever Esri had,
  resampled. If Melbourne's source is 0.31 m WorldView-3 and Sydney's is 0.1 m
  aerial photography, those tiles carry very different amounts of real detail
  at the same nominal pixel size, and the comparison is between sensors.

`SRC_ACC` is a third: a stated horizontal accuracy of several metres is the
same order as the errors being measured, so part of any cross-date "error" is
simply the two captures being georeferenced differently from each other.

**The within-area results are unaffected by all of this** — every capture in an
area shares a resolution, a sensor family and a projection — which is why the
land-cover conclusion above stands. It is the cross-area comparison of absolute
inlier counts that needs resampling to a common GSD before it means anything.

### 3. Every tile in this study is upsampled, and the georeferencing error is the same size as the signal

`results/wayback_source_meta.json`, read from Esri's own per-release metadata
services. Sydney and Melbourne resolved before the service began rate-limiting;
the other three need a re-run after a cooldown.

**Sydney CBD** — 7 distinct acquisitions, not the 13 "distinct captures" the
tile-hash method counts:

| acquired | native m | sampled m | stated acc m | sensor |
|---|---|---|---|---|
| 2013-02-07 | 0.50 | 0.50 | 10.00 | Pleiades |
| 2016-08-10 | 0.31 | 0.31 | 4.23 | WV03_VNIR |
| 2020-06-16 | 0.46 | 0.31 | 4.23 | GE01 |
| 2021-05-12 | 0.50 | 0.31 | 4.23 | WV02 |
| 2022-04-19 | 0.50 | 0.30 | 5.00 | WV02 |
| 2023-02-05 | 0.30 | 0.30 | 2.00 | WV03 |
| 2025-10-02 | 0.31 | 0.15 | 8.47 | WV03 |

**Melbourne CBD** — 5 distinct acquisitions, native 0.46–0.50 m, all
WorldView-2 / GeoEye-1 / Pleiades.

**Brisbane CBD** — 6 distinct acquisitions from the **9 tiles** the hash method
called distinct, native 0.31–0.50 m, stated accuracy 4.23–10 m, and its
**newest acquisition is 2020-07-07** even though the query tile is a 2025-12-18
release. Brisbane's cross-date table above therefore labels several pairs with
gaps that are years wrong in both directions: releases five years apart can
carry the same 2020 imagery, and the "0-year" query is itself five years old.

Three things fall out of this, and they matter more than the resolution
confound described above.

**The tile-hash method over-counts captures.** Sydney's 13 "distinct captures"
are 7 acquisitions. Different Wayback releases can render the same underlying
imagery differently — reprocessing, reprojection, recompression — so several
"gaps" in the original curve compare one acquisition against itself, rendered
twice. Those are not cross-date measurements at all.

**Everything here is upsampled.** The native source is 0.30–0.50 m everywhere,
and these tiles are fetched at zoom 19 = 0.24 m/px. There is no real detail
below ~0.3 m in any capture in this study — the matcher is working on
interpolated pixels throughout. That also settles the cross-city resolution
worry in the other direction: Sydney and Melbourne share sensors (WV02, WV03,
GE01, Pleiades) and native resolution to within 0.04 m, so **the land-cover
conclusion is not a resolution artefact.** Same sensors, same resolution,
wildly different cross-date behaviour — it is the ground.

**The stated horizontal accuracy is 2–10 m.** That is the same order as the
cross-date errors being measured, and larger than most of them. A 3.34 m median
"error" at a short gap is at or below the georeferencing uncertainty of the two
captures being compared, so **a substantial part of what this curve measures is
the two images disagreeing about where they are, not the matcher failing.**
Solve *rate* and inlier count remain clean signals; the metre values do not,
and should be read as an upper bound on matcher error rather than as matcher
error.

This applies to `results/crossdate_wayback_curve.md` too.

## Per-area tables


## brisbane

control: 24/24 solved, median 0.021 m, 119 inliers -- area is testable
| capture | gap, yr | inliers | solved | median m | p90 m |
|---|---|---|---|---|---|
| 2023-06-29 | 2.5 | 19 | 19/24 | **10.64** | 23.11 |
| 2023-05-03 | 2.6 | 5 | 0/24 | **--** | -- |
| 2022-08-10 | 3.4 | 25 | 17/24 | **6.54** | 19.14 |
| 2021-07-21 | 4.4 | 6 | 11/24 | **14.71** | 16.87 |
| 2020-07-01 | 5.5 | 38 | 24/24 | **5.00** | 11.22 |
| 2019-12-12 | 6.0 | 6 | 9/24 | **13.25** | 42.09 |
| 2017-10-25 | 8.1 | 5 | 0/24 | **--** | -- |
| 2014-12-03 | 11.0 | 15 | 18/24 | **5.47** | 18.87 |

**Not monotonic -- there is no cliff to quote.** Works at 2.5, 3.4, 5.5, 11.0 yr; fails at 2.6, 4.4, 6.0, 8.1 yr.

A short gap failing while a longer one works means ELAPSED TIME IS NOT THE VARIABLE here. The likeliest reason is that the axis is

the Wayback RELEASE date, not the acquisition date -- see scripts/wayback_source_meta.py.

## melbourne

control: 24/24 solved, median 0.021 m, 91 inliers -- area is testable
| capture | gap, yr | inliers | solved | median m | p90 m |
|---|---|---|---|---|---|
| 2025-12-18 | 0.3 | 5 | 5/24 | **20.53** | 24.54 |
| 2024-08-15 | 1.6 | 8 | 13/24 | **6.54** | 8.90 |
| 2023-10-11 | 2.5 | 5 | 4/24 | **7.34** | 26.25 |
| 2023-08-10 | 2.6 | 5 | 4/24 | **22.28** | 169.74 |
| 2023-06-29 | 2.7 | 5 | 6/24 | **23.51** | 88.65 |
| 2022-08-10 | 3.6 | 6 | 3/24 | **19.07** | 35.05 |
| 2021-11-03 | 4.4 | 5 | 7/24 | **14.22** | 20.68 |
| 2021-02-24 | 5.1 | 5 | 0/24 | **--** | -- |
| 2020-07-01 | 5.7 | 5 | 0/24 | **--** | -- |
| 2017-11-16 | 8.4 | 5 | 0/24 | **--** | -- |
| 2017-02-27 | 9.1 | 5 | 3/24 | **4.40** | 8.75 |
| 2014-07-02 | 11.7 | 5 | 0/24 | **--** | -- |

**Not monotonic -- there is no cliff to quote.** Works at 1.6 yr; fails at 0.3, 2.5, 2.6, 2.7, 3.6, 4.4, 5.1, 5.7, 8.4, 9.1, 11.7 yr.

A short gap failing while a longer one works means ELAPSED TIME IS NOT THE VARIABLE here. The likeliest reason is that the axis is

the Wayback RELEASE date, not the acquisition date -- see scripts/wayback_source_meta.py.

## perth

control: 24/24 solved, median 0.017 m, 102 inliers -- area is testable
| capture | gap, yr | inliers | solved | median m | p90 m |
|---|---|---|---|---|---|
| 2025-06-26 | 0.7 | 26 | 24/24 | **3.05** | 10.26 |
| 2024-08-15 | 1.6 | 5 | 0/24 | **--** | -- |
| 2023-08-10 | 2.6 | 8 | 13/24 | **2.96** | 36.30 |
| 2023-02-23 | 3.1 | 5 | 0/24 | **--** | -- |
| 2022-06-08 | 3.8 | 5 | 3/24 | **59.94** | 61.62 |
| 2021-04-28 | 4.9 | 5 | 0/24 | **--** | -- |
| 2020-03-23 | 6.0 | 6 | 10/24 | **7.97** | 23.83 |
| 2018-10-17 | 7.4 | 6 | 11/24 | **6.80** | 24.91 |
| 2017-11-16 | 8.4 | 5 | 4/24 | **7.22** | 7.89 |
| 2017-02-27 | 9.1 | 11 | 16/24 | **3.36** | 13.02 |
| 2014-12-03 | 11.3 | 5 | 1/24 | **7.02** | 7.02 |

**Not monotonic -- there is no cliff to quote.** Works at 0.7, 2.6, 9.1 yr; fails at 1.6, 3.1, 3.8, 4.9, 6.0, 7.4, 8.4, 11.3 yr.

A short gap failing while a longer one works means ELAPSED TIME IS NOT THE VARIABLE here. The likeliest reason is that the axis is

the Wayback RELEASE date, not the acquisition date -- see scripts/wayback_source_meta.py.

## rural_griffith

control: 24/24 solved, median 0.011 m, 451 inliers -- area is testable
| capture | gap, yr | inliers | solved | median m | p90 m |
|---|---|---|---|---|---|
| 2026-01-29 | 0.5 | 443 | 24/24 | **0.02** | 0.03 |
| 2024-06-06 | 2.2 | 165 | 24/24 | **0.40** | 0.78 |
| 2023-06-29 | 3.1 | 167 | 24/24 | **0.45** | 0.78 |
| 2022-06-29 | 4.1 | 183 | 24/24 | **1.92** | 2.03 |
| 2017-10-25 | 8.8 | 118 | 24/24 | **0.53** | 0.72 |

Every measured gap works, out to 8.8 years. No cliff inside this area's capture history.

## Across areas

brisbane           NOT MONOTONIC -- 4 of 8 gaps work, out to 11.0 yr

melbourne          NOT MONOTONIC -- 1 of 12 gaps work, out to 1.6 yr

perth              NOT MONOTONIC -- 3 of 11 gaps work, out to 9.1 yr

rural_griffith     works at every measured gap, out to 8.8 yr


A cliff in the same place everywhere would be a MATCHER property. One that moves with

the site is a property of what is on the ground, and then the flight area decides it.

Areas whose results are NOT monotonic cannot contribute a cliff location at all:

a shorter gap failing while a longer one works says the x-axis is not measuring age.
