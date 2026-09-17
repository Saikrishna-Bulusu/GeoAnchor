# Neither recommended scene can contribute, and no scene in the dataset fixes this

17 Sept 2026. The parent repo's `docs/step20_scene_priority.py` recommends
**Scene_20 first, then Scene_13**, to widen the covariance estimator's base —
"the estimator's tail rests on one scene: excluding Scene_21 leaves 441 fixes,
only 28 worse than 50 m, and 20 of those 28 are Scene_10."

Both recommendations fail, and measuring why says something about the dataset
that changes the plan rather than the scene list.

## Scene_20 delivers zero

It is already downloaded. Measured over its 252 frames:

| | |
|---|---|
| frames in the env80 envelope (50–100 m **and** ≥80°) | **0** |
| frames in the training envelope (40–120 m, ≥65°) | 12 |
| altitude, min / median / max | 93 / 121 / 123 m |

It flies at 93–123 m. Only **one** of its 252 frames is inside 50–100 m, and
that one is not near-nadir.

## Scene_13 cannot deliver either, and this was knowable without downloading it

`step20_scene_priority.py` carries the published per-scene statistics, and its
own table says:

```
"Scene_13": (533, 113, 126, 24, 90, 0.050, 0.136)
              n   alt_min alt_max ...
```

**113–126 m — zero overlap with the 50–100 m envelope.** Scene_13 cannot
contribute a single env80 frame. The ranking put it second anyway, because it
scored on frame count and resolution band rather than on whether the altitude
range intersects the envelope at all.

AnyVisLoc is distributed through Baidu NetDisk, slow and manual, so this is a
download not worth starting.

## The whole dataset, ranked by what it could contribute

Altitude overlap with 50–100 m × frame count, against what the seven downloaded
scenes actually delivered:

| scene | have | n | altitude | in 50–100 | naive est | **measured env80** |
|---|---|---|---|---|---|---|
| Scene_04 | yes | 2990 | 39–299 m | 19% | 575 | **3** |
| Scene_10 | yes | 651 | 33–90 m | 70% | 457 | **192** |
| Scene_09 | yes | 699 | 50–137 m | 57% | 402 | **134** |
| Scene_21 | yes | 1161 | 87–150 m | 21% | 240 | **4** |
| Scene_07 | — | 1219 | 51–301 m | 20% | 239 | — |
| Scene_06 | — | 1358 | 6–292 m | 17% | 237 | — |
| Scene_16 | yes | 295 | 56–112 m | 79% | 232 | **9** |
| Scene_08 | — | 1072 | 29–300 m | 18% | 198 | — |
| Scene_22 | yes | 220 | 40–80 m | 75% | 165 | **8** |
| Scene_20 | yes | 252 | 93–123 m | 23% | 59 | **0** |
| Scene_13 | — | 533 | 113–126 m | **0%** | **0** | — |
| Scene_01/02/03/11/15/18/23 | — | — | all above 100 m | 0% | 0 | — |
| Scene_12 | — | 563 | 56–73 m | 100% | **0** | — (pitch tops out at 76°) |

**The naive estimate over-predicts by 6.1×** across the seven measured scenes
(2129 predicted, 350 delivered). CLAUDE.md already records this failure mode
once — "frame estimate is an upper bound: measured median 0.27× actual" — and
this is the third time the same ranking has been wrong by it.

## Why the estimate fails, and what actually binds

Altitude overlap is the wrong predictor because altitude is not uniformly
distributed inside a scene's range. But the deeper problem is the **view
angle**. Of the frames that ARE inside 50–100 m, the fraction that also reach
≥80° is:

| scene | in altitude | also ≥80° | |
|---|---|---|---|
| Scene_10 | 638 | 192 | 30% |
| Scene_09 | 669 | 134 | 20% |
| Scene_04 | 51 | 3 | 6% |
| Scene_16 | 227 | 9 | **4%** |
| Scene_22 | 208 | 8 | **4%** |

**Scene_09 and Scene_10 are unusual in flying near-nadir at all.** Everywhere
else the view-angle gate removes 94–96% of the frames that pass altitude.
Scene_16 and Scene_22 both sit squarely inside the altitude band — 56–112 m and
40–80 m — and still contribute single digits, because they are flown oblique.

## What follows

**env80 is Scene_09 + Scene_10 by construction, not by accident:** 326 of 350
frames, 93%. No scene in the published 24 changes that, because the envelope
asks for low *and* nadir and the dataset is mostly high or oblique.

So the estimator's single-scene tail dependence is **not fixable by adding
scenes from this dataset**. The three real options, in order of cost:

1. **Relax the envelope and say so.** 50–120 m and ≥70° raises the usable set
   from 350 to 959 frames — but 527 of those are Scene_21, which is excluded as
   a matcher failure, leaving 432, of which Scene_09+10 are still 78%. This buys
   less than it looks like.
2. **Train per-envelope and report the domain shift**, rather than pretending a
   wider envelope is the same problem.
3. **Different data.** The concentration is a property of AnyVisLoc's flight
   profiles, not of this pipeline.

Recorded so the next session does not spend a manual Baidu download on
Scene_13, and so `step20_scene_priority.py`'s ranking is read with its measured
6.1× optimism applied.
