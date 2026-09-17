# Why Scene_09 and Scene_10 disagree about which XFeat wins

`CLAUDE.md` has carried this as unexplained: *"Scene_09 favours xfeat_mnn over
xfeat_lg 4.4:1 on plausible rate; Scene_10 reverses it 1.9:1. Until that is
explained, 'xfeat_mnn wins' is a claim about Scene_09."*

## The mechanism found

`solve.py:152` calls `method.match(f_frame, f_ref)` **once**, against the
concatenated reference keypoints of every candidate tile. Tiles partition the
**RANSAC input only** — never the matching. So the two scenes hand the matcher
very different problems:

| | tiles | reference keypoints seen by one `match()` call |
|---|---|---|
| Scene_09 | 9 | 9 × 2048 = **18432** |
| Scene_10 | 1 | **2048** |

A 9× difference in the number of candidate correspondences, and nothing in the
sweep's configuration says so.

## Tested directly, in both directions

Same scenes, same frames, same matchers — only `--ref-keypoints` changed.

**Scene_10 (1 tile), given Scene_09's reference size:**

| matcher | @2048 | @18432 | |
|---|---|---|---|
| `xfeat_mnn` | 18.8% | **22.4%** | improves, inliers 26 → 42 |
| `xfeat_lg` | 35.9% | **25.0%** | **degrades**, latency 284 → 1794 ms |

**Scene_09 (9 tiles), starved to Scene_10's reference size:**

| matcher | @18432 | @2052 | |
|---|---|---|---|
| `xfeat_mnn` | 46.3% | **18.7%** | **collapses** |
| `xfeat_lg` | 10.4% | **14.2%** | improves |

**The effect is real, large, and opposite for the two matchers.** More
reference keypoints help mutual-nearest-neighbour and hurt LighterGlue, on both
scenes, in both directions. MNN is a per-descriptor argmax that simply gets
more chances at the true correspondence; LighterGlue is a learned matcher whose
attention has to discriminate the true match among 9× the distractors, and it
pays 6× the latency to do it worse.

## But it does NOT explain the reversal, and that is the honest result

At **matched** reference keypoints the two scenes still disagree, in the same
directions as before:

| reference keypoints | Scene_09 | Scene_10 |
|---|---|---|
| ~2048 | mnn wins **1.32:1** | lg wins **1.91:1** |
| 18432 | mnn wins **4.45:1** | lg wins **1.12:1** |

Scene_09 favours `xfeat_mnn` at both sizes. Scene_10 favours `xfeat_lg` at
both. Equalising the reference size narrows the gap on Scene_09 from 4.45:1 to
1.32:1 and on Scene_10 from 1.91:1 to 1.12:1 — it **amplifies** the
disagreement without **causing** it.

So the hypothesis was half right. Reference-keypoint count explains the
*magnitude* of the disagreement, not its *sign*. Something about the scenes
themselves — content, reference imagery, altitude distribution — still decides
which matcher wins, and that remains open.

## What is now safe to say

- **"xfeat_mnn wins" is still a claim about Scene_09**, and the reversal is not
  a tiling artefact. That much is settled.
- **Reference-keypoint count is a first-class experimental variable** and was
  previously an uncontrolled one. Any comparison between scenes with different
  tile counts is confounded by it unless `--ref-keypoints` is matched, and the
  existing `results/env80_sweep/` numbers are 9× apart on this axis without
  saying so.
- **`xfeat_lg` degrading as the search widens is a deployment fact, not a
  curiosity.** It is the same shape as the finding that LighterGlue costs most
  when it fails: at 18432 reference keypoints on Scene_10 it took 1794 ms
  median, 6× its 284 ms at 2048, to produce a *worse* answer. A wider cold-start
  search makes it both slower and less accurate, which is the opposite of how a
  search budget is supposed to behave.
- **`xfeat_mnn` is the one that needs a wide reference.** Starving Scene_09
  from 18432 to 2052 cost it 46.3% → 18.7%. Anything that narrows the search —
  a tighter prior, fewer tiles, a smaller `top_k` — takes that away, and the
  `top_k` front should be read with this in mind.

## Reproduce

```bash
python scripts/env80_sweep.py --scenes Scene_10 --methods xfeat_mnn,xfeat_lg \
    --ref-keypoints 18432 --out results/refkp/s10_18432
python scripts/env80_sweep.py --scenes Scene_09 --methods xfeat_mnn,xfeat_lg \
    --ref-keypoints 228   --out results/refkp/s09_228
```

`--ref-keypoints` is **per tile**, so 228 × 9 tiles ≈ 2052 on Scene_09 and
18432 × 1 tile on Scene_10.
