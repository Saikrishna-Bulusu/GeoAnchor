# The learned estimator's advantage is mostly a rejection result, measured again

17 Sept 2026. Training a covariance model from the **runtime's own sweep**
rather than the parent harness turned up something that changes how the
headline number should be read.

## The number

`CLAUDE.md` carries the learned estimator at Spearman ≈ 0.55 against realised
error, leave-one-scene-out. Reproduced exactly on the harness data:

| population | n | Spearman |
|---|---|---|
| all rows, as published | 439 | **0.551** |
| error ≤ 200 m | 439 | 0.551 |
| error ≤ 50 m | 394 | 0.535 |
| **error ≤ 20 m** | 291 | **0.220** |

`XFEAT_MNN`, satellite, Scene_21 excluded, same features, same
leave-one-scene-out. **Predictability more than halves once the catastrophes
are removed.**

## Why that matters

The catastrophes are removed *before the covariance backend ever runs*. The
runtime's `solve()` rejects a geometrically implausible homography and returns
**no position at all** — there is nothing to attach a covariance to. Measured
on the runtime's own sweep over the training scenes: 808 frames, 293 plausible,
and **zero** implausible rows carry an error, because an implausible solve has
no position.

So the population the estimator is asked about in flight is the post-gate one,
and on that population its ordering signal is weak. Measured directly, not
inferred — training from the runtime sweep, 3 scenes:

| method | n (post-gate) | Spearman |
|---|---|---|
| `xfeat_mnn` | 141 | **0.185** |
| `edgepoint2_s64` | 152 | **0.072** (p = 0.38) |

Two independent routes to the same place: restricting the harness data to
low-error rows gives 0.220, and measuring the real post-gate population gives
0.185. They agree.

## What this does and does not overturn

**It does not overturn the finding.** "Rejection and covariance are different
problems, and different methods win" stands — and this is further evidence for
it, from the other direction. An inlier threshold wins rejection; a learned
estimator is the only thing that emits metres. Both remain true.

**It does deflate the headline number.** Much of the 0.551 is the model
learning to separate catastrophes from good fixes, which is *the rejection
problem*, and which the inlier gate and the plausibility check already solve
for free. Scored on what it is actually for — ranking the error of fixes that
already passed — it is closer to 0.2.

**It does not mean the covariance is useless.** EKF3 needs *a* number, and a
weakly-ordered estimate that is right on average beats a constant 8 m and beats
Eq. 7's saturation at 10.00 m. The leave-one-scene-out ratios of predicted to
actual median were 0.73–1.42 on the harness data; that is the useful property,
and it is not what Spearman measures.

**The honest framing** is therefore: the learned estimator's value is
*calibration* — getting the magnitude roughly right across a 4× range — not
*discrimination* among accepted fixes, where the signal is weak.

## Why the runtime sweep is the better training source anyway

`scripts/train_covariance.py --sweep` now reads `env80_sweep.py` output
directly. Three reasons it is preferred over `--root`:

- **The features are definitionally identical to the flying code's**, because
  the same `solve()` produced them. All eight of `covariance.FEATURES`, not the
  five that survive intersecting with the harness's differently-defined
  `fix_quality`.
- **It works for any method the runtime has**, including EdgePoint2, which the
  parent harness does not implement.
- **It scores the population the estimator will actually see.** That is the
  point of this document — and it is why the numbers it produces are lower and
  more honest than the harness's.

## Caveats on this measurement

- **n is small**: 141–152 post-gate rows across 3 scenes, against the harness's
  439 across 4. Scene_21 and Scene_22 were still sweeping when this was
  written; the numbers should be re-taken with all five.
- **"error ≤ 20 m" is a proxy for the post-gate population, not the population
  itself.** The gate is on inliers, not on error. The direct runtime-sweep
  measurement is the better evidence and it agrees.
- **Scene_16 is badly predicted by both methods** (predicted 2.5–3.0 m, actual
  10.2–10.7 m). With three scenes, holding it out leaves two to train on, and
  `CLAUDE.md`'s group-identity warning applies.
