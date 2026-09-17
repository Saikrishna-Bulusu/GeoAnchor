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

### Re-taken on the complete sweep — the signal is not weak, it is absent

The numbers above were taken while two scenes were still sweeping. All ten
CSVs are now in `results/train_sweep`, and the retrained models are **worse,
not better**:

| method | n | scenes | Spearman | p |
|---|---|---|---|---|
| `edgepoint2_s64` | 156 | 4 | **−0.032** | 0.7 |
| `xfeat_mnn` | 145 | 4 | **0.003** | 0.97 |

Both are indistinguishable from zero. **On the population this estimator is
actually for — fixes that already passed the gate — it does not order error at
all.**

The leave-one-scene-out breakdown says why, and it is the same story for both:

```
                 held out    n   med pred   med actual   ratio
edgepoint2_s64   Scene_09   43       6.76         2.78    2.43
                 Scene_10   82       5.29         3.84    1.38
                 Scene_16   27       2.08        10.71    0.19
xfeat_mnn        Scene_09   67      10.45         3.02    3.46
                 Scene_10   36      29.81         3.48    8.56
                 Scene_16   38       2.99        10.17    0.29
```

**Scene_16 is inverted and it is inverted on both methods** — the model calls
it the most confident scene and it is the worst one. Scene_09 and Scene_10 are
over-estimated by 1.4–8.6x in the other direction. So the model has not learned
a weak ordering, it has learned a **scene-identity ordering that is wrong on
the held-out scene**, which is exactly the failure mode `CLAUDE.md` warns about
under "cross-validate grouped by scene, never by row" — and the grouping is
what exposed it.

The out-of-fold calibration table shows the same thing from the other side.
For `edgepoint2_s64` the *lowest* predicted-sigma bin has the *highest* median
actual error:

```
    pred sigma     n   median actual m   p90 actual m
          2.06    38              5.95         17.22
          4.98    38              3.78          4.58
          5.52    38              3.88          5.92
          8.45    38              3.26        136.85
```

The one property that survives is in the last column: the p90 of the
highest-sigma bin is 136.85 m against 17.22 m for the lowest. **The model still
finds the tail even though it has lost the middle** — which is, once more, the
rejection problem rather than the covariance one.

### What this changes

The deflation in this document was stated as "closer to 0.2". On complete data
it is **zero**. Everything below about calibration-not-discrimination still
holds and is now the *only* defensible claim for the learned estimator:
it gets the magnitude into the right range, and it does not rank within it.

This does not touch `CLAUDE.md`'s THE FINDING, which is measured on the
*pre-gate* harness population and is about Eq. 7's saturation and
descriptor-specificity. It does mean the learned estimator should be described
as a **magnitude model**, never as a confidence ranking.

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

- **n is small**: 145–156 post-gate rows on complete data. And "4 scenes"
  overstates it — **Scene_22 contributes 4 rows**, so leave-one-scene-out is
  really three scenes deep and never reports Scene_22 as a fold. Scene_21 is
  excluded per `CLAUDE.md`. Three effective scenes is thin for a group-wise
  claim in either direction, including this document's negative one.
- **The retrain used the same features and the same splits as the first pass**,
  so the move from 0.185 to 0.003 is data, not method.
- **"error ≤ 20 m" is a proxy for the post-gate population, not the population
  itself.** The gate is on inliers, not on error. The direct runtime-sweep
  measurement is the better evidence and it agrees.
- **Scene_16 is badly predicted by both methods** (predicted 2.1–3.0 m, actual
  10.2–10.7 m). With three effective scenes, holding it out leaves two to train
  on, and `CLAUDE.md`'s group-identity warning applies.
- **More scenes is the obvious next move and may not rescue it.** The failure
  is not noise — both methods invert the same scene in the same direction,
  which reads as a real property of Scene_16 that the eight features do not
  carry.
