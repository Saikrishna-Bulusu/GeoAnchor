# The learned covariance estimator, exported and running

17 Sept 2026. `scripts/train_covariance.py` trains on the parent repo's
`results/s8_train` and writes a pickle that `processing_layer/covariance.py`'s
`Learned` backend loads directly.

**The gap this closes.** `Learned` has been in the runtime since the beginning
with no model to load, so every flight ran on `gate_only` — a fixed 8 m that
labels itself a placeholder in every record. The project's headline result is
that only a learned estimator produces metres that track actual error, and the
runtime was not carrying it.

```bash
python scripts/train_covariance.py --matcher XFEAT_STAR --out models/covariance_xfeat_star.pkl
GEOANCHOR_SET="processing_layer.covariance.backend=learned;\
processing_layer.covariance.model_path=models/covariance_xfeat_star.pkl" bash run.sh
```

## What it predicts, and how well

sigma in metres, regressed as `log(error)` — the log is not cosmetic, because
errors span 0.1 m to 168 m and a squared-error fit in linear metres is
dominated entirely by the tail.

Five features, and the choice is deliberate: `inliers`, `inlier_ratio`,
`matches`, `reproj_err_px`, `altitude_m`. The runtime's `FEATURES` list also
names `keypoints`, `scale` and `tiles_searched`, but those mean something
**different** in the harness — its `n_pnp_input` is not the runtime's keypoint
count and its `retrieval_k` is not a tile count. A feature that means one thing
at training time and another at inference is worse than an absent one, because
nothing detects it. The exported model carries its own feature list, so
`Learned.sigma_m` looks up exactly the five.

### Calibration, out of fold, binned by predicted sigma

The headline claim is a *tracking* claim, so it is checked as one: bin by what
the model predicted, report what actually happened.

| matcher | pred → actual (median m, four quartile bins) | Spearman |
|---|---|---|
| `XFEAT_STAR` | 8.1→7.3, 12.0→11.9, 16.4→16.4, 33.2→29.6 | **0.581** |
| `XFEAT_MNN` | 6.5→6.9, 12.0→11.8, 19.3→16.7, 35.7→25.9 | 0.551 |
| `XFEAT_LG` | 6.9→6.9, 9.0→11.4, 10.9→9.9, 21.1→20.7 | 0.399 |

Monotone across a 4× range of sigma on all three. Leave-one-scene-out ratios of
predicted to actual median are **0.73–1.42** across four held-out scenes — against
NGPS Eq. 7's 2.4× understatement and its saturation at 10.00 m.

**Spearman varies 0.399 to 0.581 across three matchers of the same family.**
That is the project's descriptor-specificity finding reproduced from the other
direction: even within XFeat, how predictable the error is depends on which
matcher produced it.

### Validation rules, each of which has produced a wrong answer once

- **Ground truth is never a predictor.** `PDE`, `R_i`, `pdm_at_k`,
  `recall_at_k`, `retrieval_gt_rank`, `pred_error`, `location_error_list`,
  `truePos.*` are all distance from the true position under different names.
  The feature set is an **allowlist**, which fails safe where a blocklist does not.
- **Grouped by scene, never by row.** Rows in a scene share a reference tile,
  an altitude band and a capture.
- **Scene_21 excluded** — 93% bad, 140 of 141 catastrophes. A matcher failure,
  not a localisation result.
- **Errors above 10 km dropped from the regression.** The harness sets
  `pnp_success` on `inliers > 0` alone, so degenerate solves reaching 1e93 m are
  in the data. Those belong to the *rejection* problem, which the inlier gate
  already wins; keeping them makes the fit chase a number no covariance can
  express.

## Running live

59 fixes on the Sydney replay with `backend: learned`:

```
sigma  min 5.74   median 6.87   max 11.89
distinct sigma values: 14        (gate_only would be 1)
```

**The model refuses to believe the synthetic task, and that is correct.** The
Sydney replay's errors are 0.003–0.012 m because its frames are cut from the
very tile they are matched against, and the model still says ~6.9 m. It was
trained on real AnyVisLoc frames against a real satellite basemap, where 270
inliers genuinely means several metres. A covariance that reported 0.006 m here
would be a covariance that had learned the fixture.

## The limitation that matters most

**These models are XFeat-family, and the runtime's default matcher is
`edgepoint2_s64`.** Applying one to the other is precisely the failure this
project exists to document: *a system that swaps matchers silently inherits a
covariance model that no longer works.* It would be self-refuting to ship it.

So `configs/system.yaml` **stays on `gate_only`**. Switching to `learned` is a
deliberate act that requires naming a model, and the model's `meta` records
which matcher, which scenes and which reference it was trained on.

An EdgePoint2 model needs harness runs with EdgePoint2 over the training
scenes, and the parent repo has none — `s8_train` covers `XFEAT_LG`,
`XFEAT_MNN` and `XFEAT_STAR` only. That is the next experiment, and it is a
harness run, not a modelling problem.

Two smaller caveats, stated rather than buried:

- **Four scenes is not many.** The leave-one-scene-out spread (0.73–1.42)
  is the honest estimate of how far off a new scene could be.
- **The model is not monotone in inliers at the good end** — 120 inliers gives
  6.93 m where 40 gives 4.54 m on a synthetic probe. It fits five features
  jointly, so single-feature sweeps need not be monotone, but it is worth
  knowing before reading the sigma as "inliers, transformed".
