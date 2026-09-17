#!/usr/bin/env python3
"""Train the covariance estimator and export it for the runtime's `learned` backend.

    python scripts/train_covariance.py --sweep results/train_sweep --method edgepoint2_s64
    python scripts/train_covariance.py --root ~/GeoAnchor/results/s8_train

THE GAP THIS FILLS. `processing_layer/covariance.py` has shipped a working
`Learned` backend and no model, so every flight has run on `gate_only` -- a
fixed 8 m that labels itself a placeholder in every record. The project's
headline result is that only a learned estimator produces metres that track
actual error, and the runtime has not been carrying it.

WHAT IS PREDICTED. sigma in metres, as `log(error)`, regressed against the
realised distance from ground truth. The log is not cosmetic: errors here span
0.1 m to 168 m, so a squared-error fit in linear metres is dominated entirely
by the tail and predicts a near-constant large sigma for everything.

THE THREE RULES THAT MAKE THE NUMBER MEAN ANYTHING, all from CLAUDE.md and all
of which have already produced a wrong answer once when broken:

  * **Ground truth may never be a predictor.** `PDE`, `R_i`, `pdm_at_k`,
    `recall_at_k`, `retrieval_gt_rank`, `pred_error`, `location_error_list` and
    `truePos.*` are all distance from the true position under different names.
    They are the TARGET and are banned from X. `_FEATURES` below is an
    allowlist, not a blocklist, which is the safer direction.
  * **Cross-validate grouped by SCENE, never by row.** Rows within a scene
    share a reference tile, an altitude band and a capture; a row-wise split
    leaks all three and reports a model that cannot fly.
  * **Scene_21 is excluded.** It is a matcher failure -- 93% bad, 140 of 141
    catastrophes -- not a localisation result, and training on it fits the
    estimator to a broken matcher.

TWO SOURCES, AND --sweep IS THE BETTER ONE.

  --sweep   this runtime's own env80_sweep.py output. Its CSV carries ALL
            EIGHT of covariance.FEATURES under the definitions the flying code
            uses, because the same `solve()` produced them. A model trained
            here cannot suffer a train/inference mismatch, and it works for any
            method the runtime has -- including EdgePoint2, which the parent
            harness does not implement.

  --root    the parent repo's harness output (results/s8_train). More scenes,
            but its fix_quality features share NAMES with the runtime's while
            meaning different things, so only five survive the intersection
            below.

Prefer --sweep. --root is kept because it is the only source with XFeat
harness runs already on disk.

FEATURES ARE THE INTERSECTION, DELIBERATELY (--root only). The harness records more than the
runtime can compute in flight, and three of the runtime's own `FEATURES`
(`keypoints`, `scale`, `tiles_searched`) have a DIFFERENT MEANING in the
harness -- its `n_pnp_input` is not the runtime's keypoint count and its
`retrieval_k` is not a tile count. A feature that means one thing at training
time and another at inference is worse than an absent one, because nothing
detects it. Only the five that are genuinely the same quantity are used, and
the exported model carries its own feature list so `Learned.sigma_m` looks up
exactly those.
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import pickle
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

# The runtime names, for exactly the quantities that are the same on both sides.
# geoanchor/processing_layer/covariance.py::features_from produces these.
_FEATURES = ["inliers", "inlier_ratio", "matches", "reproj_err_px", "altitude_m"]

# Named so a reader can see they were considered and rejected, rather than
# wondering whether they were forgotten.
_BANNED = {"pred_error", "pred_error_m", "location_error_list", "retrieval_gt_rank",
           "recall_at_k", "PDE", "R_i", "pdm_at_k", "truePos"}


def load(root: str, reference: str = "satellite", exclude=("Scene_21",)):
    """Rows of (scene, matcher, features, actual_error_m) from a harness run."""
    rows = []
    pattern = os.path.join(root, "**", f"{reference}-*-*-yp", "**", "VG_data_*.pkl")
    for p in glob.glob(pattern, recursive=True):
        try:
            d = pickle.load(open(p, "rb"))
        except Exception:
            continue
        scene = d.get("scene_name")
        if scene in exclude:
            continue
        fq = d.get("fix_quality")
        if not fq:
            continue                      # predates the step9 patch
        i = d.get("best_index")
        if i is None:
            continue

        def at(key):
            v = fq.get(key)
            if isinstance(v, list):
                return float(v[i]) if 0 <= i < len(v) else None
            return float(v) if v is not None else None

        # Altitude is xyz[2] in the scene frame -- there is no altitude key.
        alt = None
        try:
            alt = float(np.load(d["npz_path"], allow_pickle=True)["xyz"][2])
        except Exception:
            tp = d.get("truePos") or {}
            alt = float(tp["z"]) if "z" in tp else None

        feats = {
            "inliers": at("n_pnp_inliers"),
            "inlier_ratio": at("inlier_ratio"),
            "matches": at("n_matches_total"),
            "reproj_err_px": at("reproj_mean_px"),
            "altitude_m": alt,
        }
        if any(v is None for v in feats.values()):
            continue
        err = d.get("pred_error")
        if err is None or not np.isfinite(err) or err <= 0:
            continue
        # The harness marks pnp_success on `inliers > 0` alone, so a degenerate
        # solve is in here with an error of 1e93. Those are real failures and
        # belong in the REJECTION problem, not in a regression on metres --
        # keeping them makes the fit chase a number no covariance can express.
        if err > 1e4:
            continue
        matcher = Path(p).parents[2].name.split("-")[2]
        rows.append((scene, matcher, feats, float(err)))
    return rows


# The runtime's own feature list, in covariance.FEATURES order. Available in
# full ONLY from --sweep, where the same solve() that flies produced them.
_SWEEP_FEATURES = ["inliers", "inlier_ratio", "matches", "keypoints",
                   "reproj_err_px", "altitude_m", "scale", "tiles_searched"]


def load_sweep(root: str, method: str = None, exclude=("Scene_21",)):
    """Rows from env80_sweep.py CSVs. Same shape load() returns."""
    import csv as _csv
    rows = []
    for f in sorted(glob.glob(os.path.join(root, "*.csv"))):
        for r in _csv.DictReader(open(f)):
            if r.get("scene") in exclude:
                continue
            if method and r.get("method") != method:
                continue
            # Only SOLVED frames carry an error to regress against. A frame the
            # solver rejected has no position and therefore no error; those
            # belong to the rejection problem, which the inlier gate wins.
            if str(r.get("plausible")).lower() not in ("true", "1"):
                continue
            try:
                err = float(r["error_m"])
            except (TypeError, ValueError, KeyError):
                continue
            if not (0 < err <= 1e4):
                continue
            try:
                feats = {k: float(r[k]) for k in _SWEEP_FEATURES}
            except (TypeError, ValueError, KeyError):
                continue
            rows.append((r["scene"], r["method"], feats, err))
    return rows


def fit(X, y):
    from sklearn.ensemble import GradientBoostingRegressor
    m = GradientBoostingRegressor(n_estimators=200, max_depth=3, learning_rate=0.05,
                                  subsample=0.9, random_state=0)
    m.fit(X, y)
    return m


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=str(Path.home() / "GeoAnchor/results/s8_train"),
                    help="parent-repo harness output (5 features after the "
                         "intersection)")
    ap.add_argument("--sweep", default=None,
                    help="this runtime's env80_sweep.py output directory. "
                         "PREFERRED: all 8 runtime features, same definitions "
                         "as the flying code, works for any method.")
    ap.add_argument("--reference", default="satellite")
    ap.add_argument("--matcher", "--method", dest="matcher", default=None,
                    help="train on one matcher only. The finding is that Eq. 7 is "
                         "DESCRIPTOR-SPECIFIC, so a per-matcher model is the "
                         "honest default; pooling is the thing to justify.")
    ap.add_argument("--out", default="models/covariance.pkl")
    ap.add_argument("--exclude-scene", default="Scene_21")
    a = ap.parse_args()

    excl = tuple(s for s in a.exclude_scene.split(",") if s)
    if a.sweep:
        feature_names = _SWEEP_FEATURES
        rows = load_sweep(a.sweep, a.matcher, exclude=excl)
        source = f"sweep:{a.sweep}"
    else:
        feature_names = _FEATURES
        rows = load(a.root, a.reference, exclude=excl)
        if a.matcher:
            rows = [r for r in rows if r[1] == a.matcher]
        source = f"harness:{a.root}"
    if not rows:
        print(f"no usable rows under {a.root}")
        print("  Needs fix_quality, which only runs made after the step9 patch carry.")
        print("  Or use --sweep <env80_sweep output dir>, which needs none of it.")
        return 1

    scenes = sorted({r[0] for r in rows})
    matchers = sorted({r[1] for r in rows})
    print(f"{len(rows)} fixes | scenes {scenes} | matchers {matchers}")
    print(f"excluded: {', '.join(excl) if excl else 'nothing'}")
    if len(scenes) < 3:
        print("\nWARNING: fewer than 3 scenes. Leave-one-scene-out on 2 scenes is "
              "\n  a 50/50 split and the group-identity guard in CLAUDE.md applies: "
              "\n  altitude separates two scenes perfectly and the model splits on it.")

    X = np.array([[r[2][k] for k in feature_names] for r in rows], float)
    y_m = np.array([r[3] for r in rows], float)
    y = np.log(y_m)
    groups = np.array([r[0] for r in rows])

    # ---- leave one SCENE out -------------------------------------------
    print(f"\n\033[1mLeave-one-scene-out\033[0m  (n={len(rows)}, "
          f"{len(feature_names)} features from {source})")
    print(f"  {'held out':12} {'n':>5} {'med pred':>9} {'med actual':>11} {'ratio':>7}")
    preds = np.zeros(len(rows))
    for s in scenes:
        te = groups == s
        tr = ~te
        if tr.sum() < 20 or te.sum() < 5:
            continue
        m = fit(X[tr], y[tr])
        preds[te] = np.exp(m.predict(X[te]))
        mp, ma = np.median(preds[te]), np.median(y_m[te])
        print(f"  {s:12} {te.sum():5d} {mp:9.2f} {ma:11.2f} {mp/ma:7.2f}")

    # ---- does the predicted sigma TRACK the actual error? ---------------
    # The headline claim is a tracking claim (12.0->12.0, 21->21, 38->45), so
    # it is checked the same way: bin by PREDICTED sigma, report the actual
    # error in each bin. A flat actual column means the model is not tracking,
    # whatever its average error looks like.
    ok = preds > 0
    print(f"\n\033[1mCalibration\033[0m -- binned by predicted sigma, out-of-fold")
    print(f"  {'pred sigma':>12} {'n':>5} {'median actual m':>16} {'p90 actual m':>13}")
    qs = np.quantile(preds[ok], [0, .25, .5, .75, 1.0])
    for lo, hi in zip(qs[:-1], qs[1:]):
        sel = ok & (preds >= lo) & (preds <= hi)
        if sel.sum() < 5:
            continue
        print(f"  {np.median(preds[sel]):12.2f} {sel.sum():5d} "
              f"{np.median(y_m[sel]):16.2f} {np.quantile(y_m[sel], .9):13.2f}")
    # Spearman: does the ORDER survive? That is what a covariance has to get
    # right, and it is robust to the scale being off.
    from scipy.stats import spearmanr
    rho, p = spearmanr(preds[ok], y_m[ok])
    print(f"\n  Spearman(predicted, actual) = {rho:.3f}  (p={p:.2g})")
    print("  A covariance has to get the ORDER right above all; the scale can be "
          "recalibrated,\n  a wrong ordering cannot.")

    # ---- final model on everything, and export --------------------------
    model = fit(X, y)
    out = REPO / a.out if not os.path.isabs(a.out) else Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    blob = {
        "model": model,
        "features": feature_names,
        "meta": {
            "target": "log_sigma",
            "calibrated": True,
            "validation": f"leave-one-scene-out over {len(scenes)} scenes, "
                          f"spearman {rho:.3f}",
            "n_train": len(rows),
            "scenes": scenes,
            "matchers": matchers,
            "matcher_filter": a.matcher,
            "reference": a.reference,
            "excluded_scenes": list(excl),
            "banned_as_predictors": sorted(_BANNED),
            "source": source,
            "note": "sigma in METRES. Trained on log(error); Learned.sigma_m "
                    "exponentiates because meta.target == 'log_sigma'.",
        },
    }
    with open(out, "wb") as fh:
        pickle.dump(blob, fh)
    print(f"\nwrote {out}  ({out.stat().st_size / 1024:.0f} KB)")
    print("\nUse it:")
    print(f'  GEOANCHOR_SET="processing_layer.covariance.backend=learned;'
          f'processing_layer.covariance.model_path={a.out}" bash run.sh')
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
