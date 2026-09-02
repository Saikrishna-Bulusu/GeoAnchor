"""How wrong is this fix, in metres.

EKF3 consumes metres squared and clamps horizontal position variance to
[0.01, 100] m. Whatever goes in that field decides how much the filter trusts
the fix, so it is not a diagnostic -- it is a control input.

Measured on 1323 fixes across 4 scenes, leave-one-scene-out, 31 Aug 2026:

    rejection AUC        covariance in metres
    inlier threshold     0.724  wins       cannot produce one
    NGPS Eq. 7 + Eq. 8   0.467             2.4x understated, saturates at 10 m
    learned estimator    0.641             tracks: 12.0->12.0, 21->21, 38->45

Two different problems with two different winners. The threshold rejects, in
solve.py. Only a learned estimator produces a usable number, and Eq. 7 is worse
than useless here because its clip floor of 0.1 caps sigma at 10.00 m -- 31% of
rows pin there -- while actual errors reach 168 m and EKF3 would have accepted
anything up to 100.

Until a model is exported, `gate_only` emits a fixed conservative sigma and
labels itself in every record as a placeholder. That label is the point: a
number with no provenance in a covariance field is exactly the failure the
project is trying to document.
"""
from __future__ import annotations

from pathlib import Path

FEATURES = [
    "inliers", "inlier_ratio", "matches", "keypoints",
    "reproj_err_px", "altitude_m", "scale", "tiles_searched",
]


def features_from(result, altitude_m=None) -> dict:
    return {
        "inliers": float(result.inliers),
        "inlier_ratio": float(result.inlier_ratio),
        "matches": float(result.matches),
        "keypoints": float(result.keypoints),
        "reproj_err_px": float(result.reproj_err_px) if result.reproj_err_px is not None else -1.0,
        "altitude_m": float(altitude_m) if altitude_m is not None else -1.0,
        "scale": float(result.scale) if result.scale is not None else 1.0,
        "tiles_searched": float(result.tiles_searched),
    }


class CovarianceEstimator:
    name = "base"

    def sigma_m(self, feats: dict) -> float:
        raise NotImplementedError

    def describe(self) -> dict:
        return {"backend": self.name}


class GateOnly(CovarianceEstimator):
    """A constant. Honest about being one.

    This is deliberately the same shape as ap_vo2's `position_covariance_xy:
    2.0` -- one ROS parameter written to cov[0] and cov[7] for every fix at
    every altitude over every terrain. Naming that as the hole the contribution
    fills is more useful than filling it with an invented formula.
    """
    name = "gate_only"

    def __init__(self, fixed_sigma_m: float = 8.0, clamp=(0.1, 100.0)):
        self.fixed = float(fixed_sigma_m)
        self.clamp = tuple(clamp)

    def sigma_m(self, feats: dict) -> float:
        return min(max(self.fixed, self.clamp[0]), self.clamp[1])

    def describe(self) -> dict:
        return {"backend": self.name, "sigma_m": self.fixed, "clamp_m": list(self.clamp),
                "calibrated": False,
                "note": "fixed placeholder, not calibrated against measured error"}


class Learned(CovarianceEstimator):
    """A trained estimator. Cross-validated by SCENE, never by row.

    The grouping matters and has already caused one wrong result: with only two
    scenes, `rho`, `log_rho` and `altitude_m` separate the scenes perfectly and
    a tree splits on scene identity first, so the model appears to lose. Any
    model loaded here must have been validated leave-one-scene-out.

    Ground truth may never be an input. PDE, R_i, pdm_at_k, recall_at_k,
    retrieval_gt_rank, pred_error and location_error_list are all distance from
    the true position under different names.
    """
    name = "learned"

    def __init__(self, model_path, clamp=(0.1, 100.0)):
        self.clamp = tuple(clamp)
        self.path = Path(model_path)
        if not self.path.exists():
            raise FileNotFoundError(f"covariance model not found: {self.path}")
        self.model, self.feature_names, self.meta = self._load()

    def _load(self):
        import pickle
        with open(self.path, "rb") as fh:
            blob = pickle.load(fh)
        if isinstance(blob, dict) and "model" in blob:
            return blob["model"], blob.get("features", FEATURES), blob.get("meta", {})
        return blob, FEATURES, {}

    def sigma_m(self, feats: dict) -> float:
        import numpy as np
        x = np.array([[feats.get(k, -1.0) for k in self.feature_names]], dtype=float)
        y = float(self.model.predict(x)[0])
        if self.meta.get("target") == "log_sigma":
            y = float(np.exp(y))
        return min(max(y, self.clamp[0]), self.clamp[1])

    def describe(self) -> dict:
        return {"backend": self.name, "model": str(self.path), "features": self.feature_names,
                "clamp_m": list(self.clamp), "calibrated": bool(self.meta.get("calibrated", False)),
                "validation": self.meta.get("validation", "unstated")}


def build(cfg: dict) -> CovarianceEstimator:
    backend = (cfg or {}).get("backend", "gate_only")
    clamp = tuple((cfg or {}).get("clamp_m", (0.1, 100.0)))
    if backend == "learned":
        return Learned(cfg["model_path"], clamp=clamp)
    return GateOnly((cfg or {}).get("fixed_sigma_m", 8.0), clamp=clamp)
