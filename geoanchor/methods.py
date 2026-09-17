"""Feature methods. Every one of them runs on the CPU.

That constraint is not a limitation to work around, it is the design. A
pipeline with no CUDA in it is the same pipeline on the Pi 5, the AGX Xavier,
the Orin Nano and the 2019 Jetson Nano, so joules per fix across those four
boards is a comparison of the boards and not of four different pipelines. The
moment one board runs a TensorRT engine and another runs PyTorch, the headline
measurement compares implementations instead.

Shared deliberately between map preprocessing and the processing layer. The
reference descriptors in the store and the frame descriptors at flight time
MUST come from the same extractor, so there is exactly one place that decides
what an extractor is.
"""
from __future__ import annotations

import importlib
import os
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent


@dataclass
class Features:
    kpts: np.ndarray                  # (N, 2) float32, x y in image pixels
    desc: np.ndarray                  # (N, D)
    scores: np.ndarray = None         # (N,)
    image_size: tuple = None          # (w, h) -- LighterGlue needs it
    extra: dict = field(default_factory=dict)

    def __len__(self) -> int:
        return 0 if self.kpts is None else len(self.kpts)


class Method:
    name = "base"
    kind = "float"          # "binary" for ORB/AKAZE
    learned = False

    def available(self) -> tuple:
        return True, ""

    def detect(self, image_bgr: np.ndarray) -> Features:
        raise NotImplementedError

    def match(self, fa: Features, fb: Features) -> tuple:
        """Return (idx_a, idx_b, confidence)."""
        raise NotImplementedError

    def describe(self) -> dict:
        return {"name": self.name, "kind": self.kind, "learned": self.learned,
                "label": label(self.name), "blurb": blurb(self.name)}


# --------------------------------------------------------------------------
# What a human should see. The keys below are NOT renamed, and must not be.
#
# A method name is load-bearing in three places that outlive any one run:
# the feature store id hashes it in (`ref_tile__xfeat_mnn__<hash>`), every
# session.json record carries `"method": "xfeat_mnn"`, and the parent repo's
# results/ tables are keyed on it. Renaming `xfeat_mnn` to something friendly
# would invalidate every built store on every board and make every exported
# session unreadable, to change a string on a dropdown.
#
# So the identifier stays and the label is separate. `mnn` is mutual nearest
# neighbour, `lg` is LighterGlue, `t`/`s` are EdgePoint2's tiny and small
# networks and the number is the descriptor width -- all of which are obvious
# once you know and opaque until you do.
# --------------------------------------------------------------------------
DISPLAY = {
    "orb":   ("ORB",   "Classical. Fastest thing here and it runs on any board. "
                       "Returned zero geometrically plausible fixes against real "
                       "satellite reference -- fast is not the same as usable."),
    "sift":  ("SIFT",  "Classical. Beats XFeat for speed on ARM, which surprises "
                       "people. Still no plausible fixes on Scene_10."),
    "akaze": ("AKAZE", "Classical. 6 plausible fixes in 192 frames on Scene_10, "
                       "all inside 5 m. The one classical method that is not "
                       "simply broken on real reference."),
    "xfeat_mnn": ("XFeat + Nearest-Neighbour",
                  "Learned detector, plain descriptor matching. The general-"
                  "purpose default. Wants an inlier gate around 10."),
    "xfeat_lg":  ("XFeat + LighterGlue",
                  "Learned detector AND learned matcher. Most accurate on paper "
                  "and unusable on a CPU board: 2-3.6 s per fix, and 9x SLOWER on "
                  "the frames it fails than on the ones it solves."),
    "edgepoint2_t32": ("EdgePoint2 Tiny (32-D)",
                       "Smallest network, narrow descriptor. Collapses on the "
                       "hard scene -- 0.8% accept where the 64-D gets 5.2%."),
    "edgepoint2_s32": ("EdgePoint2 Small (32-D)",
                       "Narrow descriptor buys no speed here and costs accuracy. "
                       "Measured for the comparison, not recommended."),
    "ripepp": ("RIPE++ (MegaDepth)",
               "Learns from positive pairs only, no correspondence labels, and "
               "ships its own trainable matcher -- the one path to a fine-tuned "
               "descriptor that does not break its matcher. ACADEMIC LICENCE: "
               "measurable, never shippable."),
    "ripepp_tokyo": ("RIPE++ (MegaDepth + Tokyo 24/7)",
                     "The only published checkpoint trained for appearance "
                     "change over time, which is the nearest analogue to the "
                     "cross-date problem. Same academic licence."),
    "ripepp_scared": ("RIPE++ (SCARED)",
                      "Endoscopy checkpoint. Included for completeness; there "
                      "is no reason to expect it to transfer to aerial."),
    "edgepoint2_s64": ("EdgePoint2 Small (64-D)",
                       "Faster than XFeat on both scenes and its worst error is "
                       "8.4 m where XFeat's is 187 m. Wants a LOWER inlier gate "
                       "(8, not 10) -- using XFeat's gate throws away a third of "
                       "its fixes."),
}


def label(name: str) -> str:
    """Human-readable name. Falls back to the identifier for anything new."""
    return DISPLAY.get(name, (name, ""))[0]


def blurb(name: str) -> str:
    """One-paragraph 'should I pick this', from measured numbers only."""
    return DISPLAY.get(name, (name, ""))[1]


# --------------------------------------------------------------------------
# Classical. No weights, no torch, works everywhere including a fresh JetPack.
# --------------------------------------------------------------------------
class _OpenCVMethod(Method):
    def __init__(self, name: str, max_keypoints: int = 4096):
        self.name = name
        self.max_keypoints = int(max_keypoints)
        self._det = None
        self._bf = None

    def _detector(self):
        if self._det is None:
            if self.name == "orb":
                self._det = cv2.ORB_create(nfeatures=self.max_keypoints)
            elif self.name == "sift":
                self._det = cv2.SIFT_create(nfeatures=self.max_keypoints)
            elif self.name == "akaze":
                # AKAZE has no nfeatures; it is capped after detection instead.
                self._det = cv2.AKAZE_create()
            else:
                raise ValueError(self.name)
        return self._det

    def available(self) -> tuple:
        ctor = {"orb": "ORB_create", "sift": "SIFT_create", "akaze": "AKAZE_create"}[self.name]
        if not hasattr(cv2, ctor):
            return False, (
                f"cv2.{ctor} is missing. OpenCV 5 moved the 2D-features constructors; "
                "this project pins opencv-python-headless<5 for exactly that reason."
            )
        return True, ""

    def detect(self, image_bgr: np.ndarray) -> Features:
        gray = image_bgr if image_bgr.ndim == 2 else cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        kp, desc = self._detector().detectAndCompute(gray, None)
        if desc is None or len(kp) == 0:
            h, w = gray.shape[:2]
            return Features(np.zeros((0, 2), np.float32), np.zeros((0, 1), np.float32),
                            np.zeros((0,), np.float32), (w, h))
        if len(kp) > self.max_keypoints:
            order = np.argsort([-k.response for k in kp])[: self.max_keypoints]
            kp = [kp[i] for i in order]
            desc = desc[order]
        pts = np.array([k.pt for k in kp], dtype=np.float32)
        sc = np.array([k.response for k in kp], dtype=np.float32)
        h, w = gray.shape[:2]
        return Features(pts, desc, sc, (w, h))

    def match(self, fa: Features, fb: Features) -> tuple:
        if len(fa) == 0 or len(fb) == 0:
            return np.zeros(0, int), np.zeros(0, int), np.zeros(0, np.float32)
        norm = cv2.NORM_HAMMING if self.name in ("orb", "akaze") else cv2.NORM_L2
        if self._bf is None:
            # crossCheck IS mutual nearest neighbour, done inside OpenCV.
            self._bf = cv2.BFMatcher(norm, crossCheck=True)
        m = self._bf.match(fa.desc, fb.desc)
        if not m:
            return np.zeros(0, int), np.zeros(0, int), np.zeros(0, np.float32)
        ia = np.array([x.queryIdx for x in m], dtype=int)
        ib = np.array([x.trainIdx for x in m], dtype=int)
        d = np.array([x.distance for x in m], dtype=np.float32)
        conf = 1.0 / (1.0 + d / (d.mean() + 1e-6))
        return ia, ib, conf


# --------------------------------------------------------------------------
# XFeat. Apache 2.0, 64-D descriptors, authors target CPU explicitly.
# --------------------------------------------------------------------------
def _xfeat_root() -> Path | None:
    candidates = [
        os.environ.get("XFEAT_ROOT"),
        REPO_ROOT / "xfeat",
        REPO_ROOT.parent / "third_party" / "accelerated_features",
        Path.home() / "GeoAnchor" / "third_party" / "accelerated_features",
    ]
    for c in candidates:
        if not c:
            continue
        p = Path(c)
        if (p / "modules" / "xfeat.py").exists():
            return p
    return None


class XFeatMethod(Method):
    kind = "float"
    learned = True

    def __init__(self, name: str = "xfeat_mnn", max_keypoints: int = 4096,
                 min_cossim: float = 0.82, threads: int = 0):
        self.name = name
        self.use_lighterglue = name.endswith("_lg")
        self.max_keypoints = int(max_keypoints)
        self.min_cossim = float(min_cossim)
        self.threads = threads
        self._x = None
        self._torch = None

    def available(self) -> tuple:
        root = _xfeat_root()
        if root is None:
            return False, ("accelerated_features not found. Set XFEAT_ROOT, or place the "
                           "clone at <repo>/xfeat. bootstrap.sh does this for you.")
        try:
            importlib.import_module("torch")
        except ImportError:
            return False, "torch is not installed -- run bootstrap.sh"
        if not (root / "weights" / "xfeat.pt").exists():
            return False, f"weights missing at {root/'weights'/'xfeat.pt'}"
        if self.use_lighterglue and not (root / "weights" / "xfeat-lighterglue.pt").exists():
            return False, "xfeat-lighterglue.pt missing"
        return True, ""

    def _load(self):
        if self._x is not None:
            return
        root = _xfeat_root()
        if root is None:
            raise RuntimeError("accelerated_features not found")
        if str(root) not in sys.path:
            sys.path.insert(0, str(root))
        import torch
        self._torch = torch
        if self.threads:
            torch.set_num_threads(int(self.threads))
        if torch.version.cuda is not None:
            # Not fatal, but worth knowing: a CUDA build on an ARM board means
            # pip resolved the wrong wheel and dragged in a gigabyte of dead code.
            print("[methods] WARNING: this torch is a CUDA build; the CPU-only wheel was expected",
                  file=sys.stderr)
        mod = importlib.import_module("modules.xfeat")
        self._x = mod.XFeat(top_k=self.max_keypoints)
        self._x.dev = torch.device("cpu")
        self._x.net = self._x.net.to("cpu").eval()
        if self.use_lighterglue:
            lg = importlib.import_module("modules.lighterglue")
            self._x.lighterglue = lg.LighterGlue().to("cpu").eval()

    def detect(self, image_bgr: np.ndarray) -> Features:
        self._load()
        torch = self._torch
        img = image_bgr
        if img.ndim == 2:
            img = np.repeat(img[:, :, None], 3, axis=2)
        rgb = img[:, :, ::-1].copy()                     # XFeat trains on RGB
        t = torch.from_numpy(rgb).permute(2, 0, 1).float()[None]
        with torch.inference_mode():
            out = self._x.detectAndCompute(t, top_k=self.max_keypoints)[0]
        h, w = img.shape[:2]
        f = Features(
            kpts=out["keypoints"].cpu().numpy().astype(np.float32),
            desc=out["descriptors"].cpu().numpy().astype(np.float32),
            scores=out["scores"].cpu().numpy().astype(np.float32) if "scores" in out else None,
            image_size=(w, h),
        )
        f.extra["_t"] = out
        return f

    def match(self, fa: Features, fb: Features) -> tuple:
        self._load()
        torch = self._torch
        if len(fa) == 0 or len(fb) == 0:
            return np.zeros(0, int), np.zeros(0, int), np.zeros(0, np.float32)

        if self.use_lighterglue:
            d0, d1 = self._as_dict(fa), self._as_dict(fb)
            with torch.inference_mode():
                # Returns (mkpts0, mkpts1, idxs). The third value is the (N, 2)
                # index pairs, which is what the solver wants; recovering
                # indices by matching the returned coordinates back would be
                # both slower and ambiguous when two keypoints coincide.
                _, _, idxs = self._x.match_lighterglue(d0, d1)
            idxs = np.asarray(idxs)
            if idxs.size == 0:
                return np.zeros(0, int), np.zeros(0, int), np.zeros(0, np.float32)
            ia = idxs[:, 0].astype(int)
            ib = idxs[:, 1].astype(int)
            return ia, ib, np.ones(len(ia), np.float32)

        da = torch.from_numpy(fa.desc)
        db = torch.from_numpy(fb.desc)
        with torch.inference_mode():
            ia, ib = self._x.match(da, db, min_cossim=self.min_cossim)
        ia = np.asarray(ia.cpu() if hasattr(ia, "cpu") else ia, dtype=int).ravel()
        ib = np.asarray(ib.cpu() if hasattr(ib, "cpu") else ib, dtype=int).ravel()
        n = min(len(ia), len(ib))
        return ia[:n], ib[:n], np.ones(n, np.float32)

    def _as_dict(self, f: Features) -> dict:
        """Shapes matter here and the failure is not obvious.

        match_lighterglue does `d['image_size'][None, ...]` internally, so
        image_size must arrive as (2,) and become (1, 2). Handing it (1, 2) --
        which reads more natural, and is what a lot of example code does --
        makes it (1, 1, 2), the keypoint normalisation broadcasts an extra
        axis, and the failure surfaces far away as
        `mat1 and mat2 shapes cannot be multiplied` inside kornia's attention.
        """
        torch = self._torch
        w, h = f.image_size
        d = {
            "keypoints": torch.from_numpy(f.kpts),
            "descriptors": torch.from_numpy(f.desc),
            "image_size": torch.tensor([float(w), float(h)], dtype=torch.float32),
        }
        if f.scores is not None:
            d["scores"] = torch.from_numpy(f.scores)
        return d


# --------------------------------------------------------------------------
# EdgePoint2. MIT, 32/48/64-D descriptors, trained for embedded CPU inference.
#
# The reason to have it is the DESCRIPTOR WIDTH, not the network. On this
# hardware the published CPU gain over XFeat is 1.16x (Orin-NX, 40.84 vs 35.33
# FPS) even though it uses 2.7x fewer GFLOPs -- the CPU path is memory-bound,
# not FLOP-bound, which is the same reason detection here uses only 2.7 of 8
# cores. Matching, however, is a dense descriptor product and is linear in
# width, so 32-D against XFeat's 64-D should roughly halve it, and halves the
# reference store on disk and in RAM with it.
#
# Whether the accuracy holds is the open question and the reason this is a
# method rather than a replacement: the paper evaluates homography, relative
# pose, FM-Bench and visual localization, none of which are cross-view aerial
# to satellite. Measure on env80 before believing anything.
# --------------------------------------------------------------------------
def _edgepoint2_root() -> Path | None:
    candidates = [
        os.environ.get("EDGEPOINT2_ROOT"),
        REPO_ROOT / "edgepoint2",
        REPO_ROOT.parent / "third_party" / "EdgePoint2",
        Path.home() / "GeoAnchor" / "third_party" / "EdgePoint2",
    ]
    for c in candidates:
        if not c:
            continue
        p = Path(c)
        if (p / "edgepoint2.py").exists() and (p / "model" / "model.py").exists():
            return p
    return None


class EdgePoint2Method(Method):
    kind = "float"
    learned = True

    # T/S/M/L/E is network size, the number is descriptor width. T32 is the
    # cheapest at 130 KB of weights; E64 is the most accurate and the biggest.
    CONFIGS = ("T32", "T48", "S32", "S48", "S64", "M32", "M48", "M64",
               "L32", "L48", "L64", "E32", "E48", "E64")

    # The upstream default is score=-5, and on env80 query frames that gate --
    # not top_k -- is what decides the keypoint count: -5 yields 2278 where
    # XFeat yields 4096, and the top_k sweep showed accept rate rising
    # monotonically with keypoint count (0% at 512 to 15.1% at 4096). -12
    # saturates top_k on the same frames, which puts the two methods on the
    # same budget and leaves top_k as the single knob, which is what the sweep
    # is measuring. Raise it towards -5 to trade keypoints back for speed.
    DEFAULT_SCORE = -12.0

    # Reference-keypoint count above which the transpose-product matcher wins.
    # The crossover is a cache effect and is therefore BOARD-SPECIFIC: this is
    # the AGX Xavier's, and it must be re-measured on a Pi 5 or a TX2 rather
    # than assumed. Getting it wrong costs tens of milliseconds, never
    # correctness -- both forms return identical index sets.

    def __init__(self, name: str = "edgepoint2_t32", max_keypoints: int = 4096,
                 min_cossim: float = 0.82, threads: int = 0, score: float = None):
        self.name = name
        self.cfg = name.rsplit("_", 1)[-1].upper()
        self.max_keypoints = int(max_keypoints)
        self.min_cossim = float(min_cossim)
        self.threads = threads
        self.score = self.DEFAULT_SCORE if score is None else float(score)
        self._m = None
        self._torch = None

    def available(self) -> tuple:
        root = _edgepoint2_root()
        if root is None:
            return False, ("EdgePoint2 not found. Set EDGEPOINT2_ROOT, or clone "
                           "https://github.com/HITCSC/EdgePoint2 to <repo>/edgepoint2.")
        if self.cfg not in self.CONFIGS:
            return False, f"unknown config '{self.cfg}'. Known: {', '.join(self.CONFIGS)}"
        try:
            importlib.import_module("torch")
        except ImportError as exc:
            # Report what actually failed. "torch is not installed" is a guess,
            # and it sends you to bootstrap.sh when the real fault is a broken
            # link or a shadowed symbol in an installed torch.
            return False, f"torch unusable ({exc}) -- run bootstrap.sh"
        if not (root / "weights" / f"{self.cfg}.pth").exists():
            return False, f"weights missing at {root/'weights'/f'{self.cfg}.pth'}"
        return True, ""

    def _load(self):
        if self._m is not None:
            return
        root = _edgepoint2_root()
        if root is None:
            raise RuntimeError("EdgePoint2 not found")
        if str(root) not in sys.path:
            sys.path.insert(0, str(root))
        import torch
        self._torch = torch
        if self.threads:
            torch.set_num_threads(int(self.threads))
        mod = importlib.import_module("edgepoint2")
        # EdgePoint2Wrapper.__init__ does torch.load(f'./weights/{cfg}.pth'),
        # a path relative to the PROCESS cwd rather than to the package. Every
        # layer here runs from the repo root, so loading it unaided raises
        # FileNotFoundError -- and it would do so lazily, on the first frame.
        # chdir for the duration of construction only, and restore it even if
        # the load raises, because a layer that silently changed the working
        # directory would break every relative path in the config after it.
        cwd = os.getcwd()
        try:
            os.chdir(root)
            self._m = mod.EdgePoint2Wrapper(self.cfg, top_k=self.max_keypoints,
                                            score=self.score).eval()
        finally:
            os.chdir(cwd)

    def detect(self, image_bgr: np.ndarray) -> Features:
        self._load()
        torch = self._torch
        img = image_bgr
        if img.ndim == 2:
            img = np.repeat(img[:, :, None], 3, axis=2)
        # RGB and 0..1, unlike XFeat which takes 0..255. Feeding it 0..255
        # produces keypoints and descriptors without complaint, just far worse
        # ones, so there is nothing to catch downstream except a bad inlier
        # count that looks like a hard scene.
        rgb = img[:, :, ::-1].copy()
        t = torch.from_numpy(rgb).permute(2, 0, 1).float()[None] / 255.0
        with torch.inference_mode():
            out = self._m(t)[0]
        h, w = img.shape[:2]
        return Features(
            kpts=out["keypoints"].cpu().numpy().astype(np.float32),
            desc=out["descriptors"].cpu().numpy().astype(np.float32),
            scores=out["scores"].cpu().numpy().astype(np.float32),
            image_size=(w, h),
        )

    def match(self, fa: Features, fb: Features) -> tuple:
        self._load()
        torch = self._torch
        if len(fa) == 0 or len(fb) == 0:
            return np.zeros(0, int), np.zeros(0, int), np.zeros(0, np.float32)
        # Descriptors leave the network L2-normalised (model.sample defaults to
        # norm=True), so the dot product IS cosine similarity and the same
        # min_cossim threshold as XFeat applies unchanged.
        da = torch.from_numpy(fa.desc)
        db = torch.from_numpy(fb.desc)
        with torch.inference_mode():
            # Mutual-nearest-neighbour needs the argmax along BOTH axes, and
            # the second one has two forms: reduce the existing matrix the
            # other way (max(dim=0)), or pay for a second GEMM and reduce that
            # along the contiguous axis. This used to pick between them on
            # reference size, via WIDE_REF. That was wrong in shape, not just
            # in value -- see CLAUDE.md. max(dim=0) is not monotonic in N,
            # because torch dispatches different reduction kernels by shape:
            # on this frame's real store it costs 227 ms at 10 tiles and
            # 166 ms at 9, and on a Pi 5 it reaches 2019 ms at N=32768 against
            # the second GEMM's 444. Which side of that discontinuity a frame
            # lands on is decided by how many tiles the prior selects, which is
            # the worst possible input to a latency budget.
            #
            # So: always the second GEMM, which is what XFeat's own matcher
            # does (xfeat/modules/xfeat.py:328). Measured on the real store,
            # identical matches at every size, Xavier at MAXN:
            #
            #     tiles  ref kp   max(dim=0)   2nd GEMM    delta
            #         1    2048         21.1       19.5     -1.6
            #         4    8192         85.9       51.5    -34.4
            #         9   18432        166.4      123.2    -43.3
            #        10   20480        226.7      113.5   -113.2
            #        13   26624        224.7      153.3    -71.4
            #        25   51200        448.3      332.1   -116.1
            #
            # Faster at every geometry on both boards, and monotonic. The
            # synthetic sweep in scripts/wide_ref_sweep.py showed a window
            # where max(dim=0) won; it does not survive contact with the real
            # store's descriptor layout, so trust the store measurement.
            cossim = da @ db.T
            best, m12 = cossim.max(dim=1)
            _, m21 = (db @ da.T).max(dim=1)
            idx1 = torch.arange(len(m12))
            keep = (m21[m12] == idx1) & (best > self.min_cossim)
            idx1, idx2, conf = idx1[keep], m12[keep], best[keep]
        return (idx1.cpu().numpy().astype(int),
                idx2.cpu().numpy().astype(int),
                conf.cpu().numpy().astype(np.float32))

    def describe(self) -> dict:
        d = super().describe()
        d["config"] = self.cfg
        d["descriptor_dim"] = int(self.cfg[1:])
        d["score_threshold"] = self.score
        return d


# --------------------------------------------------------------------------
# RIPE++. LIMIT@ECCV 2026, arXiv 2608.19693, Fraunhofer HHI.
#
# LICENCE FIRST, because it is the one thing that constrains where this can go.
# RIPE++ is **NOT** open source: it ships under Fraunhofer's "Software Copyright
# License for Academic Use", which permits "non-commercial purposes of
# evaluation, testing and academic research" and prohibits commercial use or
# exploitation of the code or any modification of it. German law, Munich
# jurisdiction. Every other matcher here is permissive -- XFeat Apache 2.0,
# EdgePoint2 MIT, SIFT/ORB/AKAZE in OpenCV -- so this is the first one that can
# be MEASURED and reported but can never ship in anything flown commercially or
# released. Treat it as a benchmark reference point, not a candidate.
#
# THE REASON IT IS INTERESTING is not that it is a ninth row. It learns from
# POSITIVE IMAGE PAIRS ONLY, with no correspondence labels, and it ships a
# trainable matcher of its own. That second half is what makes a fine-tune
# viable at all here: fine-tuning XFeat's descriptor breaks LighterGlue,
# because LighterGlue's first layer was fitted to XFeat's descriptor basis, so
# the accuracy leader and the fine-tuning path are mutually exclusive. RIPE++
# brings its own matcher and sidesteps that.
#
# Three checkpoints are published: MegaDepth (default), MegaDepth + Tokyo 24/7
# (aachen_day_night), and SCARED (endoscopy). None is aerial-to-satellite, and
# none is cross-date, so env80 decides whether any of it transfers. The Tokyo
# variant is the one worth trying second: it is the only one trained for
# appearance change across time of day, which is the nearest published analogue
# to the cross-date problem in results/crossdate_wayback_curve.md.
#
# The adapter matches with a plain mutual nearest neighbour, the same reduction
# every other learned method here uses, so the comparison isolates the
# DESCRIPTOR. Its own trainable matcher is a separate experiment and would not
# be comparable to the rest of this table.
# --------------------------------------------------------------------------
def _ripepp_root() -> Path | None:
    candidates = [
        os.environ.get("RIPEPP_ROOT"),
        REPO_ROOT / "ripepp",
        REPO_ROOT.parent / "third_party" / "RIPEpp",
    ]
    for c in candidates:
        if not c:
            continue
        p = Path(c)
        if (p / "ripepp" / "models" / "ripepp.py").exists():
            return p
    return None


class RIPEppMethod(Method):
    kind = "float"
    learned = True

    VARIANTS = {"ripepp": "default",
                "ripepp_tokyo": "aachen_day_night",
                "ripepp_scared": "scared"}

    # Upstream's demo uses threshold=0.5 with top_k=2048. The threshold is a
    # heatmap gate and, as with EdgePoint2's `score`, it and not top_k is what
    # decides the keypoint count on a low-contrast frame -- which is every
    # frame in this project. Kept at the published value so the first env80
    # number is comparable with the paper; sweep it before drawing conclusions
    # about keypoint budget, exactly as EdgePoint2's DEFAULT_SCORE had to be.
    DEFAULT_THRESHOLD = 0.5

    def __init__(self, name: str = "ripepp", max_keypoints: int = 4096,
                 min_cossim: float = 0.82, threads: int = 0,
                 threshold: float = None):
        self.name = name
        self.variant = self.VARIANTS.get(name, "default")
        self.max_keypoints = int(max_keypoints)
        self.min_cossim = float(min_cossim)
        self.threads = threads
        self.threshold = self.DEFAULT_THRESHOLD if threshold is None else float(threshold)
        self._m = None
        self._torch = None

    def available(self) -> tuple:
        root = _ripepp_root()
        if root is None:
            return False, ("RIPE++ not found. Set RIPEPP_ROOT, or clone "
                           "https://github.com/fraunhoferhhi/RIPEpp to <repo>/ripepp.")
        if self.name not in self.VARIANTS:
            return False, f"unknown variant '{self.name}'. Known: {', '.join(self.VARIANTS)}"
        for mod in ("torch", "torchvision", "omegaconf"):
            try:
                importlib.import_module(mod)
            except ImportError as exc:
                return False, f"{mod} unusable ({exc}) -- see bootstrap.sh"
        # The checkpoint is downloaded on first use from a Fraunhofer server.
        # Report that rather than letting a layer discover it on frame one, on
        # a board that may have no network at all.
        if self._checkpoint() is None:
            return False, (f"checkpoint for '{self.variant}' not present. It downloads "
                           "from cvg.hhi.fraunhofer.de on first use -- do that on a "
                           "networked machine, not on the aircraft.")
        return True, ""

    def _checkpoint(self) -> Path | None:
        """The local checkpoint, if it is already there. Never downloads."""
        root = _ripepp_root()
        if root is None:
            return None
        names = {"default": "ripe++.ckpt",
                 "aachen_day_night": "ripe++_tokyo_megadepth.ckpt",
                 "scared": "ripe++_scared.ckpt"}
        fn = names[self.variant]
        import torch
        for c in (root / "weights" / fn,
                  Path(torch.hub.get_dir()) / "checkpoints" / fn):
            if c.exists():
                return c
        return None

    def _load(self):
        if self._m is not None:
            return
        root = _ripepp_root()
        if root is None:
            raise RuntimeError("RIPE++ not found")
        if str(root) not in sys.path:
            sys.path.insert(0, str(root))
        import torch
        self._torch = torch
        if self.threads:
            torch.set_num_threads(int(self.threads))
        ckpt = self._checkpoint()
        if ckpt is None:
            raise RuntimeError(
                "RIPE++ checkpoint is not downloaded. Run it once on a networked "
                "machine, or place the .ckpt in <ripepp>/weights/.")
        mod = importlib.import_module("ripepp")
        # device="cpu" explicitly, and it is the upstream default too. CLAUDE.md:
        # no CUDA anywhere in this pipeline, because the headline measurement is
        # joules per fix ACROSS BOARDS and a CUDA path on one of them compares
        # implementations instead.
        self._m = mod.load_model_from_checkpoint(ckpt, device="cpu").eval()

    def detect(self, image_bgr: np.ndarray) -> Features:
        self._load()
        torch = self._torch
        img = image_bgr
        if img.ndim == 2:
            img = np.repeat(img[:, :, None], 3, axis=2)
        # RGB, 0..1, NCHW -- the same convention as EdgePoint2 and the opposite
        # of XFeat's 0..255. Feeding the wrong range produces keypoints without
        # complaint, just far worse ones, so there is nothing downstream to
        # catch it except an inlier count that looks like a hard scene.
        rgb = img[:, :, ::-1].copy()
        t = torch.from_numpy(rgb).permute(2, 0, 1).float()[None] / 255.0
        with torch.inference_mode():
            kpts, desc, scores = self._m.detectAndCompute(
                t, threshold=self.threshold, top_k=self.max_keypoints)
        h, w = img.shape[:2]
        return Features(
            kpts=kpts.squeeze(0).cpu().numpy().astype(np.float32),
            desc=desc.squeeze(0).cpu().numpy().astype(np.float32),
            scores=scores.squeeze(0).cpu().numpy().astype(np.float32),
            image_size=(w, h),
        )

    def match(self, fa: Features, fb: Features) -> tuple:
        self._load()
        torch = self._torch
        if len(fa) == 0 or len(fb) == 0:
            return np.zeros(0, int), np.zeros(0, int), np.zeros(0, np.float32)
        da = torch.from_numpy(fa.desc)
        db = torch.from_numpy(fb.desc)
        with torch.inference_mode():
            # Descriptors are not guaranteed L2-normalised the way XFeat's and
            # EdgePoint2's are, and min_cossim is a COSINE threshold, so
            # normalise rather than assume. Skipping this silently turns the
            # gate into a magnitude test.
            da = torch.nn.functional.normalize(da, dim=1)
            db = torch.nn.functional.normalize(db, dim=1)
            # Two GEMMs, never max(dim=0). Same reasoning as EdgePoint2Method:
            # the single-matmul reduction is not monotonic in reference size
            # and puts a 2x latency discontinuity where the prior's tile count
            # decides which side a frame lands on. See CLAUDE.md.
            cossim = da @ db.T
            best, m12 = cossim.max(dim=1)
            _, m21 = (db @ da.T).max(dim=1)
            idx1 = torch.arange(len(m12))
            keep = (m21[m12] == idx1) & (best > self.min_cossim)
            idx1, idx2, conf = idx1[keep], m12[keep], best[keep]
        return (idx1.cpu().numpy().astype(int),
                idx2.cpu().numpy().astype(int),
                conf.cpu().numpy().astype(np.float32))

    def describe(self) -> dict:
        d = super().describe()
        d["variant"] = self.variant
        d["threshold"] = self.threshold
        d["licence"] = "Fraunhofer academic use only -- non-commercial, not shippable"
        return d


# --------------------------------------------------------------------------
REGISTRY = {
    "orb":       lambda **kw: _OpenCVMethod("orb", kw.get("max_keypoints", 4096)),
    "sift":      lambda **kw: _OpenCVMethod("sift", kw.get("max_keypoints", 4096)),
    "akaze":     lambda **kw: _OpenCVMethod("akaze", kw.get("max_keypoints", 4096)),
    "xfeat_mnn": lambda **kw: XFeatMethod("xfeat_mnn", **_xf(kw)),
    # T32 is the cheapest and the widest departure from XFeat (32-D vs 64-D);
    # S64 matches XFeat's descriptor width, so the pair separates "the network
    # is better" from "the descriptor is narrower".
    "edgepoint2_t32": lambda **kw: EdgePoint2Method("edgepoint2_t32", **_ep2(kw)),
    "edgepoint2_s32": lambda **kw: EdgePoint2Method("edgepoint2_s32", **_ep2(kw)),
    "edgepoint2_s64": lambda **kw: EdgePoint2Method("edgepoint2_s64", **_ep2(kw)),
    "xfeat_lg":  lambda **kw: XFeatMethod("xfeat_lg", **_xf(kw)),
    # Academic licence only -- measurable, never shippable. See the class.
    "ripepp":        lambda **kw: RIPEppMethod("ripepp", **_ripe(kw)),
    "ripepp_tokyo":  lambda **kw: RIPEppMethod("ripepp_tokyo", **_ripe(kw)),
    "ripepp_scared": lambda **kw: RIPEppMethod("ripepp_scared", **_ripe(kw)),
}


def _xf(kw: dict) -> dict:
    return {
        "max_keypoints": kw.get("max_keypoints", 4096),
        "min_cossim": kw.get("min_cossim", 0.82),
        "threads": kw.get("threads", 0),
    }


def _ripe(kw: dict) -> dict:
    d = _xf(kw)
    d["threshold"] = kw.get("threshold")   # None means RIPEppMethod.DEFAULT_THRESHOLD
    return d


def _ep2(kw: dict) -> dict:
    d = _xf(kw)
    d["score"] = kw.get("score")          # None means EdgePoint2Method.DEFAULT_SCORE
    return d


def build(name: str, **kw) -> Method:
    if name not in REGISTRY:
        raise KeyError(f"unknown method '{name}'. Known: {', '.join(sorted(REGISTRY))}")
    return REGISTRY[name](**kw)


def survey(**kw) -> dict:
    """What can actually run here. The dashboard greys out the rest."""
    out = {}
    for name in REGISTRY:
        try:
            ok, why = build(name, **kw).available()
        except Exception as exc:
            ok, why = False, f"{type(exc).__name__}: {exc}"
        out[name] = {"available": ok, "reason": why,
                     "label": label(name), "blurb": blurb(name)}
    return out


if __name__ == "__main__":
    import json
    print(json.dumps(survey(), indent=2))
    rng = np.random.default_rng(0)
    a = (rng.random((320, 400, 3)) * 255).astype(np.uint8)
    b = np.roll(a, 23, axis=1)
    for name, info in survey().items():
        if not info["available"]:
            print(f"{name:11s} unavailable: {info['reason'][:70]}")
            continue
        m = build(name)
        t0 = time.perf_counter(); fa = m.detect(a); fb = m.detect(b)
        t1 = time.perf_counter(); ia, ib, _ = m.match(fa, fb); t2 = time.perf_counter()
        print(f"{name:11s} kp {len(fa):5d}/{len(fb):5d}  matches {len(ia):5d}  "
              f"detect {(t1-t0)*500:6.1f} ms/img  match {(t2-t1)*1000:6.1f} ms")
