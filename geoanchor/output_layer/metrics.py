"""Error, error rate, and loss.

One rule governs everything here: NEVER REPORT A MEAN. Satellite runs in this
project contain fixes wrong by up to 2.06e93 m, because a homography or PnP
solve can converge on a degenerate solution that passes an `inliers > 0` check.
A single such row destroys a mean and an RMSE. Medians, percentiles and the
fraction inside a distance band survive it. That rules out RMSE, which is what
NGPS and most of the literature report, and saying so is itself a finding.

`loss` is the negative log-likelihood of the actual error under the emitted
covariance, not a plain squared error. That choice is deliberate: the question
this project asks is whether the stated uncertainty matches the error actually
made, and NLL is the quantity that answers it. A pipeline can lower a squared
error and still be badly calibrated; it cannot lower NLL without its sigma
becoming honest.
"""
from __future__ import annotations

import math
import random

from ..geo import distance_m

TWO_PI = 2.0 * math.pi


def position_error_m(actual: dict, predicted: dict):
    if not actual or not predicted:
        return None
    if predicted.get("lat") is None or actual.get("lat") is None:
        return None
    return distance_m(actual["lat"], actual["lon"], predicted["lat"], predicted["lon"])


def nll(error_m: float, sigma_m: float) -> float:
    """Isotropic 2D Gaussian. Radial error e under sigma:

        L = e^2 / (2 sigma^2) + ln(2 pi sigma^2)

    Falling sigma lowers the second term and raises the first, so the minimum
    sits where sigma matches the error actually being made. That is what makes
    it a calibration measure rather than an accuracy measure.
    """
    s = max(float(sigma_m), 1e-3)
    return (float(error_m) ** 2) / (2.0 * s * s) + math.log(TWO_PI * s * s)


def l2(error_m: float, sigma_m=None) -> float:
    return float(error_m) ** 2


LOSSES = {"nll": nll, "l2": l2}


def compute_loss(kind: str, error_m, sigma_m):
    if error_m is None:
        return None
    fn = LOSSES.get(kind, nll)
    if fn is nll and sigma_m is None:
        return None
    return fn(error_m, sigma_m)


def _percentile(values: list, q: float):
    # Drop non-finite values BEFORE sorting. A NaN does not compare true against
    # anything, so sorted() leaves it wherever it started rather than pushing it
    # to an end: the result is not a NaN median that announces itself, it is a
    # median silently taken from the wrong row, with every percentile past that
    # point shifted too. Dropping is the only defensible reading -- a fix whose
    # error is NaN has no position on the distribution.
    s = sorted(v for v in values if isinstance(v, (int, float)) and math.isfinite(v))
    if not s:
        return None
    if len(s) == 1:
        return s[0]
    pos = (len(s) - 1) * q
    lo, hi = int(math.floor(pos)), int(math.ceil(pos))
    return s[lo] if lo == hi else s[lo] + (s[hi] - s[lo]) * (pos - lo)


class Running:
    """Running aggregates for the dashboard and the session summary.

    Every sample list here is capped. Unbounded, they are four floats per fix
    held for the life of the process, which is fine for a flight and not fine
    for a replay feed on loop -- the same failure that made session.json grow
    until the output layer was killed.

    What the cap must not do is move the numbers. So the quantities that can be
    maintained exactly are maintained exactly and never read off a sample:
    `max_error_m` is a running maximum, and the within-band fractions are
    running counters. That matters more here than it usually would, because the
    outliers are the point -- satellite runs reach 2.06e93 m, and a sampled
    maximum could simply miss the row that proves it.

    Only the percentiles come from the sample, and only once past the cap, where
    a uniform reservoir puts them within a fraction of a percent. The RNG is
    seeded so two runs over the same data report the same figures.
    """

    def __init__(self, bands=(5.0, 10.0, 20.0), max_samples: int = 200_000):
        self.bands = bands
        self.max_samples = max(1, int(max_samples))
        self.errors: list = []
        self.losses: list = []
        self.sigmas: list = []
        self.latencies: list = []
        self.n_fix = 0
        self.n_accepted = 0
        self.n_scored = 0
        self.n_unpaired = 0
        self.n_nonfinite = 0
        self.max_error_m = None                      # exact, never sampled
        self.n_within = {float(b): 0 for b in bands}  # exact, never sampled
        self._seen: dict = {}                        # values offered per list
        self._rng = random.Random(0)

    def _sample(self, key: str, lst: list, value: float) -> None:
        """Keep an unbiased fixed-size sample of an unbounded stream.

        Below the cap this is a plain append and the list is the population.
        Above it, standard reservoir sampling: the nth value replaces a uniformly
        chosen slot with probability max_samples/n, which leaves every value ever
        offered equally likely to be present.
        """
        n = self._seen.get(key, 0) + 1
        self._seen[key] = n
        if len(lst) < self.max_samples:
            lst.append(value)
            return
        j = self._rng.randrange(n)
        if j < self.max_samples:
            lst[j] = value

    @property
    def sampled(self) -> bool:
        """True once any list has overflowed, i.e. percentiles are estimates."""
        return any(v > self.max_samples for v in self._seen.values())

    def add(self, *, accepted: bool, error_m=None, loss=None, sigma_m=None, latency_ms=None):
        self.n_fix += 1
        if accepted:
            self.n_accepted += 1
        if _finite(latency_ms):
            self._sample("latencies", self.latencies, float(latency_ms))
        if error_m is None:
            self.n_unpaired += 1
            return
        if not _finite(error_m):
            # Keep NaN and inf out of the distributions entirely rather than
            # filtering them at every read. A non-finite error is not a large
            # error -- it is the absence of one -- and letting it into the list
            # corrupts sorted(), max() and the within-band denominators in three
            # different ways. Counted so it is visible rather than discarded.
            self.n_nonfinite += 1
            return
        self.n_scored += 1
        e = float(error_m)
        self._sample("errors", self.errors, e)
        if self.max_error_m is None or e > self.max_error_m:
            self.max_error_m = e
        for b in self.n_within:
            if e <= b:
                self.n_within[b] += 1
        if _finite(loss):
            self._sample("losses", self.losses, float(loss))
        if _finite(sigma_m):
            self._sample("sigmas", self.sigmas, float(sigma_m))

    @property
    def error_rate(self):
        """Fraction of fixes that did NOT survive the rejection gate.

        Not an accuracy figure. It is the number that decides whether EKF3 sees
        enough fixes to stay out of dead reckoning, so it belongs next to the
        rate, not next to the error.
        """
        return None if not self.n_fix else 1.0 - self.n_accepted / self.n_fix

    def summary(self) -> dict:
        e = self.errors
        out = {
            "n_fixes": self.n_fix,
            "n_accepted": self.n_accepted,
            "n_scored": self.n_scored,
            "n_unpaired": self.n_unpaired,
            "n_nonfinite": self.n_nonfinite,
            "accept_rate": round(self.n_accepted / self.n_fix, 4) if self.n_fix else None,
            "error_rate": round(self.error_rate, 4) if self.error_rate is not None else None,
            "median_error_m": _round(_percentile(e, 0.50)),
            "p90_error_m": _round(_percentile(e, 0.90)),
            "p99_error_m": _round(_percentile(e, 0.99)),
            "max_error_m": _round(self.max_error_m),
            "median_loss": _round(_percentile(self.losses, 0.50), 4),
            "median_sigma_m": _round(_percentile(self.sigmas, 0.50)),
            "median_latency_ms": _round(_percentile(self.latencies, 0.50), 1),
            "p95_latency_ms": _round(_percentile(self.latencies, 0.95), 1),
            "note": "no mean or RMSE by design -- a single degenerate solve destroys both",
        }
        if self.sampled:
            out["percentiles_sampled"] = {
                "note": "percentiles estimated from a uniform reservoir; counts, "
                        "max_error_m and the within-band fractions are exact",
                "max_samples": self.max_samples,
                "offered": dict(self._seen),
            }
        for b in self.bands:
            key = f"within_{int(b)}m"
            # From the exact counter, not the sample: this is the number the
            # accuracy claim rests on.
            out[key] = (round(self.n_within[float(b)] / self.n_scored, 4)
                        if self.n_scored else None)
        return out


def _finite(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def _round(v, nd: int = 3):
    return None if v is None else round(float(v), nd)
