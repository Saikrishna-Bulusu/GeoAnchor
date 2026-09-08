"""Pace the feed to whatever the processing layer can actually sustain.

WHY THIS EXISTS
---------------
`feed.fps` was a hand-tuned constant, and it had to be re-measured on every
board because the right value depends entirely on how long that board takes to
match a frame. The Xavier wanted 2, a board that matches in 250 ms wants 4, and
inheriting the wrong one silently is exactly the failure the portability rule
exists to prevent. Two boards was already one too many to keep in sync.

WHAT THE MEASUREMENTS SAY (Xavier, 8 Sept 2026, ~410 ms per fix)
----------------------------------------------------------------
    fps   fixes/s   latency med   staleness
      2      2.01         390.6        12.1
      4      2.60         506.5       136.0
      8      2.49         462.7        66.5
     16      2.23         482.0        45.3
     30      1.87         712.5       190.2

The shape is not monotonic and the reason matters. `drain(keep_latest_of=
[T_FRAME])` conflates, so a frame only ever costs latency by SITTING in the
queue. Publish slower than the consumer and the consumer blocks on an empty
queue: every frame is picked up the moment it lands, and staleness collapses to
the transport cost. Publish faster and the consumer never waits, but the frame
does -- by the mean gap between publishes. Publish much faster and staleness
starts falling again (a fuller queue means a fresher newest), right up until
the producer's own encoding starts stealing cores from the matcher, at which
point everything gets worse at once.

So the target is the left-hand side: **the consumer should wait on the queue,
never the frame in it.** Publish one frame every (service time x margin), and
the queue is empty when the consumer comes back for it.

WHY IT PACES OFF stage_ms AND NOT THE FIX INTERVAL
--------------------------------------------------
The obvious signal -- how often fixes come back -- is poisoned. The fix
interval is partly set by how fast we publish, so pacing off it is a loop that
feeds its own output back in: publish slower, fixes arrive slower, conclude the
board got slower, publish slower still. It converges on the floor.

`sum(stage_ms)` is the consumer's intrinsic cost: decode, rectify, detect,
match, ransac. It does not move when we change the publish rate (379-388 ms
across the 2, 4 and 8 fps arms above), which is exactly the property a control
input needs. That is why the previous commit's decision to persist stage_ms
mattered for more than post-hoc analysis.
"""
from __future__ import annotations

import statistics
from collections import deque

# stage_ms carries one entry that is a COUNT, not a duration. Summing it in
# would add ~10 to every service-time estimate and slow the feed for no reason.
NOT_A_DURATION = frozenset({"tiles_fitted"})


class AdaptivePacer:
    """Turns observed fix cost into a publish rate. Pure: no I/O, no clock."""

    def __init__(self, margin: float = 1.15, bounds: tuple = (0.5, 10.0),
                 window: int = 15, warmup: int = 3, hysteresis: float = 0.10):
        self.margin = max(1.0, float(margin))
        self.lo, self.hi = float(bounds[0]), float(bounds[1])
        self.window = int(window)
        # The first fix on a cold process pays the model load -- 3.2 s on the
        # Xavier, against a steady state of 0.41 s. Pacing off that would idle
        # the feed at the floor for the whole window it takes to age out.
        self.warmup = int(warmup)
        self.hysteresis = float(hysteresis)

        self._samples: deque = deque(maxlen=self.window)
        self._seen = 0
        self.fps: float | None = None
        self.service_ms: float | None = None

    def observe(self, stage_ms: dict | None) -> bool:
        """Feed one fix's stage breakdown in. True if the rate changed."""
        if not stage_ms:
            return False
        total = sum(v for k, v in stage_ms.items()
                    if k not in NOT_A_DURATION and isinstance(v, (int, float)))
        if total <= 0:
            return False
        self._seen += 1
        if self._seen <= self.warmup:
            return False
        self._samples.append(total)
        if len(self._samples) < 3:
            return False

        # Median, not mean. A single 7 s xfeat_lg failure or one cold tile load
        # should not reset the feed rate for the next fifteen frames.
        self.service_ms = statistics.median(self._samples)
        target = 1000.0 / (self.service_ms * self.margin)
        target = min(self.hi, max(self.lo, target))

        if self.fps is not None and abs(target - self.fps) <= self.hysteresis * self.fps:
            return False
        self.fps = target
        return True

    def describe(self) -> dict:
        return {"mode": "auto", "fps": round(self.fps, 2) if self.fps else None,
                "service_ms": round(self.service_ms, 1) if self.service_ms else None,
                "samples": len(self._samples), "margin": self.margin,
                "bounds": [self.lo, self.hi]}
