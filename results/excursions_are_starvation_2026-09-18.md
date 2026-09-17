# The excursions are starvation, not poisoning. The hypothesis was wrong.

18 Sept 2026. `results/sim_gnss_denied_2026-09-18.md` proposed that the
excursions in the GNSS-denied runs were **a wrong fix passing the inlier gate
and being fused with an 8 m sigma that said it was trustworthy** — and called
that "the strongest evidence yet for the rejection half of THE FINDING",
flagged as not proven. It is now measured, and it is **not what happens.**

Reproduce (300 s of denial, with the fix stream tapped):

```bash
.venv/bin/python scripts/sim_gnss_denied.py --baseline 30 --seconds 300
```

`scripts/sim_gnss_denied.py` now subscribes to the processing layer's fix
stream and scores **every** fix against Gazebo truth on the same clock as the
estimator samples, writing `results/sim_gnss_fixes_*.csv`. The pipeline's own
`error_m` could not answer this: under denial it is computed against MAVLink,
which is the estimate the fix is driving, so a fix that drags the estimate
500 m looks like a fix that agrees with the estimate perfectly.

## What the fixes actually looked like

**953 accepted fixes over 300 s. Median 3.4 m from truth. Worst 24.0 m.**

Not one bad fix reached the estimator. Inside the excursion windows the
accepted fixes were median 7.7 m and 8.5 m, worst 14.9 m. There is no poisoning
to find.

**And the gate was not the thing rejecting, either.** Of the fixes the
processing layer emitted, 87–100% were accepted in every window:

| window | fixes emitted | accepted | inliers (median) |
|---|---|---|---|
| starved, 240–300 s | **15** | 13 (87%) | 33 |
| healthy, 180–210 s | **172** | 172 (100%) | 43 |
| healthy, 90–150 s | 240 | 237 (99%) | 57 |
| starved, 30–90 s | 46 | 45 (98%) | 35 |

The starved window emitted **15 fixes in 60 seconds** where a healthy one
emitted **172 in 30**. That is a ~23x collapse in throughput with the
acceptance *ratio* essentially unchanged.

## What actually happens

The estimator is **starved**, and it dead-reckons through the gap.

Ten gaps between accepted fixes exceeded AGP's 5 s timeout. The largest:

    gap  53.6 s : error  12.7 ->  117.6 m   (+117.3 m/min)
    gap  23.6 s : error   2.4 ->   12.9 m   ( +26.7 m/min)
    gap  21.6 s : error   3.7 ->   25.9 m   ( +61.7 m/min)
    gap  14.1 s : error  10.1 ->   26.3 m   ( +68.9 m/min)

**+117.3 m/min against the control's measured +119.91 m/min** — the
no-vision dead-reckoning rate from the same rig, to within 2%. The estimator is
not being pulled anywhere. It is coasting, at exactly the rate coasting
produces, because nothing is arriving to correct it.

Binned over the run, the relationship is monotonic and unambiguous:

    bin (s)    accepted    Hz     median err    max err
      0- 30         56    1.87        1.4 m      27.4 m
     90-120        129    4.30        4.9        8.3
    120-150        108    3.60        5.5        8.1
    180-210        172    5.73        6.7       14.9
    240-270          1    0.03       26.6       51.9
    270-300         12    0.40       75.5      117.8

Above ~3 Hz the error sits at 5–7 m. At 0.03 Hz it is 26 m and climbing.

## What this changes

**It refutes the hypothesis and strengthens the finding it was meant to
support.** `CLAUDE.md`'s claim is that an inlier threshold is the best rejector
available. Here it — and the plausibility check ahead of it — kept every bad
fix out of the estimator across 300 s of GNSS denial, with the worst survivor
24 m out. That is the rejection machinery working, not failing.

**The binding constraint is availability, not correctness.** Every previous
framing in this project treats a fix as a thing to be judged. Under denial the
question is instead *how long since the last one*, measured against AGP's 5 s
timeout (and ArduPilot's 7 s `posTimeout`). A pipeline that emits a perfect fix
every 20 seconds is useless here and a pipeline that emits an 8 m fix every
second is fine.

**So the thing to optimise is the rate of usable fixes, not their accuracy.**
That reframes the top_k work: the point of bringing a matcher inside the
latency budget is not a better median, it is closing the gaps.

## What is NOT established

- **Why throughput collapses is indicated, not isolated.** `PLE-08`
  ("solution implausible — outside the map, or a degenerate solve") is the
  dominant code in the run at 109 occurrences, rejecting on shear (0.289
  against a 0.25 limit) and scale (0.592, more than 0.35 from 1.0), and both
  are the geometric signature of an obliquely-viewed frame. That is consistent
  with the bank-angle finding in `results/sim_fixedwing_2026-09-17.md`. But the
  log rows are **throttled**, so their per-window counts are emission counts
  rather than event rates and cannot carry the claim. Isolating it needs
  per-frame reject reasons on the same clock.
- **The starved arcs are on one side of the orbit** — both at roughly N−450
  E−225 from the map centre, both healthy windows elsewhere — which points at
  land cover as well as geometry, per the 30-vs-96-inlier result in the same
  document. Two candidate causes, not separated.
- **One run.** The earlier two runs produced single excursions of 4.9 s and
  13.8 s; this one produced a 53.6 s gap. The mechanism is consistent across
  all three, the magnitudes are not.

## A note on the arithmetic that produced the wrong hypothesis

The original claim reasoned: *542 m in 13.8 s is impossible by dead reckoning
at 120 m/min (which gives 28 m), therefore the estimate was pulled.* The
premise is right and the conclusion did not follow — it assumed the only two
options were "coasting" and "poisoned". This run shows a third: the gap can be
far longer than the excursion window measured at a 5x-median threshold, because
the error is already large when the window is entered and the window ends when
a fix finally lands. The 53.6 s gap here produced a 43.8 s window at that
threshold.

**The lesson is the one the rest of this project keeps relearning: measure the
input, not just the output.** The fix stream was never being recorded, so every
statement about what the estimator was being fed was an inference.
