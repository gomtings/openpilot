# Experimental Ford full path prediction, v4

This document archives v4. The [current v5 controller](ford_model_action_no_yaw_damping.md)
retains this prediction and removes yaw damping.

V4 removes the extra 15 cm / 25% limit on the geometric prediction introduced
in [v3](ford_model_action_prediction.md). Those numbers were hand-chosen tuning
bounds, not identified Ford response limits. The current user request is to
remove that restriction; the existing default-off Sunnylink toggle remains.

The controller now uses the full predicted offset from the same model path,
assuming 150 ms of motion along the selected, upstream-limited curvature. The
150 ms horizon remains an engineering assumption. Available model geometry
still limits the prediction distance, with endpoint hold for short paths and
fallback to the valid base offset if prediction arithmetic is nonfinite.

The existing total command limits (C0 ±5.11 m, C1 ±0.5 rad), independent slew
rates (4 m/s, 0.5 rad/s), quantization, yaw damping, input/service gates and
zero C2/C3 remain unchanged. Only two control states persist. No integrator,
model history, extra toggle or PSCM feedback loop is added.

Removing the adjustment cap also permits the predicted C0 to oppose the
original offset or become nonzero from a zero original offset. For example,
a straight path with a nonzero selected turn request can have an opposing
future-frame offset. Tests cover that behavior, mirrored turn releases,
return to zero, and unchanged slew; sign preservation of the original C0 is
no longer claimed. The yaw damper still cannot reverse its input target.

## Evidence and interpretation

The PSCM reports a generic `LimitReached` state. It does not tell us whether
an incoming target is geometrically correct or well timed. Its internal
limits cannot establish the tracking performance of this predictor. Removal
is an experiment supported by command comparisons, not by an assumption that
the PSCM will correct an excessive or mistimed request.

Compared with capped v3 on identical recorded inputs:

| Interval | Effect of removing the extra cap |
| --- | --- |
| Latest tight-left entry, 173–175.4 s | Mean C0 magnitude +0.032 m, maximum change 0.07 m |
| Earlier right entry, 637–640 s | Mean magnitude +0.028 m, maximum change 0.10 m |
| Earlier right exit, 642.7–643.852 s | All 115 commands unchanged |
| Driver-clean low requests above 8 m/s, routes9b/9e | Mean absolute change 0.0018 / 0.0014 m; maximum 0.02 m |
| All eligible samples on either recent route | Maximum absolute command change 0.13 m |

C1 and command eligibility are exactly identical to v3 on both recent routes.
Some command signs change near zero: this is an intended consequence of using
the full transform, not proof those corrections improve driving. Entry windows
include driver input, reported in the validation record. The magnitude changes
above describe controller C0, not measured lateral vehicle displacement.

The earlier v1 archive is also reproduced exactly on routes90/95, separately
from the current controller pass. All replay uses original timestamps;
publication times proxy computation, exact consumed model frames and causal
carState are retained, and complete SubMaster health is unavailable. The
recorded model and vehicle motion remain fixed. There is no measured physical
improvement, stability result or new desired-versus-actual steering trajectory.

Final validation passes 374 tests and 26 subtests with 100% controller statement
and branch coverage, 299,604 original route cycles and 817,346 Float32/CAN
round trips. The controller is 190 total lines / 122 code lines excluding
blanks, comments and docstrings; two control states persist.

See `ford_model_action_full_prediction_validation.json` for final test counts,
coverage, dependency pins, source hashes and route/packing results. The initial
uncapped variant was evaluated in a separate lab file before editing production;
final route checks execute the production v4 source.

## Reproduce and select

Use the dependencies and combined suite command in the
[drive-test guide](ford_model_action_drive_test.md). For the recent routes:

```sh
python -m tools.ford_pscm_lab.damping_replay /path/to/route9e/rlogs --baseline v3 --candidate current --window left_entry 173 175.4 --window left_peak 175.4 178.3 --window left_exit 178.3 180.5 --window reversal 728 734 --output /path/to/separate/route9e-results
python -m tools.ford_pscm_lab.damping_replay /path/to/route9b/rlogs --baseline v3 --candidate current --window right_entry 637 640 --window right_exit 642.7 643.852 --output /path/to/separate/route9b-results
python -m tools.ford_pscm_lab.stress_model_action --cycles 200000 --seed 20260907 --opendbc-revision c21a9013700734dd20b09e05aa68329ad8cc20f9 --output /path/to/stress.json
```

The same **Selected-Action Path Tracking (Experimental)** toggle selects v4
on the CAN FD F-150 Lightning. An installation with the toggle already enabled
selects v4 after updating and restarting controlsd. Diagnostics identify
`model-action-c0-c1-prediction-v4`; `calibration_approved=false` remains explicit.
Deployment branch: `sunnypilot/sunnypilot`, `hiimisaac-dev`. This work does not
install software on the device or change its settings.
