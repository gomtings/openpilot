# Experimental Ford path prediction, v3

The latest driven route9e used v1 (`5fc16abc7`), before the v2 yaw damping.
It often follows the requested steering angle closely, but some tight turns
fall behind after a reasonable initial turn-in. The requested angle is replanned
from the car's changing position; a large late request may partly be a recovery
request after arriving wide. It is not proof that the original turn required
that much steering. The generic PSCM limit flag does not identify a torque,
rate or mechanical limit, and the miss starts before our C1 cap in the clearest
left turn.

Short measured-motion integrations against earlier frozen model paths are
consistent with a growing miss, but model uncertainty, reference timing and
some driver input prevent a conclusive causal attribution. V3 tests a bounded
change to initial path demand. No counterfactual physical tracking score is
claimed from replaying fixed logs.

## Change and bounds

Start with the current model's lateral offset at 7 m of path arc length, as in
v1/v2. Advance the reference pose by speed × 0.15 s along the **selected,
upstream-limited curvature**, and read the same model path 7 m beyond that
advance, expressed in the predicted ego frame. Bound the change from the
original offset to both ±0.15 m and ±25% of its magnitude. Prediction cannot
reverse that target or create C0 from a zero offset.

For advance `d`, selected curvature `k`, rotation `theta = k*d`, and model
point `(x, y)` at arc station `7+d`, the predicted lateral coordinate is:

```
y_predicted = cos(theta)*y - sin(theta)*x + (1-cos(theta))/k
```

The code evaluates the last term continuously at zero curvature without
cancellation. Available path horizon limits `d`; prediction tapers to zero as
the horizon approaches 7 m. Nonfinite prediction falls back to the validated
current offset. The existing endpoint hold remains for paths shorter than 7 m.

A matched constant-radius path retains essentially the same C0, subject to
sample interpolation. Developing and flattening bends can move the target
earlier. This is a geometric hypothesis assuming motion along selected
curvature, not an identified 150 ms actuator delay or a calibrated plant model.
Errors in that assumption can increase or reduce useful steering demand.

The predicted offset passes through the existing ±5.11 m clip, v2 excess-yaw
damping, independent 4 m/s slew and 0.01 m quantization. C1 construction, clip,
slew and quantization are unchanged. C2=C3=0. Input sanity, freshness, service
health and startup selection gates are unchanged. There are still only two
control states (C0 and C1 slew positions), plus three adapter timestamps.
The module is 193 total lines, including 125 code lines excluding blanks,
comments and docstrings (18 more code lines than v2). No model history, integral
or turn state machine is added. The bounds above
apply to the prediction target, not arbitrary differences between separately
slewed controllers after different histories.

## Offline results and tradeoff

All four supplied routes run at their original controls timestamps. Routes90/95
also reproduce the archived v1 command construction exactly. The newer routes
compare immutable v2 code with v3 using identical measured yaw, selected
curvature, exact consumed model and causal carState. Publication times proxy
computation time; complete SubMaster health is unavailable. Neither v2 nor v3
was driven on these recordings. V2-versus-recorded error is therefore not a
reconstruction accuracy measurement.

| Recorded interval | Command change versus v2 |
| --- | --- |
| route9e left entry, 173–175.4 s | Mean C0 magnitude +0.143 m; same 2.0 m level reached 0.203 s earlier |
| route9e left peak, 175.4–178.3 s | Mean magnitude +0.095 m; prediction also increases some late demand |
| route9e reversal, 728–734 s | Peak C0 magnitude 0.24 → 0.22 m |
| route9b right exit, 642.7–643.852 s | Mean C0 +0.024 m, partially offsetting v2 damping |
| Driver-clean low requests above 8 m/s, routes9b/9e | Mean absolute C0 change ≈0.0015 m; maximum 0.02 m |

The entry C0 crossings at 0.5, 1.0, 1.5, 2.0 and 2.4 m move earlier by 61, 64,
367, 203 and 90 ms respectively. These are command-level crossing times,
not measured improvements in wheel response. Several entry/peak windows
contain driver input, quantified in the validation record.

On all 115 earlier right-exit cycles before strong intervention, v3 remains
below the driven v1 reconstruction: mean C0 is 0.370 m for v1, 0.290 m for v2,
and 0.314 m for v3. This tradeoff is retained explicitly; v3 does not improve
every exit command relative to v2. There is no evidence yet that it reduces
the late model request or the physical miss.

Route9b now includes full rlogs 12/13, added after the archived v2 evaluation.
Its 14-rlog totals therefore differ from the historical 12-rlog report. The
focused segment-10 comparison uses identical timestamps and data.

Validation passes 372 tests and 26 subtests, including the real extracted
controlsd selection/limiter/publication path and downstream CAN builder, with
100% controller statement and branch coverage. Four routes cover 299,604
original cycles. Randomized testing adds 200,000 core updates plus their
mirrors against an independent analytic geometry/damping/slew oracle; boundary
and route checks total 817,346 Float32/CAN round trips. These checks establish
command construction and retained gates, not closed-loop vehicle behavior.
See `ford_model_action_prediction_validation.json` for provenance and counts.

## Reproduce and select

Use the native dependencies and suite command in the
[drive-test guide](ford_model_action_drive_test.md). New-route comparisons use
the deployment opendbc pin `c21a9013700734dd20b09e05aa68329ad8cc20f9`:

```sh
python -m tools.ford_pscm_lab.damping_replay /path/to/route9e/rlogs --baseline v2 --candidate current --window left_entry 173 175.4 --window left_peak 175.4 178.3 --window left_exit 178.3 180.5 --window reversal 728 734 --window final_entry 822 825.5 --output /path/to/separate/route9e-results
python -m tools.ford_pscm_lab.damping_replay /path/to/route9b/rlogs --baseline v2 --candidate current --window right_entry 637 640 --window right_exit_before_strong_input 642.7 643.852 --output /path/to/separate/route9b-results
python -m tools.ford_pscm_lab.stress_model_action --cycles 200000 --seed 20260907 --opendbc-revision c21a9013700734dd20b09e05aa68329ad8cc20f9 --output /path/to/stress.json
```

Historical replay loads trusted controller source from immutable local Git
commits; those objects must exist in the checkout. The original replay tool
still requires its explicit historical opendbc pin. Neither tool downloads
code or drives the car.

The existing default-off Sunnylink **Selected-Action Path Tracking
(Experimental)** toggle selects v3 on the CAN FD F-150 Lightning. Updating
with that toggle already enabled selects v3 at the next controlsd startup.
Diagnostics identify `model-action-c0-c1-prediction-v3` and keep
`calibration_approved=false`. No new device installation, hardware build,
physical calibration or device boot is part of this offline validation.
