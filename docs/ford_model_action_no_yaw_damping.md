# Experimental Ford selected-action controller, v5

V5 removes the excess-yaw C0 attenuation at the user's request. The damping
function, 0.02 rad/s deadband and 0.2 s reduction scale are deleted. Valid measured
yaw no longer changes either command target. Existing yaw input-health checks
and logging remain. No replacement gain, increment gate or controller state is added.

The full 150 ms geometric prediction introduced in v4 remains, along with the
7 m offset station and one-second heading scale. This is not a return to v1:
v1 did not predict the offset. C0/C1 bounds, slew, quantization, service gates,
engagement, downstream driver arbitration and zero C2/C3 are unchanged.

The latest supplied route a0 ran v2, not v4. Prior same-input comparisons found
identical v1/v2 commands during its driver-clean minor-bend warning intervals
and the preceding five seconds. That evidence does not identify damping as the
cause of those misses. Removing damping can restore C0 demand where the damper
was active, including turn exits; it is not evidence of improved tracking or
reduced oversteer.

## Offline validation

The regression suite checks yaw-independent commands through mirrored turn
entry, release and reversal, including valid yaw extremes and small yaw offsets.
Six cases fail with v4 damping present and pass after removal. Invalid yaw still
resets the controller. Actual controlsd selection, upstream limiting, Float32
publication and downstream CAN tests cover both model and maneuver references.

Full-rlog comparisons run pinned v4 against production v5 on routes9b, 9e and a0.
They preserve original clocks, exact consumed model frames and causal carState;
publication times proxy computation time, and complete SubMaster health is
unavailable. The numerical stress run checks independent geometric targets,
scalar slew, mirrored turns and Float32/CAN packing. Recorded vehicle motion
stays fixed: none of these checks establishes counterfactual steering response,
closed-loop stability or a physical tracking improvement.

Results and source hashes are recorded in
`ford_model_action_no_yaw_damping_validation.json`. Earlier validation documents
remain archives of their specified controller versions.

Validation passes 356 tests and 26 subtests with 100% controller statement and
branch coverage, 280,636 recorded route cycles and 779,410 Float32/CAN round trips,
including 200,000 random stress cycles. C1 and input eligibility match v4 exactly
on all three routes. Commands during all 1,578 driver-clean ordinary-bend warning
cycles on route a0 also remain identical to v4. At the earlier right-turn exit,
removing damping increases C0 magnitude by a mean 0.079 m, maximum 0.13 m; these
are command offsets, not measured vehicle displacement.

The module is 171 total lines, or 111 code lines excluding blanks, comments and
docstrings, with two control states. No hardware build or device boot was performed.

## Reproduce and select

Use the dependency setup and combined suite in the
[drive-test guide](ford_model_action_drive_test.md). Replay and stress commands:

```sh
python -m tools.ford_pscm_lab.damping_replay /path/to/route9b/rlogs --baseline v4 --candidate current --window right_entry 637 640 --window right_exit 642.7 643.852 --output /path/to/separate/route9b-results
python -m tools.ford_pscm_lab.damping_replay /path/to/route9e/rlogs --baseline v4 --candidate current --window left_entry 173 175.4 --window left_peak 175.4 178.3 --window left_exit 178.3 180.5 --output /path/to/separate/route9e-results
python -m tools.ford_pscm_lab.damping_replay /path/to/routea0/rlogs --baseline v4 --candidate current --window bends_5min 298 338 --window bend_7min 449 458 --window bend_9min 579 588 --output /path/to/separate/routea0-results
python -m tools.ford_pscm_lab.stress_model_action --cycles 200000 --seed 20260908 --opendbc-revision c21a9013700734dd20b09e05aa68329ad8cc20f9 --output /path/to/stress.json
```

The same default-off **Selected-Action Path Tracking (Experimental)** Sunnylink
toggle selects v5 on the CAN FD F-150 Lightning. Deployment remains
`sunnypilot/sunnypilot`, branch `hiimisaac-dev`. After updating, restart controlsd
through a real offroad-to-onroad cycle. Diagnostics identify
`model-action-c0-c1-prediction-v5`; `calibration_approved=false` remains explicit.
