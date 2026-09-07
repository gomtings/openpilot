# Experimental Ford offset damping, v2

This document and its validation counts describe the archived v2 source. The
[current v4 experiment](ford_model_action_full_prediction.md) uses full path prediction.

Segment 10 of the supplied route9b recording shows measured turning persisting
as requested right curvature falls. At about 643.0 s, before strong driver
intervention, device-gyro curvature is approximately 0.01786/m against a
0.01172/m request. Around 643.9 s, heading demand has reversed slightly but
C0 still requests approximately +0.12 m into the turn. Strong column input
starts around 643.852 s; later motion cannot establish autonomous recovery.
Earlier light driver input also exists.

The outgoing CAN commands match preceding publications. All 70,937 decoded
frames have zero C2/C3 and valid checksums. Focused exit diagnostics have fresh
model/carState inputs and targets within normal quantization of the outputs.
This supports trying less residual C0; it does not identify PSCM dynamics or
prove C0 alone caused the physical oversteer.

## Change

C0 starts from the clipped current model offset at 7 m. When C0 and measured
host yaw point in the same direction, compute:

```
requested_yaw = max(0, sign(C0) * speed * desiredCurvature)
excess = max(0, sign(C0) * yaw - requested_yaw - 0.02 rad/s)
reduction = 7 m * 0.2 s * excess
target = sign(C0) * max(0, abs(C0) - reduction)
```

Opposing centering demand is unchanged. The correction cannot increase the
target's magnitude or reverse its sign. Opposite-direction planned curvature
cannot amplify a small yaw bias into a correction. Existing 4 m/s C0 slew still
applies; this target bound is not a claim that every stateful output is smaller than a
separate v1 controller after arbitrary direction reversals. C1 construction,
clipping and slew are unchanged; C2=C3=0. Only C0/C1 slew states persist.
There is no integral, model-history filter, turn state machine or fitted plant.

Host yaw is `-carState.yawRate`, as in the existing Ford call path. The
0.02 rad/s deadband exceeds the approximately 0.008 rad/s offset measured
against the device gyro on quiet straights. The 0.2 s scale is an initial
engineering choice, not an identified delay or gain. Both remain physically
unvalidated. Large biased or noisy yaw within the existing sanity gate can
still attenuate useful centering; fixed-input replay cannot establish stability.

## Offline evidence

The complete 12-rlog route is replayed at original controls publication times,
with exact consumed model geometry, causal carState, and carControl matched
within 5 ms. These times proxy computation; full SubMaster health is unavailable.
V1 reconstruction is within one field quantum of all 64,701 paired active
publications. V2 has identical eligibility and exactly identical C1.

On 381.78 seconds of driver-clean low requests above 8 m/s, only 3 of 37,937 cycles
change C0, each by one 0.01 m quantum. Across the 642.7–643.852 s exit window,
C0 changes on all 115 cycles, averaging 0.080 m reduction. Entry/peak C0
maximum stays 2.80 m; some entry-window samples decrease by up to 0.05 m.
These are command comparisons on recorded inputs, not predicted tracking.
Low request means requested lateral acceleration below 0.15 m/s²; it is a
proxy for straight driving and does not establish a physically straight path.

The original routes90/95 also run through v2. Their zero-yaw baseline pass
checks archived v1 compatibility; the measured-yaw adapter pass checks current
construction, eligibility, field limits and packing. It is not an exact match
to v1 or v8. Randomized testing checks the damping against an independent
piecewise oracle, mirror symmetry, resets and slew, with real Float32/CAN
round trips. See `ford_model_action_damping_validation.json` for counts and hashes.

Final validation passes 325 tests and 26 subtests, with 100% controller
statement/branch coverage, 204,946 original route cycles and 628,030 CAN round
trips. The module is 166 total lines, including 107 code lines excluding
comments, blanks and docstrings. Standards and Spec reviews have no remaining findings.

## Reproduce

Use the dependency setup and suite command in the [drive-test guide](ford_model_action_drive_test.md).
The new route replay requires the deployment opendbc pin recorded there:

```sh
python -m tools.ford_pscm_lab.damping_replay /path/to/complete/rlogs --baseline v1 --candidate v2 --window segment10_entry_peak 637 640 --window segment10_exit_before_strong_input 642.7 643.852 --output /path/to/separate/results
python -m tools.ford_pscm_lab.stress_model_action --cycles 200000 --seed 20260907 --opendbc-revision c21a9013700734dd20b09e05aa68329ad8cc20f9 --output /path/to/stress.json
```

At the v2 revision, the same default-off Sunnylink toggle selects v2; no additional setting is
introduced. Updating an installation with the toggle already enabled selects
v2 at the next controlsd startup. `calibration_approved=false` remains explicit.
No physical fix, hardware build or device boot is established by these checks.
