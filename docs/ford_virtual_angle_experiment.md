# Ford C2-free model-pose tracking with measured feedback

Hypothesis `model-pose-c0-c1-feedback-v8` retains the model-pose C0/C1 base
and adds two guarded release policies. When measured turning exceeds both
current and delayed requests, a separate output guard prevents same-direction
C0/C1 growth, including while feedback history rebuilds after driver input.
When turning instead falls below both requests and is no longer increasing,
bounded C1 tracking can use remaining release-entry command headroom.
Existing opposing-bias recovery still stops at zero bias. Geometry, blending,
feedback gain, slew rates and field limits are unchanged; C2/C3 remain zero.

This is an experimental outer controller around the multivariable PSCM.
Its geometry does not define a calibrated C0/C1-to-wheel mapping or an angle
servo. V8 has offline validation only. Command replay cannot establish the
truck's response, closed-loop stability, or an overshoot improvement.

## Evidence and scope

Route80 ran v3 and contains both sustained under-response and over-response.
Representative eligible windows had median CAN response/request ratios of
0.78, 1.77 and 0.69 with a declared 0.2-second comparison interval. These
are descriptive tracking ratios, not identified controller gains.

V4 replaced separate model-heading C1 with selected-curvature C1 and reduced
heading demand in several large maneuvers. The user subsequently reported
weak turning and steering repeatedly stopping near 85 degrees. Older logs
contain larger wheel angles; the inspected host code has no fixed 85-degree
wheel stop, although upstream curvature limits depend on speed.

Route83 had the Sunnylink toggle on, but omitted EPS firmware responses.
The former firmware gate selected the default `FordPathController`; replay
reproduced its recorded C0/C1/C2 requests. Its favorable turns are evidence
for the existing model-pose construction, not validation of v5 or v6.
V6 reuses that construction while replacing its remaining C2 request with
C0/C1 geometry. Removing C2 changes the request received by the PSCM, so
matching large C0/C1 commands does not guarantee matching vehicle motion.

Route8a ran v6 and was reported as the best drive. Route8e ran v7 throughout
with the experiment enabled; it includes entry lag and excessive turning
while requests release. Fixed-input v6/v7 replay produced identical commands
in the main reversal and over-response examples, so the v7 recovery change
does not directly explain their command behavior. In the over-response
example, model C0/C1 grew while selected curvature fell and driver resets
repeatedly removed feedback history. Another exit remained deficient after
opposing bias reached zero. These observations motivate the v8 guards; they
do not isolate an EPS transfer function or demonstrate the proposed response.

## Base request

controlsd selects valid `lateralManeuverPlan.desiredCurvature`, otherwise
`modelV2.action.desiredCurvature`, after the existing curvature limiter.
This action already includes upstream delay handling; it receives no extra
response advance here.

The model contribution uses the existing allocator's raw forward pose and
bounded short-pose correction. `_model_pose` advances 0.1 seconds, retains
the model's remaining forward geometry, and separately corrects the short
pose using measured curvature and its recent change. Its offset preview is
up to 7 m and its heading preview is up to max(7 m, speed × 1 s), bounded by
available path length. This raw pose is not passed through a second model
filter. The filtered, ego-aligned reference remains available for comparison
and the existing geometry-validity checks.

```text
share(k) = clip((k - 0.006/m) / (0.012/m - 0.006/m), 0, 1)
aligned = desired_curvature × model_forward_heading > 0
model_share = min(share(abs(desired_curvature)), share(model_curvature_demand))
              if aligned, otherwise 0
model_pair = existing_pose_encoder(model_pose, model_share, C2=0)

remaining_curvature = desired_curvature × (1 - model_share)
L0 = max(8 m, speed × 1 s)
L1 = max(7 m, speed × 1 s)
curvature_C0 = 0.5 × remaining_curvature × L0²
curvature_C1 = remaining_curvature × L1
C0_base = clip(model_pair.C0 + curvature_C0, ±5.11 m)
C1_base = clip(model_pair.C1 + curvature_C1, ±0.5 rad)
```

`model_curvature_demand` is the larger absolute curvature implied by the
forward offset and heading previews. The share uses the existing allocator's
0.006–0.012/m thresholds. Both model and action must request a substantial
turn in the same direction before model pose supplies the full base.
Small, flat, opposed or zero requests use the curvature contribution; zero
action produces a zero base. Partial shares combine both contributions.
The existing pose encoder retains its quantization and field-allocation rules.
The residual-curvature lift is geometric, not a claim of EPS equivalence to C2.

The inherited pose encoder allocates heading overflow using its asymmetric
limits (+0.5235/−0.5 rad), before the symmetric final ±0.5 rad
heading bound. On clipped tails, this can leave mirrored C0 requests differing
by up to 0.0235 rad × 7 m = 0.1645 m. The favorable comparison anchors lie
below that heading cap; full model-base odd symmetry is not claimed.

## Measured feedback and limits

```text
past_request = selected curvature held at or before (measurement_time - delay)
yaw_error = measured_speed × past_request - measured_yaw_rate
bias_trial = released_bias + feedback_gain × yaw_error × measurement_dt
C1_unconstrained = clip(C1_base + accepted_bias, ±0.5 rad)
C1_target = temporary_backoff_ceiling(C1_unconstrained) if backoff_active
            otherwise C1_unconstrained
```

Measured yaw is negated Ford CAN yaw, matching the control sign convention.
The historical request uses zero-order hold; it never interpolates toward a
future publication. Nominal comparison delay is `CP.steerActuatorDelay`
(0.2 seconds on the source vehicle). Feedback compares against selected
curvature, not curvature inferred from the model-pose coefficients.

| Quantity | Value |
|---|---:|
| C0 / C1 final bounds | ±5.11 m / ±0.5 rad |
| Independent C0 / C1 slew | 4 m/s / 0.5 rad/s |
| Feedback integration scale | 1.0 |
| Feedback minimum speed | 2 m/s |
| Maximum PSCM/core input age | 150 ms |
| Allowed timestamp lead | 5 ms |
| Release comparison tolerance | one C1 wire quantum, 0.0005 rad |

The integration scale, preview distances and blend thresholds are effective
gains; none establishes stability. No wheel-response gain is fitted.
Zero yaw error retains acquired bias while an eligible turn continues.
Host anti-windup admits reachable correction within the combined C1 field
and slew limits. Feedback overflow is not transferred into C0.

The release logic scales bias as the bounded base decreases and resets on
zero/reversal. When delayed curvature still represents a stronger or opposing
request, or PSCM reports LimitReached, new integration is normally frozen.
One exception permits measured-error backoff: measured turning must exceed
both the delayed and current selected yaw requests in the base's direction,
and total heading must still have the base's sign. Exceeding only an older,
smaller request during turn-in does not qualify. The accepted increment may
only reduce that existing total toward zero; it cannot grow the request or
carry it through zero. Existing host field and slew limits still apply.

The existing release-recovery exception requires fresh valid PSCM status with
limit below 2, retained bias opposing the base, and both current and delayed
requests aligned with that base. Measured turning must be below both requests
in their direction. It then uses the current yaw deficit × the existing
feedback gain × measurement interval to unwind only the opposing bias toward
zero. The increment is clipped so recovery cannot cross zero bias or create
demand beyond the existing base. Common host anti-windup still limits what
can be accepted. A separate release-tracking exception is described below;
other constrained cases remain frozen. PSCM limit 2 never permits either
request-increasing exception.
The no-new-bias restriction applies to `release_recovery`. It does not apply
to the separate bounded `release_tracking` branch. Once release ends,
ordinary eligible integration can add correction beyond the base as before;
its existing limits and guards are unchanged.

`release_recovery` and `feedback_recovery_active=true` indicate that the
recovery branch actually changed bias on that update. If host anti-windup
blocks the entire increment, the status remains `host_limit` and the flag is
false. Recovery is evaluated only on fresh measurements; the flag is false
on repeated-measurement updates and after reset.

Diagnostics distinguish `release_backoff` and `pscm_backoff`; a release takes
precedence when both conditions apply. While `feedback_backoff_active` is
true, total C1 is also capped at the preceding continuous heading request in
the current request direction and at zero in the opposite direction. This
ceiling affects the output only: it is not stored or projected into bias.
The measured-error increment can still update bias under the normal limits,
but a changing model base does not create persistent integral suppression.
The ceiling persists between repeated measurements; C1 cannot grow or reverse
while it applies. The next fresh measurement clears it unless backoff is
again warranted. It does not cap C0, and normal feedback has its own rules
outside backoff. Independent slew remains 0.5 rad/s for C1 and 4 m/s for C0.
Backoff still compares against the delayed reference, so response lag remains.
Reducing a request does not demonstrate that physical overshoot is resolved.

## V8 release guard and tracking

`ReleaseGuard` retains selected-request history independently of feedback
bias history. Driver-related feedback resets do not erase that reference,
but the guard still requires current fresh valid PSCM status, no current
driver override, and the existing input and speed eligibility. Invalid core
input or disengagement resets its history with the controller.

During release, measured yaw must exceed both the current and delay-matched
requests in the requested turn direction. Only then does the guard cap
same-direction C0/C1 growth at each preceding continuous request. Terms
already reducing the turn, including an opposing C0 centering offset, remain
available. The guard follows base allocation and C1 feedback, so changing
model geometry cannot bypass it. Its ceilings affect outputs, never stored
bias. No scalar-curvature cap replaces strong model geometry during turn-in
or undertracking. Existing independent slew and field limits still apply.

`release_tracking` addresses an eligible release deficit once bias is zero
or already in the base's direction. Both current and delayed requests must
align with that base, measured turning must be below both, and measured
curvature must not be rising in the turn direction across the response
interval by more than one C1 wire quantum after scaling by heading preview.
Fresh valid PSCM status with limit below 2 is required. The current yaw deficit
uses the existing integration gain and measurement interval;
new C1 tracking increments are limited by command headroom captured at
release entry, tapered with remaining desired curvature. The allowance is
`max(0, entry_command_magnitude - abs(base)) × min(1, abs(desired) / entry_reference)`
above the current base; any existing same-direction bias consumes it first.
This limits new tracking integration, not the existing model base or bias.
Only that additional allowance is tapered; strong model geometry remains
available. A brief pause does not reacquire a higher entry
ceiling; a full response interval without release ends the retained episode.
Common host anti-windup, field and slew bounds still apply. Opposing bias
continues through `release_recovery`, which stops at zero, before any separate
tracking exception can be considered.

Neither exception relaxes the PSCM LimitReached growth restriction. The
reference delay and finite response time remain; these output policies are
command-construction changes, not evidence of improved physical tracking.

## PSCM status and driver handling

card publishes `Lane_Assist_Data3_FD1` in `carStateSP.fordPscmStatus`, retaining
the original CAN receipt timestamp. Republishing carStateSP or receiving
unrelated frames cannot refresh it. The opendbc submodule is unchanged.

Feedback requires valid fresh status, InProgress lateral state (2), capability
LimitedModeAvailable or ExtendedModeAvailable (1 or 2), and no denial.
Missing, malformed, stale, backward-timestamped, denied or unavailable status
clears feedback bias/history and disables the separate release guard,
leaving the base subject to its core validity gates.
LimitReached (2) permits only the bounded request-reducing backoff described
above and otherwise freezes integration. LimitWithDriverActive (3) clears
feedback. Backoff still requires fresh, valid, InProgress status with an
available capability and no denial. These generic PSCM reports do not identify
a specific torque or rate limit.

`steeringPressed`, raw torque above the existing Ford driver allowance, or
nonfinite torque clear feedback. Below 2 m/s feedback also clears. A fresh
feedback reference interval is required after override; the independent
release guard can use retained valid request history once its current gates
are satisfied. Base requests retain normal
PSCM driver arbitration while lateral control remains authorized; an unset
override flag cannot rule out subthreshold driver influence.

## Gates and Sunnylink selection

Core model/action/car-state freshness, finite-value, clock and speed checks
remain in place. Invalid core inputs reset both commands and clear latActive.
Raw model geometry is validated on every update, including repeated model
timestamps; an invalid raw path cannot reuse the cached valid reference.
Missing PSCM status disables feedback, not an otherwise valid base request.

Vehicle → Ford → **C2-Free Path Tracking (Experimental)** retains the
`FordVirtualAngleController` key, default-off setting and offroad/onroad cycle
requirement. Enabled selects v8 on Ford CAN FD `FORD_F_150_LIGHTNING_MK1`
regardless of missing or different EPS firmware-query results. Other platforms
retain their existing controller. V8 takes priority over PSCM Coefficient
Observer while selected; disabling and cycling offroad/onroad restores the
previous selection. Controller selection does not force lateral engagement.

The analyzed firmware is `RL38-14D003-AA`; removing the eligibility check
is not validation of other firmware. No live device setting is changed.

## Diagnostics and verification

The 5 Hz `Ford C2-free path tracking` event keeps its name and identifies v8.
`model_offset_base` / `model_heading_base` report the already weighted and
encoded model contribution; `curvature_offset_base` / `curvature_heading_base`
report the residual-curvature contribution. `model_share` and `base_guard`
identify model-pose, blended, curvature-only, opposed-model and zero-request
cases. `heading_base` is the bounded pre-feedback C1. `offset_target` and
`heading_target` are the final targets after the independent release guard;
`offset_target_unguarded` and `heading_target_unguarded` retain the inputs to
that guard. The latter C1 already includes its normal feedback/backoff policy.

The event retains source timestamps, measured curvature/yaw, final commands,
slew scales, feedback bias/status/history, raw torque and PSCM status/age.
`feedback_backoff_active` records the persistent heading ceiling, including
cycles whose feedback status is `no_new_measurement`.
`release_guard_active` and `release_guard_reference_curvature` expose the
independent C0/C1 guard and its retained delayed reference.
`feedback_release_tracking_active`, `feedback_release_ceiling` and
`feedback_curvature_delta` identify accepted release
tracking, the total-heading threshold used to admit new bias, and the
measured-curvature change across the response interval (1/m). The tracking
flag is true only when the branch accepts a bias change on a new measurement;
it is false on repeated measurements. The ceiling/trend fields can describe
an evaluated condition even when no increment is accepted.
`feedback_recovery_active` records an accepted recovery increment on this
update only; it does not persist between measurements.
`feedback_yaw_error` retains its delayed-reference meaning. Recovery instead
uses current error, reconstructed from logged `desired_curvature`,
synchronized car-state speed and `yaw_rate`; those two errors can differ.
During backoff or the independent release guard, `heading_target` can be lower in the request direction than
the bounded sum of `heading_base` and `heading_bias`, because the temporary
ceiling is not part of the stored bias.
`model_heading_target` remains a filtered comparison reference; it is not the
weighted model contribution. `angleState.saturated` is not an EPS-limit signal.

Validation must cover large recorded maneuvers, flat-model centering, both
turn directions, model/action disagreement, share transitions, release and
reversal, release/limit backoff without growth or zero crossing, status/driver
resets, reference causality, bounds, slew and CAN packing with C2/C3 zero.
Recovery checks cover both directions, stopping at zero bias, repeated
measurements, current-and-delayed agreement, and rejection at PSCM limit 2.
Old v3/v4 command-equality expectations do not define
v8 success. Guard checks also cover driver reset/history rebuilding,
same-direction growth, opposing coefficients, repeated measurements,
undertracking and invalid-status inhibition. Tracking checks cover delayed
curvature trends and tapered release-entry headroom. Historical v5–v7 replay
results remain historical observations.

The v8 recorded-input fixture contains 15,273 cycles with 4,879 selected
evidence samples. Base allocation and output eligibility match v7. In the
clean deficient exit, median absolute C1 changes from 0.0665 to 0.0845 rad
while C0 stays unchanged. The growth guard also acts while feedback history
rebuilds; the largest over-growth witness includes nearby driver input and
is excluded from the strict autonomous tracking score. Both good comparison
curves in that fixture retain their median requests, and the older large-turn
fixtures retain their required command scale.

On the earlier good drive, one comparison curve retains extra C1 after
eligible release tracking: median magnitude changes from 0.121 to 0.128 rad.
In its 103–110 s interval, tracking increments occur only while measured
turning falls short, with a median current response/request ratio of 0.895.
Acquired bias can persist after matching, as with ordinary integral feedback.
This collateral command change remains a reason to compare new vehicle logs.
Replay fixes recorded motion and planner outputs, so enabled vehicle logs
are still required to assess tracking error, oscillation and interventions.
