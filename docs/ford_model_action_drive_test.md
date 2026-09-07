# Ford selected-action drive-test branch

The candidate is selectable on the **Ford CAN FD F-150 Lightning** behind
its own persistent, default-off Sunnylink toggle. Version 2 adds
[bounded excess-yaw offset damping](ford_model_action_damping.md) to the
[original candidate](ford_model_action_candidate.md). Input gates are unchanged.
`calibration_approved=false`: offline checks do not establish physical tracking,
turn-exit behavior or closed-loop stability.

## Select and restore

1. Install branch `hiimisaac-dev` from
   `sunnypilot/sunnypilot` on the device using your normal branch-switch process.
   Allow its build to finish before changing the setting.
2. While offroad, open Sunnylink device settings → Vehicle → Ford and enable
   **Selected-Action Path Tracking (Experimental)** (`FordModelActionController`).
3. Complete a real offroad-to-onroad cycle. Selection occurs when `controlsd`
   starts; changing a stored toggle or disengaging alone cannot swap an active
   controller. Initial physical evaluation remains controlled testing.

The startup log event `Ford path controller selected` should report
`FordModelActionController`. Periodic `Ford C2-free path tracking` events
identify `hypothesis=model-action-c0-c1-yaw-damping-v2` and report host yaw and the command tuple.

Turning the new toggle off and completing another offroad-to-onroad cycle
restores **PSCM Coefficient Observer** if selected, otherwise the original
Ford path controller. The stored observer selection is preserved. The candidate
takes priority on the supported vehicle, independently of EPS firmware query
results. Other vehicles retain their existing selection.

The v8 implementation, its Sunnylink toggle and its dedicated tests are removed.
A leftover `FordVirtualAngleController=1` file cannot enable the new controller.
The shared Float32/CAN rounding helper now lives in `ford_model_action.py`;
unused v8 PSCM-feedback plumbing is removed. Historical v8 route evidence remains
in Git history and the archived validation documents.

## Wiring and validation

`Controls.__init__` selects the candidate once at startup. It shares the
existing Ford call path, selected upstream-limited curvature, service gates,
invalid-output disengagement, Float32 publication and downstream CAN builder.
C2 and C3 stay zero. No opendbc pointer or Panda safety change is included.

Sunnylink publishes the toggle through its generated settings schema and
writes the registered Boolean through the existing parameter endpoint. The
offroad UI rule and `needs_onroad_cycle` metadata describe when it can be
changed and when it takes effect. An onroad backend write changes storage
only; the controller continues using its startup selection.

Native validation also exposed a pre-existing `params_keys_by_flag` bug:
every returned buffer referenced the same reusable string. Sunnylink backup
key enumeration could therefore return corrupted names. The bridge now
returns separate strings owned by the parameter handle. Regression tests
check distinct registered keys across flags, and toggle tests check its
persistence and backup registration using the rebuilt native library.

The current validation record is `ford_model_action_damping_validation.json`;
the [damping notes](ford_model_action_damping.md) explain its scope and limitations.
`ford_model_action_drive_test_validation.json` archives v1 wiring validation
at the recorded source hashes, including 284 tests and 26 subtests. Its counts
and 145-line controller size describe v1. The 469-line v8 module remains removed.

The original 133,550-cycle route reconstruction, 485,238 packing round trips
and mutation probes remain recorded separately in
`ford_model_action_validation.json` at the offline-stage source hashes.

## Reproduce deployment checks

Initialize the branch's exact opendbc submodule (`c21a9013700734dd20b09e05aa68329ad8cc20f9`)
and build the native Params library from this branch before testing.

```sh
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH=.:opendbc_repo
python -m pytest -q -p no:cacheprovider openpilot/selfdrive/controls/tests/test_ford_*.py tools/ford_pscm_lab openpilot/selfdrive/car/tests/test_ford_pscm_status.py openpilot/sunnypilot/sunnylink/tests openpilot/common/tests/test_params.py opendbc_repo/opendbc/car/ford/tests/test_ford.py
python -m tools.ford_pscm_lab.stress_model_action --cycles 200000 --seed 20260907 --opendbc-revision c21a9013700734dd20b09e05aa68329ad8cc20f9 --output .cache/ford_model_action_drive_test/stress.json
```

The full hardware build and device boot are not performed by these offline
tests. Installing the branch and enabling the toggle are separate actions;
pushing the branch does not change a device's selected software or settings.
