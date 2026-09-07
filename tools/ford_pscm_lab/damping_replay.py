"""Compare v1 construction and v2 damping on complete local rlogs, offline.

Original controls publication times proxy computation time. Consumed model
timestamps are exact; carState is causal and carControl is matched within 5 ms.
This compares commands on a fixed recording, never counterfactual vehicle motion.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import zstandard

from openpilot.cereal import log
from openpilot.selfdrive.controls.lib import ford_model_action
from openpilot.selfdrive.controls.lib.ford_model_action import FordModelActionController
from tools.ford_pscm_lab.model_action_replay import WireCheck, field_checks, sample, verify_dependency


DEPLOYMENT_OPENDBC = 'c21a9013700734dd20b09e05aa68329ad8cc20f9'


def extract(directory):
  columns = {'cs': 't valid can_valid speed yaw torque pressed', 'controls': 't valid desired model_ns',
             'cc': 't valid active', 'params': 't valid', 'path': 't valid active c0 c1', 'model': 't valid ns'}
  rows = {name: [] for name in columns}
  models, sources = [], {}
  t0 = None
  files = sorted(directory.glob('*--rlog.zst'), key=lambda p: int(p.name.split('--')[-2]))
  if not files:
    raise ValueError('No complete rlogs found')
  for file in files:
    compressed = file.read_bytes()
    sources[file.name] = hashlib.sha256(compressed).hexdigest()
    data = zstandard.ZstdDecompressor().stream_reader(compressed).read()
    for event in log.Event.read_multiple_bytes(data):
      kind, t, valid = event.which(), event.logMonoTime*1e-9, event.valid
      if t0 is None:
        t0 = t
      if kind == 'carState':
        cs = event.carState
        rows['cs'].append((t, valid, cs.canValid, cs.vEgo, -cs.yawRate, cs.steeringTorque, cs.steeringPressed))
      elif kind == 'controlsState':
        cs = event.controlsState
        rows['controls'].append((t, valid, cs.desiredCurvature, cs.lateralPlanMonoTime))
      elif kind == 'carControl':
        rows['cc'].append((t, valid, event.carControl.latActive))
      elif kind == 'vehicleParameters':
        rows['params'].append((t, valid))
      elif kind == 'carControlSP':
        path = event.carControlSP.fordLateralPath
        rows['path'].append((t, valid, path.valid, path.pathOffset, path.pathAngle))
      elif kind == 'modelV2':
        model = event.modelV2
        models.append(SimpleNamespace(position=SimpleNamespace(x=list(model.position.x), y=list(model.position.y)),
                                      orientation=SimpleNamespace(z=list(model.orientation.z))))
        rows['model'].append((t, valid, event.logMonoTime))
      elif kind == 'lateralManeuverPlan':
        raise ValueError('This replay requires routes without a separate maneuver reference')
  streams = {}
  for name, fields in columns.items():
    values = np.array(rows[name])
    if not len(values) or np.any(np.diff(values[:, 0]) < 0.):
      raise ValueError(f'Missing or backward {name} stream')
    streams[name] = dict(zip(fields.split(), values.T, strict=True))
  return streams, models, sources, t0


def run(directory, output):
  directory, output = directory.resolve(), output.resolve()
  if output == directory or directory in output.parents:
    raise ValueError('Output must be outside the source route directory')
  verify_dependency(DEPLOYMENT_OPENDBC)
  streams, models, sources, t0 = extract(directory)
  controls, model = streams['controls'], streams['model']
  t = controls['t']
  cs, params = (sample(streams[name], t) for name in ('cs', 'params'))
  cc, recorded = (sample(streams[name], t, nearest=True) for name in ('cc', 'path'))
  mi = np.clip(np.searchsorted(model['ns'], controls['model_ns']), 0, len(models)-1)
  exact = model['ns'][mi] == controls['model_ns']
  services = ((controls['valid'] == 1) & (cc['valid'] == 1) & (abs(cc['t']-t) < .005) &
              (cs['valid'] == 1) & (cs['can_valid'] == 1) & (params['valid'] == 1) &
              (t-params['t'] >= 0.) & (t-params['t'] <= .15) & exact & (model['valid'][mi] == 1))
  baseline, candidate, wire = FordModelActionController(), FordModelActionController(), WireCheck()
  before, after = np.zeros((len(t), 4)), np.zeros((len(t), 4))
  eligible = np.zeros(len(t), bool)
  reasons = Counter()
  for i, now in enumerate(t):
    kwargs = {'speed': cs['speed'][i], 'now': now, 'measurement_time': cs['t'][i], 'model_time': model['t'][mi[i]],
              'reference_time': model['t'][mi[i]], 'active': bool(cc['active'][i]), 'valid': bool(services[i])}
    geometry = models[mi[i]] if exact[i] else None
    # Zero yaw retains v1 targets. Both passes enforce the actual yaw range gate.
    kwargs['valid'] &= bool(np.isfinite(cs['yaw'][i]) and abs(cs['yaw'][i]) <= 3.)
    a = baseline.update(geometry, controls['desired'][i], yaw_rate=0., **kwargs)
    b = candidate.update(geometry, controls['desired'][i], yaw_rate=cs['yaw'][i], **kwargs)
    assert a.valid == b.valid and a.path_angle == b.path_angle
    before[i] = a.path_offset, a.path_angle, a.curvature, a.curvature_rate
    after[i] = b.path_offset, b.path_angle, b.curvature, b.curvature_rate
    eligible[i] = b.valid
    reasons[candidate.diagnostics['status']] += 1
    wire.check(a)
    wire.check(b)
  field_checks(before, eligible, t)
  field_checks(after, eligible, t)
  clean = eligible & (cs['pressed'] == 0) & (abs(cs['torque']) <= 1.)
  # Erode driver eligibility by one second in each direction on original time.
  bad = np.r_[0, np.cumsum(~clean)]
  left, right = np.searchsorted(t, t-1.), np.searchsorted(t, t+1., side='right')
  clean &= (bad[right] == bad[left]) & (t >= t[0]+1.) & (t <= t[-1]-1.)
  relative = t-t0
  masks = {'eligible': eligible, 'driver_clean': clean,
           'driver_clean_low_request_above_8mps': clean & (cs['speed'] >= 8.) & (abs(controls['desired'])*cs['speed']**2 < .15),
           'turn': clean & (abs(controls['desired'])*cs['speed']**2 >= .5),
           'segment10_entry_peak': eligible & (relative >= 637.) & (relative < 640.),
           'segment10_exit_before_strong_input': eligible & (relative >= 642.7) & (relative < 643.852)}
  weight = np.minimum(np.diff(t, append=t[-1]+.01), .03)
  difference = abs(before[:, 0]-after[:, 0])
  cohorts = {}
  for name, mask in masks.items():
    if mask.any():
      cohorts[name] = {'cycles': int(mask.sum()), 'seconds': float(weight[mask].sum()),
                       'changed_c0_cycles': int((difference[mask] > 1e-9).sum()),
                       'mean_absolute_c0_change_m': float(np.average(difference[mask], weights=weight[mask])),
                       'max_absolute_c0_change_m': float(difference[mask].max()),
                       'v1_peak_absolute_c0_m': float(abs(before[mask, 0]).max()),
                       'v2_peak_absolute_c0_m': float(abs(after[mask, 0]).max())}
  paired = eligible & (recorded['valid'] == 1) & (recorded['active'] == 1) & (abs(recorded['t']-t) < .005)
  actual = np.column_stack((recorded['c0'], recorded['c1']))
  error = abs(before[:, :2]-actual)
  report = {'scope': 'Fixed-input command replay only; no physical improvement or stability claim.', 'calibration_approved': False,
            'cycles': len(t), 'eligible_cycles': int(eligible.sum()), 'status_counts': dict(reasons),
            'c1_exactly_unchanged': True, 'same_validity': True, 'field_slew_zero_c2_c3_pass': True,
            'float32_can_round_trips': wire.count, 'cohorts': cohorts,
            'v1_reconstruction_vs_recorded': {'paired_cycles': int(paired.sum()),
              'within_one_quantum_cycles': int(np.all(error[paired] <= [.010001, .0005001], axis=1).sum()),
              'maximum_absolute_error_c0_c1': np.max(error[paired], axis=0).tolist()},
            'timing': 'Publication-time proxy, causal carState, exact consumed model; full SubMaster health unavailable.',
            'baseline': 'Current adapter with zero yaw retains v1 targets and actual-yaw sanity gate.',
            'source_rlog_sha256': sources, 'opendbc_head': DEPLOYMENT_OPENDBC,
            'source_sha256': {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in (Path(__file__), Path(ford_model_action.__file__))}}
  output.mkdir(parents=True, exist_ok=True)
  np.savez_compressed(output/'commands.npz', t=relative, before=before, after=after, eligible=eligible,
                      speed=cs['speed'], yaw=cs['yaw'], desired=controls['desired'], torque=cs['torque'])
  (output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
  print(json.dumps({k: v for k, v in report.items() if k not in ('source_rlog_sha256', 'source_sha256')}, indent=2))


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('rlog_directory', type=Path)
  parser.add_argument('--output', type=Path, required=True)
  args = parser.parse_args()
  run(args.rlog_directory, args.output)
