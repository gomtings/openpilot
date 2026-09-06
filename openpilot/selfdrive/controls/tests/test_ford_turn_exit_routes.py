"""Recorded-input regression checks; changed commands do not predict motion."""
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np

from openpilot.selfdrive.controls.lib.ford_virtual_angle import FordVirtualAngleController, PscmStatus


class TestFordTurnExitRoutes(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.fixture = Path(__file__).parent / 'fixtures/ford_turn_exit_requests.npz'
    cls.metadata = json.loads(cls.fixture.with_suffix('.json').read_text())
    cls.data = dict(np.load(cls.fixture, allow_pickle=False))
    d = cls.data
    models = [SimpleNamespace(position=SimpleNamespace(x=p[0], y=p[1]), orientation=SimpleNamespace(z=p[2])) for p in d['models']]
    commands, gates, rows, previous_commands, prior_biases = [], [], [], [], []
    episode = None
    for i, now in enumerate(d['t']):
      if d['episode'][i] != episode:
        controller = FordVirtualAngleController(response_delay=cls.metadata['response_delay'])
        episode = d['episode'][i]
      previous_commands.append([controller.offset_request, controller.heading_request])
      old_base, old_bias = controller.feedback.previous_base, controller.feedback.bias
      eps = PscmStatus(float(d['pscm_timestamp'][i]), int(d['pscm_lateral_state'][i]), int(d['pscm_limit'][i]),
                        int(d['pscm_capability'][i]), bool(d['pscm_denied'][i]), bool(d['pscm_valid'][i]))
      path = controller.update(models[d['model_index'][i]], d['desired_curvature'][i], yaw_rate=d['yaw_rate'][i], speed=d['speed'][i],
                               now=now, measurement_time=d['measurement_time'][i], model_time=d['model_time'][i],
                               reference_time=d['reference_time'][i], active=bool(d['active'][i]), valid=bool(d['valid'][i]),
                               steering_pressed=bool(d['pressed'][i]), steering_torque=d['steering_torque'][i], pscm_status=eps)
      row = dict(controller.diagnostics)
      base = row.get('heading_base', 0.)
      retained = old_bias if old_base is not None and old_base * base >= 0. else 0.
      if old_base and old_base * base >= 0.:
        retained *= min(1., abs(base / old_base))
      prior_biases.append(retained)
      commands.append([path.path_offset, path.path_angle, path.curvature, path.curvature_rate])
      gates.append(path.valid)
      rows.append(row)
    cls.commands, cls.gates = np.array(commands), np.array(gates)
    cls.previous_commands, cls.prior_biases = np.array(previous_commands), np.array(prior_biases)
    cls.status = np.array([row['feedback_status'] for row in rows])
    for key in ('heading_base', 'heading_bias', 'offset_target', 'heading_target', 'offset_target_unguarded', 'heading_target_unguarded',
                'feedback_yaw_error', 'feedback_reference_curvature', 'release_guard_reference_curvature',
                'feedback_release_ceiling', 'feedback_curvature_delta', 'heading_horizon'):
      setattr(cls, key, np.array([row.get(key, np.nan) for row in rows], dtype=float))
    cls.guarded = np.array([row.get('release_guard_active', False) for row in rows])
    cls.tracking = np.array([row.get('feedback_release_tracking_active', False) for row in rows])

  def window(self, name):
    index = next(i for i, window in enumerate(self.metadata['windows']) if window['name'] == name)
    return self.data['window_masks'][:, index]

  def test_fixture_provenance_and_minimal_signal_schema(self):
    self.assertEqual(hashlib.sha256(self.fixture.read_bytes()).hexdigest(), self.metadata['fixture_sha256'])
    self.assertEqual(self.metadata['baseline_revision'], 'dfcfddb91ce2409511f5b2dbce25d06d5056b3d6')
    self.assertEqual(self.metadata['baseline_hypothesis'], 'model-pose-c0-c1-feedback-v7')
    self.assertEqual(set(self.data), set(self.metadata['retained_fields']))
    self.assertGreater(self.metadata['samples'], 15000)
    self.assertEqual(len(self.data['t']), self.metadata['samples'])
    self.assertGreater(int(self.data['evidence'].sum()), 4000)
    self.assertTrue(np.isfinite(self.data['models']).all())
    self.assertEqual(self.data['t'][0], 0.)
    for key in ('wheel_deg', 'wheel_rate', 'eps_torque', 'recorded_wheel_curvature', 'origin_ns', 'publication_time'):
      self.assertNotIn(key, self.data)
    for key in ('commands', 'valid', 'heading_bias'):
      self.assertTrue(self.metadata['compact_full_baseline_evidence_parity'][key]['exact'])
    for digest in self.metadata['baseline_source_hashes'].values():
      self.assertRegex(digest, r'^[0-9a-f]{64}$')

  def test_recorded_model_growth_is_guarded_while_feedback_rebuilds_history(self):
    d = self.data
    # Select the problem using pinned baseline status and recorded inputs,
    # not candidate success. Nearby driver input is deliberately retained.
    selected = self.window('first_reversal') | self.window('second_reversal') | self.window('over_growth')
    index = np.searchsorted(d['t'], d['measurement_time'] - self.metadata['response_delay'], side='right') - 1
    safe_index = np.maximum(index, 0)
    delayed = d['desired_curvature'][safe_index]
    direction = np.sign(d['desired_curvature'])
    horizon = np.maximum(7., d['speed'])
    mask = (selected & (d['baseline_status'] == 'history') & d['baseline_valid'] & (index >= 0) &
            (d['episode'][safe_index] == d['episode']) & ~d['pressed'] & (abs(d['steering_torque']) <= 1.) &
            (delayed * d['desired_curvature'] > 0.) & ((abs(delayed) - abs(d['desired_curvature'])) * horizon > .0005) &
            ((d['yaw_rate'] - d['speed'] * delayed) * direction > 0.) &
            ((d['yaw_rate'] - d['speed'] * d['desired_curvature']) * direction > 0.))
    self.assertGreater(int(mask.sum()), 25)
    self.assertGreater(int((mask & self.window('over_growth')).sum()), 10)
    self.assertTrue(self.guarded[mask].all())
    self.assertTrue((self.status[mask] == 'history').all())
    np.testing.assert_array_equal(self.heading_bias[mask], 0.)
    targets = np.column_stack((self.offset_target, self.heading_target))
    unguarded = np.column_stack((self.offset_target_unguarded, self.heading_target_unguarded))
    for sign in (-1, 1):
      case = mask & (direction == sign)
      self.assertGreater(int(case.sum()), 0)
      growth_removed = (unguarded[case] - targets[case]) * sign
      self.assertTrue((growth_removed >= -1e-12).all())
      self.assertTrue((growth_removed.max(axis=0) > [.01, .0005]).all())

  def test_every_release_guard_ceiling_preserves_opposing_path_terms(self):
    mask = self.guarded
    d = self.data
    self.assertGreater(int(mask.sum()), 100)
    direction = np.sign(d['desired_curvature'][mask])[:, None]
    targets = np.column_stack((self.offset_target, self.heading_target))[mask]
    unguarded = np.column_stack((self.offset_target_unguarded, self.heading_target_unguarded))[mask]
    previous = self.previous_commands[mask]
    same_direction = unguarded * direction > 0.
    self.assertTrue((targets * direction <= np.maximum(previous * direction, 0.) + 1e-12)[same_direction].all())
    np.testing.assert_array_equal(targets[~same_direction], unguarded[~same_direction])
    self.assertTrue((~d['pressed'][mask] & (abs(d['steering_torque'][mask]) <= 1.) & (d['pscm_limit'][mask] < 3)).all())

  def test_recorded_zero_bias_release_can_track_with_bounded_new_correction(self):
    d = self.data
    base = d['baseline_heading_base']
    current_error = d['speed'] * d['desired_curvature'] - d['yaw_rate']
    mask = (self.window('zero_bias_release') & d['clean_rawtorque'] & (d['demand'] >= .5) &
            (d['baseline_status'] == 'release') & (abs(d['baseline_heading_bias']) <= 1e-9) & (d['pscm_limit'] < 2) &
            (current_error * base > 0.) & (d['baseline_feedback_yaw_error'] * base > 0.) &
            (d['desired_curvature'] * base > 0.) & (d['baseline_feedback_reference_curvature'] * base > 0.))
    self.assertGreater(int(mask.sum()), 100)
    direction = np.sign(d['desired_curvature'][mask])
    increase = (self.commands[mask, 1] - d['baseline_commands'][mask, 1]) * direction
    self.assertGreater(float(np.median(increase)), .005)
    self.assertGreater(int((mask & self.tracking).sum()), 25)
    np.testing.assert_array_equal(self.commands[mask, 0], d['baseline_commands'][mask, 0])

  def test_release_tracking_admits_only_eligible_reachable_headroom(self):
    d, mask = self.data, self.tracking
    self.assertGreater(int(mask.sum()), 25)
    base, bias = self.heading_base[mask], self.heading_bias[mask]
    sign = np.sign(base)
    self.assertTrue((self.status[mask] == 'release_tracking').all())
    self.assertTrue((d['pscm_limit'][mask] < 2).all())
    self.assertTrue((d['pscm_valid'][mask] & self.gates[mask] & ~d['pressed'][mask]).all())
    self.assertTrue((abs(d['steering_torque'][mask]) <= 1.).all())
    self.assertTrue((self.prior_biases[mask] * base >= 0.).all())
    self.assertTrue((self.feedback_yaw_error[mask] * base > 0.).all())
    current_error = d['speed'][mask] * d['desired_curvature'][mask] - d['yaw_rate'][mask]
    self.assertTrue((current_error * base > 0.).all())
    self.assertTrue((d['desired_curvature'][mask] * base > 0.).all())
    self.assertTrue((self.feedback_reference_curvature[mask] * base > 0.).all())
    self.assertTrue((sign * self.feedback_curvature_delta[mask] * self.heading_horizon[mask] <= .0005 + 1e-12).all())
    self.assertTrue(((bias - self.prior_biases[mask]) * sign > 0.).all())
    self.assertTrue(((base + bias) * sign <= self.feedback_release_ceiling[mask] + 1e-12).all())

  def test_raw_model_bases_and_validity_remain_unchanged_on_evidence(self):
    d, mask = self.data, self.data['evidence']
    np.testing.assert_array_equal(self.gates[mask], d['baseline_valid'][mask])
    valid = mask & self.gates
    np.testing.assert_array_equal(self.heading_base[valid], d['baseline_heading_base'][valid])
    np.testing.assert_array_equal(self.offset_target_unguarded[valid], d['baseline_offset_target'][valid])

  def test_preselected_good_curves_retain_command_scale(self):
    d = self.data
    for name in ('good_curve_a', 'good_curve_b'):
      with self.subTest(window=name):
        mask = self.window(name) & d['clean_rawtorque'] & (d['demand'] >= .5)
        self.assertGreater(int(mask.sum()), 200)
        old, new = d['baseline_commands'][mask, :2], self.commands[mask, :2]
        # These are collateral command bounds, not a new-motion prediction.
        self.assertTrue((np.median(abs(new), axis=0) >= .95 * np.median(abs(old), axis=0)).all())
        self.assertTrue((np.median(abs(new), axis=0) <= 1.05 * np.median(abs(old), axis=0)).all())
        self.assertTrue((np.quantile(abs(new - old), .9, axis=0) <= [.02, .005]).all())

  def test_all_commands_keep_zero_c2_c3_and_existing_field_and_rate_limits(self):
    d = self.data
    np.testing.assert_array_equal(self.commands[:, 2:], 0.)
    self.assertTrue(np.isfinite(self.commands).all())
    self.assertTrue((abs(self.commands[:, :2]) <= [5.110000001, .500000001]).all())
    continuous = (d['episode'][1:] == d['episode'][:-1]) & self.gates[1:] & self.gates[:-1]
    allowed = np.diff(d['t'])[:, None] * [4., .5] + [.01, .0005] + np.array([1e-8, 1e-8])
    self.assertTrue((abs(np.diff(self.commands[:, :2], axis=0))[continuous] <= allowed[continuous]).all())


if __name__ == '__main__':
  unittest.main()
