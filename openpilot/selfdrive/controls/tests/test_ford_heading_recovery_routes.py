import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np

from openpilot.selfdrive.controls.lib.ford_virtual_angle import FordVirtualAngleController, PscmStatus


class TestFordHeadingRecoveryRoutes(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.fixture = Path(__file__).parent / 'fixtures/ford_heading_recovery_requests.npz'
    cls.metadata = json.loads(cls.fixture.with_suffix('.json').read_text())
    cls.data = dict(np.load(cls.fixture))
    d = cls.data
    models = [SimpleNamespace(position=SimpleNamespace(x=p[0], y=p[1]), orientation=SimpleNamespace(z=p[2])) for p in d['models']]
    commands, gates, rows = [], [], []
    previous_episode = None
    for i, now in enumerate(d['t']):
      if d['episode'][i] != previous_episode:
        controller = FordVirtualAngleController(response_delay=cls.metadata['response_delay'])
        previous_episode = d['episode'][i]
      eps = PscmStatus(float(d['pscm_timestamp'][i]), int(d['pscm_lateral_state'][i]), int(d['pscm_limit'][i]),
                        int(d['pscm_capability'][i]), bool(d['pscm_denied'][i]), bool(d['pscm_valid'][i]))
      prior_bias, prior_base = controller.feedback.bias, controller.feedback.previous_base
      path = controller.update(models[d['model_index'][i]], d['desired_curvature'][i], yaw_rate=d['yaw_rate'][i], speed=d['speed'][i],
                               now=now, measurement_time=d['measurement_time'][i], model_time=d['model_time'][i],
                               reference_time=d['reference_time'][i], active=bool(d['active'][i]), valid=bool(d['valid'][i]),
                               steering_pressed=bool(d['pressed'][i]), steering_torque=d['steering_torque'][i], pscm_status=eps)
      row = dict(controller.diagnostics)
      base = row.get('heading_base', 0.)
      retained_bias = prior_bias if prior_base is not None and prior_base * base >= 0. else 0.
      if prior_base and prior_base * base >= 0.:
        retained_bias *= min(1., abs(base / prior_base))
      row['bias_before_update'] = retained_bias
      commands.append((path.path_offset, path.path_angle, path.curvature, path.curvature_rate))
      gates.append(path.valid)
      rows.append(row)
    cls.commands, cls.gates = np.array(commands), np.array(gates)
    cls.status = np.array([row['feedback_status'] for row in rows])
    cls.recovering = np.array([row.get('feedback_recovery_active', False) for row in rows])
    cls.backoff = np.array([row.get('feedback_backoff_active', False) for row in rows])
    for key in ('heading_base', 'heading_target', 'heading_bias', 'bias_before_update', 'feedback_reference_curvature', 'feedback_yaw_error'):
      setattr(cls, key, np.array([row.get(key, np.nan) for row in rows], dtype=float))

  def test_fixture_hash_and_original_controller_provenance(self):
    self.assertEqual(hashlib.sha256(self.fixture.read_bytes()).hexdigest(), self.metadata['fixture_sha256'])
    self.assertEqual(self.metadata['baseline_revision'], '61dac4977bf9c36504398e8a4959dfed79cf6f05')
    self.assertEqual(len(self.metadata['windows']), 3)
    self.assertGreater(int(self.data['evidence'].sum()), 1000)

  def test_recorded_turn_exit_releases_opposing_correction(self):
    d = self.data
    # Select the captured command problem from the old policy, not from the
    # candidate result: release was freezing an opposing bias while both
    # current and delayed requests still exceeded measured turning.
    base, bias = d['baseline_heading_base'], d['baseline_heading_bias']
    current_error = d['desired_curvature'] * d['speed'] - d['yaw_rate']
    delayed = d['baseline_feedback_reference_curvature']
    mask = (d['window_masks'][:, 0] & (d['baseline_status'] == 'release') & (bias * base < 0.) &
            (current_error * base > 0.) & (d['baseline_feedback_yaw_error'] * base > 0.) &
            (d['desired_curvature'] * base > 0.) & (delayed * base > 0.) & (d['pscm_limit'] < 2))
    self.assertGreater(int(mask.sum()), 100)
    along_turn = np.sign(d['desired_curvature'][mask])
    increase = (self.commands[mask, 1] - d['baseline_commands'][mask, 1]) * along_turn
    self.assertGreater(float(np.median(increase)), .005)
    self.assertGreater(int((self.recovering & mask).sum()), 25)
    self.assertLess(float(np.median(abs(self.heading_bias[mask]))), float(np.median(abs(bias[mask]))) - .005)

  def test_recovery_only_cancels_bias_with_both_requests_undertracked(self):
    d, mask = self.data, self.recovering
    self.assertGreater(int(mask.sum()), 25)
    self.assertTrue((self.status[mask] == 'release_recovery').all())
    self.assertTrue((d['pscm_limit'][mask] < 2).all())
    self.assertTrue((d['pscm_valid'][mask] & self.gates[mask] & ~d['pressed'][mask]).all())
    self.assertTrue((abs(d['steering_torque'][mask]) <= 1.).all())
    self.assertFalse(self.backoff[mask].any())
    self.assertTrue((self.bias_before_update[mask] * self.heading_base[mask] < 0.).all())
    self.assertTrue((self.feedback_yaw_error[mask] * self.heading_base[mask] > 0.).all())
    current_error = d['speed'] * d['desired_curvature'] - d['yaw_rate']
    self.assertTrue((current_error[mask] * self.heading_base[mask] > 0.).all())
    self.assertTrue((d['desired_curvature'][mask] * self.heading_base[mask] > 0.).all())
    self.assertTrue((self.feedback_reference_curvature[mask] * self.heading_base[mask] > 0.).all())
    self.assertTrue((abs(self.heading_bias[mask]) < abs(self.bias_before_update[mask])).all())
    self.assertTrue((self.heading_bias[mask] * self.bias_before_update[mask] >= -1e-12).all())
    self.assertTrue((abs(self.heading_target[mask]) <= abs(self.heading_base[mask]) + 1e-12).all())

  def test_guard_does_not_add_c0_and_preserves_heading_base_and_validity(self):
    evidence = self.data['evidence']
    # The release guard now intentionally prevents same-direction C0 growth.
    # Keep the original fixture's no-extra-demand requirement and its separate
    # good-curve retention checks, rather than insisting on old excess demand.
    direction = np.sign(self.data['desired_curvature'][evidence])
    self.assertTrue(((self.commands[evidence, 0] - self.data['baseline_commands'][evidence, 0]) * direction <= 1e-12).all())
    np.testing.assert_array_equal(self.heading_base[evidence], self.data['baseline_heading_base'][evidence])
    np.testing.assert_array_equal(self.gates[evidence], self.data['baseline_valid'][evidence])

  def test_well_tracked_curves_keep_command_scale(self):
    for index in (1, 2):
      with self.subTest(window=self.metadata['windows'][index]['name']):
        mask = self.data['window_masks'][:, index]
        old, new = self.data['baseline_commands'][mask, 1], self.commands[mask, 1]
        # This bounds collateral command change; it cannot guarantee the same
        # future vehicle response on a drive with the candidate installed.
        self.assertGreaterEqual(float(np.median(abs(new))), .95 * float(np.median(abs(old))))
        self.assertLess(float(np.quantile(abs(new - old), .9)), .02)

  def test_all_fixture_commands_respect_field_and_rate_limits(self):
    d = self.data
    np.testing.assert_array_equal(self.commands[:, 2:], 0.)
    self.assertTrue(np.isfinite(self.commands).all())
    self.assertTrue((abs(self.commands[:, :2]) <= [5.110000001, .500000001]).all())
    continuous = (d['episode'][1:] == d['episode'][:-1]) & self.gates[1:] & self.gates[:-1]
    allowed = np.diff(d['t'])[:, None] * [4., .5] + [.01, .0005] + np.array([1e-8, 1e-8])
    self.assertTrue((abs(np.diff(self.commands[:, :2], axis=0))[continuous] <= allowed[continuous]).all())


if __name__ == '__main__':
  unittest.main()
