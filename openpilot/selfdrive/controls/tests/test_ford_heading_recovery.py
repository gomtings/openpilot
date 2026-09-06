import unittest

from openpilot.selfdrive.controls.lib.ford_virtual_angle import HeadingFeedback, PathTuning, PscmStatus


def update(feedback, sign, now, *, base=.2, desired=.02, yaw=.1, previous=.2, measurement=None, limit=0, **overrides):
  inputs = {'yaw_rate': sign * yaw, 'speed': 10., 'now': now, 'measurement_time': now if measurement is None else measurement,
            'dt': .01, 'previous_command': sign * previous, 'heading_horizon': 10., 'driver_override': False,
            'pscm_status': PscmStatus(now, 2, limit, 2, False)}
  inputs.update(overrides)
  return feedback.update(sign * base, sign * desired, **inputs)


def acquired_correction(sign, yaw=.4, desired=.03):
  feedback = HeadingFeedback(.2, PathTuning())
  previous = .3
  for i in range(60):
    target = update(feedback, sign, i * .01, base=.3, desired=desired, yaw=yaw, previous=previous)
    previous = sign * target
  return feedback, previous


class TestFordHeadingRecovery(unittest.TestCase):
  def test_release_recovers_opposing_bias_using_current_error(self):
    for sign in (-1, 1):
      with self.subTest(sign=sign):
        feedback, previous = acquired_correction(sign)
        retained = feedback.bias * .2 / .3
        self.assertLess(sign * retained, -.01)
        target = update(feedback, sign, .6, previous=previous)
        # Current request needs 0.2 rad/s, delayed request 0.3 rad/s, measured
        # yaw is 0.1 rad/s. Use the smaller current deficit, not the old turn.
        self.assertAlmostEqual(sign * (feedback.bias - retained), .1 * .01)
        self.assertLessEqual(sign * feedback.bias, 0.)
        self.assertLessEqual(sign * target, .2)
        self.assertEqual(feedback.diagnostics['feedback_status'], 'release_recovery')
        self.assertTrue(feedback.diagnostics['feedback_recovery_active'])

  def test_recovery_stops_at_zero_bias_with_batched_measurement(self):
    for sign in (-1, 1):
      with self.subTest(sign=sign):
        feedback, previous = acquired_correction(sign, yaw=.301)
        self.assertLess(sign * feedback.bias, 0.)
        target = update(feedback, sign, .65, previous=previous, yaw=0.)
        self.assertAlmostEqual(feedback.bias, 0.)
        self.assertAlmostEqual(sign * target, .2)
        self.assertEqual(feedback.diagnostics['feedback_status'], 'release_recovery')
        # The recovery update stops exactly at zero. A later new observation
        # can enter the separately bounded release-tracking policy.
        target = update(feedback, sign, .66, yaw=0.)
        self.assertGreater(sign * feedback.bias, 0.)
        self.assertLessEqual(sign * target, feedback.diagnostics['feedback_release_ceiling'])
        self.assertEqual(feedback.diagnostics['feedback_status'], 'release_tracking')
        self.assertFalse(feedback.diagnostics['feedback_recovery_active'])
        self.assertTrue(feedback.diagnostics['feedback_release_tracking_active'])

  def test_opposing_delayed_request_blocks_recovery_even_with_both_positive_errors(self):
    for sign in (-1, 1):
      feedback, previous = acquired_correction(sign, yaw=-.2, desired=-.03)
      retained = feedback.bias * .2 / .3
      self.assertLess(sign * retained, 0.)
      update(feedback, sign, .6, previous=previous, yaw=-.5)
      self.assertGreater(sign * feedback.diagnostics['feedback_yaw_error'], 0.)
      self.assertAlmostEqual(feedback.bias, retained)
      self.assertEqual(feedback.diagnostics['feedback_status'], 'release')
      self.assertFalse(feedback.diagnostics['feedback_recovery_active'])

  def test_new_base_cannot_fabricate_recovery_beyond_available_slew(self):
    for sign in (-1, 1):
      for partial in (False, True):
        with self.subTest(sign=sign, partial=partial):
          feedback, previous = acquired_correction(sign)
          bias = feedback.bias
          before = .4 + sign * bias
          if partial:
            previous = before - .0045  # Only .0005 rad of the .001 recovery is deliverable.
          target = update(feedback, sign, .6, base=.4, previous=previous)
          self.assertAlmostEqual(sign * (feedback.bias - bias), .0005 if partial else 0.)
          self.assertLessEqual(sign * target, .4)
          self.assertEqual(feedback.diagnostics['feedback_status'], 'release_recovery' if partial else 'host_limit')
          self.assertEqual(feedback.diagnostics['feedback_recovery_active'], partial)

  def test_recovery_requires_both_undertracking_errors_and_no_eps_limit(self):
    for sign in (-1, 1):
      for overrides in ({'yaw': .25}, {'yaw': .2}, {'limit': 2}, {'desired': 0.}, {'desired': -.02}):
        with self.subTest(sign=sign, overrides=overrides):
          feedback, previous = acquired_correction(sign)
          retained = feedback.bias * .2 / .3
          update(feedback, sign, .6, previous=previous, **overrides)
          self.assertAlmostEqual(feedback.bias, retained)
          self.assertNotEqual(feedback.diagnostics['feedback_status'], 'release_recovery')

  def test_same_direction_bias_uses_release_tracking_instead_of_opposing_bias_recovery(self):
    for sign in (-1, 1):
      feedback, previous = acquired_correction(sign, yaw=.2)
      retained = feedback.bias * .2 / .3
      self.assertGreater(sign * retained, 0.)
      update(feedback, sign, .6, previous=previous)
      self.assertGreater(sign * feedback.bias, sign * retained)
      self.assertEqual(feedback.diagnostics['feedback_status'], 'release_tracking')
      self.assertFalse(feedback.diagnostics['feedback_recovery_active'])

  def test_fresh_recovery_clears_backoff_without_reusing_measurements(self):
    for sign in (-1, 1):
      with self.subTest(sign=sign):
        feedback, previous = acquired_correction(sign)
        target = update(feedback, sign, .6, previous=previous, yaw=.4)
        self.assertTrue(feedback.backoff_active)
        bias = feedback.bias
        # The new current request alone cannot recover from an old observation.
        repeated = update(feedback, sign, .61, measurement=.6, previous=sign * target, yaw=.4)
        self.assertEqual(feedback.bias, bias)
        self.assertTrue(feedback.backoff_active)
        self.assertLessEqual(sign * repeated, sign * target)
        update(feedback, sign, .62, previous=sign * repeated)
        self.assertGreater(sign * feedback.bias, sign * bias)
        self.assertFalse(feedback.backoff_active)
        self.assertEqual(feedback.diagnostics['feedback_status'], 'release_recovery')
        bias = feedback.bias
        update(feedback, sign, .63, measurement=.62)
        self.assertEqual(feedback.bias, bias)
        self.assertEqual(feedback.diagnostics['feedback_status'], 'no_new_measurement')
        self.assertFalse(feedback.diagnostics['feedback_recovery_active'])

  def test_driver_or_missing_status_clears_the_correction(self):
    for sign in (-1, 1):
      for overrides in ({'driver_override': True}, {'pscm_status': None},
                        {'pscm_status': PscmStatus(.3, 2, 0, 2, False)}):
        with self.subTest(sign=sign, overrides=overrides):
          feedback, previous = acquired_correction(sign)
          target = update(feedback, sign, .6, previous=previous, **overrides)
          self.assertEqual(feedback.bias, 0.)
          self.assertAlmostEqual(sign * target, .2)
          self.assertNotEqual(feedback.diagnostics['feedback_status'], 'release_recovery')


if __name__ == '__main__':
  unittest.main()
