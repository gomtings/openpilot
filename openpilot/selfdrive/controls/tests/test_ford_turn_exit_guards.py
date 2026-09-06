"""Independent turn-exit guard properties, not a PSCM response simulation."""
import unittest

from openpilot.selfdrive.controls.lib.ford_virtual_angle import FordVirtualAngleController, HeadingFeedback, PathTuning, PscmStatus, ReleaseGuard


AUTO_STATUS = object()


def guard_step(guard, sign, now, *, desired=.02, yaw=.4, speed=10., measurement=None, pscm=AUTO_STATUS, **overrides):
  inputs = {'yaw_rate': sign * yaw, 'speed': speed, 'now': now, 'measurement_time': now if measurement is None else measurement,
            'heading_horizon': 10., 'driver_override': False,
            'pscm_status': PscmStatus(now, 2, 0, 2, False) if pscm is AUTO_STATUS else pscm}
  inputs.update(overrides)
  return guard.update(sign * desired, **inputs)


def warm_guard(guard, sign, last_override=False):
  for i in range(40):
    guard_step(guard, sign, i * .01, desired=.03, yaw=.3, driver_override=last_override and i == 39)


def feedback_step(feedback, sign, now, *, base=.1, desired=.02, yaw=.1, speed=10., previous=.3, measurement=None, **overrides):
  inputs = {'yaw_rate': sign * yaw, 'speed': speed, 'now': now, 'measurement_time': now if measurement is None else measurement,
            'dt': .01, 'previous_command': sign * previous, 'heading_horizon': 10., 'driver_override': False,
            'pscm_status': PscmStatus(now, 2, 0, 2, False)}
  inputs.update(overrides)
  return feedback.update(sign * base, sign * desired, **inputs)


def warm_feedback(sign, speed=10., yaw=.3):
  feedback = HeadingFeedback(.2, PathTuning())
  previous = .3
  for i in range(60):
    target = feedback_step(feedback, sign, i * .01, base=.3, desired=.03, yaw=yaw, speed=speed, previous=previous)
    previous = sign * target
  return feedback, previous


class TestFordReleaseGuard(unittest.TestCase):
  def test_only_growth_in_the_requested_direction_is_capped(self):
    for sign in (-1, 1):
      with self.subTest(sign=sign):
        guard = ReleaseGuard(.2)
        warm_guard(guard, sign)
        self.assertTrue(guard_step(guard, sign, .4))
        self.assertAlmostEqual(guard.limit(sign * .5, sign * .2), sign * .2)
        self.assertAlmostEqual(guard.limit(sign * .1, sign * .2), sign * .1)
        self.assertAlmostEqual(guard.limit(-sign * .5, sign * .2), -sign * .5)
        self.assertAlmostEqual(guard.limit(sign * .5, -sign * .2), 0.)

  def test_driver_reset_does_not_erase_valid_request_history(self):
    for sign in (-1, 1):
      guard = ReleaseGuard(.2)
      warm_guard(guard, sign, last_override=True)
      self.assertFalse(guard.active)
      self.assertTrue(guard_step(guard, sign, .4))
      self.assertAlmostEqual(guard.reference_curvature, sign * .03)

  def test_repeated_measurement_keeps_guard_but_current_override_disables_it(self):
    for sign in (-1, 1):
      guard = ReleaseGuard(.2)
      warm_guard(guard, sign)
      self.assertTrue(guard_step(guard, sign, .4))
      self.assertTrue(guard_step(guard, sign, .41, measurement=.4))
      self.assertFalse(guard_step(guard, sign, .42, measurement=.4, driver_override=True))
      self.assertAlmostEqual(guard.limit(sign * .5, sign * .2), sign * .5)
      self.assertTrue(guard_step(guard, sign, .43))

  def test_status_speed_zero_and_reversal_disable_action(self):
    cases = [
      {'pscm': None}, {'pscm': PscmStatus(.1, 2, 0, 2, False)},
      {'pscm': PscmStatus(.4, 2, 0, 2, False, False)}, {'pscm': PscmStatus(.4, 2, 0, 2, True)},
      {'pscm': PscmStatus(.4, 1, 0, 2, False)}, {'pscm': PscmStatus(.4, 2, 0, 0, False)},
      {'pscm': PscmStatus(.4, 2, 3, 2, False)}, {'pscm': PscmStatus(float('nan'), 2, 0, 2, False)},
      {'speed': 1.99}, {'desired': 0.}, {'desired': -.02},
    ]
    for sign in (-1, 1):
      for overrides in cases:
        with self.subTest(sign=sign, overrides=overrides):
          guard = ReleaseGuard(.2)
          warm_guard(guard, sign)
          self.assertFalse(guard_step(guard, sign, .4, **overrides))
          self.assertAlmostEqual(guard.limit(sign * .5, sign * .2), sign * .5)

  def test_repeated_backward_status_cannot_restore_guard_authority(self):
    guard = ReleaseGuard(.2)
    warm_guard(guard, 1)
    self.assertTrue(guard_step(guard, 1, .4))
    self.assertFalse(guard_step(guard, 1, .41, pscm=PscmStatus(.39, 2, 0, 2, False)))
    self.assertFalse(guard_step(guard, 1, .42, pscm=PscmStatus(.39, 2, 0, 2, False)))
    self.assertTrue(guard_step(guard, 1, .43, pscm=PscmStatus(.43, 2, 0, 2, False)))

  def test_invalid_future_status_does_not_poison_later_fresh_status(self):
    guard = ReleaseGuard(.2)
    warm_guard(guard, 1)
    self.assertFalse(guard_step(guard, 1, .4, pscm=PscmStatus(10., 2, 0, 2, False)))
    self.assertTrue(guard_step(guard, 1, .41, pscm=PscmStatus(.41, 2, 0, 2, False)))

  def test_fresh_unavailable_status_prevents_older_in_progress_reactivation(self):
    for unavailable in (PscmStatus(.41, 1, 0, 2, False), PscmStatus(.41, 2, 0, 2, True), PscmStatus(.41, 2, 0, 0, False)):
      with self.subTest(unavailable=unavailable):
        guard = ReleaseGuard(.2)
        warm_guard(guard, 1)
        self.assertTrue(guard_step(guard, 1, .4))
        self.assertFalse(guard_step(guard, 1, .41, pscm=unavailable))
        self.assertFalse(guard_step(guard, 1, .42, pscm=PscmStatus(.4, 2, 0, 2, False)))
        self.assertFalse(guard_step(guard, 1, .43, pscm=PscmStatus(.4, 2, 0, 2, False)))
        self.assertAlmostEqual(guard.limit(.5, .2), .5)
        self.assertTrue(guard_step(guard, 1, .44, pscm=PscmStatus(.44, 2, 0, 2, False)))

  def test_parent_reset_clears_the_independent_history(self):
    controller = FordVirtualAngleController()
    warm_guard(controller.release_guard, 1)
    self.assertTrue(guard_step(controller.release_guard, 1, .4))
    controller.reset()
    self.assertFalse(guard_step(controller.release_guard, 1, .41))
    self.assertAlmostEqual(controller.release_guard.limit(.5, .2), .5)


class TestFordReleaseTrackingGuards(unittest.TestCase):
  def test_repeated_measurement_does_not_add_another_release_correction(self):
    for sign in (-1, 1):
      feedback, previous = warm_feedback(sign)
      target = feedback_step(feedback, sign, .6, previous=previous)
      bias = feedback.bias
      self.assertGreater(sign * bias, 0.)
      feedback_step(feedback, sign, .61, previous=sign * target, measurement=.6)
      self.assertEqual(feedback.bias, bias)
      self.assertFalse(feedback.diagnostics['feedback_release_tracking_active'])
      feedback_step(feedback, sign, .62, previous=sign * target)
      self.assertGreater(sign * feedback.bias, sign * bias)

  def test_response_trend_uses_curvature_despite_opposite_yaw_rate_trend(self):
    # First case: yaw rises .025->.04, but curvature falls .005->.004.
    # Second case: yaw falls .05->.04, but curvature rises .005->.008.
    for sign in (-1, 1):
      for old_speed, old_yaw, new_speed, allowed in ((5., .025, 10., True), (10., .05, 5., False)):
        with self.subTest(sign=sign, allowed=allowed):
          feedback, previous = warm_feedback(sign, speed=old_speed, yaw=old_yaw)
          retained = feedback.bias * (.1 / .3)
          feedback_step(feedback, sign, .6, speed=new_speed, yaw=.04, previous=previous)
          self.assertEqual(feedback.diagnostics['feedback_release_tracking_active'], allowed)
          if allowed:
            self.assertGreater(sign * feedback.bias, sign * retained)
          else:
            self.assertAlmostEqual(feedback.bias, retained)

  def test_zero_release_headroom_does_not_reduce_base_or_store_boost(self):
    for sign in (-1, 1):
      feedback, previous = warm_feedback(sign)
      target = feedback_step(feedback, sign, .6, base=.4, previous=previous)
      self.assertAlmostEqual(feedback.bias, 0.)
      self.assertAlmostEqual(sign * target, .4)
      self.assertFalse(feedback.diagnostics['feedback_release_tracking_active'])

  def test_release_tracking_needs_current_deficit_and_no_eps_limit(self):
    for sign in (-1, 1):
      for overrides in ({'yaw': .25}, {'yaw': .2}, {'pscm_status': PscmStatus(.6, 2, 2, 2, False)}):
        with self.subTest(sign=sign, overrides=overrides):
          feedback, previous = warm_feedback(sign)
          target = feedback_step(feedback, sign, .6, previous=previous, **overrides)
          self.assertAlmostEqual(feedback.bias, 0.)
          self.assertAlmostEqual(sign * target, .1)
          self.assertFalse(feedback.diagnostics['feedback_release_tracking_active'])

  def test_release_tracking_respects_blocked_and_partial_slew_admission(self):
    for sign in (-1, 1):
      for partial in (False, True):
        with self.subTest(sign=sign, partial=partial):
          feedback, previous = warm_feedback(sign)
          feedback_step(feedback, sign, .6, previous=previous)
          retained = feedback.bias * (.09 / .1)
          heading_before = .09 + sign * retained
          previous = heading_before - (.0045 if partial else .005)
          feedback_step(feedback, sign, .61, base=.09, desired=.0199, previous=previous)
          self.assertAlmostEqual(sign * (feedback.bias - retained), .0005 if partial else 0.)
          self.assertEqual(feedback.diagnostics['feedback_release_tracking_active'], partial)

  def test_brief_release_pause_cannot_capture_a_larger_entry_command(self):
    for sign in (-1, 1):
      with self.subTest(sign=sign):
        feedback, previous = warm_feedback(sign)
        feedback_step(feedback, sign, .6, previous=previous)
        # Hold the smaller request until the delayed reference catches up,
        # but not for a full response interval after release becomes false.
        for i in range(61, 84):
          feedback_step(feedback, sign, i * .01, desired=.02, yaw=.2)
        feedback_step(feedback, sign, .84, desired=.019, previous=.45)
        self.assertAlmostEqual(feedback.diagnostics['feedback_release_ceiling'], .1 + (.3 - .1) * (.019 / .03))
        # A full quiet response interval starts a new independent episode.
        for i in range(85, 129):
          feedback_step(feedback, sign, i * .01, desired=.019, yaw=.19)
        feedback_step(feedback, sign, 1.29, desired=.018, previous=.45)
        self.assertAlmostEqual(feedback.diagnostics['feedback_release_ceiling'], .1 + (.45 - .1) * (.018 / .019))


if __name__ == '__main__':
  unittest.main()
