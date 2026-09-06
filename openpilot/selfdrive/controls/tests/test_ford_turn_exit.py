"""Request-level turn-exit regressions; these do not simulate EPS response."""
import unittest

from openpilot.selfdrive.controls.lib.ford_virtual_angle import FordVirtualAngleController
from openpilot.selfdrive.controls.tests.test_ford_curvature_c0 import step
from openpilot.selfdrive.controls.tests.test_ford_heading_recovery import acquired_correction, update
from openpilot.selfdrive.controls.tests.test_ford_path_reference import circle
from openpilot.selfdrive.controls.lib.ford_virtual_angle import PscmStatus


class TestFordTurnExit(unittest.TestCase):
  def test_release_can_correct_a_deficit_after_opposing_bias_is_gone(self):
    for sign in (-1, 1):
      with self.subTest(sign=sign):
        feedback, previous = acquired_correction(sign, yaw=.3)
        self.assertAlmostEqual(feedback.bias, 0.)
        target = update(feedback, sign, .6, base=.1, desired=.02, yaw=.1, previous=previous)
        self.assertGreater(sign * target, .1)
        first_bias = sign * feedback.bias
        target = update(feedback, sign, .61, base=.0995, desired=.0199, yaw=.1, previous=sign * target)
        self.assertGreater(sign * feedback.bias, first_bias)
        self.assertLessEqual(sign * target, previous)

  def test_model_growth_cannot_defeat_release_after_driver_bias_reset(self):
    for sign in (-1, 1):
      with self.subTest(sign=sign):
        controller = FordVirtualAngleController()
        for i in range(300):
          now = i * .01
          step(controller, now, sign * .03, circle(sign * .035), speed=10., yaw_rate=sign * .3,
               steering_pressed=(i == 299), pscm_status=PscmStatus(now, 2, 0, 2, False))
        prior_offset, prior_heading = controller.offset_request, controller.heading_request
        self.assertEqual(controller.feedback.bias, 0.)
        step(controller, 3., sign * .025, circle(sign * .06), speed=10., yaw_rate=sign * .4,
             pscm_status=PscmStatus(3., 2, 0, 2, False))
        self.assertLessEqual(sign * controller.offset_request, sign * prior_offset + 1e-12)
        self.assertLessEqual(sign * controller.heading_request, sign * prior_heading + 1e-12)
        # The same strong model pose stays available once new measured motion
        # shows a deficit. The guard must not impose a fixed geometry cap.
        step(controller, 3.01, sign * .024, circle(sign * .06), speed=10., yaw_rate=sign * .1,
             pscm_status=PscmStatus(3.01, 2, 0, 2, False))
        self.assertGreater(sign * controller.offset_request, sign * prior_offset)


if __name__ == '__main__':
  unittest.main()
