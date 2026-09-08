"""Measured yaw gates input health but cannot attenuate path demand."""
import math

import pytest

from openpilot.selfdrive.controls.lib.ford_model_action import ModelActionController
from openpilot.selfdrive.controls.lib.ford_path import FordPath
from openpilot.selfdrive.controls.tests.test_ford_model_action import straight


@pytest.mark.parametrize('sign', [-1., 1.])
@pytest.mark.parametrize('yaw', [-3., -.2, -.008, 0., .008, .2, 3.])
def test_valid_yaw_cannot_change_commands_during_entry_release_or_reversal(sign, yaw):
  reference, measured = ModelActionController(), ModelActionController()
  for i in range(400):
    offset, desired = ((.4, .02), (.4, .001), (.12, -.0004078), (-.4, -.02))[i//100]
    model = straight(sign*offset)
    expected = reference.update(model, sign*desired, speed=10., dt=.01)
    actual = measured.update(model, sign*desired, speed=10., dt=.01, yaw_rate=yaw)
    assert actual == expected
    assert actual.curvature == actual.curvature_rate == 0.


@pytest.mark.parametrize('yaw', [math.nan, math.inf, -math.inf, None, 'bad', 3.001, -3.001])
def test_invalid_yaw_still_resets_core(yaw):
  controller = ModelActionController()
  controller.update(straight(.4), .01, speed=10., dt=.01)
  assert controller.update(straight(.4), .01, speed=10., dt=.01, yaw_rate=yaw) == FordPath()
  assert controller.c0 == controller.c1 == 0.
