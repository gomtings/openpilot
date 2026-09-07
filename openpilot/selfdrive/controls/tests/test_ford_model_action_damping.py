"""Bounded excess-yaw damping: remove offset demand without integral or modes."""
import math
import pytest

from openpilot.selfdrive.controls.lib.ford_model_action import ModelActionController, damp_offset
from openpilot.selfdrive.controls.lib.ford_path import FordPath
from openpilot.selfdrive.controls.tests.test_ford_model_action import straight


@pytest.mark.parametrize('sign', [-1., 1.])
def test_recorded_turn_exit_reduces_same_direction_offset_before_driver_intervention(sign):
  # Route9b, segment10, about643.0s; fresh Ford yaw in host coordinates.
  c0, desired, speed, yaw = sign*.5, sign*.0117238564, 9.71, sign*.180
  reduced = damp_offset(c0, desired, speed, yaw)
  assert reduced == pytest.approx(sign*(.5-1.4*(.180-9.71*.0117238564-.02)))
  assert 0. < sign*reduced < .45


@pytest.mark.parametrize('sign', [-1., 1.])
def test_recorded_late_exit_removes_remaining_offset_without_creating_countersteer(sign):
  # About643.9s. C1 is already slightly opposite; C0 still points into the turn.
  assert damp_offset(sign*.12, sign*-.0004078, 10.77, sign*.1002) == pytest.approx(sign*.00772)
  assert damp_offset(sign*.12, 0., 10.77, sign*.2) == 0.


@pytest.mark.parametrize('sign', [-1., 1.])
@pytest.mark.parametrize('bias', [-.013, -.008, 0., .008, .013])
def test_matched_turn_and_straight_bias_cannot_reduce_offset(sign, bias):
  for desired in (0., sign*.01, sign*.05):
    assert damp_offset(sign*.4, desired, 10., 10.*desired+bias) == sign*.4


@pytest.mark.parametrize('sign', [-1., 1.])
@pytest.mark.parametrize('bias', [-.013, -.008, 0., .008, .013, .02])
def test_opposed_plan_cannot_amplify_small_yaw_bias(sign, bias):
  for desired in (-sign*.01, -sign*.1):
    assert damp_offset(sign*.4, desired, 20., sign*bias) == sign*.4


@pytest.mark.parametrize('sign', [-1., 1.])
def test_damping_begins_continuously_above_the_yaw_deadband(sign):
  assert damp_offset(sign*.4, -sign*.1, 20., sign*.020001) == pytest.approx(sign*(.4-1.4e-6))


@pytest.mark.parametrize('sign', [-1., 1.])
def test_entry_deficit_opposing_centering_and_zero_offset_are_preserved(sign):
  assert damp_offset(sign*2.75, sign*.053889, 5.283, sign*.272) == sign*2.75
  assert damp_offset(sign*-.25, sign*.001, 10., sign*.2) == sign*-.25
  assert damp_offset(0., sign*.001, 10., sign*.2) == 0.


@pytest.mark.parametrize('sign', [-1., 1.])
def test_core_keeps_heading_unchanged_and_slews_offset_independently(sign):
  baseline, damped = ModelActionController(), ModelActionController()
  for i in range(300):
    desired = sign*(.02 if i < 100 else .001)
    a = baseline.update(straight(sign*.4), desired, speed=10., dt=.01)
    b = damped.update(straight(sign*.4), desired, speed=10., dt=.01, yaw_rate=sign*.2)
    assert a.path_angle == b.path_angle
    assert a.curvature == b.curvature == a.curvature_rate == b.curvature_rate == 0.
  assert a.path_offset == pytest.approx(sign*.4)
  assert b.path_offset == pytest.approx(sign*.16)
  # No damping memory: a fresh copied pair of actuator states behaves identically.
  copied = ModelActionController()
  copied.c0, copied.c1 = damped.c0, damped.c1
  assert copied.update(straight(sign*.4), 0., speed=10., dt=.01, yaw_rate=0.) == damped.update(
    straight(sign*.4), 0., speed=10., dt=.01, yaw_rate=0.)


@pytest.mark.parametrize('yaw', [math.nan, math.inf, -math.inf, None, 'bad', 3.001, -3.001])
def test_invalid_yaw_resets_core(yaw):
  c = ModelActionController()
  c.update(straight(.4), .01, speed=10., dt=.01)
  assert c.update(straight(.4), .01, speed=10., dt=.01, yaw_rate=yaw) == FordPath()
  assert c.c0 == c.c1 == 0.
