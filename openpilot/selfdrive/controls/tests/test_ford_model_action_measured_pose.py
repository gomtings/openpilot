import math
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib.ford_model_action import ModelActionController, encode_model_action
from openpilot.selfdrive.controls.lib.ford_path import _model_path


def circle(k, length=70.0):
  s = np.linspace(0.0, length, 1401)
  if k:
    x, y = np.sin(k * s) / k, (1 - np.cos(k * s)) / k
  else:
    x, y = s, np.zeros_like(s)
  return SimpleNamespace(position=SimpleNamespace(x=x, y=y), orientation=SimpleNamespace(z=k * s))


@pytest.mark.parametrize('speed', [0.3, 4.0, 15.0, 30.0, 55.0])
@pytest.mark.parametrize('curvature', [-0.08, -0.005, 0.0, 0.005, 0.08])
def test_measured_pose_equals_requested_pose_when_tracking(speed, curvature):
  m = circle(curvature)
  # Values over the measurement's health range fall back to the same baseline.
  assert encode_model_action(m, curvature, speed, pose_yaw_rate=speed * curvature) == encode_model_action(m, curvature, speed)


@pytest.mark.parametrize('measured_curvature', [-0.02, 0.0, 0.004, 0.008, 0.02])
@pytest.mark.parametrize('speed', [4.0, 15.0, 30.0])
def test_offset_is_exact_rigid_transform_using_measured_pose(speed, measured_curvature):
  m = circle(0.008)
  out = encode_model_action(m, 0.008, speed, pose_yaw_rate=speed * measured_curvature)
  s, x, y, _ = _model_path(m)
  d = min(speed * 0.15, max(0.0, s[-1] - 7.0))
  angle = measured_curvature * d
  vehicle_x = math.sin(angle) / measured_curvature if measured_curvature else d
  vehicle_y = (1 - math.cos(angle)) / measured_curvature if measured_curvature else 0.0
  target_x = np.interp(7.0 + d, s, x)
  target_y = np.interp(7.0 + d, s, y)
  expected = -math.sin(angle) * (target_x - vehicle_x) + math.cos(angle) * (target_y - vehicle_y)
  assert out.path_offset == pytest.approx(expected, abs=1e-12)
  assert out.path_angle == encode_model_action(m, 0.008, speed).path_angle


@pytest.mark.parametrize('invalid', [None, float('nan'), float('inf'), -float('inf'), 3.01, -3.01])
def test_unavailable_measurement_retains_baseline_target(invalid):
  m = circle(0.01)
  assert encode_model_action(m, 0.01, 15.0, pose_yaw_rate=invalid) == encode_model_action(m, 0.01, 15.0)


def test_undertracking_adds_offset_and_overtracking_releases_offset():
  m = circle(0.01)
  base = encode_model_action(m, 0.01, 15.0)
  under = encode_model_action(m, 0.01, 15.0, pose_yaw_rate=0.10)
  over = encode_model_action(m, 0.01, 15.0, pose_yaw_rate=0.20)
  assert under.path_offset > base.path_offset > over.path_offset
  assert under.path_angle == base.path_angle == over.path_angle


def test_two_state_core_keeps_caps_slew_and_reset():
  c = ModelActionController()
  old = (0.0, 0.0)
  for _ in range(200):
    out = c.update(circle(0.2), 0.2, speed=20.0, dt=0.01, pose_yaw_rate=0.1)
    assert out.valid and out.curvature == out.curvature_rate == 0.0
    assert abs(c.c0 - old[0]) <= 0.0400000001
    assert abs(c.c1 - old[1]) <= 0.0050000001
    assert abs(out.path_offset) <= 5.11 and abs(out.path_angle) <= 0.5
    old = c.c0, c.c1
  assert c.__slots__ == ('c0', 'c1')
  assert not c.update(circle(0.01), 0.01, speed=15.0, dt=0.01, active=False, pose_yaw_rate=0.1).valid
  assert (c.c0, c.c1) == (0.0, 0.0)


def test_measurement_fallback_and_recovery_keep_slew_and_selected_c1():
  c = ModelActionController()
  original = ModelActionController()
  old = (0.0, 0.0)
  for index in range(180):
    curvature = 0.02 if index < 90 else -0.015
    measured = 0.05 if index < 50 else None if index < 100 else -0.03
    model = circle(curvature)
    out = c.update(model, curvature, speed=15.0, dt=0.01, pose_yaw_rate=measured)
    ref = original.update(model, curvature, speed=15.0, dt=0.01)
    assert out.path_angle == ref.path_angle
    assert abs(c.c0 - old[0]) <= 0.0400000001
    assert abs(c.c1 - old[1]) <= 0.0050000001
    assert out.curvature == out.curvature_rate == 0.0
    old = c.c0, c.c1
