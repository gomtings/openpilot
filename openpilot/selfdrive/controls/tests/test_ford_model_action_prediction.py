"""Geometric prediction checks; these do not simulate the Ford steering plant."""
import math

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib.ford_model_action import encode_model_action
from openpilot.selfdrive.controls.tests.test_ford_model_action import circle, make_model, straight


def geometric_offset(model):
  station = np.r_[0., np.cumsum(np.hypot(np.diff(model.position.x), np.diff(model.position.y)))]
  return float(np.interp(7., station, model.position.y))


@pytest.mark.parametrize('sign', [-1., 1.])
def test_developing_bend_advances_offset_with_bounded_request(sign):
  x = np.linspace(0., 30., 3001)
  model = make_model(x, sign*.001*x**3, np.zeros_like(x))
  base = geometric_offset(model)
  target = encode_model_action(model, 0., 10.)
  assert sign*target.path_offset > sign*base+.05
  assert target.path_offset == pytest.approx(base*1.25)
  assert target.path_angle == target.curvature == target.curvature_rate == 0.


@pytest.mark.parametrize('sign', [-1., 1.])
def test_flattening_bend_reduces_offset_before_the_near_path_disappears(sign):
  station = np.linspace(0., 30., 3001)
  heading = sign*.01*np.minimum(station, 5.)
  ds = station[1]-station[0]
  x = np.r_[0., np.cumsum(np.cos((heading[:-1]+heading[1:])/2)*ds)]
  y = np.r_[0., np.cumsum(np.sin((heading[:-1]+heading[1:])/2)*ds)]
  model = make_model(x, y, heading)
  base = geometric_offset(model)
  target = encode_model_action(model, sign*.01, 20.)
  assert 0. < sign*target.path_offset < sign*base-.02
  assert target.path_angle == pytest.approx(sign*.2)


@pytest.mark.parametrize('speed', [.3, 5., 20., 55.])
@pytest.mark.parametrize('curvature', [-.05, -.01, .01, .05])
def test_matched_constant_circle_is_not_given_a_blanket_gain_increase(speed, curvature):
  model = circle(curvature)
  assert encode_model_action(model, curvature, speed).path_offset == pytest.approx(geometric_offset(model), abs=1e-4)


@pytest.mark.parametrize('offset', [-.4, 0., .4])
@pytest.mark.parametrize('curvature', [-1e-300, 0., 1e-300])
def test_straight_centering_and_near_zero_curvature_remain_well_conditioned(offset, curvature):
  for speed in (.3, 20., 55.):
    assert encode_model_action(straight(offset), curvature, speed).path_offset == pytest.approx(offset, abs=1e-12)


@pytest.mark.parametrize('offset', [-4., -.4, -.001, 0., .001, .4, 4.])
def test_prediction_cannot_reverse_or_swamp_existing_centering(offset):
  for curvature in (-1., -.1, .1, 1.):
    value = encode_model_action(straight(offset), curvature, 55.).path_offset
    assert abs(value-offset) <= min(.15, .25*abs(offset))+1e-12
    assert value*offset >= 0.


def test_short_horizon_holds_endpoint_and_available_prediction_tapers_to_zero():
  for length in (1., 6.99, 7.):
    assert encode_model_action(make_model([0., length], [.4, .4], [0., 0.]), .01, 20.).path_offset == .4
  near = encode_model_action(make_model([0., 7.000001], [.4, .4], [0., 0.]), .01, 20.).path_offset
  assert abs(near-.4) < 1e-6


def test_unrepresentable_predicted_geometry_keeps_the_valid_current_offset():
  large = 1.79e308
  model = make_model([-large, np.nextafter(-large, 0.)], [large, large], [0., 0.])
  target = encode_model_action(model, .3, 20.)
  assert target.valid and math.isfinite(target.path_offset)
  assert target.path_offset == large
