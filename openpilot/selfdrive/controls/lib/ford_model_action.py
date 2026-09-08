"""Experimental Ford C2-free controller: nearby offset and selected-action heading.

Selected only by its explicit toggle. The 7 m station and one-second scale are
engineering choices, not identified PSCM gains or physical calibration.
"""
import math
import struct

import numpy as np

from opendbc.car.ford.values import FordFlags
from openpilot.selfdrive.controls.lib.ford_path import FordPath, _model_path


OFFSET_STATION_M = 7.0
HEADING_TIME_S = 1.0
CALIBRATION_APPROVED = False
PREDICTION_TIME_S = .15  # geometric preview, not an identified actuator delay


def _predict_offset(path, c0, pose_curvature, speed):
  """Read the same path from a predicted constant-curvature vehicle pose.

  A matched constant-radius path retains its offset. Developing/flattening
  bends can move the target earlier. The core retains field limits and slew.
  Use only available geometry; shortened horizons taper prediction to zero.
  """
  station, longitudinal, lateral, _ = path
  distance = min(speed*PREDICTION_TIME_S, max(0., station[-1]-OFFSET_STATION_M))
  if distance == 0.:
    return c0
  x = float(np.interp(OFFSET_STATION_M+distance, station, longitudinal))
  y = float(np.interp(OFFSET_STATION_M+distance, station, lateral))
  rotation = pose_curvature*distance
  # (1-cos(rotation))/curvature, evaluated without cancellation or division by zero.
  translation = distance*math.sin(rotation/2)*float(np.sinc(rotation/(2*math.pi)))
  predicted = math.cos(rotation)*y-math.sin(rotation)*x+translation
  if not _finite(predicted):
    return c0
  return predicted


def _packed(value, resolution, offset):
  """Mirror Float32 carControlSP and sign-reversed CANPacker rounding."""
  value = struct.unpack("f", struct.pack("f", value))[0]
  return -(math.floor((-value - offset) / resolution + 0.5) * resolution + offset)


def _finite(*values):
  try:
    return all(math.isfinite(value) for value in values)
  except (TypeError, ValueError, OverflowError):
    return False


def encode_model_action(model, desired_curvature, speed, *, pose_yaw_rate=None):
  """Encode predicted y(7) and max(7, v*1s)*selected limited curvature.

  Preserve the reviewed core's endpoint hold when the path ends before 7 m.
  This samples the available geometry; it does not extrapolate an unseen path.
  Calibrated measured yaw predicts the vehicle pose when available; otherwise
  retain the selected-curvature prediction. This is geometric yaw feedback,
  whose sensitivity depends on the existing preview time and path distance.
  """
  if not _finite(desired_curvature, speed) or not .3 <= speed <= 55 or abs(desired_curvature) > 1:
    return FordPath()
  try:
    path = _model_path(model)
  except OverflowError:
    return FordPath()
  if path is None or not all(_finite(*values) for values in path):
    return FordPath()
  station, _, lateral, _ = path
  c0 = float(np.interp(min(OFFSET_STATION_M, station[-1]), station, lateral))
  pose_curvature = desired_curvature
  if pose_yaw_rate is not None and _finite(pose_yaw_rate) and abs(pose_yaw_rate) <= 3:
    pose_curvature = pose_yaw_rate / speed
  c0 = _predict_offset(path, c0, pose_curvature, speed)
  c1 = max(OFFSET_STATION_M, speed*HEADING_TIME_S)*desired_curvature
  return FordPath(True, c0, c1, 0., 0.) if _finite(c0, c1) else FordPath()


class ModelActionController:
  """Only two control states: unquantized, independently slewed C0 and C1.

  Freshness and engagement belong to the caller. Raw Ford yaw checks input
  health only. A separate calibrated yaw input can change the offset forecast;
  heading always follows the selected limited curvature.
  """
  __slots__ = ('c0', 'c1')

  def __init__(self):
    self.reset()

  def reset(self):
    self.c0 = self.c1 = 0.

  def update(self, model, desired_curvature, *, speed, dt, yaw_rate=0., active=True, valid=True, pose_yaw_rate=None):
    # Raw Ford yaw remains an input-health check, not a pose measurement.
    if not active or not valid or not _finite(dt, yaw_rate) or not .002 <= dt <= .1 or abs(yaw_rate) > 3:
      self.reset()
      return FordPath()
    target = encode_model_action(model, desired_curvature, speed, pose_yaw_rate=pose_yaw_rate)
    if not target.valid:
      self.reset()
      return FordPath()
    c0 = float(np.clip(target.path_offset, -5.11, 5.11))
    c1 = float(np.clip(target.path_angle, -.5, .5))
    self.c0 += float(np.clip(c0-self.c0, -4.*dt, 4.*dt))
    self.c1 += float(np.clip(c1-self.c1, -.5*dt, .5*dt))
    return FordPath(True, _packed(self.c0, .01, -5.12), _packed(self.c1, .0005, -.5), 0., 0.)


class FordModelActionController:
  """Input adapter for the opt-in selected-action controller.

  controlsd owns upstream selection/limiting and service health. This adapter
  checks ages and clock order, then supplies elapsed time to the two-state
  core. Pose age gates measured-motion use; diagnostics do not feed back into
  the core. Raw model geometry is checked even at a repeated model timestamp.

  Raw Ford yaw supplies diagnostics and input-health checks. Fresh, healthy
  calibrated motion supplies pose prediction; unavailable motion falls back
  to the existing requested-pose forecast without resetting the slew states.
  PSCM status and driver torque are not control-law inputs.
  """
  def __init__(self):
    self.core = ModelActionController()
    self.reset()

  def reset(self, status='inactive'):
    self.core.reset()
    self.last_time = self.last_measurement_time = self.last_model_time = None
    self.diagnostics = {'status': status, 'hypothesis': 'model-action-measured-pose-v6',
                        'calibration_approved': CALIBRATION_APPROVED, 'command': (0., 0., 0., 0.)}

  def update(self, model, desired_curvature, *, yaw_rate, speed, now, measurement_time, model_time, reference_time,
             active, valid=True, pose_yaw_rate=None, pose_time=None, pose_valid=False):
    reason = None
    if not active:
      reason = 'inactive'
    elif not valid:
      reason = 'invalid_service'
    elif not _finite(desired_curvature, yaw_rate, speed, now, measurement_time, model_time, reference_time):
      reason = 'nonfinite'
    elif not all(-.005 <= now - timestamp <= .15 for timestamp in (measurement_time, model_time, reference_time)):
      reason = 'stale_input'
    elif not .3 <= speed <= 55 or abs(yaw_rate) > 3 or abs(desired_curvature) > 1:
      reason = 'input_range'
    if reason is not None:
      self.reset(reason)
      return FordPath()

    dt = .01 if self.last_time is None else now - self.last_time
    if not .002 <= dt <= .1 or (self.last_measurement_time is not None and measurement_time < self.last_measurement_time) or (
      self.last_model_time is not None and model_time < self.last_model_time
    ):
      self.reset('timing_reset')
      return FordPath()
    pose_age = now - pose_time if _finite(pose_time) else None
    if not _finite(pose_age):
      pose_age = None
    use_pose = (pose_valid and pose_yaw_rate is not None and pose_age is not None and
                _finite(pose_yaw_rate) and abs(pose_yaw_rate) <= 3 and -.005 <= pose_age <= .15)
    command = self.core.update(model, desired_curvature, speed=speed, dt=dt, yaw_rate=yaw_rate,
                               pose_yaw_rate=pose_yaw_rate if use_pose else None)
    if not command.valid:
      self.reset('invalid_path')
      return command
    self.last_time, self.last_measurement_time, self.last_model_time = now, measurement_time, model_time
    self.diagnostics = {'status': 'active', 'hypothesis': 'model-action-measured-pose-v6',
                        'calibration_approved': CALIBRATION_APPROVED, 'desired_curvature': desired_curvature,
                        'yaw_rate': yaw_rate, 'pose_source': 'measured' if use_pose else 'requested',
                        'pose_yaw_rate': pose_yaw_rate if use_pose else None, 'pose_age': pose_age,
                        'model_age': now - model_time, 'measurement_age': now - measurement_time, 'reference_age': now - reference_time,
                        'dt': dt, 'offset_request': self.core.c0, 'heading_request': self.core.c1,
                        'command': (command.path_offset, command.path_angle, 0., 0.)}
    return command


def select_model_action_controller(CP, enabled, previous_controller):
  """The separate default-off toggle takes priority on the CAN FD Lightning."""
  compatible = CP.brand == 'ford' and CP.flags & FordFlags.CANFD and CP.carFingerprint == 'FORD_F_150_LIGHTNING_MK1'
  if enabled and compatible:
    return FordModelActionController()
  return previous_controller
