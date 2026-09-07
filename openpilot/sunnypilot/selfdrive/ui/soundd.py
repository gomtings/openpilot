import numpy as np

from openpilot.cereal import messaging
from openpilot.selfdrive.ui.soundd import Soundd
from openpilot.sunnypilot.selfdrive.ui.custom_audio import CustomAudio


class SounddSP(Soundd):
  def __init__(self):
    super().__init__()
    self.custom_audio = CustomAudio()
    self.gear_sock = messaging.sub_sock("carState", conflate=False)

  def callback(self, data_out: np.ndarray, frames: int, time, status) -> None:
    super().callback(data_out, frames, time, status)
    alert_data = data_out[:frames, 0]
    alert_peak = float(np.max(np.abs(alert_data)))
    custom_data = self.custom_audio.player.render(frames, alert_peak > 0) * 0.25
    custom_peak = float(np.max(np.abs(custom_data)))
    gain = min(1.0, max(0.0, 1.0 - alert_peak) / max(custom_peak, 1e-9))
    alert_data += custom_data * gain

  def get_audible_alert(self, sm):
    super().get_audible_alert(sm)
    self.custom_audio.update(messaging.drain_sock(self.gear_sock, wait_for_one=False))
