import numpy as np

from openpilot.cereal import messaging
from openpilot.selfdrive.ui.soundd import AudibleAlert, Soundd
from openpilot.sunnypilot.selfdrive.ui.custom_audio import CustomAudio


class SounddSP(Soundd):
  def __init__(self):
    super().__init__()
    self.custom_audio = CustomAudio()
    self.gear_sock = messaging.sub_sock("carState", conflate=False)

  def callback(self, data_out: np.ndarray, frames: int, time, status) -> None:
    alert_active = self.current_alert != AudibleAlert.none
    super().callback(data_out, frames, time, status)
    alert_active = alert_active or self.current_alert != AudibleAlert.none or bool(np.any(data_out[:frames, 0]))
    custom_data = self.custom_audio.player.render(frames, alert_active)
    if not alert_active:
      data_out[:frames, 0] = custom_data

  def get_audible_alert(self, sm):
    super().get_audible_alert(sm)
    self.custom_audio.update(messaging.drain_sock(self.gear_sock, wait_for_one=False))
