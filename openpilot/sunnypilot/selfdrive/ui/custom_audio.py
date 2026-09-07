from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import time
import wave

import numpy as np

from openpilot.common.swaglog import cloudlog

AUDIO_DIR = Path("/data/media/0/custom_audio")
SAMPLE_RATE = 48000
MAX_SECONDS = 300
GEAR_TIMEOUT = 0.5
RESUME_FRAMES = int(0.15 * SAMPLE_RATE)


class GearTransition:
  def __init__(self):
    self.previous = None
    self.last_time = None

  def update(self, gear, timestamp, valid=True):
    if not valid or gear == "unknown":
      self.previous = self.last_time = None
      return None
    if self.last_time is None or not 0 < timestamp - self.last_time <= GEAR_TIMEOUT:
      self.previous = None
    transition = gear if self.previous is not None and gear != self.previous else None
    self.previous = gear
    self.last_time = timestamp
    return transition


def load_audio(filename):
  path = AUDIO_DIR / filename
  if not path.is_file():
    return None
  with wave.open(str(path), "rb") as wav:
    if (wav.getnchannels(), wav.getsampwidth(), wav.getframerate(), wav.getcomptype()) != (1, 2, SAMPLE_RATE, "NONE"):
      raise ValueError("Custom audio requires 48 kHz mono 16-bit PCM WAV")
    count = wav.getnframes()
    if not 0 < count <= MAX_SECONDS * SAMPLE_RATE:
      raise ValueError("Custom audio must be between 0 and 300 seconds")
    raw = wav.readframes(count)
    if len(raw) != count * 2:
      raise ValueError("Truncated custom audio")
    return np.frombuffer(raw, dtype="<i2").astype(np.float32) / 32768


class AudioPlayer:
  def __init__(self):
    self.command = None
    self.seen_command = None
    self.samples = np.empty(0, dtype=np.float32)
    self.position = 0
    self.gain = 1.0
    self.paused = False

  def render(self, frames, alert_active):
    command = self.command
    if command is not self.seen_command:
      self.seen_command = command
      self.samples = command if command is not None else np.empty(0, dtype=np.float32)
      self.position = 0
      self.gain = 1.0
      self.paused = False
    out = np.zeros(frames, dtype=np.float32)
    if self.position == len(self.samples):
      return out
    if alert_active:
      self.paused = True
      self.gain = 0.0
      return out
    if self.paused:
      self.paused = False
      self.gain = 0.0
    n = min(frames, len(self.samples) - self.position)
    gains = np.minimum(1, self.gain + np.arange(n) / RESUME_FRAMES)
    out[:n] = self.samples[self.position:self.position + n] * gains
    self.position += n
    if n:
      self.gain = float(gains[-1])
    return out


class CustomAudio:
  def __init__(self):
    self.player = AudioPlayer()
    self.transition = GearTransition()
    self.loader = ThreadPoolExecutor(max_workers=1, thread_name_prefix="custom-audio-source")
    self.pending = None

  def update(self, messages):
    now = time.monotonic()
    event = None
    if self.transition.last_time is not None and now - self.transition.last_time > GEAR_TIMEOUT:
      self.transition.update(None, now, False)
    for msg in messages:
      timestamp = msg.logMonoTime / 1e9
      valid = msg.valid and msg.carState.canValid and msg.carState.gearShifter != "unknown" and 0 <= now - timestamp <= GEAR_TIMEOUT
      transition = self.transition.update(str(msg.carState.gearShifter), timestamp, valid)
      if not valid:
        event = None
      elif transition:
        event = transition

    if event:
      self.player.command = None
      if self.pending:
        self.pending.cancel()
      self.pending = self.loader.submit(load_audio, f"{event}.wav")

    if self.pending and self.pending.done():
      try:
        samples = self.pending.result()
        if samples is not None:
          self.player.command = samples
      except Exception:
        cloudlog.exception("Unable to load custom audio")
      self.pending = None

