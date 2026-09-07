# Custom Audio

Put short **48 kHz, mono, 16-bit PCM WAV** files in `/data/media/0/custom_audio/` (maximum five minutes each).

Each gear change plays `<gear>.wav`: `park.wav`, `drive.wav`, `sport.wav`, `reverse.wav`, `neutral.wav`, `low.wav`, `brake.wav`, `eco.wav`, or `manumatic.wav`.

- Entering a gear starts its audio from the beginning at full volume, with no fade-in.
- Changing gears cuts the previous audio without a fade-out. If the new file is missing or invalid, playback stays silent.
- Returning to a previous gear restarts its audio from the beginning. Staying in the same gear does not replay it.
- Safety alerts pause custom audio immediately. After the alert, it resumes from its saved position with a **150 ms fade-in**. Audio triggered during an alert waits, then fades in.
- Startup and recovery from unknown/invalid gear data or data gaps establish a silent baseline.

No settings or dismiss control. Replace or remove files to change future playback.

Convert files with:

```sh
ffmpeg -i input_audio -ar 48000 -ac 1 -c:a pcm_s16le drive.wav
```
