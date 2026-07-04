# Camera — Raspberry Pi 5 + Camera Module 3

Live video / frame capture for the avionics bay, built on **Picamera2**
(the libcamera stack). The legacy `raspistill` / `raspivid` tools and the
old `picamera` pip library do **not** work on a Pi 5 — Picamera2 is the
supported path.

`camera_capture.py` is fully self-contained: nothing else in the repo
imports it, and it never raises on camera failure, so a missing or broken
camera cannot take down GPSReader or the roll-control programs.

---

## Dependencies (on the Pi 5)

| Dependency | Install | Notes |
|------------|---------|-------|
| Picamera2  | `sudo apt install -y python3-picamera2` | **apt, not pip.** Preinstalled on full Raspberry Pi OS images; the pip wheel misses the libcamera bindings. |
| ffmpeg     | `sudo apt install -y ffmpeg` | Preinstalled on Raspberry Pi OS. Used to wrap H.264 into `.mp4`; without it the script falls back to raw `.h264`. |

No changes to the Python deps used by the rest of the repo
(`pyserial`, `pynmea2`, `openpyxl` are unaffected).

> If you run the avionics code inside a venv, create it with
> `python3 -m venv --system-site-packages venv` so the apt-installed
> picamera2 is visible inside it.

---

## Usage

```bash
python3 camera_capture.py --mode video                 # record until Ctrl+C
python3 camera_capture.py --mode video --duration 30   # 30-second clip
python3 camera_capture.py --mode frames --interval 0.5 # JPEG every 0.5 s
python3 camera_capture.py --mode both                  # video + periodic JPEGs
python3 camera_capture.py --mode still                 # one 4608x2592 photo
```

Output goes to `logs/camera_YYYYMMDD_HHMMSS/` at the repo root, next to
the flight logs. Resolution / framerate / bitrate / autofocus are constants
in the CONFIGURATION block at the top of `camera_capture.py`.

From another program:

```python
from camera_capture import CameraCapture   # or Camera.camera_capture from repo root

cam = CameraCapture()
if cam.start():            # returns False instead of raising if no camera
    cam.start_video()      # H.264/MP4 recording
    cam.start_frame_grab() # periodic JPEGs alongside the video
    frame = cam.get_latest_frame()  # numpy array for onboard vision
    cam.stop()
```

---

## First hardware test checklist

1. With the Pi **powered off**, connect the Camera Module 3 ribbon to a CSI
   port (contacts facing the correct way per the port markings).
2. Boot, then confirm the OS sees the camera:
   ```bash
   rpicam-hello --list-cameras     # should list "imx708"
   ```
   If nothing is listed, reseat the ribbon at both ends before debugging
   software.
3. Confirm Picamera2 is present: `python3 -c "import picamera2; print('ok')"`
4. Smoke test the module: `python3 camera_capture.py --mode still`
   → a JPEG should appear under `logs/camera_*/`.
5. Video test: `python3 camera_capture.py --mode video --duration 10`
   → play the resulting `.mp4` (VLC on the Pi, or copy it off).
6. Failure-handling test: unplug the camera (Pi off, then boot without it)
   and run the script again — it must print `[CAM] No camera detected` and
   exit cleanly, without a traceback.

## Assumptions to verify on real hardware

- **Software H.264 load**: the Pi 5 has no hardware video encoder;
  1080p30 software encode is known-good, but watch CPU/temps if the roll
  controller runs at the same time. Drop `VIDEO_RESOLUTION` to 1280x720 if
  needed.
- **`start_encoder(encoder, output)` / `FfmpegOutput` API**: matches
  Picamera2 as shipped on Raspberry Pi OS Bookworm (0.3.x). If the Pi has a
  very old image, `sudo apt full-upgrade` first.
- **Autofocus**: continuous AF is enabled by default. For flight you likely
  want it **off** (lens hunting during boost is useless) — set
  `AUTOFOCUS_MODE = None` in the CONFIGURATION block.
- **Camera port index**: `CAMERA_INDEX = 0` assumes one camera; if you use
  the second CSI port and it enumerates differently, set it to 1.
