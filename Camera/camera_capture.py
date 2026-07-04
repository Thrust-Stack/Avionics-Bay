"""
Avionics Camera - Raspberry Pi 5 + Camera Module 3

Captures live video and/or still frames using Picamera2 (the libcamera
stack). This is the correct stack for the Pi 5 + Camera Module 3 — the
legacy raspistill/raspivid tools and the old `picamera` library do NOT
work on a Pi 5.

Setup (Raspberry Pi OS Bookworm or later):
  1. Connect the Camera Module 3 ribbon to one of the Pi 5's two CSI ports
  2. sudo apt update && sudo apt install -y python3-picamera2
     (Picamera2 is installed via apt on Raspberry Pi OS, NOT pip)
  3. Verify the camera is detected:  rpicam-hello --list-cameras
  4. python3 camera_capture.py --mode video

Usage:
  python3 camera_capture.py --mode video                 # record until Ctrl+C
  python3 camera_capture.py --mode video --duration 30   # record 30 seconds
  python3 camera_capture.py --mode frames                # JPEG every FRAME_INTERVAL_S
  python3 camera_capture.py --mode both                  # video + periodic JPEGs
  python3 camera_capture.py --mode still                 # one full-res still, then exit

Each session is saved to logs/camera_YYYYMMDD_HHMMSS/ at the repo root,
alongside the flight logs.

Other programs can also drive this module directly:

  from camera_capture import CameraCapture
  cam = CameraCapture()
  if cam.start():                # False (never an exception) if no camera
      cam.start_video()
      cam.start_frame_grab()
      ...
      cam.stop()

Every hardware failure path prints a [CAM] message and disables the camera
instead of raising, so a missing/broken camera can never take down the rest
of the avionics stack.
"""

import argparse
import datetime
import os
import sys
import threading
import time

# Picamera2 is imported lazily so this module stays importable on machines
# without the camera stack (dev laptops, CI). See _import_picamera2().
_PICAMERA2_IMPORT_ERROR = None

# ============================================================
# CONFIGURATION
# ============================================================
CAMERA_INDEX = 0              # Pi 5 has two CSI ports; 0 = first detected camera

# Video: Camera Module 3 sensor modes top out at 2304x1296@56fps.
# 1920x1080@30 is a safe default; the Pi 5 has no hardware H.264 encoder,
# so encoding runs in software (fine at 1080p30, heavy at 1296p56).
VIDEO_RESOLUTION = (1920, 1080)
VIDEO_FRAMERATE = 30
VIDEO_BITRATE = 10_000_000    # bits/sec

# Still capture (--mode still) uses the full 4608x2592 sensor resolution.

# Continuous frame grab: how often to save a JPEG, in seconds.
FRAME_INTERVAL_S = 1.0

# Autofocus: Camera Module 3 has a motorized lens. "continuous" keeps it
# hunting for focus; set to None to leave the lens at its default position.
AUTOFOCUS_MODE = "continuous"

# Seconds to let auto-exposure/white-balance settle after the sensor starts.
WARMUP_S = 2.0
# ============================================================

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# Camera sessions live beside the flight logs in <repo root>/logs/
LOGS_DIR = os.path.join(os.path.dirname(SCRIPT_DIR), "logs")


def _import_picamera2():
    """Import the picamera2 classes on first use. Returns True on success."""
    global Picamera2, H264Encoder, FfmpegOutput, FileOutput, _PICAMERA2_IMPORT_ERROR
    if _PICAMERA2_IMPORT_ERROR is not None:
        return False
    if "Picamera2" in globals():
        return True
    try:
        from picamera2 import Picamera2
        from picamera2.encoders import H264Encoder
        from picamera2.outputs import FfmpegOutput, FileOutput
        return True
    except ImportError as e:
        _PICAMERA2_IMPORT_ERROR = str(e)
        return False


def now_stamp():
    return datetime.datetime.now().strftime("%Y%m%d_%H%M%S")


class CameraCapture:
    """
    Owns one Picamera2 camera. All methods are safe to call even if the
    camera failed to initialize — they print a [CAM] message and return
    False/None instead of raising.
    """

    def __init__(self, output_dir=None, resolution=VIDEO_RESOLUTION,
                 framerate=VIDEO_FRAMERATE, bitrate=VIDEO_BITRATE):
        self.resolution = resolution
        self.framerate = framerate
        self.bitrate = bitrate
        self.output_dir = output_dir  # created on start()

        self._picam2 = None
        self._encoder = None
        self._video_path = None
        self._frame_thread = None
        self._stop_event = threading.Event()
        self._frame_lock = threading.Lock()
        self._frame_count = 0

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    @property
    def running(self):
        return self._picam2 is not None

    @property
    def recording(self):
        return self._encoder is not None

    def start(self):
        """
        Initialize and start the camera sensor. Returns True on success,
        False if picamera2 is missing or no camera is attached.
        """
        if self.running:
            return True

        if not _import_picamera2():
            print("[CAM] Picamera2 not available — camera disabled.")
            print(f"[CAM]   ({_PICAMERA2_IMPORT_ERROR})")
            print("[CAM]   On Raspberry Pi OS install it with:")
            print("[CAM]     sudo apt install -y python3-picamera2")
            return False

        try:
            cameras = Picamera2.global_camera_info()
        except Exception as e:
            print(f"[CAM] Could not enumerate cameras — camera disabled: {e}")
            return False

        if len(cameras) <= CAMERA_INDEX:
            print("[CAM] No camera detected — camera disabled.")
            print("[CAM]   Check the ribbon cable, then run: rpicam-hello --list-cameras")
            return False

        try:
            self._picam2 = Picamera2(CAMERA_INDEX)
            config = self._picam2.create_video_configuration(
                main={"size": self.resolution},
                controls={"FrameRate": self.framerate},
            )
            self._picam2.configure(config)
            self._picam2.start()
        except Exception as e:
            print(f"[CAM] Failed to start camera — camera disabled: {e}")
            self._teardown()
            return False

        self._set_autofocus()

        if self.output_dir is None:
            self.output_dir = os.path.join(LOGS_DIR, f"camera_{now_stamp()}")
        os.makedirs(self.output_dir, exist_ok=True)

        model = cameras[CAMERA_INDEX].get("Model", "unknown")
        print(f"[CAM] Camera started: {model}  "
              f"{self.resolution[0]}x{self.resolution[1]} @ {self.framerate}fps")
        print(f"[CAM] Session dir: {self.output_dir}")

        time.sleep(WARMUP_S)  # let AE/AWB settle before first capture
        return True

    def _set_autofocus(self):
        if AUTOFOCUS_MODE != "continuous":
            return
        try:
            from libcamera import controls
            self._picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})
            print("[CAM] Continuous autofocus enabled")
        except Exception:
            # Older stack or fixed-focus module — not fatal.
            print("[CAM] Continuous autofocus not available (continuing without)")

    def stop(self):
        """Stop frame grabbing, recording, and the camera. Always safe to call."""
        self._stop_event.set()
        if self._frame_thread is not None:
            self._frame_thread.join(timeout=5)
            self._frame_thread = None
        self.stop_video()
        self._teardown()

    def _teardown(self):
        if self._picam2 is not None:
            try:
                self._picam2.stop()
            except Exception:
                pass
            try:
                self._picam2.close()
            except Exception:
                pass
            self._picam2 = None

    # ------------------------------------------------------------------
    # Video recording
    # ------------------------------------------------------------------

    def start_video(self, filename=None):
        """
        Begin recording H.264 video to an .mp4 in the session directory.
        Returns the output path, or None on failure.
        """
        if not self.running:
            print("[CAM] start_video ignored — camera not running")
            return None
        if self.recording:
            print(f"[CAM] Already recording: {self._video_path}")
            return self._video_path

        path = os.path.join(self.output_dir, filename or f"video_{now_stamp()}.mp4")
        try:
            encoder = H264Encoder(bitrate=self.bitrate)
            self._picam2.start_encoder(encoder, FfmpegOutput(path))
        except Exception as e:
            # FfmpegOutput needs the ffmpeg binary (preinstalled on Raspberry
            # Pi OS). Fall back to a raw .h264 stream if it isn't there.
            print(f"[CAM] MP4 output failed ({e}) — falling back to raw .h264")
            path = path.rsplit(".", 1)[0] + ".h264"
            try:
                encoder = H264Encoder(bitrate=self.bitrate)
                self._picam2.start_encoder(encoder, FileOutput(path))
            except Exception as e2:
                print(f"[CAM] Could not start recording: {e2}")
                return None

        self._encoder = encoder
        self._video_path = path
        print(f"[CAM] Recording: {path}")
        return path

    def stop_video(self):
        """Stop the current recording, if any. Returns the video path or None."""
        if not self.recording:
            return None
        path = self._video_path
        try:
            self._picam2.stop_encoder()
        except Exception as e:
            print(f"[CAM] Error stopping encoder: {e}")
        self._encoder = None
        self._video_path = None
        print(f"[CAM] Recording saved: {path}")
        return path

    # ------------------------------------------------------------------
    # Frame capture
    # ------------------------------------------------------------------

    def capture_frame(self, path=None):
        """
        Save one JPEG from the live stream (works while recording video).
        Returns the file path, or None on failure.
        """
        if not self.running:
            print("[CAM] capture_frame ignored — camera not running")
            return None
        if path is None:
            with self._frame_lock:
                self._frame_count += 1
                path = os.path.join(self.output_dir,
                                    f"frame_{self._frame_count:06d}.jpg")
        try:
            self._picam2.capture_file(path)
            return path
        except Exception as e:
            print(f"[CAM] Frame capture failed: {e}")
            return None

    def get_latest_frame(self):
        """
        Return the newest frame as a numpy array (for onboard vision use),
        or None on failure. Note: the array is the raw main-stream format
        (typically 4-channel XBGR), not RGB JPEG data.
        """
        if not self.running:
            return None
        try:
            return self._picam2.capture_array()
        except Exception as e:
            print(f"[CAM] Frame read failed: {e}")
            return None

    def start_frame_grab(self, interval_s=FRAME_INTERVAL_S):
        """Background thread: save a JPEG every interval_s seconds until stop()."""
        if not self.running:
            print("[CAM] start_frame_grab ignored — camera not running")
            return False
        if self._frame_thread is not None:
            return True

        def _grab_loop():
            while not self._stop_event.wait(interval_s):
                path = self.capture_frame()
                if path:
                    print(f"[CAM] Frame saved: {os.path.basename(path)}")

        self._stop_event.clear()
        self._frame_thread = threading.Thread(target=_grab_loop, daemon=True)
        self._frame_thread.start()
        print(f"[CAM] Frame grab every {interval_s}s")
        return True

    # ------------------------------------------------------------------
    # Full-resolution still (uses still configuration; camera must be idle)
    # ------------------------------------------------------------------

    def capture_still(self, path=None):
        """
        Reconfigure for a full-resolution (4608x2592) still, capture it, and
        return to video configuration. Do not call while recording video.
        Returns the file path, or None on failure.
        """
        if not self.running:
            print("[CAM] capture_still ignored — camera not running")
            return None
        if self.recording:
            print("[CAM] capture_still ignored — stop video recording first")
            return None
        if path is None:
            path = os.path.join(self.output_dir, f"still_{now_stamp()}.jpg")
        try:
            still_config = self._picam2.create_still_configuration()
            self._picam2.switch_mode_and_capture_file(still_config, path)
            print(f"[CAM] Still saved: {path}")
            return path
        except Exception as e:
            print(f"[CAM] Still capture failed: {e}")
            return None


def main():
    parser = argparse.ArgumentParser(
        description="Record video / capture frames from the Pi Camera Module 3")
    parser.add_argument("--mode", choices=["video", "frames", "both", "still"],
                        default="video",
                        help="video: record mp4; frames: periodic JPEGs; "
                             "both: video + JPEGs; still: one full-res photo")
    parser.add_argument("--duration", type=float, default=None,
                        help="Seconds to run before stopping (default: until Ctrl+C)")
    parser.add_argument("--interval", type=float, default=FRAME_INTERVAL_S,
                        help=f"Seconds between JPEGs in frames/both mode "
                             f"(default: {FRAME_INTERVAL_S})")
    parser.add_argument("--output", default=None,
                        help="Session output directory (default: logs/camera_<timestamp>)")
    args = parser.parse_args()

    print("==========================================")
    print("  Avionics Camera")
    print("==========================================")
    print(f"  Mode:    {args.mode}")
    print(f"  Length:  {'until Ctrl+C' if args.duration is None else f'{args.duration}s'}")
    print("==========================================\n")

    cam = CameraCapture(output_dir=args.output)
    if not cam.start():
        sys.exit(1)

    try:
        if args.mode == "still":
            ok = cam.capture_still() is not None
            cam.stop()
            sys.exit(0 if ok else 1)

        if args.mode in ("video", "both"):
            if cam.start_video() is None:
                cam.stop()
                sys.exit(1)
        if args.mode in ("frames", "both"):
            cam.start_frame_grab(args.interval)

        if args.duration is not None:
            time.sleep(args.duration)
        else:
            while True:
                time.sleep(1)

    except KeyboardInterrupt:
        print("\n\nStopped.")
    finally:
        cam.stop()
        print(f"Session saved to: {cam.output_dir}")


if __name__ == "__main__":
    main()
