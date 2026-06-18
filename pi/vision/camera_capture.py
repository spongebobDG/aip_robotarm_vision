"""
camera_capture.py — CSI camera capture for Phase 3.

Wraps the camera behind a simple "latest frame wins" interface: a background
thread continuously grabs frames and overwrites a single buffer, so a slow
consumer (fusion/AI) never blocks the capture rate and never queues up stale
frames (PLAN.md Phase 3/4: each multiprocessing stage must not block the
others). For multiprocess use, run the capture in its own process and have it
publish into a `multiprocessing.shared_memory` block or a `Queue(maxsize=1)`
with `block=False` puts — not wired up yet since there is no consumer process
until fusion.py/app/main.py exist.

Two interchangeable backends, auto-selected at start():
  - Picamera2Backend — the libcamera/Picamera2 stack (Raspberry Pi OS).
  - V4L2Backend      — pure V4L2 raw-Bayer capture for boards where Picamera2
    is unavailable (e.g. Ubuntu Server on a Pi 4, where python3-picamera2 is
    not in the apt repos). It drives the unicam raw path directly: media-ctl
    matches the capture-node format to the sensor's raw-Bayer media-bus format,
    `v4l2-ctl` streams raw frames, and we debayer + gray-world white-balance in
    OpenCV ourselves. There is NO hardware ISP/3A in this path, so we also run
    a simple software auto-gain loop against the sensor's analogue_gain control.
    Configured under `camera.v4l2` in camera_config.yaml.

Backends return HxWx3 uint8 RGB.

Run standalone to verify the camera independent of fusion/MQTT:
    python -m vision.camera_capture
"""
from __future__ import annotations

import subprocess
import threading
import time
from pathlib import Path
from typing import Optional

import yaml

_CONFIG = Path(__file__).resolve().parents[1] / "config" / "camera_config.yaml"


# --------------------------------------------------------------------------- #
# Picamera2 backend (Raspberry Pi OS / libcamera present)
# --------------------------------------------------------------------------- #
class Picamera2Backend:
    """Original Picamera2 path. Lazily imports picamera2 so it's only required
    on hosts that actually have the libcamera stack installed."""

    def __init__(self, cfg: dict):
        self._width = int(cfg["width"])
        self._height = int(cfg["height"])
        self._fps = int(cfg["fps"])
        self._fmt = cfg.get("format", "RGB888")
        self._picam2 = None

    def open(self) -> None:
        from picamera2 import Picamera2  # imported lazily: only present with libcamera

        self._picam2 = Picamera2()
        config = self._picam2.create_video_configuration(
            main={"size": (self._width, self._height), "format": self._fmt},
            controls={"FrameRate": self._fps},
        )
        self._picam2.configure(config)
        self._picam2.start()

    def read(self):
        return self._picam2.capture_array()

    def close(self) -> None:
        if self._picam2:
            self._picam2.stop()


# --------------------------------------------------------------------------- #
# V4L2 raw-Bayer backend (Ubuntu / no Picamera2)
# --------------------------------------------------------------------------- #
# OpenCV Bayer-to-BGR conversion codes, selected by the node's raw fourcc.
_BAYER_CODES = {
    "GB10": "COLOR_BayerGB2BGR",
    "BG10": "COLOR_BayerBG2BGR",
    "RG10": "COLOR_BayerRG2BGR",
    "BA10": "COLOR_BayerGR2BGR",
}


class V4L2Backend:
    """Raw-Bayer capture over the unicam node, debayered + white-balanced in
    OpenCV. Uses only stock Ubuntu packages (v4l-utils, python3-opencv); needs
    no libcamera/Picamera2."""

    def __init__(self, cfg: dict):
        import cv2  # noqa: F401  (fail fast if opencv is missing)
        import numpy as np  # noqa: F401

        v = cfg.get("v4l2", {})
        self._width = int(cfg["width"])
        self._height = int(cfg["height"])
        self._device = v.get("device", "/dev/video0")
        self._subdev = v.get("subdev", "/dev/v4l-subdev0")
        self._media = v.get("media_device", "/dev/media1")
        self._sensor_entity = v.get("sensor_entity", "ov5647 10-0036")
        self._sensor_fmt = v.get("sensor_format", "SGBRG10_1X10")
        self._pixelformat = v.get("pixelformat", "GB10")
        self._bayer_code = _BAYER_CODES.get(self._pixelformat, "COLOR_BayerGB2BGR")
        self._gain = int(v.get("gain", 200))
        self._exposure = int(v.get("exposure", 300))
        self._auto_gain = bool(v.get("auto_gain", True))
        self._target_mean = float(v.get("auto_gain_target", 110))
        self._gain_min = int(v.get("gain_min", 16))
        self._gain_max = int(v.get("gain_max", 1023))
        self._awb = bool(v.get("white_balance", True))

        # GB10/BG10/... are 16-bit containers holding 10 valid bits per pixel.
        self._frame_bytes = self._width * self._height * 2
        self._proc: Optional[subprocess.Popen] = None
        self._frames_since_ae = 0

    # -- sensor/pipeline setup via media-ctl + v4l2-ctl --------------------- #
    def _setup_pipeline(self) -> None:
        subprocess.run(
            ["media-ctl", "-d", self._media, "--set-v4l2",
             f'"{self._sensor_entity}":0[fmt:{self._sensor_fmt}/{self._width}x{self._height}]'],
            check=True, capture_output=True,
        )
        self._set_ctrl("exposure", self._exposure)
        self._set_ctrl("analogue_gain", self._gain)

    def _set_ctrl(self, name: str, value: int) -> None:
        subprocess.run(
            ["v4l2-ctl", "-d", self._subdev, f"--set-ctrl={name}={value}"],
            check=False, capture_output=True,
        )

    def open(self) -> None:
        self._setup_pipeline()
        # Continuous raw stream to stdout; we read fixed-size frames off the pipe.
        self._proc = subprocess.Popen(
            ["v4l2-ctl", "-d", self._device,
             f"--set-fmt-video=width={self._width},height={self._height},"
             f"pixelformat={self._pixelformat}",
             "--stream-mmap", "--stream-count=0", "--stream-to=-"],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0,
        )

    def _read_exact(self, n: int) -> Optional[bytes]:
        buf = bytearray()
        stdout = self._proc.stdout
        while len(buf) < n:
            chunk = stdout.read(n - len(buf))
            if not chunk:
                return None
            buf += chunk
        return bytes(buf)

    def read(self):
        import cv2
        import numpy as np

        raw = self._read_exact(self._frame_bytes)
        if raw is None:
            return None
        bayer16 = np.frombuffer(raw, dtype="<u2").reshape(self._height, self._width)
        bayer8 = (bayer16 >> 2).astype(np.uint8)  # 10-bit -> 8-bit
        bgr = cv2.cvtColor(bayer8, getattr(cv2, self._bayer_code))

        if self._awb:  # gray-world: pull B/R channel means up to G
            means = bgr.reshape(-1, 3).mean(axis=0) + 1e-6
            g = means[1]
            scale = np.array([g / means[0], 1.0, g / means[2]], dtype=np.float32)
            bgr = np.clip(bgr.astype(np.float32) * scale, 0, 255).astype(np.uint8)

        if self._auto_gain:
            self._auto_gain_step(float(bayer8.mean()))

        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    def _auto_gain_step(self, mean_brightness: float) -> None:
        # Cheap proportional auto-gain: nudge analogue_gain toward target mean
        # every ~15 frames (changing the control every frame is needless churn).
        self._frames_since_ae += 1
        if self._frames_since_ae < 15:
            return
        self._frames_since_ae = 0
        err = self._target_mean - mean_brightness
        if abs(err) < 8:
            return
        # ~0.5% gain step per brightness unit of error, clamped.
        new_gain = int(self._gain * (1.0 + 0.005 * err))
        new_gain = max(self._gain_min, min(self._gain_max, new_gain))
        if new_gain != self._gain:
            self._gain = new_gain
            self._set_ctrl("analogue_gain", new_gain)

    def close(self) -> None:
        if self._proc:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self._proc.kill()


# --------------------------------------------------------------------------- #
# Public capture object (backend-agnostic, latest-frame-wins)
# --------------------------------------------------------------------------- #
class CameraCapture:
    def __init__(self, config_path: "str | Path" = _CONFIG):
        self._cfg = yaml.safe_load(Path(config_path).read_text(encoding="utf-8"))["camera"]
        self._width = int(self._cfg["width"])
        self._height = int(self._cfg["height"])
        self._fps = int(self._cfg["fps"])

        # Software crop to reduce barrel distortion and match thermal FOV.
        crop_cfg = self._cfg.get("crop", {})
        self._crop_enabled = bool(crop_cfg.get("enabled", False))
        if self._crop_enabled:
            self._crop_left = int(crop_cfg["left"])
            self._crop_top = int(crop_cfg["top"])
            self._crop_width = int(crop_cfg["width"])
            self._crop_height = int(crop_cfg["height"])
        else:
            self._crop_left = self._crop_top = 0
            self._crop_width = self._width
            self._crop_height = self._height

        self._backend = None
        self._lock = threading.Lock()
        self._frame = None
        self._frame_count = 0
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

    def _make_backend(self):
        forced = self._cfg.get("backend", "auto")
        if forced == "v4l2":
            return V4L2Backend(self._cfg)
        if forced == "picamera2":
            return Picamera2Backend(self._cfg)
        # auto: prefer Picamera2 if the libcamera stack is importable, else V4L2.
        try:
            import picamera2  # noqa: F401
            return Picamera2Backend(self._cfg)
        except ImportError:
            return V4L2Backend(self._cfg)

    def start(self) -> None:
        self._backend = self._make_backend()
        self._backend.open()
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._backend:
            self._backend.close()

    def _loop(self) -> None:
        while not self._stop.is_set():
            frame = self._backend.read()
            if frame is None:
                continue
            if self._crop_enabled:
                frame = frame[self._crop_top:self._crop_top+self._crop_height,
                              self._crop_left:self._crop_left+self._crop_width]
            with self._lock:
                self._frame = frame
                self._frame_count += 1

    def latest_frame(self):
        """Returns the most recent frame (np.ndarray, HxWx3 RGB) or None before
        the first capture."""
        with self._lock:
            return self._frame

    @property
    def frame_count(self) -> int:
        with self._lock:
            return self._frame_count


if __name__ == "__main__":
    cap = CameraCapture()
    cap.start()
    output_size = f"{cap._crop_width}x{cap._crop_height}" if cap._crop_enabled else f"{cap._width}x{cap._height}"
    print(f"capturing {cap._width}x{cap._height} @ {cap._fps}fps, "
          f"output {output_size} (crop {'enabled' if cap._crop_enabled else 'disabled'}) — Ctrl+C to stop")
    try:
        t0 = time.time()
        last_count = 0
        while True:
            time.sleep(2.0)
            n = cap.frame_count
            fps = (n - last_count) / (time.time() - t0)
            print(f"frame_count={n}  ~{fps:.1f} fps")
            t0 = time.time()
            last_count = n
    except KeyboardInterrupt:
        pass
    finally:
        cap.stop()
