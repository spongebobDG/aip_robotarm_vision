"""
detector.py — person / motion detection for tracking + intruder events (Phase 4).

Self-contained: uses only detectors built into OpenCV (no external model files,
which this headless server doesn't have -- cv2.data and the haarcascade XMLs are
absent). Two backends, chosen in surveillance_config.yaml:

  hog_people : cv2.HOGDescriptor default people SVM, run on a downscaled frame
               (~15 fps on a Pi4). Detects full bodies -> "intruder" / track target.
  motion     : frame differencing (largest moving blob). Trivially cheap (~200 fps),
               robust in a static surveillance scene; good when you just need to
               track whatever is moving.

Both return a unified DetectionResult with the target centre in NORMALIZED
coordinates (cx, cy in 0..1) so the Tracker doesn't care about resolution.

    det = Detector()                 # reads surveillance_config.yaml
    r = det.detect(rgb_frame)
    if r.found: tracker.update(r.cx, r.cy)
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml

_CONFIG = Path(__file__).resolve().parents[1] / "config" / "surveillance_config.yaml"
_PROC_W, _PROC_H = 320, 240   # detectors run at this size for speed


@dataclass
class DetectionResult:
    found: bool
    cx: float = 0.5        # normalized 0..1
    cy: float = 0.5
    w: float = 0.0         # normalized box size
    h: float = 0.0
    conf: float = 0.0


class Detector:
    def __init__(self, config_path: "str | Path" = _CONFIG):
        cfg = yaml.safe_load(Path(config_path).read_text(encoding="utf-8"))["perception"]
        self.backend = cfg.get("detector", "hog_people")
        self._hog = None
        if self.backend != "motion":   # hog_people (or any non-motion) needs the SVM
            self._hog = cv2.HOGDescriptor()
            self._hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        self._prev_gray = None  # motion backend state

    def detect(self, rgb: np.ndarray) -> DetectionResult:
        small = cv2.resize(rgb, (_PROC_W, _PROC_H))
        if self.backend == "motion":
            return self._detect_motion(small)
        return self._detect_hog(small)

    def _detect_hog(self, small: np.ndarray) -> DetectionResult:
        rects, weights = self._hog.detectMultiScale(small, winStride=(8, 8))
        if len(rects) == 0:
            return DetectionResult(False)
        # pick the most confident detection
        i = int(np.argmax(weights)) if len(weights) else 0
        x, y, w, h = rects[i]
        return DetectionResult(
            True,
            cx=(x + w / 2) / _PROC_W, cy=(y + h / 2) / _PROC_H,
            w=w / _PROC_W, h=h / _PROC_H, conf=float(weights[i]) if len(weights) else 1.0,
        )

    def _detect_motion(self, small: np.ndarray) -> DetectionResult:
        gray = cv2.GaussianBlur(cv2.cvtColor(small, cv2.COLOR_BGR2GRAY), (5, 5), 0)
        if self._prev_gray is None:
            self._prev_gray = gray
            return DetectionResult(False)
        diff = cv2.absdiff(self._prev_gray, gray)
        self._prev_gray = gray
        _, th = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)
        th = cv2.dilate(th, None, iterations=2)
        cnts, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return DetectionResult(False)
        c = max(cnts, key=cv2.contourArea)
        area = cv2.contourArea(c)
        if area < 200:  # ignore tiny flicker/noise
            return DetectionResult(False)
        x, y, w, h = cv2.boundingRect(c)
        return DetectionResult(
            True,
            cx=(x + w / 2) / _PROC_W, cy=(y + h / 2) / _PROC_H,
            w=w / _PROC_W, h=h / _PROC_H, conf=min(area / (_PROC_W * _PROC_H), 1.0),
        )


if __name__ == "__main__":
    import time
    from vision.camera_capture import CameraCapture

    det = Detector()
    print("backend:", det.backend)
    cam = CameraCapture()
    cam.start()
    time.sleep(2)
    n, hits, t0 = 0, 0, time.time()
    while time.time() - t0 < 5:
        f = cam.latest_frame()
        if f is None:
            continue
        r = det.detect(f)
        n += 1
        if r.found:
            hits += 1
            print(f"  found @ ({r.cx:.2f},{r.cy:.2f}) conf={r.conf:.2f}")
        time.sleep(0.05)
    cam.stop()
    print(f"{n} frames in 5s ({n/5:.1f} fps), {hits} detections")
