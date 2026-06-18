"""
fusion.py — overlay the low-res thermal frame onto the RGB camera frame.

Pipeline: normalize thermal (degC) to 0-255 -> colormap -> upscale to RGB
frame size -> warpAffine with the calibrated registration matrix -> alpha
blend onto RGB (PLAN.md Phase 3). The affine matrix corrects for the
vertical parallax between the stacked camera/thermal mounts and must be
recalibrated (pi/tools/fusion_calibrate.py) if mounting geometry or working
distance changes.

Usage:
    from vision.fusion import Fusion
    fusion = Fusion()
    overlaid = fusion.overlay(rgb_frame, thermal_frame_c)
"""
from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import yaml

_CONFIG = Path(__file__).resolve().parents[1] / "config" / "fusion_calib.yaml"


class Fusion:
    def __init__(self, config_path: "str | Path" = _CONFIG):
        cfg = yaml.safe_load(Path(config_path).read_text(encoding="utf-8"))
        # Registration matrix: a 2x3 affine (warpAffine) OR a 3x3 homography
        # (warpPerspective). A wide-angle camera adds perspective/edge skew that a
        # similarity/affine can't fully model, so the calibration tool may store a
        # homography; we pick the warp by matrix shape. `affine` key kept for
        # backward compat; `warp` is preferred if present.
        mat = cfg.get("warp", cfg.get("affine"))
        self.warp = np.array(mat, dtype=np.float32)
        self.t_min_c = float(cfg["display"]["t_min_c"])
        self.t_max_c = float(cfg["display"]["t_max_c"])
        self.alpha = float(cfg["overlay"]["alpha"])

    # backward-compat alias (old code referenced .affine)
    @property
    def affine(self) -> np.ndarray:
        return self.warp

    def colormap(self, thermal_c: np.ndarray) -> np.ndarray:
        span = max(self.t_max_c - self.t_min_c, 1e-6)
        norm = np.clip((thermal_c - self.t_min_c) / span, 0.0, 1.0)
        gray8 = (norm * 255).astype(np.uint8)
        return cv2.applyColorMap(gray8, cv2.COLORMAP_JET)

    def register(self, thermal_up: np.ndarray, w: int, h: int) -> np.ndarray:
        """Warp the upscaled thermal image onto the RGB grid using whichever
        transform was calibrated (2x3 affine -> warpAffine, 3x3 homography ->
        warpPerspective)."""
        if self.warp.shape == (3, 3):
            return cv2.warpPerspective(thermal_up, self.warp, (w, h))
        return cv2.warpAffine(thermal_up, self.warp, (w, h))

    def overlay(self, rgb: np.ndarray, thermal_c: np.ndarray) -> np.ndarray:
        h, w = rgb.shape[:2]
        thermal_color = self.colormap(thermal_c)
        thermal_up = cv2.resize(thermal_color, (w, h), interpolation=cv2.INTER_CUBIC)
        thermal_registered = self.register(thermal_up, w, h)
        return cv2.addWeighted(rgb, 1.0 - self.alpha, thermal_registered, self.alpha, 0)


if __name__ == "__main__":
    # Synthetic smoke test (no camera/thermal hardware needed).
    rgb = np.zeros((480, 640, 3), dtype=np.uint8)
    thermal_c = np.full((24, 32), 22.0, dtype=np.float64)
    thermal_c[10:14, 14:18] = 55.0  # fake hotspot
    out = Fusion().overlay(rgb, thermal_c)
    print("overlay output shape:", out.shape, out.dtype)
