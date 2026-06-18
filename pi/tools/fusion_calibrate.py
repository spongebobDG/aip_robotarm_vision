"""
fusion_calibrate.py — interactive RGB<->thermal registration (Phase 3).

Heat several point targets (e.g. a soldering iron tip, a warm mug) spread
across the shared field of view, run this at your real operating distance
(camera/thermal mounts are stacked, so vertical parallax is distance-
dependent), then click matching points in the RGB and thermal-colormap
windows in the SAME order. Computes a 2x3 affine via
cv2.estimateAffinePartial2D and writes it into fusion_calib.yaml's `affine`
key (other keys are preserved).

Run:   python -m tools.fusion_calibrate [--points N]      (from pi/ directory)
Keys (in either image window): click to add a point, 'u' undo last, 'q' quit
once both windows have >= N points.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # allow `vision`/`comms` import

import cv2
import numpy as np
import yaml

from comms.thermal_serial import ThermalSerial  # noqa: E402
from vision.camera_capture import CameraCapture  # noqa: E402
from vision.fusion import Fusion, _CONFIG as FUSION_CONFIG  # noqa: E402

_rgb_points: list[tuple[int, int]] = []
_thermal_points: list[tuple[int, int]] = []


def _on_click(points: list) -> "callable":
    def handler(event, x, y, flags, userdata):
        if event == cv2.EVENT_LBUTTONDOWN:
            points.append((x, y))
            print(f"  point added: ({x}, {y})  total={len(points)}")

    return handler


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--points", type=int, default=4, help="point pairs to collect (>=3)")
    args = ap.parse_args()

    cam = CameraCapture()
    therm = ThermalSerial()
    cam.start()
    therm.start()

    fusion = Fusion()
    print("waiting for first camera frame...")
    rgb = None
    while rgb is None:
        rgb = cam.latest_frame()
    thermal_c = therm.read_frame()
    thermal_color = cv2.resize(
        fusion.colormap(thermal_c), (rgb.shape[1], rgb.shape[0]), interpolation=cv2.INTER_CUBIC
    )

    cv2.namedWindow("rgb")
    cv2.namedWindow("thermal")
    cv2.setMouseCallback("rgb", _on_click(_rgb_points))
    cv2.setMouseCallback("thermal", _on_click(_thermal_points))

    print(__doc__)
    try:
        while True:
            disp_rgb = rgb.copy()
            disp_th = thermal_color.copy()
            for img, pts in ((disp_rgb, _rgb_points), (disp_th, _thermal_points)):
                for p in pts:
                    cv2.circle(img, p, 5, (0, 255, 0), -1)
            cv2.imshow("rgb", disp_rgb)
            cv2.imshow("thermal", disp_th)
            key = cv2.waitKey(30) & 0xFF
            if key == ord("u"):
                if _rgb_points:
                    _rgb_points.pop()
                if _thermal_points:
                    _thermal_points.pop()
            elif key == ord("q"):
                break
    finally:
        cv2.destroyAllWindows()
        cam.stop()
        therm.stop()

    n = min(len(_rgb_points), len(_thermal_points))
    if n < 3 or n < args.points:
        print(f"need >= max(3, --points) matched point pairs, got {n} — aborting without saving")
        return

    src = np.array(_thermal_points[:n], dtype=np.float32)
    dst = np.array(_rgb_points[:n], dtype=np.float32)
    affine, inliers = cv2.estimateAffinePartial2D(src, dst)
    if affine is None:
        print("estimateAffinePartial2D failed to find a solution — aborting without saving")
        return

    cfg = yaml.safe_load(FUSION_CONFIG.read_text(encoding="utf-8"))
    cfg["affine"] = affine.tolist()
    FUSION_CONFIG.write_text(yaml.safe_dump(cfg, sort_keys=False), encoding="utf-8")
    print(f"saved affine to {FUSION_CONFIG}:\n{affine}")


if __name__ == "__main__":
    main()
