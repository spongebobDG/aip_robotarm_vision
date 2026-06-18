"""
vision_web_preview.py — LIVE side-by-side RGB + thermal preview (Phase 3).

Server-friendly live view for aiming/aligning the stacked camera + thermal
module: this Ubuntu Server box has no X display, so both feeds stream to a
browser as MJPEG (same idea as camera_web_preview.py) and refresh in real time.
Move the thermal module and watch both panes update together until they frame
the same scene. Then switch to tools/fusion_calibrate_web.py to compute the
affine. (Run only ONE of these at a time -- they share the single camera and
the single serial port.)

Thermal pane: per-frame AUTO-CONTRAST colormap (2-98 percentile stretch) so warm
targets pop even in a near-uniform room, NEAREST upscale so the 24x32 grid is
crisp, a marker on the hottest pixel, a center crosshair for aiming, and a
min/max/mean readout. Dead pixels are already repaired by ThermalSerial.

A single background thread reads the serial port (latest-frame-wins); the HTTP
handlers only serve the latest encoded frame, so multiple browser tabs never
cause two readers to fight over /dev/serial0.

Run on the Pi (from the pi/ directory):
    python -m tools.vision_web_preview
Then from any browser on the same LAN:   http://<pi-ip>:8082/
"""
from __future__ import annotations

import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # allow vision/comms import

import cv2
import numpy as np

from comms.thermal_serial import ThermalSerial  # noqa: E402
from vision.camera_capture import CameraCapture  # noqa: E402
from vision.fusion import Fusion  # noqa: E402

PORT = 8082
BOUNDARY = "frame"
OUT_W, OUT_H = 640, 480


class ThermalLatest:
    """Background single-reader: keeps the most recent thermal JPEG + stats."""

    def __init__(self, therm: ThermalSerial):
        self._therm = therm
        self._lock = threading.Lock()
        self._jpeg: bytes | None = None
        self._frame_c: np.ndarray | None = None  # latest raw degC frame (for fusion)
        self._running = False

    def start(self):
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def stop(self):
        self._running = False

    def _render(self, frame_c: np.ndarray) -> bytes:
        # orientation is already corrected in ThermalSerial.read_frame() (config
        # `orientation`), so the same fix applies to calibration + Fusion too.
        lo, hi = np.percentile(frame_c, [2, 98])
        norm = np.clip((frame_c - lo) / max(hi - lo, 1e-6), 0.0, 1.0)
        color = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_JET)
        img = cv2.resize(color, (OUT_W, OUT_H), interpolation=cv2.INTER_NEAREST)
        # hottest-pixel marker (mapped from 24x32 grid to output size)
        r, c = np.unravel_index(int(np.argmax(frame_c)), frame_c.shape)
        px = int((c + 0.5) * OUT_W / frame_c.shape[1])
        py = int((r + 0.5) * OUT_H / frame_c.shape[0])
        cv2.circle(img, (px, py), 12, (255, 255, 255), 2)
        # center crosshair for aiming
        cv2.drawMarker(img, (OUT_W // 2, OUT_H // 2), (255, 255, 255),
                       cv2.MARKER_CROSS, 30, 1)
        ta = self._therm.last_ta_c
        txt = (f"min {frame_c.min():.1f}  max {frame_c.max():.1f}  "
               f"mean {frame_c.mean():.1f}  Ta {ta:.1f}  badpx {self._therm.last_bad_pixels}")
        cv2.putText(img, txt, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
        cv2.putText(img, txt, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return buf.tobytes()

    def _loop(self):
        while self._running:
            try:
                frame_c = self._therm.read_frame()
            except Exception:  # noqa: BLE001 -- keep streaming through transient errors
                time.sleep(0.05)
                continue
            jpeg = self._render(frame_c)
            with self._lock:
                self._jpeg = jpeg
                self._frame_c = frame_c

    def latest_jpeg(self) -> bytes | None:
        with self._lock:
            return self._jpeg

    def latest_frame_c(self) -> np.ndarray | None:
        with self._lock:
            return self._frame_c


_PAGE = """<!doctype html><html><head><meta charset="utf-8"><title>vision live</title>
<style>body{margin:0;background:#111;color:#eee;font-family:sans-serif}
 .row{display:flex;flex-wrap:wrap;gap:2px}
 figure{margin:0} img{display:block;width:48vw;min-width:320px;background:#000}
 figcaption{padding:4px 8px}</style></head><body>
<div class="row">
 <figure><figcaption>RGB camera</figcaption><img src="/rgb.mjpg"></figure>
 <figure><figcaption>Thermal (auto-contrast, &#9711;=hottest, &#10133;=center)</figcaption><img src="/thermal.mjpg"></figure>
 <figure><figcaption>Fusion (calibrated thermal over RGB)</figcaption><img src="/fusion.mjpg"></figure>
</div>
<p style="padding:0 8px">Fusion pane uses the calibrated affine in fusion_calib.yaml (fixed 15-40&deg;C range, same as the Phase 4 fire threshold). Aiming/lens checks: cover the thermal lens with your palm &mdash; the thermal pane should go uniformly warm if it is seeing forward.</p>
</body></html>"""


class _Handler(BaseHTTPRequestHandler):
    cam: CameraCapture = None
    thermal: ThermalLatest = None
    fusion: Fusion = None

    def log_message(self, fmt, *args):
        pass

    def do_GET(self):
        if self.path == "/rgb.mjpg":
            self._stream(self._rgb_frame)
        elif self.path == "/thermal.mjpg":
            self._stream(self.thermal.latest_jpeg)
        elif self.path == "/fusion.mjpg":
            self._stream(self._fusion_frame)
        else:
            body = _PAGE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    def _rgb_frame(self) -> bytes | None:
        frame = self.cam.latest_frame()
        if frame is None:
            return None
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return buf.tobytes()

    def _fusion_frame(self) -> bytes | None:
        rgb = self.cam.latest_frame()
        thermal_c = self.thermal.latest_frame_c()
        if rgb is None or thermal_c is None:
            return None
        overlaid = self.fusion.overlay(rgb, thermal_c)
        ok, buf = cv2.imencode(".jpg", overlaid, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return buf.tobytes()

    def _stream(self, get_jpeg):
        self.send_response(200)
        self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={BOUNDARY}")
        self.end_headers()
        try:
            while True:
                jpg = get_jpeg()
                if jpg is None:
                    time.sleep(0.05)
                    continue
                self.wfile.write(f"--{BOUNDARY}\r\n".encode())
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(jpg)}\r\n\r\n".encode())
                self.wfile.write(jpg)
                self.wfile.write(b"\r\n")
                time.sleep(0.08)
        except (BrokenPipeError, ConnectionResetError):
            pass


def main():
    cam = CameraCapture()
    therm = ThermalSerial()
    cam.start()
    therm.start()
    thermal = ThermalLatest(therm)
    thermal.start()
    _Handler.cam = cam
    _Handler.thermal = thermal
    _Handler.fusion = Fusion()
    server = ThreadingHTTPServer(("0.0.0.0", PORT), _Handler)
    print(f"vision live preview: http://<pi-ip>:{PORT}/  (Ctrl+C to stop)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        thermal.stop()
        cam.stop()
        therm.stop()


if __name__ == "__main__":
    main()
