"""
camera_web_preview.py — TEMPORARY headless camera check tool (Phase 3).

Standalone and throwaway: NOT part of the Phase 3/4 app. It exists only to
verify the CSI camera is producing frames when there's no GUI/VNC display
available to run cv2.imshow against. Serves an MJPEG stream over plain HTTP
using stdlib + Pillow only (both already needed/available), so there's
nothing extra to install just to look at the feed.

Run on the Pi (from the pi/ directory):
    python -m tools.camera_web_preview

Then from any browser on the same LAN:
    http://<pi-ip>:8080/

Note: Picamera2's "RGB888" format is actually byte-order BGR in practice, so
colors may look swapped here — irrelevant for this tool's only job, which is
confirming frames are arriving.
"""
from __future__ import annotations

import io
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from PIL import Image

from vision.camera_capture import CameraCapture

BOUNDARY = "frame"
PORT = 8080


class _Handler(BaseHTTPRequestHandler):
    capture: CameraCapture = None  # set in main() before the server starts

    def log_message(self, fmt, *args):
        pass

    def do_GET(self):
        if self.path == "/stream.mjpg":
            self._stream_mjpeg()
        else:
            self._index()

    def _index(self):
        body = b"<html><body><h3>camera preview (temporary check)</h3><img src='/stream.mjpg'></body></html>"
        self.send_response(200)
        self.send_header("Content-Type", "text/html")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _stream_mjpeg(self):
        self.send_response(200)
        self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={BOUNDARY}")
        self.end_headers()
        try:
            while True:
                frame = self.capture.latest_frame()
                if frame is None:
                    time.sleep(0.05)
                    continue
                buf = io.BytesIO()
                Image.fromarray(frame).save(buf, format="JPEG", quality=80)
                jpg = buf.getvalue()
                self.wfile.write(f"--{BOUNDARY}\r\n".encode())
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(jpg)}\r\n\r\n".encode())
                self.wfile.write(jpg)
                self.wfile.write(b"\r\n")
                time.sleep(0.1)
        except (BrokenPipeError, ConnectionResetError):
            pass


def main():
    capture = CameraCapture()
    capture.start()
    _Handler.capture = capture
    server = ThreadingHTTPServer(("0.0.0.0", PORT), _Handler)
    print(f"camera preview: http://<pi-ip>:{PORT}/  (Ctrl+C to stop)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        capture.stop()


if __name__ == "__main__":
    main()
