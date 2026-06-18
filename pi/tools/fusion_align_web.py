"""
fusion_align_web.py — LIVE manual RGB<->thermal alignment (Phase 3/4).

An alternative to point-click calibration (fusion_calibrate_web.py). On a low-res
24x32 thermal sensor the warm blob is fuzzy, so clicking exact correspondences is
hard and high-DOF fits (homography) overfit the click noise into edge skew. Here
you instead watch the LIVE fused overlay and drag a few sliders until the thermal
hot region sits on top of the real object in the RGB image. Because you only
control 5 intuitive, well-behaved parameters (scale x/y, translate x/y, rotation)
there is nothing to overfit -- it's a constrained, human-in-the-loop registration.

The transform is built around the image center:
    A = R(rot) @ diag(sx, sy)              # 2x2 rot+scale
    b = center + (tx, ty) - A @ center     # so rot/scale pivot at the center
    M = [A | b]                            # 2x3 affine, thermal_up px -> rgb px
"Save" writes M into fusion_calib.yaml (`warp`), which Fusion.overlay then uses.

Run on the Pi (from the pi/ dir):  python -m tools.fusion_align_web
Browser on the same LAN:           http://<pi-ip>:8083/
"""
from __future__ import annotations

import json
import math
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse, parse_qs

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # allow vision/comms import

import cv2
import numpy as np
import yaml

from comms.thermal_serial import ThermalSerial  # noqa: E402
from vision.camera_capture import CameraCapture  # noqa: E402
from vision.fusion import _CONFIG as FUSION_CONFIG  # noqa: E402

PORT = 8083
BOUNDARY = "frame"
OUT_W, OUT_H = 640, 480

# live alignment parameters (server-side, mutated by /set)
_params = {"sx": 0.85, "sy": 0.85, "tx": 0.0, "ty": 0.0, "rot": 0.0, "alpha": 0.5}
_plock = threading.Lock()


def _matrix() -> np.ndarray:
    with _plock:
        p = dict(_params)
    th = math.radians(p["rot"])
    cos, sin = math.cos(th), math.sin(th)
    R = np.array([[cos, -sin], [sin, cos]], dtype=np.float64)
    A = R @ np.diag([p["sx"], p["sy"]])
    center = np.array([OUT_W / 2.0, OUT_H / 2.0])
    b = center + np.array([p["tx"], p["ty"]]) - A @ center
    return np.array([[A[0, 0], A[0, 1], b[0]], [A[1, 0], A[1, 1], b[1]]], dtype=np.float32)


class ThermalLatest:
    """Background single-reader keeping the latest raw degC frame (no lag)."""

    def __init__(self, therm: ThermalSerial):
        self._therm = therm
        self._lock = threading.Lock()
        self._frame_c: "np.ndarray | None" = None
        self._running = False

    def start(self):
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def stop(self):
        self._running = False

    def _loop(self):
        while self._running:
            try:
                self._frame_c = self._therm.read_frame()
            except Exception:  # noqa: BLE001
                time.sleep(0.05)

    def latest(self) -> "np.ndarray | None":
        return self._frame_c


def _thermal_color(frame_c: np.ndarray) -> np.ndarray:
    """Auto-contrast colormap upscaled to output size (alignment visibility)."""
    lo, hi = np.percentile(frame_c, [2, 98])
    norm = np.clip((frame_c - lo) / max(hi - lo, 1e-6), 0.0, 1.0)
    color = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_JET)
    return cv2.resize(color, (OUT_W, OUT_H), interpolation=cv2.INTER_NEAREST)


_PAGE = """<!doctype html><html><head><meta charset="utf-8"><title>fusion align</title>
<style>
 body{font-family:sans-serif;margin:12px;background:#111;color:#eee}
 img{display:block;border:1px solid #444;width:min(90vw,640px)}
 .ctl{margin:8px 0;max-width:640px}
 label{display:inline-block;width:90px} input[type=range]{width:60%;vertical-align:middle}
 span.v{display:inline-block;width:60px;text-align:right;font-family:monospace}
 button{font-size:15px;padding:6px 12px;margin:6px 6px 0 0}
 #msg{font-family:monospace;margin-top:8px}
</style></head><body>
<h3>Live RGB &harr; Thermal alignment</h3>
<p>Hold a warm target (palm) in view, then drag sliders until the thermal hot region sits on the object in the RGB below. No clicking. Then Save.</p>
<img id="ov" src="/overlay.mjpg">
<div id="ctls"></div>
<button onclick="save()">Save to fusion_calib.yaml</button>
<button onclick="reset()">Reset</button>
<div id="msg"></div>
<script>
const SPEC=[['sx','scale X',0.3,1.6,0.005],['sy','scale Y',0.3,1.6,0.005],
 ['tx','move X',-260,260,1],['ty','move Y',-260,260,1],
 ['rot','rotate',-30,30,0.25],['alpha','blend',0,1,0.02]];
let P={};
function build(){let h='';for(const[k,lbl,mn,mx,st]of SPEC)
 h+=`<div class="ctl"><label>${lbl}</label><input type="range" id="${k}" min="${mn}" max="${mx}" step="${st}" oninput="oc('${k}')"><span class="v" id="v_${k}"></span></div>`;
 document.getElementById('ctls').innerHTML=h;}
function setUI(){for(const[k]of SPEC){document.getElementById(k).value=P[k];document.getElementById('v_'+k).textContent=(+P[k]).toFixed(3);}}
async function oc(k){P[k]=+document.getElementById(k).value;document.getElementById('v_'+k).textContent=P[k].toFixed(3);
 await fetch('/set?'+k+'='+P[k]);}
function msg(t){document.getElementById('msg').textContent=t;}
async function save(){const j=await (await fetch('/save',{method:'POST'})).json();
 msg(j.ok?('SAVED to fusion_calib.yaml:\\n'+j.matrix):('ERROR: '+j.error));}
async function reset(){const j=await (await fetch('/reset',{method:'POST'})).json();P=j.params;setUI();msg('reset');}
build();fetch('/params').then(r=>r.json()).then(j=>{P=j.params;setUI();});
</script></body></html>"""


class _Handler(BaseHTTPRequestHandler):
    cam: CameraCapture = None
    thermal: ThermalLatest = None
    defaults = dict(_params)

    def log_message(self, fmt, *args):
        pass

    def _json(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/overlay.mjpg":
            self._stream()
        elif path == "/set":
            q = parse_qs(urlparse(self.path).query)
            with _plock:
                for k, v in q.items():
                    if k in _params:
                        _params[k] = float(v[0])
            self._json({"ok": True})
        elif path == "/params":
            with _plock:
                self._json({"ok": True, "params": dict(_params)})
        else:
            body = _PAGE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    def do_POST(self):
        if self.path == "/save":
            try:
                M = _matrix()
                cfg = yaml.safe_load(FUSION_CONFIG.read_text(encoding="utf-8"))
                cfg["warp"] = M.tolist()
                cfg.pop("affine", None)
                FUSION_CONFIG.write_text(yaml.safe_dump(cfg, sort_keys=False), encoding="utf-8")
                self._json({"ok": True, "matrix": np.array2string(M, precision=4)})
            except Exception as e:  # noqa: BLE001
                self._json({"ok": False, "error": f"{type(e).__name__}: {e}"})
        elif self.path == "/reset":
            with _plock:
                _params.update(self.defaults)
            self._json({"ok": True, "params": dict(self.defaults)})
        else:
            self._json({"ok": False, "error": "unknown endpoint"}, 404)

    def _overlay_jpeg(self) -> "bytes | None":
        rgb = self.cam.latest_frame()
        frame_c = self.thermal.latest()
        if rgb is None or frame_c is None:
            return None
        with _plock:
            alpha = _params["alpha"]
        thermal_up = _thermal_color(frame_c)
        reg = cv2.warpAffine(thermal_up, _matrix(), (OUT_W, OUT_H))
        out = cv2.addWeighted(cv2.resize(rgb, (OUT_W, OUT_H)), 1.0 - alpha, reg, alpha, 0)
        ok, buf = cv2.imencode(".jpg", out, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return buf.tobytes()

    def _stream(self):
        self.send_response(200)
        self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={BOUNDARY}")
        self.end_headers()
        try:
            while True:
                jpg = self._overlay_jpeg()
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
    server = ThreadingHTTPServer(("0.0.0.0", PORT), _Handler)
    print(f"fusion align: http://<pi-ip>:{PORT}/  (Ctrl+C to stop)")
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
