"""
fusion_align_web.py — LIVE manual RGB<->thermal alignment + camera FOV (Phase 3/4).

An alternative to point-click calibration (fusion_calibrate_web.py). On a low-res
24x32 thermal sensor the warm blob is fuzzy, so clicking exact correspondences is
hard and high-DOF fits (homography) overfit the click noise into edge skew. Here
you instead watch the LIVE fused overlay and drag a few sliders until the thermal
hot region sits on top of the real object in the RGB image. Because you only
control a few intuitive, well-behaved parameters (scale x/y, translate x/y,
rotation) there is nothing to overfit -- it's a constrained, human-in-the-loop
registration.

Also includes a "zoom" slider that crops the RGB sensor frame live (narrower
crop = narrower FOV, less barrel distortion at the edges, closer match to the
thermal sensor's fixed ~55 deg FOV). This tool always reads the FULL sensor
frame (CameraCapture(force_full_frame=True)) regardless of whatever crop is
currently saved in camera_config.yaml, so zoom can go wider OR narrower than
the saved crop.

The registration transform is built around the (cropped) frame's center:
    A = R(rot) @ diag(sx, sy)              # 2x2 rot+scale
    b = center + (tx, ty) - A @ center     # so rot/scale pivot at the center
    M = [A | b]                            # 2x3 affine, thermal_up px -> rgb px
"Save" writes M into fusion_calib.yaml (`warp`) and the crop box into
camera_config.yaml (`camera.crop`), so the deployed app sees exactly what was
tuned here.

IMPORTANT: run only ONE camera/thermal-reading tool at a time. CameraCapture
and ThermalSerial each open the underlying device/port exclusively; two
processes reading the same UART concurrently corrupt and delay frames for
both (this is why a previous session saw severe thermal lag — vision_web_preview
and this tool were both running against the same /dev/ttyS0 + camera at once).

Run on the Pi (from the pi/ dir):  python -m tools.fusion_align_web
Browser on the same LAN:           http://<pi-ip>:8083/
"""
from __future__ import annotations

import json
import math
import re
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
from vision.camera_capture import CameraCapture, _CONFIG as CAMERA_CONFIG  # noqa: E402
from vision.fusion import _CONFIG as FUSION_CONFIG  # noqa: E402

PORT = 8083
BOUNDARY = "frame"
ZOOM_MIN, ZOOM_MAX = 0.4, 1.0  # crop fraction of full sensor frame (1.0 = no crop)

_plock = threading.Lock()


def _full_sensor_size() -> "tuple[int, int]":
    cam_cfg = yaml.safe_load(CAMERA_CONFIG.read_text(encoding="utf-8"))["camera"]
    return int(cam_cfg["width"]), int(cam_cfg["height"])


def _decompose_warp(M: np.ndarray, w: int, h: int) -> dict:
    """Recover sx,sy,tx,ty,rot from a saved [A|b] matrix built the same way
    _matrix() builds one, using (w/2, h/2) as the pivot it was built with."""
    A, b = M[:, :2].astype(np.float64), M[:, 2].astype(np.float64)
    rot = math.degrees(math.atan2(A[1, 0], A[0, 0]))
    sx = math.hypot(A[0, 0], A[1, 0])
    sy = math.hypot(A[0, 1], A[1, 1])
    center = np.array([w / 2.0, h / 2.0])
    tx, ty = b - center + A @ center
    return {"sx": sx, "sy": sy, "tx": float(tx), "ty": float(ty), "rot": rot}


def _load_initial_params() -> dict:
    """Start sliders from whatever is already saved, instead of generic
    defaults, so re-opening this tool resumes the previous calibration."""
    full_w, full_h = _full_sensor_size()
    cam_cfg = yaml.safe_load(CAMERA_CONFIG.read_text(encoding="utf-8"))["camera"]
    crop = cam_cfg.get("crop", {})
    crop_w = int(crop["width"]) if crop.get("enabled") else full_w
    zoom = max(ZOOM_MIN, min(ZOOM_MAX, crop_w / full_w))
    crop_h = int(round(full_h * zoom))

    fcfg = yaml.safe_load(FUSION_CONFIG.read_text(encoding="utf-8"))
    mat = fcfg.get("warp", fcfg.get("affine"))
    alpha = float(fcfg.get("overlay", {}).get("alpha", 0.45))
    try:
        p = _decompose_warp(np.array(mat, dtype=np.float64), crop_w, crop_h)
    except Exception:  # noqa: BLE001 -- fall back to neutral if shape mismatches
        p = {"sx": 0.85, "sy": 0.85, "tx": 0.0, "ty": 0.0, "rot": 0.0}
    p["alpha"] = alpha
    p["zoom"] = zoom
    return p


_params = _load_initial_params()


def _crop_box(zoom: float, full_w: int, full_h: int) -> "tuple[int, int, int, int]":
    """Centered crop box (left, top, w, h) for a given zoom fraction."""
    w = max(64, int(round(full_w * zoom)))
    h = max(48, int(round(full_h * zoom)))
    return (full_w - w) // 2, (full_h - h) // 2, w, h


def _matrix(p: dict, w: int, h: int) -> np.ndarray:
    th = math.radians(p["rot"])
    cos, sin = math.cos(th), math.sin(th)
    R = np.array([[cos, -sin], [sin, cos]], dtype=np.float64)
    A = R @ np.diag([p["sx"], p["sy"]])
    center = np.array([w / 2.0, h / 2.0])
    b = center + np.array([p["tx"], p["ty"]]) - A @ center
    return np.array([[A[0, 0], A[0, 1], b[0]], [A[1, 0], A[1, 1], b[1]]], dtype=np.float32)


class ThermalLatest:
    """Background single-reader keeping the latest raw degC frame (no lag)."""

    def __init__(self, therm: ThermalSerial):
        self._therm = therm
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


def _thermal_color(frame_c: np.ndarray, w: int, h: int) -> np.ndarray:
    """Auto-contrast colormap upscaled to (w, h) for alignment visibility."""
    lo, hi = np.percentile(frame_c, [2, 98])
    norm = np.clip((frame_c - lo) / max(hi - lo, 1e-6), 0.0, 1.0)
    color = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_JET)
    return cv2.resize(color, (w, h), interpolation=cv2.INTER_NEAREST)


def _patch_camera_crop(enabled: bool, left: int, top: int, w: int, h: int) -> None:
    """Rewrite only the nested values under `camera: crop:` in camera_config.yaml,
    preserving all the surrounding comments (a full yaml.safe_dump would strip
    them, and the v4l2 section documents real hardware quirks worth keeping)."""
    text = CAMERA_CONFIG.read_text(encoding="utf-8")
    m = re.search(r"(  crop:\n(?:    .*\n)*)", text)
    if not m:
        raise RuntimeError("camera_config.yaml: 'crop:' block not found")
    block = m.group(1)
    patched = block
    patched = re.sub(r"enabled: \S+", f"enabled: {'true' if enabled else 'false'}", patched, count=1)
    patched = re.sub(r"left: \S+", f"left: {left}", patched, count=1)
    patched = re.sub(r"top: \S+", f"top: {top}", patched, count=1)
    patched = re.sub(r"width: \S+", f"width: {w}", patched, count=1)
    patched = re.sub(r"height: \S+", f"height: {h}", patched, count=1)
    CAMERA_CONFIG.write_text(text.replace(block, patched), encoding="utf-8")


_PAGE = """<!doctype html><html><head><meta charset="utf-8"><title>fusion align</title>
<style>
 body{font-family:sans-serif;margin:12px;background:#111;color:#eee}
 img{display:block;border:1px solid #444;width:min(90vw,640px)}
 .ctl{margin:8px 0;max-width:640px}
 label{display:inline-block;width:90px} input[type=range]{width:60%;vertical-align:middle}
 span.v{display:inline-block;width:60px;text-align:right;font-family:monospace}
 button{font-size:15px;padding:6px 12px;margin:6px 6px 0 0}
 #msg{font-family:monospace;margin-top:8px;white-space:pre-wrap}
</style></head><body>
<h3>Live RGB &harr; Thermal alignment + camera FOV</h3>
<p>Hold a warm target (palm) in view. "zoom" crops the RGB camera (lower = narrower FOV,
less edge distortion). Drag the rest until the thermal hot region sits on the object below.
No clicking. Then Save.</p>
<img id="ov" src="/overlay.mjpg">
<div id="ctls"></div>
<button onclick="save()">Save (warp + camera crop)</button>
<button onclick="reset()">Reset to last saved</button>
<div id="msg"></div>
<script>
const SPEC=[['zoom','camera FOV (zoom)',0.4,1.0,0.01],
 ['sx','scale X',0.3,1.6,0.005],['sy','scale Y',0.3,1.6,0.005],
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
 msg(j.ok?('SAVED.\\nwarp:\\n'+j.matrix+'\\ncamera crop: '+j.crop):('ERROR: '+j.error));}
async function reset(){const j=await (await fetch('/reset',{method:'POST'})).json();P=j.params;setUI();msg('reset to last saved values');}
build();fetch('/params').then(r=>r.json()).then(j=>{P=j.params;setUI();});
</script></body></html>"""


class _Handler(BaseHTTPRequestHandler):
    cam: CameraCapture = None
    thermal: ThermalLatest = None

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
                        val = float(v[0])
                        if k == "zoom":
                            val = max(ZOOM_MIN, min(ZOOM_MAX, val))
                        _params[k] = val
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
                with _plock:
                    p = dict(_params)
                full_w, full_h = self.cam._width, self.cam._height
                left, top, w, h = _crop_box(p["zoom"], full_w, full_h)
                M = _matrix(p, w, h)

                fcfg = yaml.safe_load(FUSION_CONFIG.read_text(encoding="utf-8"))
                fcfg["warp"] = M.tolist()
                fcfg.pop("affine", None)
                fcfg.setdefault("overlay", {})["alpha"] = p["alpha"]
                FUSION_CONFIG.write_text(yaml.safe_dump(fcfg, sort_keys=False), encoding="utf-8")

                _patch_camera_crop(True, left, top, w, h)

                self._json({
                    "ok": True,
                    "matrix": np.array2string(M, precision=4),
                    "crop": f"{w}x{h} @ ({left},{top}) of {full_w}x{full_h}",
                })
            except Exception as e:  # noqa: BLE001
                self._json({"ok": False, "error": f"{type(e).__name__}: {e}"})
        elif self.path == "/reset":
            fresh = _load_initial_params()
            with _plock:
                _params.update(fresh)
            self._json({"ok": True, "params": dict(fresh)})
        else:
            self._json({"ok": False, "error": "unknown endpoint"}, 404)

    def _overlay_jpeg(self) -> "bytes | None":
        raw = self.cam.latest_frame()
        frame_c = self.thermal.latest()
        if raw is None or frame_c is None:
            return None
        full_h, full_w = raw.shape[:2]
        with _plock:
            p = dict(_params)
        left, top, w, h = _crop_box(p["zoom"], full_w, full_h)
        cropped = raw[top:top + h, left:left + w]
        thermal_up = _thermal_color(frame_c, w, h)
        M = _matrix(p, w, h)
        reg = cv2.warpAffine(thermal_up, M, (w, h))
        out = cv2.addWeighted(cropped, 1.0 - p["alpha"], reg, p["alpha"], 0)
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
    cam = CameraCapture(force_full_frame=True)
    therm = ThermalSerial()
    cam.start()
    therm.start()
    thermal = ThermalLatest(therm)
    thermal.start()
    _Handler.cam = cam
    _Handler.thermal = thermal
    server = ThreadingHTTPServer(("0.0.0.0", PORT), _Handler)
    print(f"fusion align: http://<pi-ip>:{PORT}/  (Ctrl+C to stop)")
    print("NOTE: stop any other tool that reads the camera/thermal serial first "
          "(e.g. vision_web_preview) -- running two at once corrupts/delays both.")
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
