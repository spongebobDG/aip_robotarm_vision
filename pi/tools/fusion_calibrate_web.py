"""
fusion_calibrate_web.py — browser-based RGB<->thermal registration (Phase 3).

Server-friendly replacement for tools/fusion_calibrate.py: this Ubuntu Server
box has no X display (cv2.imshow fails with "Can't initialize GTK backend"), so
calibration is done in a browser instead of cv2 windows -- same idea as
tools/camera_web_preview.py. Uses only stdlib http.server + cv2 (imencode/
estimateAffinePartial2D/warpAffine are headless-safe; only highgui needs X).

Workflow (do it at your real operating distance -- camera/thermal mounts are
stacked, so vertical parallax is distance-dependent):
  1. Heat several point targets (soldering iron tip, warm mug...) spread across
     the shared field of view so they're visible in BOTH images.
  2. Run on the Pi (from the pi/ directory):
         python -m tools.fusion_calibrate_web
  3. From any browser on the same LAN:  http://<pi-ip>:8081/
  4. Click "Capture snapshot" to freeze a matching RGB + thermal pair.
  5. Click each target in the RGB image, then the SAME targets in the SAME order
     in the thermal image (>= 3 pairs; 4+ recommended).
  6. Click "Compute & save" -> writes the 2x3 affine into fusion_calib.yaml
     (other keys preserved) and shows the blended overlay so you can sanity-check
     the registration. Re-capture and redo if it looks off.
"""
from __future__ import annotations

import json
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # allow vision/comms import

import cv2
import numpy as np
import yaml

from comms.thermal_serial import ThermalSerial  # noqa: E402
from vision.camera_capture import CameraCapture  # noqa: E402
from vision.fusion import Fusion, _CONFIG as FUSION_CONFIG  # noqa: E402

PORT = 8081

_PAGE = """<!doctype html><html><head><meta charset="utf-8"><title>fusion calibrate</title>
<style>
 body{font-family:sans-serif;margin:12px;background:#111;color:#eee}
 .row{display:flex;gap:16px;flex-wrap:wrap}
 .pane{border:1px solid #444}
 canvas{cursor:crosshair;display:block}
 button{font-size:15px;padding:6px 12px;margin:4px 4px 4px 0}
 #msg{white-space:pre-wrap;font-family:monospace;margin-top:8px}
 h4{margin:6px 0}
</style></head><body>
<h3>RGB &harr; Thermal calibration</h3>
<div>
 <button onclick="capture()">1. Capture snapshot</button>
 <button onclick="undo()">Undo last point</button>
 <button onclick="reset()">Clear points</button>
 <button onclick="save()">2. Compute &amp; save</button>
</div>
<p>Click each heated target in <b>RGB</b>, then the same targets in the <b>same order</b> in <b>Thermal</b>. Need &ge;3 pairs.</p>
<div class="row">
 <div><h4>RGB (<span id="nr">0</span>)</h4><div class="pane"><canvas id="rgb"></canvas></div></div>
 <div><h4>Thermal (<span id="nt">0</span>)</h4><div class="pane"><canvas id="th"></canvas></div></div>
 <div><h4>Overlay preview</h4><div class="pane"><img id="ov" width="320"></div></div>
</div>
<div id="msg"></div>
<script>
let rgbPts=[],thPts=[],imgs={};
function draw(id,src,pts){const c=document.getElementById(id),x=c.getContext('2d'),im=new Image();
 im.onload=()=>{c.width=im.width;c.height=im.height;x.drawImage(im,0,0);
  pts.forEach((p,i)=>{x.fillStyle='#0f0';x.beginPath();x.arc(p[0],p[1],5,0,7);x.fill();
   x.fillStyle='#0f0';x.font='14px sans-serif';x.fillText(i+1,p[0]+7,p[1]-7);});};
 im.src=src;imgs[id]=im;}
function redraw(){draw('rgb',imgs.rgb.src,rgbPts);draw('th',imgs.th.src,thPts);
 document.getElementById('nr').textContent=rgbPts.length;
 document.getElementById('nt').textContent=thPts.length;}
function clickHandler(id,pts){const c=document.getElementById(id);
 c.onclick=e=>{const r=c.getBoundingClientRect();
  pts.push([Math.round((e.clientX-r.left)*c.width/r.width),Math.round((e.clientY-r.top)*c.height/r.height)]);redraw();};}
async function capture(){const r=await fetch('/frames');const j=await r.json();
 draw('rgb','data:image/png;base64,'+j.rgb,rgbPts);draw('th','data:image/png;base64,'+j.thermal,thPts);
 setTimeout(redraw,100);msg('snapshot captured ('+j.note+')');}
function undo(){rgbPts.pop()||0;thPts.length>rgbPts.length&&thPts.pop();redraw();}
function reset(){rgbPts=[];thPts=[];redraw();}
function msg(t){document.getElementById('msg').textContent=t;}
async function save(){if(rgbPts.length<3||thPts.length<3){msg('need >=3 points in each');return;}
 const n=Math.min(rgbPts.length,thPts.length);
 const r=await fetch('/save',{method:'POST',headers:{'Content-Type':'application/json'},
  body:JSON.stringify({rgb:rgbPts.slice(0,n),thermal:thPts.slice(0,n)})});
 const j=await r.json();if(j.ok){document.getElementById('ov').src='data:image/png;base64,'+j.overlay;
  msg('SAVED affine to fusion_calib.yaml:\\n'+j.affine);}else{msg('ERROR: '+j.error);}}
clickHandler('rgb',rgbPts);clickHandler('th',thPts);
</script></body></html>"""


class _Handler(BaseHTTPRequestHandler):
    cam: CameraCapture = None
    therm: ThermalSerial = None
    fusion: Fusion = None
    snap_rgb = None      # frozen BGR frame from the last /frames
    snap_thermal_c = None  # frozen thermal degC frame from the last /frames

    def log_message(self, fmt, *args):
        pass

    def _send(self, code, ctype, body: bytes):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj, code=200):
        self._send(code, "application/json", json.dumps(obj).encode())

    @staticmethod
    def _png_b64(bgr: np.ndarray) -> str:
        import base64
        ok, buf = cv2.imencode(".png", bgr)
        return base64.b64encode(buf.tobytes()).decode()

    @staticmethod
    def _thermal_display(thermal_c: np.ndarray, w: int, h: int) -> np.ndarray:
        """Per-frame AUTO-CONTRAST colormap for the calibration display only.
        Fusion's fixed 15-40C range maps a room (25-36C) into a narrow band that
        looks uniform -- you can't see the heated targets to click them. Here we
        stretch to the frame's own 2-98 percentile so targets pop, and use
        NEAREST upscaling so the 24x32 pixel grid is crisp for precise clicks.
        This only affects what's shown; the saved affine geometry and the real
        Fusion overlay (fixed range, for the Phase 4 fire threshold) are
        unchanged -- clicking coordinates are in the same upscaled (w,h) grid."""
        lo, hi = np.percentile(thermal_c, [2, 98])
        norm = np.clip((thermal_c - lo) / max(hi - lo, 1e-6), 0.0, 1.0)
        color = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_JET)
        return cv2.resize(color, (w, h), interpolation=cv2.INTER_NEAREST)

    def do_GET(self):
        if self.path == "/frames":
            self._frames()
        else:
            self._send(200, "text/html", _PAGE.encode())

    def _frames(self):
        rgb = None
        while rgb is None:
            rgb = self.cam.latest_frame()
        # drain the serial backlog so the snapshot is the CURRENT scene, not a
        # frame buffered 15-20s ago (this tool only reads on demand). Retry a
        # few times: a re-sync can occasionally land on a false marker and raise.
        from comms.thermal_serial import ChecksumError, FrameSyncError
        thermal_c = None
        for _ in range(5):
            try:
                thermal_c = self.therm.read_fresh()
                break
            except (ChecksumError, FrameSyncError):
                continue
        if thermal_c is None:
            self._json({"ok": False, "error": "thermal read failed (frame sync)"}, 503)
            return
        type(self).snap_rgb = rgb.copy()
        type(self).snap_thermal_c = thermal_c.copy()
        # auto-contrast thermal display, upscaled to RGB size -- same grid the
        # user clicks on and the same grid the affine maps from (Fusion.overlay).
        th_color = self._thermal_display(thermal_c, rgb.shape[1], rgb.shape[0])
        note = (f"thermal {thermal_c.min():.1f}-{thermal_c.max():.1f}C "
                f"(display auto-stretched), bad_px {self.therm.last_bad_pixels}")
        self._json({"rgb": self._png_b64(rgb), "thermal": self._png_b64(th_color), "note": note})

    def do_POST(self):
        if self.path != "/save":
            self._json({"ok": False, "error": "unknown endpoint"}, 404)
            return
        try:
            length = int(self.headers.get("Content-Length", 0))
            data = json.loads(self.rfile.read(length))
            if self.snap_rgb is None:
                raise RuntimeError("capture a snapshot first")
            src = np.array(data["thermal"], dtype=np.float32)  # upscaled-thermal px
            dst = np.array(data["rgb"], dtype=np.float32)      # rgb px
            if len(src) < 3 or len(src) != len(dst):
                raise RuntimeError("need >=3 matched point pairs")
            affine, _ = cv2.estimateAffinePartial2D(src, dst)
            if affine is None:
                raise RuntimeError("estimateAffinePartial2D found no solution")

            cfg = yaml.safe_load(FUSION_CONFIG.read_text(encoding="utf-8"))
            cfg["affine"] = affine.tolist()
            FUSION_CONFIG.write_text(yaml.safe_dump(cfg, sort_keys=False), encoding="utf-8")

            # overlay preview using the just-saved affine on the frozen frames
            preview = Fusion().overlay(self.snap_rgb, self.snap_thermal_c)
            self._json({"ok": True, "affine": np.array2string(affine, precision=3),
                        "overlay": self._png_b64(preview)})
        except Exception as e:  # noqa: BLE001 -- report any failure to the browser
            self._json({"ok": False, "error": f"{type(e).__name__}: {e}"})


def main():
    cam = CameraCapture()
    therm = ThermalSerial()
    cam.start()
    therm.start()
    _Handler.cam = cam
    _Handler.therm = therm
    _Handler.fusion = Fusion()
    server = ThreadingHTTPServer(("0.0.0.0", PORT), _Handler)
    print(f"fusion calibrate: http://<pi-ip>:{PORT}/  (Ctrl+C to stop)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        cam.stop()
        therm.stop()


if __name__ == "__main__":
    main()
