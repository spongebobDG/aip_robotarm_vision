"""
fusion_calibrate_web.py — browser-based RGB<->thermal registration (Phase 3/4).

Server-friendly replacement for tools/fusion_calibrate.py: this Ubuntu Server
box has no X display (cv2.imshow fails with "Can't initialize GTK backend"), so
calibration is done in a browser instead of cv2 windows. cv2 imencode/
findHomography/estimateAffine2D/warpPerspective are headless-safe; only highgui
needs X.

WIDE-ANGLE camera note: a wide lens adds perspective + edge skew that a
similarity transform (the old estimateAffinePartial2D) cannot model, so the
overlay drifts toward the edges. This tool now (a) ACCUMULATES point pairs across
many snapshots so you can cover the whole field of view -- including the corners
where it's worst -- with a single moving heat target (e.g. a palm), and (b) fits
a HOMOGRAPHY (8 DOF, perspective) when enough pairs are given, falling back to a
full affine (6 DOF) for fewer. RANSAC averages out click noise and reports the
reprojection error so you can tell how good the fit is. (For barrel-distortion-
perfect results you'd undistort the camera with a checkerboard first; with only a
palm, dense points + homography is the best achievable.)

Workflow (do it at your real operating distance):
  1. Run on the Pi (from the pi/ dir):  python -m tools.fusion_calibrate_web
  2. Browser on the same LAN:  http://<pi-ip>:8081/
  3. For each target position: "Capture snapshot" -> click the hot target in RGB
     -> click the SAME target in Thermal -> "Add pair to set". Move the target to
     a new spot (spread them out, hit the corners) and repeat. Collect >= 6 for a
     homography (8-12 well-spread is good); 3-5 gives an affine.
  4. "Compute & save" -> writes the matrix into fusion_calib.yaml (as `warp`) and
     shows the blended overlay + reprojection error. Re-add points / recompute if
     the error is high or the overlay looks off.
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
 b#np{color:#6f6} h4{margin:6px 0}
</style></head><body>
<h3>RGB &harr; Thermal calibration (multi-point, server-side)</h3>
<div>
 <button onclick="capture()">1. Capture snapshot</button>
 <button onclick="addPair()">2. Add pair to set</button>
 <button onclick="undo()">Undo point</button>
 <button onclick="clearCur()">Clear current</button>
 <button onclick="clearAll()">Clear ALL</button>
 <label style="margin-left:8px">model:
  <select id="model">
   <option value="affine" selected>affine (recommended)</option>
   <option value="homography">homography</option>
   <option value="similarity">similarity</option>
  </select></label>
 <button onclick="save()">3. Compute &amp; save</button>
</div>
<p>Per target position: Capture &rarr; click target in <b>RGB</b> &rarr; same target in <b>Thermal</b> &rarr; Add pair. Move the palm (cover the corners!) and repeat. Accumulated pairs (on server): <b id="np">0</b>. <b>affine</b> is best for the low-res thermal; homography (needs &ge;4) can overfit click noise into edge skew &mdash; recompute with another model anytime without re-clicking.</p>
<div class="row">
 <div><h4>RGB (<span id="nr">0</span>)</h4><div class="pane"><canvas id="rgb"></canvas></div></div>
 <div><h4>Thermal (<span id="nt">0</span>)</h4><div class="pane"><canvas id="th"></canvas></div></div>
 <div><h4>Overlay preview</h4><div class="pane"><img id="ov" width="320"></div></div>
</div>
<div id="msg"></div>
<script>
let rgbPts=[],thPts=[],total=0,imgs={};
function draw(id,src,pts){const c=document.getElementById(id),x=c.getContext('2d'),im=new Image();
 im.onload=()=>{c.width=im.width;c.height=im.height;x.drawImage(im,0,0);
  pts.forEach((p,i)=>{x.fillStyle='#0f0';x.beginPath();x.arc(p[0],p[1],5,0,7);x.fill();
   x.fillStyle='#0f0';x.font='14px sans-serif';x.fillText(i+1,p[0]+7,p[1]-7);});};
 im.src=src;imgs[id]=im;}
function redraw(){if(imgs.rgb)draw('rgb',imgs.rgb.src,rgbPts);if(imgs.th)draw('th',imgs.th.src,thPts);
 document.getElementById('nr').textContent=rgbPts.length;
 document.getElementById('nt').textContent=thPts.length;
 document.getElementById('np').textContent=total;}
function clickHandler(id,pts){const c=document.getElementById(id);
 c.onclick=e=>{const r=c.getBoundingClientRect();
  pts.push([Math.round((e.clientX-r.left)*c.width/r.width),Math.round((e.clientY-r.top)*c.height/r.height)]);redraw();};}
async function post(url,body){const r=await fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body||{})});return r.json();}
async function capture(){const r=await fetch('/frames');const j=await r.json();
 if(j.ok===false){msg('ERROR: '+j.error);return;}
 total=j.total;draw('rgb','data:image/png;base64,'+j.rgb,rgbPts);draw('th','data:image/png;base64,'+j.thermal,thPts);
 setTimeout(redraw,100);msg('snapshot captured ('+j.note+')');}
async function addPair(){const n=Math.min(rgbPts.length,thPts.length);
 if(n<1){msg('click the target in BOTH images first');return;}
 const pp=[];for(let i=0;i<n;i++)pp.push([rgbPts[i][0],rgbPts[i][1],thPts[i][0],thPts[i][1]]);
 const j=await post('/add',{pairs:pp});total=j.total;
 rgbPts.length=0;thPts.length=0;redraw();msg('added '+n+'; total='+total+'. Move target & capture again.');}
function undo(){rgbPts.pop();thPts.length>rgbPts.length&&thPts.pop();redraw();}
function clearCur(){rgbPts.length=0;thPts.length=0;redraw();}
async function clearAll(){const j=await post('/clear');total=j.total;rgbPts.length=0;thPts.length=0;redraw();msg('cleared all accumulated pairs');}
function msg(t){document.getElementById('msg').textContent=t;}
async function save(){const model=document.getElementById('model').value;
 const j=await post('/save',{model:model});
 if(j.ok){document.getElementById('ov').src='data:image/png;base64,'+j.overlay;
  msg('SAVED '+j.model+' to fusion_calib.yaml\\n'+'pairs='+j.n+' inliers='+j.inliers+' reproj_err='+j.err+'px\\n'+j.matrix);}
 else{msg('ERROR: '+j.error);}}
clickHandler('rgb',rgbPts);clickHandler('th',thPts);
fetch('/count').then(r=>r.json()).then(j=>{total=j.total;redraw();});
</script></body></html>"""


class _Handler(BaseHTTPRequestHandler):
    cam: CameraCapture = None
    therm: ThermalSerial = None
    fusion: Fusion = None
    snap_rgb = None      # frozen BGR frame from the last /frames
    snap_thermal_c = None  # frozen thermal degC frame from the last /frames
    pairs: list = []     # accumulated [rx, ry, tx, ty] across snapshots (server-side)

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
        This only affects what's shown; the saved geometry and the real Fusion
        overlay (fixed range, for the Phase 4 fire threshold) are unchanged --
        clicking coordinates are in the same upscaled (w,h) grid."""
        lo, hi = np.percentile(thermal_c, [2, 98])
        norm = np.clip((thermal_c - lo) / max(hi - lo, 1e-6), 0.0, 1.0)
        color = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_JET)
        return cv2.resize(color, (w, h), interpolation=cv2.INTER_NEAREST)

    def do_GET(self):
        if self.path == "/frames":
            self._frames()
        elif self.path == "/count":
            self._json({"ok": True, "total": len(self.pairs)})
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
        th_color = self._thermal_display(thermal_c, rgb.shape[1], rgb.shape[0])
        note = (f"thermal {thermal_c.min():.1f}-{thermal_c.max():.1f}C "
                f"(display auto-stretched), bad_px {self.therm.last_bad_pixels}")
        self._json({"ok": True, "rgb": self._png_b64(rgb), "total": len(self.pairs),
                    "thermal": self._png_b64(th_color), "note": note})

    @staticmethod
    def _fit(src: np.ndarray, dst: np.ndarray, model: str):
        """Fit thermal->rgb with the requested model. Returns
        (matrix, model_name, inlier_count, mean_reproj_err_px).
          similarity = rot+uniform scale+translation (4 DOF), most robust
          affine     = + shear/independent scale (6 DOF), good for wide-cam aspect
          homography = + perspective (8 DOF), can overfit low-res thermal clicks"""
        n = len(src)
        if model == "homography":
            if n < 4:
                raise RuntimeError("homography needs >=4 pairs")
            mat, mask = cv2.findHomography(src, dst, cv2.RANSAC, 5.0)
            if mat is None:
                raise RuntimeError("findHomography found no solution")
            pred = cv2.perspectiveTransform(src.reshape(-1, 1, 2), mat).reshape(-1, 2)
        elif model == "similarity":
            mat, mask = cv2.estimateAffinePartial2D(src, dst, method=cv2.RANSAC)
            if mat is None:
                raise RuntimeError("estimateAffinePartial2D found no solution")
            pred = (src @ mat[:, :2].T) + mat[:, 2]
        else:  # affine (default)
            model = "affine"
            mat, mask = cv2.estimateAffine2D(src, dst, method=cv2.RANSAC)
            if mat is None:
                raise RuntimeError("estimateAffine2D found no solution")
            pred = (src @ mat[:, :2].T) + mat[:, 2]
        m = mask.ravel().astype(bool) if mask is not None else np.ones(n, bool)
        inliers = int(m.sum())
        err = float(np.linalg.norm(pred[m] - dst[m], axis=1).mean()) if m.any() else float("nan")
        return mat, model, inliers, err

    def do_POST(self):
        try:
            length = int(self.headers.get("Content-Length", 0))
            data = json.loads(self.rfile.read(length)) if length else {}
            if self.path == "/add":
                self.pairs.extend(data.get("pairs", []))
                self._json({"ok": True, "total": len(self.pairs)})
            elif self.path == "/clear":
                self.pairs.clear()
                self._json({"ok": True, "total": 0})
            elif self.path == "/save":
                self._save(data.get("model", "affine"))
            else:
                self._json({"ok": False, "error": "unknown endpoint"}, 404)
        except Exception as e:  # noqa: BLE001 -- report any failure to the browser
            self._json({"ok": False, "error": f"{type(e).__name__}: {e}"})

    def _save(self, model: str):
        if self.snap_rgb is None:
            raise RuntimeError("capture a snapshot first")
        pairs = np.array(self.pairs, dtype=np.float32)
        if pairs.ndim != 2 or pairs.shape[1] != 4 or len(pairs) < 3:
            raise RuntimeError("need >=3 accumulated pairs (Add pair to set)")
        # contiguous copies -- estimateAffine2D's checkVector rejects strided views
        dst = np.ascontiguousarray(pairs[:, 0:2])   # rgb px
        src = np.ascontiguousarray(pairs[:, 2:4])   # upscaled-thermal px
        mat, used, inliers, err = self._fit(src, dst, model)

        cfg = yaml.safe_load(FUSION_CONFIG.read_text(encoding="utf-8"))
        cfg["warp"] = mat.tolist()
        cfg.pop("affine", None)       # supersede the old key
        FUSION_CONFIG.write_text(yaml.safe_dump(cfg, sort_keys=False), encoding="utf-8")

        preview = Fusion().overlay(self.snap_rgb, self.snap_thermal_c)
        self._json({"ok": True, "model": used, "n": len(pairs), "inliers": inliers,
                    "err": f"{err:.2f}", "matrix": np.array2string(mat, precision=4),
                    "overlay": self._png_b64(preview)})


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
