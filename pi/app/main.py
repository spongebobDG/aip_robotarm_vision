"""
main.py — Phase 4 surveillance orchestrator (Pi side).

Wires the perception + behaviour layer together and runs the event loop:

    camera (thread, latest-frame-wins) ─┐
    thermal (thread, latest-frame-wins) ┼─> tick: FireAnalyzer + Detector
                                        │         -> StateMachine
                                        └─────────────> mqtt_link -> ESP32 (50 Hz)

Threads (not processes): CameraCapture and ThermalSerial already read in
background threads, and the heavy bits (pyserial read, Picamera2/V4L2 capture,
cv2 HOG) all release the GIL, so a slow consumer doesn't stall capture and
"latest-frame-wins" keeps things current (PLAN.md Phase 3/4 non-blocking goal).
If a heavier GIL-bound detector (e.g. a Python DNN) is added later, promote the
detector to its own multiprocessing.Process with a shared-memory frame.

Safety: on any exit (Ctrl-C, error) the arm is told to HOME then RELAX, matching
the ESP32 watchdog's own link-loss behaviour.

Run on the Pi (from the pi/ dir):
    python -m app.main              # real: needs mosquitto + ESP32
    python -m app.main --dry-run    # no broker/arm: prints commands, real sensors
"""
from __future__ import annotations

import argparse
import signal
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from comms.thermal_serial import ThermalSerial  # noqa: E402
from vision.camera_capture import CameraCapture  # noqa: E402
from vision.thermal_analysis import FireAnalyzer  # noqa: E402
from vision.detector import Detector  # noqa: E402
from app.tracker import Tracker  # noqa: E402
from app.state_machine import StateMachine  # noqa: E402

TICK_HZ = 10.0  # perception/behaviour loop rate (slow layer; ESP32 does 50 Hz)


class _ThermalLatest:
    """Background single-reader keeping the latest degC frame (no serial lag)."""

    def __init__(self, therm: ThermalSerial):
        self._therm = therm
        self._frame = None
        self._run = False

    def start(self):
        self._run = True
        threading.Thread(target=self._loop, daemon=True).start()

    def stop(self):
        self._run = False

    def _loop(self):
        while self._run:
            try:
                self._frame = self._therm.read_frame()
            except Exception:  # noqa: BLE001 -- ride through transient frame errors
                time.sleep(0.02)

    def latest(self):
        return self._frame


class _DryLink:
    """Stand-in for MqttLink: prints commands instead of publishing (no broker)."""
    state = None
    status = None

    def start(self): print("[dry] mqtt_link.start()")
    def stop(self): print("[dry] mqtt_link.stop()")
    def send_joints(self, b, s, e, w): print(f"[dry] joints {b:.0f} {s:.0f} {e:.0f} {w:.0f}")
    def pan_tilt(self, dp, dt): print(f"[dry] pan_tilt {dp:+.1f} {dt:+.1f}")
    def home(self): print("[dry] HOME")
    def relax(self): print("[dry] RELAX")
    def alarm(self, p): print(f"[dry] ALARM {p}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true", help="no broker/arm; print commands")
    ap.add_argument("--seconds", type=float, default=0.0, help="auto-stop after N s (0=forever)")
    args = ap.parse_args()

    if args.dry_run:
        link = _DryLink()
    else:
        from comms.mqtt_link import MqttLink
        link = MqttLink()

    cam = CameraCapture()
    therm = ThermalSerial()
    thermal = _ThermalLatest(therm)
    fire = FireAnalyzer()
    detector = Detector()
    tracker = Tracker(link)
    fsm = StateMachine(link, tracker)

    every_n = 1
    try:
        every_n = max(1, int(__import__("yaml").safe_load(
            (Path(__file__).resolve().parents[1] / "config" / "surveillance_config.yaml")
            .read_text())["perception"]["process_every_n"]))
    except Exception:  # noqa: BLE001
        pass

    link.start()
    cam.start()
    therm.start()
    thermal.start()

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())

    print(f"surveillance running (tick {TICK_HZ:.0f} Hz, detector={detector.backend}, "
          f"dry_run={args.dry_run}). Ctrl-C to stop.")
    period = 1.0 / TICK_HZ
    last_state = None
    frame_i = 0
    t_start = time.monotonic()
    try:
        while not stop.is_set():
            t0 = time.monotonic()
            rgb = cam.latest_frame()
            thermal_c = thermal.latest()
            if rgb is None or thermal_c is None:
                time.sleep(0.05)
                continue

            fire_res = fire.update(thermal_c)
            from vision.detector import DetectionResult
            det = (detector.detect(rgb) if frame_i % every_n == 0 else DetectionResult(False))
            frame_i += 1

            state = fsm.update(fire_res, det)
            if state != last_state:
                print(f"  state -> {state}"
                      + (f"  (fire {fire_res.max_temp_c:.0f}C)" if state == "ALARM_FIRE" else "")
                      + (f"  (target {det.cx:.2f},{det.cy:.2f})" if state == "TRACKING" else ""))
                last_state = state

            if args.seconds and (time.monotonic() - t_start) >= args.seconds:
                break
            dt = time.monotonic() - t0
            if dt < period:
                time.sleep(period - dt)
    finally:
        print("shutting down: HOME then RELAX")
        try:
            link.home()
            time.sleep(0.3)
            link.relax()
        except Exception:  # noqa: BLE001
            pass
        thermal.stop()
        cam.stop()
        therm.stop()
        link.stop()


if __name__ == "__main__":
    main()
