"""
state_machine.py — high-level surveillance FSM (Phase 4).

Event-driven, priority FIRE > INTRUDER > PATROL (PLAN.md). It is the slow,
behaviour layer: it decides WHAT to do and emits low-rate setpoints through
mqtt_link; the hard 50 Hz servo interpolation runs on the ESP32. One update()
call per perception tick.

States:
  PATROL      : J1/J2 held at the watch posture, J0 sweeps slowly side to side.
  TRACKING    : a person/motion target is centred via pan/tilt visual servoing;
                if the target is lost longer than lost_timeout_s -> PATROL.
  ALARM_FIRE  : a thermal hotspot over the alarm temp; aim at it, publish
                arm/alarm, and hold until the fire clears (hysteresis in
                FireAnalyzer), then -> PATROL.

Side effects go through the injected mqtt_link (send_joints / pan_tilt / alarm)
and Tracker, so the transition logic stays unit-testable with dummies.
"""
from __future__ import annotations

import time
from pathlib import Path

import yaml

_CONFIG = Path(__file__).resolve().parents[1] / "config" / "surveillance_config.yaml"

PATROL = "PATROL"
TRACKING = "TRACKING"
ALARM_FIRE = "ALARM_FIRE"


class StateMachine:
    def __init__(self, link, tracker, config_path: "str | Path" = _CONFIG, clock=None):
        cfg = yaml.safe_load(Path(config_path).read_text(encoding="utf-8"))
        self._link = link
        self._tracker = tracker
        self._clock = clock or time.monotonic
        tr = cfg["tracking"]
        self.lost_timeout = float(tr["lost_timeout_s"])
        p = cfg["patrol"]
        self.sweep_min = float(p["sweep_min_deg"])
        self.sweep_max = float(p["sweep_max_deg"])
        self.sweep_dps = float(p["sweep_speed_dps"])
        self.watch = (float(p["watch_shoulder_deg"]), float(p["watch_elbow_deg"]),
                      float(p["watch_wrist_deg"]))
        a = cfg["alarm"]
        self.alarm_repeat_s = 1.0

        self.state = PATROL
        self._base = (self.sweep_min + self.sweep_max) / 2.0
        self._sweep_dir = 1.0
        self._last_seen = -1e9
        self._last_tick = self._clock()
        self._last_alarm_pub = -1e9

    # ---- main entry ---------------------------------------------------------
    def update(self, fire, detection) -> str:
        """Run one tick. `fire` is a FireResult, `detection` a DetectionResult.
        Returns the (possibly new) state name."""
        now = self._clock()
        dt = max(now - self._last_tick, 0.0)
        self._last_tick = now

        if fire.active:                       # priority 1
            self._enter(ALARM_FIRE)
            self._do_alarm(fire, now)
        elif detection.found:                 # priority 2
            self._enter(TRACKING)
            self._last_seen = now
            self._tracker.update(detection.cx, detection.cy)
        elif self.state == TRACKING and (now - self._last_seen) <= self.lost_timeout:
            pass                              # briefly keep TRACKING, target may reappear
        else:                                 # priority 3
            self._enter(PATROL)
            self._do_patrol(dt)
        return self.state

    # ---- per-state behaviour ------------------------------------------------
    def _do_patrol(self, dt: float) -> None:
        self._base += self._sweep_dir * self.sweep_dps * dt
        if self._base >= self.sweep_max:
            self._base, self._sweep_dir = self.sweep_max, -1.0
        elif self._base <= self.sweep_min:
            self._base, self._sweep_dir = self.sweep_min, 1.0
        self._link.send_joints(self._base, *self.watch)

    def _do_alarm(self, fire, now: float) -> None:
        # aim the head at the hotspot (visual servoing toward its centre)
        self._tracker.update(fire.cx, fire.cy)
        if now - self._last_alarm_pub >= self.alarm_repeat_s:
            self._link.alarm(f"FIRE max={fire.max_temp_c:.1f}C area={fire.area_px} "
                             f"at={fire.cx:.2f},{fire.cy:.2f}")
            self._last_alarm_pub = now

    def _enter(self, state: str) -> None:
        if state != self.state:
            self.state = state
            self._last_alarm_pub = -1e9   # let a fresh alarm publish immediately


if __name__ == "__main__":
    # Logic test with dummies (no hardware/broker).
    from dataclasses import dataclass

    @dataclass
    class _Fire:
        active: bool = False
        max_temp_c: float = 25.0
        area_px: int = 0
        cx: float = 0.5
        cy: float = 0.5

    @dataclass
    class _Det:
        found: bool = False
        cx: float = 0.5
        cy: float = 0.5

    class _Link:
        def send_joints(self, *a): print(f"   send_joints{tuple(round(x,1) for x in a)}")
        def pan_tilt(self, dp, dt): print(f"   pan_tilt({dp:.1f},{dt:.1f})")
        def alarm(self, p): print(f"   ALARM: {p}")

    class _Trk:
        def update(self, cx, cy): print(f"   track->({cx:.2f},{cy:.2f})")

    t = [0.0]
    sm = StateMachine(_Link(), _Trk(), clock=lambda: t[0])
    print("patrol:", sm.update(_Fire(), _Det())); t[0] += 1
    print("see person:", sm.update(_Fire(), _Det(found=True, cx=0.8, cy=0.4))); t[0] += 0.5
    print("fire overrides:", sm.update(_Fire(active=True, max_temp_c=70, area_px=5, cx=0.3, cy=0.6), _Det(found=True)))
    t[0] += 5
    print("fire cleared -> patrol:", sm.update(_Fire(), _Det()))
