"""
tracker.py — image-based pan/tilt visual servoing (Phase 4).

Turns a target's pixel position into a small relative pan/tilt nudge and sends it
over MQTT (arm/cmd/pantilt) to the ESP32, which does the actual 50 Hz motion.
This is deliberately a slow, proportional, dead-banded controller: the hard
real-time smoothing is the ESP32's job (PLAN.md 3-layer split), so here we just
nudge toward centering the target and rely on the deadzone to avoid jitter.

    ex = (cx - 0.5) * 2     # normalized horizontal error, -1..1
    ey = (cy - 0.5) * 2
    if |ex|,|ey| within deadzone: do nothing
    dpan  = pan_sign  * Kp_pan  * ex   (clamped)
    dtilt = tilt_sign * Kp_tilt * ey   (clamped)

cx, cy are normalized 0..1 (matches Detector and FireAnalyzer outputs), so the
tracker doesn't care about frame resolution.
"""
from __future__ import annotations

from pathlib import Path

import yaml

_CONFIG = Path(__file__).resolve().parents[1] / "config" / "surveillance_config.yaml"


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


class Tracker:
    def __init__(self, mqtt_link, config_path: "str | Path" = _CONFIG):
        cfg = yaml.safe_load(Path(config_path).read_text(encoding="utf-8"))["tracking"]
        self._link = mqtt_link
        self.kp_pan = float(cfg["kp_pan_deg"])
        self.kp_tilt = float(cfg["kp_tilt_deg"])
        self.deadzone = float(cfg["deadzone"])
        self.max_nudge = float(cfg["max_nudge_deg"])
        self.pan_sign = float(cfg["pan_sign"])
        self.tilt_sign = float(cfg["tilt_sign"])

    def compute(self, cx: float, cy: float) -> "tuple[float, float]":
        """Pure: normalized target (0..1) -> (dpan, dtilt) in deg. (0,0) inside
        the deadzone. Exposed separately so it can be unit-tested without MQTT."""
        ex = (cx - 0.5) * 2.0
        ey = (cy - 0.5) * 2.0
        if abs(ex) < self.deadzone and abs(ey) < self.deadzone:
            return 0.0, 0.0
        dpan = _clamp(self.pan_sign * self.kp_pan * ex, -self.max_nudge, self.max_nudge)
        dtilt = _clamp(self.tilt_sign * self.kp_tilt * ey, -self.max_nudge, self.max_nudge)
        return dpan, dtilt

    def update(self, cx: float, cy: float) -> "tuple[float, float]":
        """Compute and, if outside the deadzone, send the pan/tilt nudge."""
        dpan, dtilt = self.compute(cx, cy)
        if dpan != 0.0 or dtilt != 0.0:
            self._link.pan_tilt(dpan, dtilt)
        return dpan, dtilt


if __name__ == "__main__":
    # Unit test the control law with a dummy link (no broker needed).
    class _DummyLink:
        def pan_tilt(self, dpan, dtilt):
            print(f"  -> pan_tilt({dpan:.2f}, {dtilt:.2f})")

    t = Tracker(_DummyLink())
    print("centered (deadzone):", t.compute(0.5, 0.5))
    print("target right/up:", t.compute(0.9, 0.2))
    print("far corner (clamped):", t.compute(1.0, 1.0))
    print("update() far-left:")
    t.update(0.0, 0.5)
