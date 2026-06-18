"""
thermal_analysis.py — fire / over-temperature detection on thermal frames (Phase 4).

Takes a (rows, cols) degC frame from ThermalSerial (dead pixels already repaired,
orientation already corrected) and reports whether anything exceeds the alarm
temperature, where the hottest region is, and how severe it is. Uses on/off
hysteresis (t_alarm_c to latch, t_clear_c to release) plus a sustain time so the
FSM doesn't chatter right at the threshold.

The hotspot location is returned in NORMALIZED frame coordinates (cx, cy in
0..1) so the caller can map it onto the RGB frame for aiming without caring about
the thermal resolution.

Usage:
    fa = FireAnalyzer()                 # loads surveillance_config.yaml
    result = fa.update(thermal_c)       # call once per thermal frame
    if result.active: aim/alarm using result.cx, result.cy, result.max_temp_c
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

_CONFIG = Path(__file__).resolve().parents[1] / "config" / "surveillance_config.yaml"


@dataclass
class FireResult:
    active: bool            # alarm currently latched (after hysteresis/sustain)
    detected_now: bool      # this frame had hot pixels over t_alarm_c
    max_temp_c: float
    area_px: int            # number of pixels over the alarm threshold
    cx: float               # hotspot centroid, normalized 0..1 (x)
    cy: float               # hotspot centroid, normalized 0..1 (y)


class FireAnalyzer:
    def __init__(self, config_path: "str | Path" = _CONFIG, clock=None):
        cfg = yaml.safe_load(Path(config_path).read_text(encoding="utf-8"))["alarm"]
        self.t_alarm = float(cfg["t_alarm_c"])
        self.t_clear = float(cfg["t_clear_c"])
        self.min_area = int(cfg["min_area_px"])
        self.sustain_s = float(cfg["sustain_s"])
        import time
        self._clock = clock or time.monotonic
        self._active = False
        self._last_detect_t = -1e9

    def update(self, thermal_c: np.ndarray) -> FireResult:
        max_temp = float(thermal_c.max())
        # latch threshold while inactive, lower clear threshold while active
        thr = self.t_clear if self._active else self.t_alarm
        mask = thermal_c >= thr
        area = int(mask.sum())
        detected_now = area >= self.min_area and max_temp >= (
            self.t_clear if self._active else self.t_alarm
        )

        if detected_now:
            ys, xs = np.nonzero(mask)
            # weight the centroid by how far each pixel is over the clear temp
            w = (thermal_c[ys, xs] - self.t_clear).clip(min=0.0) + 1e-6
            cy = float((ys * w).sum() / w.sum()) / thermal_c.shape[0]
            cx = float((xs * w).sum() / w.sum()) / thermal_c.shape[1]
            self._last_detect_t = self._clock()
            self._active = True
        else:
            cx = cy = 0.5
            if self._active and (self._clock() - self._last_detect_t) > self.sustain_s:
                self._active = False

        return FireResult(self._active, detected_now, max_temp, area, cx, cy)


if __name__ == "__main__":
    # Synthetic test: ambient frame, then inject a hot region, then remove it.
    import time
    fa = FireAnalyzer()
    amb = np.full((24, 32), 24.0)
    print("ambient:", fa.update(amb))
    hot = amb.copy(); hot[10:13, 20:24] = 75.0
    r = fa.update(hot)
    print("hot:", r, "-> hotspot ~ (0.70, 0.46) expected:", round(r.cx, 2), round(r.cy, 2))
    print("removed (within sustain, still active):", fa.update(amb).active)
    time.sleep(0.1)
    print("note: real clear needs sustain_s to elapse with no detection")
