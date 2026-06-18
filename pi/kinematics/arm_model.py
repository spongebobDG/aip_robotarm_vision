"""
arm_model.py — forward kinematics for the 2-link planar arm + pan/tilt head.

Frame: origin at the floor under J0 (base). +x = arm "forward" (the direction
the arm points when base/shoulder/elbow/wrist are all at their configured
`home`), +z = up. `theta1` (shoulder) and `theta2` (elbow) are measured from
the horizontal in the vertical plane selected by `pan`; `theta3` (wrist/cam)
is the camera pitch relative to that same horizontal, so that
`cam_pitch = theta1 + theta2 + theta3`.

This assumes the arm's home pose (base/shoulder/elbow/wrist = HOME in
config.h / arm_config.yaml) points the link chain straight out horizontally
(theta1=theta2=theta3=0). If the physical home pose is folded/angled
differently, add a `kinematics.<axis>.sign`/`offset_deg` override in
arm_config.yaml — do not trust IK output until the Milestone 2 FK round-trip
check (command known angles, measure with a ruler, compare to
`ArmModel.fk()`) passes.

The elbow -> wrist link is a right-angle (ㄱ) bracket, not a straight rod;
`link2_mm` is the straight-line (diagonal) distance from the elbow axis to
the camera mount, which is exactly the effective length the planar model
needs. What the bend *can* break is the "elbow servo home == link2 continues
straight from link1" assumption baked into theta2=0 — if the bracket's bend
isn't bisected by the servo's zero position, theta2 has a constant angular
offset. Use `kinematics.elbow.offset_deg` to correct for that once measured
(round-trip check below will surface it as a consistent FK/real-world
mismatch that doesn't go away when other axes are at home).

Joint <-> servo-degree mapping:
`theta = offset_rad + sign * radians(servo_deg - home)`
`servo_deg = home + sign * degrees(theta - offset_rad)`
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Tuple

import yaml

_CONFIG = Path(__file__).resolve().parents[1] / "config" / "arm_config.yaml"

AXIS_NAMES = ("base", "shoulder", "elbow", "wrist")


@dataclass
class ArmGeometry:
    base_height_mm: float
    link1_mm: float
    link2_mm: float


@dataclass
class AxisMap:
    home: float
    min: float
    max: float
    sign: float = 1.0
    offset_rad: float = 0.0


class ArmModel:
    """Loads arm_config.yaml geometry/limits and converts between servo
    degrees and the theoretical (pan, theta1, theta2, theta3) angles used by
    the IK/FK math."""

    def __init__(self, config_path: "str | Path" = _CONFIG):
        cfg = yaml.safe_load(Path(config_path).read_text(encoding="utf-8"))
        g = cfg["geometry"]
        self.geom = ArmGeometry(g["base_height_mm"], g["link1_mm"], g["link2_mm"])

        axes = {a["name"]: a for a in cfg["axes"]}
        kin = cfg.get("kinematics", {})
        self.axis: dict[str, AxisMap] = {}
        for name in AXIS_NAMES:
            a = axes[name]
            axis_kin = kin.get(name, {})
            sign = float(axis_kin.get("sign", 1.0))
            offset_rad = math.radians(float(axis_kin.get("offset_deg", 0.0)))
            self.axis[name] = AxisMap(
                home=a["home"], min=a["min"], max=a["max"], sign=sign, offset_rad=offset_rad
            )

    # ---- servo degrees <-> theoretical radians ----------------------------
    def to_theta(self, name: str, servo_deg: float) -> float:
        ax = self.axis[name]
        return ax.offset_rad + ax.sign * math.radians(servo_deg - ax.home)

    def to_servo_raw(self, name: str, theta_rad: float) -> float:
        """theta -> servo degrees, NOT clamped to the axis's [min, max]."""
        ax = self.axis[name]
        return ax.home + ax.sign * math.degrees(theta_rad - ax.offset_rad)

    def clamp(self, name: str, servo_deg: float) -> float:
        ax = self.axis[name]
        return min(ax.max, max(ax.min, servo_deg))

    # ---- forward kinematics -------------------------------------------------
    def fk(self, b: float, s: float, e: float, w: float) -> Tuple[float, float, float, float]:
        """servo degrees (b, s, e, w) -> (x, y, z, cam_pitch_rad) in mm/rad."""
        pan = self.to_theta("base", b)
        th1 = self.to_theta("shoulder", s)
        th2 = self.to_theta("elbow", e)
        th3 = self.to_theta("wrist", w)

        L1, L2, H = self.geom.link1_mm, self.geom.link2_mm, self.geom.base_height_mm
        r = L1 * math.cos(th1) + L2 * math.cos(th1 + th2)
        z = H + L1 * math.sin(th1) + L2 * math.sin(th1 + th2)
        x = r * math.cos(pan)
        y = r * math.sin(pan)
        cam_pitch = th1 + th2 + th3
        return x, y, z, cam_pitch


if __name__ == "__main__":
    m = ArmModel()
    print("home FK (x,y,z,cam_pitch_rad):", m.fk(90.0, 0.0, 0.0, 90.0))
