"""
ik_solver.py — analytic IK for the 2-link planar arm + pan/tilt head.

Inverse of `arm_model.ArmModel.fk()`: given a target (x, y, z, cam_pitch) in
the same base-frame mm/rad convention, solve for servo degrees
(b, s, e, w). Raises Unreachable if the target is outside the arm's reach
or any joint's configured [min, max] limit.

Usage:
    from kinematics.ik_solver import IkSolver, Unreachable
    solver = IkSolver()
    b, s, e, w = solver.solve(x=150.0, y=0.0, z=150.0, cam_pitch=0.0)
    mqtt_link.send_joints(b, s, e, w)
"""
from __future__ import annotations

import math
from typing import Tuple

from kinematics.arm_model import ArmModel, AXIS_NAMES


class Unreachable(ValueError):
    pass


class IkSolver:
    def __init__(self, model: ArmModel | None = None):
        self.model = model or ArmModel()

    def solve(self, x: float, y: float, z: float, cam_pitch: float = 0.0) -> Tuple[float, float, float, float]:
        g = self.model.geom
        L1, L2, H = g.link1_mm, g.link2_mm, g.base_height_mm

        pan = math.atan2(y, x)
        r = math.hypot(x, y)
        zp = z - H

        c2 = (r * r + zp * zp - L1 * L1 - L2 * L2) / (2 * L1 * L2)
        if abs(c2) > 1.0:
            raise Unreachable(f"target ({x:.1f}, {y:.1f}, {z:.1f}) out of reach: |c2|={abs(c2):.3f} > 1")
        th2 = math.acos(c2)
        th1 = math.atan2(zp, r) - math.atan2(L2 * math.sin(th2), L1 + L2 * math.cos(th2))
        th3 = cam_pitch - (th1 + th2)

        thetas = {"base": pan, "shoulder": th1, "elbow": th2, "wrist": th3}
        servo = {}
        for name in AXIS_NAMES:
            deg = self.model.to_servo_raw(name, thetas[name])
            ax = self.model.axis[name]
            if deg < ax.min - 1e-6 or deg > ax.max + 1e-6:
                raise Unreachable(f"{name} angle {deg:.1f} deg outside [{ax.min}, {ax.max}]")
            servo[name] = deg

        return servo["base"], servo["shoulder"], servo["elbow"], servo["wrist"]


if __name__ == "__main__":
    solver = IkSolver()
    angles = solver.solve(x=150.0, y=0.0, z=150.0, cam_pitch=0.0)
    print("solved (b, s, e, w):", [round(a, 1) for a in angles])
