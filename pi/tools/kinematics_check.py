"""
kinematics_check.py — Phase 2 verification: FK/IK round-trip + reachability.

Does not touch MQTT/hardware — pure math sanity check against
arm_config.yaml geometry/limits before trusting IK output on the real arm.

Run:   python -m tools.kinematics_check        (from the pi/ directory)
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # allow `kinematics` import

from kinematics.arm_model import ArmModel  # noqa: E402
from kinematics.ik_solver import IkSolver, Unreachable  # noqa: E402


def round_trip(model: ArmModel, solver: IkSolver, b: float, s: float, e: float, w: float) -> None:
    x, y, z, cam_pitch = model.fk(b, s, e, w)
    try:
        b2, s2, e2, w2 = solver.solve(x, y, z, cam_pitch)
    except Unreachable as exc:
        print(f"  FAIL  ({b},{s},{e},{w}) -> xyz={x:.1f},{y:.1f},{z:.1f}  IK rejected: {exc}")
        return
    err = max(abs(b - b2), abs(s - s2), abs(e - e2), abs(w - w2))
    status = "ok" if err < 0.5 else "MISMATCH"
    print(
        f"  {status:8s} in=({b:.1f},{s:.1f},{e:.1f},{w:.1f}) "
        f"xyz=({x:.1f},{y:.1f},{z:.1f}) cam_pitch_deg={cam_pitch * 57.2958:.1f} "
        f"-> out=({b2:.1f},{s2:.1f},{e2:.1f},{w2:.1f}) max_err_deg={err:.2f}"
    )


def main() -> None:
    model = ArmModel()
    solver = IkSolver(model)

    print(f"geometry: L1={model.geom.link1_mm}mm L2={model.geom.link2_mm}mm H={model.geom.base_height_mm}mm")

    print("\nFK -> IK round-trip (known servo angles -> xyz -> back to angles):")
    for angles in [
        (90, 0, 0, 90),
        (90, 30, 40, 60),
        (60, 20, 60, 90),
        (120, 10, 90, 120),
    ]:
        round_trip(model, solver, *angles)

    print("\nreachability checks (direct xyz targets):")
    for x, y, z in [(150, 0, 95), (0, 150, 95), (1000, 0, 95), (50, 0, 400)]:
        try:
            angles = solver.solve(x, y, z, cam_pitch=0.0)
            print(f"  ok      xyz=({x},{y},{z}) -> ({', '.join(f'{a:.1f}' for a in angles)})")
        except Unreachable as exc:
            print(f"  reject  xyz=({x},{y},{z}) -> {exc}")


if __name__ == "__main__":
    main()
