"""
arm_jog.py — interactive Phase 1 verification tool (run on the Pi).

Publishes MQTT commands to the ESP32 so you can home, relax, jog single axes,
and nudge pan/tilt — confirming the full Pi -> broker -> WiFi -> ESP32 -> servo
path works, while live telemetry (arm/state) prints back.

Run:   python -m tools.arm_jog        (from the pi/ directory)

Keys:
  0..3            select axis
  + / -           jog selected axis by STEP degrees (absolute send)
  p <pan> <tilt>  pan/tilt nudge, e.g.  p 3 -2
  j b s e w       send absolute joints, e.g.  j 90 20 30 90
  h               home
  r               relax
  s               print last telemetry
  q               quit
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # allow `comms` import

from comms.mqtt_link import MqttLink  # noqa: E402

STEP = 5.0
# local mirror of targets so +/- jog can send absolute angles
HOME = [90.0, 0.0, 0.0, 90.0]
LIMITS = [(0, 180), (0, 120), (0, 140), (0, 180)]


def clamp(v, axis):
    lo, hi = LIMITS[axis]
    return max(lo, min(hi, v))


def main() -> None:
    link = MqttLink()
    link.on_status = lambda s: print(f"  [status] {s}")
    link.start()
    time.sleep(0.5)

    tgt = list(HOME)
    axis = 0
    link.home()
    print(__doc__)
    print(f"axis={axis}  targets={tgt}")

    try:
        for raw in sys.stdin:
            line = raw.strip()
            if not line:
                continue
            parts = line.split()
            c = parts[0].lower()

            if c in ("0", "1", "2", "3"):
                axis = int(c)
            elif c == "+":
                tgt[axis] = clamp(tgt[axis] + STEP, axis)
                link.send_joints(*tgt)
            elif c == "-":
                tgt[axis] = clamp(tgt[axis] - STEP, axis)
                link.send_joints(*tgt)
            elif c == "p" and len(parts) == 3:
                link.pan_tilt(float(parts[1]), float(parts[2]))
                tgt[0] = clamp(tgt[0] + float(parts[1]), 0)
                tgt[3] = clamp(tgt[3] + float(parts[2]), 3)
            elif c == "j" and len(parts) == 5:
                tgt = [clamp(float(parts[i + 1]), i) for i in range(4)]
                link.send_joints(*tgt)
            elif c == "h":
                tgt = list(HOME)
                link.home()
            elif c == "r":
                link.relax()
            elif c == "s":
                print(f"  [state] {link.state}")
            elif c == "q":
                break
            else:
                print("  ? unknown command")
                continue
            print(f"axis={axis}  targets={[round(x, 1) for x in tgt]}")
    except KeyboardInterrupt:
        pass
    finally:
        link.relax()
        link.stop()
        print("\nstopped.")


if __name__ == "__main__":
    main()
