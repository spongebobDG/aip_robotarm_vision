"""
mqtt_link.py — Pi <-> ESP32 command/telemetry link over MQTT.

The Pi hosts the Mosquitto broker; this client publishes setpoints and
subscribes to the ESP32's telemetry/status. Works with paho-mqtt 1.x and 2.x.

Usage:
    from comms.mqtt_link import MqttLink
    link = MqttLink()              # loads pi/config/mqtt_config.yaml
    link.start()
    link.home()
    link.send_joints(90, 20, 30, 90)
    link.pan_tilt(2.0, -1.0)
    print(link.state)              # latest (b, s, e, w) telemetry or None
"""
from __future__ import annotations

import os
import threading
from pathlib import Path
from typing import Callable, Optional, Tuple

import yaml
import paho.mqtt.client as mqtt

_CONFIG = Path(__file__).resolve().parents[1] / "config" / "mqtt_config.yaml"


def _make_client(client_id: str) -> mqtt.Client:
    """Construct a paho Client compatible with both 1.x and 2.x APIs."""
    try:  # paho-mqtt >= 2.0
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
    except (AttributeError, TypeError):  # paho-mqtt 1.x
        return mqtt.Client(client_id=client_id)


class MqttLink:
    def __init__(self, config_path: os.PathLike | str = _CONFIG):
        cfg = yaml.safe_load(Path(config_path).read_text(encoding="utf-8"))
        self._broker = cfg["broker"]
        self._t = cfg["topics"]
        self._lock = threading.Lock()

        self.state: Optional[Tuple[float, float, float, float]] = None
        self.status: Optional[str] = None
        self.on_state: Optional[Callable[[Tuple[float, float, float, float]], None]] = None
        self.on_status: Optional[Callable[[str], None]] = None

        self._cli = _make_client(self._broker.get("client_id", "pi-host"))
        self._cli.on_connect = self._on_connect
        self._cli.on_message = self._on_message

    # ---- lifecycle ----------------------------------------------------------
    def start(self) -> None:
        self._cli.connect(
            self._broker["host"],
            int(self._broker.get("port", 1883)),
            int(self._broker.get("keepalive", 30)),
        )
        self._cli.loop_start()

    def stop(self) -> None:
        self._cli.loop_stop()
        self._cli.disconnect()

    # ---- callbacks (flexible signatures for paho 1.x / 2.x) -----------------
    def _on_connect(self, client, userdata, flags, *args):
        client.subscribe(self._t["state"])
        client.subscribe(self._t["status"])

    def _on_message(self, client, userdata, msg):
        payload = msg.payload.decode("utf-8", "ignore").strip()
        if msg.topic == self._t["state"]:
            try:
                b, s, e, w = (float(x) for x in payload.split())
                with self._lock:
                    self.state = (b, s, e, w)
                if self.on_state:
                    self.on_state((b, s, e, w))
            except ValueError:
                pass
        elif msg.topic == self._t["status"]:
            with self._lock:
                self.status = payload
            if self.on_status:
                self.on_status(payload)

    # ---- commands -----------------------------------------------------------
    def send_joints(self, b: float, s: float, e: float, w: float) -> None:
        self._cli.publish(self._t["cmd_joints"], f"{b:.1f} {s:.1f} {e:.1f} {w:.1f}", qos=0)

    def pan_tilt(self, dpan: float, dtilt: float) -> None:
        self._cli.publish(self._t["cmd_pantilt"], f"{dpan:.2f} {dtilt:.2f}", qos=0)

    def home(self) -> None:
        self._cli.publish(self._t["cmd_mode"], "HOME", qos=1)

    def relax(self) -> None:
        self._cli.publish(self._t["cmd_mode"], "RELAX", qos=1)


if __name__ == "__main__":
    # Smoke test: connect, home, print telemetry for a few seconds.
    import time

    link = MqttLink()
    link.on_state = lambda s: print("state:", s)
    link.on_status = lambda s: print("status:", s)
    link.start()
    print("connected; sending HOME")
    link.home()
    time.sleep(5)
    link.stop()
