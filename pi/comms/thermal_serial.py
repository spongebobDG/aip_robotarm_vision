"""
thermal_serial.py — UART frame parser for the thermal module (/dev/serial0).

PROTOCOL CONFIRMED 2026-06-18 by reverse-engineering the live wire (the bare
MLX90640 sensor is I2C-only; this module has an onboard MCU re-streaming 24x32
frames over UART in its own serial format). The wire format was captured from
/dev/ttyS0 and the checksum verified against many consecutive live frames --
see docs/devlog/phase3-vision.md for the analysis. `thermal_config.yaml` holds
those confirmed values and `confirmed: true`; if it is ever flipped back to
false (e.g. swapping to an unverified module), `start()` refuses to read so a
wrong guess about the wire format can't silently produce garbage temperatures.

Frame layout (little-endian, 1544 bytes):
    5A 5A | len(2) | Ta(2) | 768 px * uint16 (1536) | checksum(2)
    temp_c = raw * value_scale + value_offset ; checksum = 16-bit word sum.

Usage:
    from comms.thermal_serial import ThermalSerial
    therm = ThermalSerial()
    therm.start()
    frame_c = therm.read_frame()   # np.ndarray, shape (rows, cols), degrees C
    ambient = therm.last_ta_c      # module-reported ambient temp (deg C)
"""
from __future__ import annotations

from pathlib import Path
from typing import List

import numpy as np
import yaml

_CONFIG = Path(__file__).resolve().parents[1] / "config" / "thermal_config.yaml"


class ProtocolUnconfirmed(RuntimeError):
    pass


class FrameSyncError(RuntimeError):
    pass


class ChecksumError(RuntimeError):
    pass


class ThermalSerial:
    def __init__(self, config_path: "str | Path" = _CONFIG):
        cfg = yaml.safe_load(Path(config_path).read_text(encoding="utf-8"))
        self._confirmed = bool(cfg.get("confirmed", False))
        self._port = cfg["port"]
        self._baud = int(cfg["baud"])
        self._rows = int(cfg["rows"])
        self._cols = int(cfg["cols"])
        self._marker: List[int] = list(cfg["start_marker"])
        self._header_bytes = int(cfg.get("header_bytes", 0))
        byteorder = cfg.get("value_byteorder", "little")
        order_char = "<" if byteorder == "little" else ">"
        self._dtype = np.dtype(order_char + np.dtype(cfg["value_dtype"]).str[1:])
        self._scale = float(cfg["value_scale"])
        self._offset = float(cfg["value_offset"])
        self._checksum = cfg.get("checksum", "none")
        self._timeout = float(cfg.get("timeout_s", 1.0))
        bp = cfg.get("bad_pixel", {}) or {}
        self._bp_repair = bool(bp.get("repair", False))
        self._bp_min_c = float(bp.get("min_valid_c", -40.0))
        self._bp_max_c = float(bp.get("max_valid_c", 300.0))
        ori = cfg.get("orientation", {}) or {}
        rot180 = bool(ori.get("rotate_180", False))
        self._flip_v = bool(ori.get("flip_vertical", False)) ^ rot180
        self._flip_h = bool(ori.get("flip_horizontal", False)) ^ rot180
        self._ser = None
        self.last_ta_c: "float | None" = None  # module ambient temp, updated per frame
        self.last_bad_pixels: int = 0  # bad pixels repaired in the most recent frame

    def start(self) -> None:
        if not self._confirmed:
            raise ProtocolUnconfirmed(
                "thermal_config.yaml frame format is unconfirmed (confirmed: false). "
                "Check the thermal module's datasheet for the real baud rate and frame "
                "layout, update thermal_config.yaml, then set confirmed: true."
            )
        import serial  # pyserial; imported lazily

        self._ser = serial.Serial(self._port, self._baud, timeout=self._timeout)

    def stop(self) -> None:
        if self._ser:
            self._ser.close()
            self._ser = None

    def read_fresh(self, max_retries: int = 8) -> np.ndarray:
        """Return the CURRENT frame, discarding any stale backlog. The module
        streams continuously into the OS serial buffer, which is capped (~4 KB on
        Linux tty): once full it KEEPS the oldest bytes and DROPS new ones, so an
        on-demand reader (e.g. the snapshot calibration tool) that reads once in
        a while gets a stale frame -- the 15-20s lag the user saw. Draining the
        buffer frame-by-frame doesn't help (those buffered frames are all old).

        So: reset_input_buffer() to drop the stale bytes and let freshly-arriving
        data through, then read. Restarting mid-stream means _sync_to_marker can
        first lock onto a false 5A 5A inside the pixel payload (raising Checksum/
        FrameSyncError); since real frames are 1544-aligned, retrying read_frame()
        re-syncs and converges onto a real boundary within a few frames.
        Continuous readers (live preview loop) keep the buffer empty and use
        read_frame() directly."""
        if self._ser:
            self._ser.reset_input_buffer()
        last_err: "Exception | None" = None
        for _ in range(max_retries):
            try:
                return self.read_frame()
            except (ChecksumError, FrameSyncError) as e:
                last_err = e
        raise last_err if last_err else FrameSyncError("read_fresh: no frame")

    def _sync_to_marker(self) -> None:
        marker = bytes(self._marker)
        window = bytearray()
        deadline_bytes = 4096  # bail out rather than spinning forever on garbage
        for _ in range(deadline_bytes):
            b = self._ser.read(1)
            if not b:
                raise FrameSyncError("timed out waiting for start marker")
            window += b
            if len(window) > len(marker):
                window = window[-len(marker):]
            if bytes(window) == marker:
                return
        raise FrameSyncError("start marker not found within read window")

    @staticmethod
    def _sum16_words(buf: bytes) -> int:
        # 16-bit sum of consecutive little-endian words, mod 0x10000.
        return int(np.frombuffer(buf, dtype="<u2").sum(dtype=np.uint64)) & 0xFFFF

    def _checksum_len(self) -> int:
        return {"none": 0, "sum8": 1, "sum16_words": 2}[self._checksum]

    def _verify_checksum(self, checksummed: bytes, chk: bytes) -> None:
        """`checksummed` is every byte the checksum is computed over (marker +
        header + payload); `chk` is the trailing checksum bytes off the wire."""
        if self._checksum == "none":
            return
        if self._checksum == "sum8":
            if (sum(checksummed) & 0xFF) != chk[0]:
                raise ChecksumError("sum8 checksum mismatch")
            return
        if self._checksum == "sum16_words":
            want = int.from_bytes(chk, "little")
            got = self._sum16_words(checksummed)
            if got != want:
                raise ChecksumError(f"sum16_words mismatch: got {got:#06x} want {want:#06x}")
            return
        raise NotImplementedError(f"checksum mode '{self._checksum}' not implemented")

    def read_frame(self) -> np.ndarray:
        """Blocks for one frame; returns (rows, cols) ndarray of degrees C.
        Also updates `self.last_ta_c` from the module's ambient-temp header."""
        self._sync_to_marker()  # consumes the marker bytes
        n_values = self._rows * self._cols
        payload_size = n_values * self._dtype.itemsize
        csum_size = self._checksum_len()

        body = self._ser.read(self._header_bytes + payload_size + csum_size)
        if len(body) != self._header_bytes + payload_size + csum_size:
            raise FrameSyncError(
                f"short read: got {len(body)}, "
                f"expected {self._header_bytes + payload_size + csum_size}"
            )

        header = body[: self._header_bytes]
        payload = body[self._header_bytes : self._header_bytes + payload_size]
        chk = body[self._header_bytes + payload_size :]

        # checksum covers marker + header + payload (everything but the csum itself)
        self._verify_checksum(bytes(self._marker) + header + payload, chk)

        # header layout (this module): [length:2][Ta:2]; expose Ta if present.
        if self._header_bytes >= 4:
            ta_raw = int.from_bytes(header[2:4], "little")
            self.last_ta_c = ta_raw * self._scale + self._offset

        values = np.frombuffer(payload, dtype=self._dtype).astype(np.float64)
        temps_c = (values * self._scale + self._offset).reshape(self._rows, self._cols)
        # orientation correction at the source -- see thermal_config.yaml notes.
        if self._flip_v:
            temps_c = np.flipud(temps_c)
        if self._flip_h:
            temps_c = np.fliplr(temps_c)
        if self._bp_repair:
            temps_c = self._repair_bad_pixels(temps_c)
        return np.ascontiguousarray(temps_c)

    def _repair_bad_pixels(self, frame: np.ndarray) -> np.ndarray:
        """Replace physically-impossible pixels (stuck/dead sensor elements that
        still checksum-clean) with the median of their valid 8-neighbours, so a
        few dead pixels can't blow up the colormap range or trip the fire
        threshold. Updates self.last_bad_pixels."""
        bad = (frame < self._bp_min_c) | (frame > self._bp_max_c)
        self.last_bad_pixels = int(bad.sum())
        if not self.last_bad_pixels:
            return frame
        out = frame.copy()
        rows, cols = frame.shape
        for r, c in np.argwhere(bad):
            r0, r1 = max(r - 1, 0), min(r + 2, rows)
            c0, c1 = max(c - 1, 0), min(c + 2, cols)
            neigh = frame[r0:r1, c0:c1]
            good = neigh[(neigh >= self._bp_min_c) & (neigh <= self._bp_max_c)]
            if good.size:
                out[r, c] = np.median(good)
        return out


if __name__ == "__main__":
    therm = ThermalSerial()
    therm.start()
    try:
        frame = therm.read_frame()
        print(f"frame shape={frame.shape} min={frame.min():.1f}C max={frame.max():.1f}C")
    finally:
        therm.stop()
