"""Dual-serial manager for the XIAO + OpenRB pair.

Auto-detects the two boards by USB VID/PID hint and by handshake:
each board responds to "e\n" with a distinctive header line ("# ENUM ..."
from XIAO, "# MOTOR ..." from OpenRB). If hints match nothing we fall
back to opening every available CDC port and identifying by handshake.

Threaded readers (one per port) decode incoming lines and push parsed
records into per-stream deques. Two helper deques receive event markers
separately so grasp.py can listen for STABLE_DETECTED / T2_CAPTURE
without scanning through thousands of sensor samples.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Tuple

import serial
import serial.tools.list_ports

from . import constants as C
from . import protocol as P


# ----------------------------------------------------------------
# Port auto-detection
# ----------------------------------------------------------------
def _hint_match(port_info, hints) -> bool:
    """Match a serial port against a (vid, pid_or_None) hint list."""
    for vid, pid in hints:
        if port_info.vid == vid and (pid is None or port_info.pid == pid):
            return True
    return False


def _candidate_ports(hints) -> list:
    return [p.device for p in serial.tools.list_ports.comports() if _hint_match(p, hints)]


def _all_candidate_ports() -> list:
    return [p.device for p in serial.tools.list_ports.comports()
            if p.vid is not None]  # exclude built-in non-USB tty entries


def _try_handshake(port: str, want: str, timeout_s: float = 2.0) -> bool:
    """Open `port`, send 'e\\n', return True iff a line containing `want` arrives in `timeout_s`."""
    try:
        ser = serial.Serial(port, C.SERIAL_BAUD, timeout=0.2)
    except (serial.SerialException, OSError):
        return False
    try:
        time.sleep(0.4)            # let the board's bootloader settle
        ser.reset_input_buffer()
        ser.write(b'e\n')
        ser.flush()
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                line = ser.readline().decode(errors='ignore')
            except Exception:
                line = ''
            if want in line:
                return True
        return False
    finally:
        ser.close()


def detect_boards() -> Tuple[Optional[str], Optional[str]]:
    """Return (xiao_port, openrb_port). Either may be None if not found."""
    # First pass: USB VID/PID hint match, then handshake-confirm.
    xiao_hints   = _candidate_ports(C.XIAO_USB_HINTS)
    openrb_hints = _candidate_ports(C.OPENRB_USB_HINTS)

    xiao_port:   Optional[str] = None
    openrb_port: Optional[str] = None

    for p in xiao_hints:
        if _try_handshake(p, 'ENUM'):
            xiao_port = p
            break

    for p in openrb_hints:
        if p == xiao_port:
            continue
        if _try_handshake(p, 'MOTOR'):
            openrb_port = p
            break

    # Second pass: brute-force handshake on any unidentified USB CDC port.
    if xiao_port is None or openrb_port is None:
        for p in _all_candidate_ports():
            if p in (xiao_port, openrb_port):
                continue
            if xiao_port is None and _try_handshake(p, 'ENUM'):
                xiao_port = p
                continue
            if openrb_port is None and _try_handshake(p, 'MOTOR'):
                openrb_port = p

    return xiao_port, openrb_port


# ----------------------------------------------------------------
# Per-port reader thread
# ----------------------------------------------------------------
class _ReaderThread(threading.Thread):
    def __init__(self, ser: serial.Serial, sample_dq: Deque, event_dq: Deque, log_dq: Deque, label: str):
        super().__init__(daemon=True, name=f'reader-{label}')
        self._ser = ser
        self._sample_dq = sample_dq
        self._event_dq  = event_dq
        self._log_dq    = log_dq
        self._stop      = threading.Event()
        self._label     = label

    def stop(self):
        self._stop.set()

    def run(self):
        buf = b''
        while not self._stop.is_set():
            try:
                chunk = self._ser.read(256)
            except (serial.SerialException, OSError):
                break
            if not chunk:
                continue
            buf += chunk
            while b'\n' in buf:
                raw, buf = buf.split(b'\n', 1)
                text = raw.decode(errors='ignore')
                recv = time.monotonic()
                rec = P.parse_line(text, recv)
                if rec is None:
                    continue
                if isinstance(rec, (P.SensorSample, P.MotorSample)):
                    self._sample_dq.append(rec)
                elif isinstance(rec, P.Event):
                    self._event_dq.append(rec)
                    self._log_dq.append(rec.raw)
                elif isinstance(rec, P.LogLine):
                    self._log_dq.append(rec.text)


# ----------------------------------------------------------------
# Main controller
# ----------------------------------------------------------------
@dataclass
class HandController:
    xiao_port:   str
    openrb_port: str
    buffer_size: int = 5000          # ~100 s of 50 Hz samples per stream

    def __post_init__(self):
        self.xiao_ser   = serial.Serial(self.xiao_port,   C.SERIAL_BAUD, timeout=0.05)
        self.openrb_ser = serial.Serial(self.openrb_port, C.SERIAL_BAUD, timeout=0.05)

        # Allow ESP32-S3 / OpenRB to finish booting after the open
        # triggers a DTR reset on some boards.
        time.sleep(0.4)
        self.xiao_ser.reset_input_buffer()
        self.openrb_ser.reset_input_buffer()

        self.sensor_dq:   Deque[P.SensorSample] = deque(maxlen=self.buffer_size)
        self.motor_dq:    Deque[P.MotorSample]  = deque(maxlen=self.buffer_size)
        self.xiao_evts:   Deque[P.Event]        = deque(maxlen=200)
        self.openrb_evts: Deque[P.Event]        = deque(maxlen=200)
        self.xiao_log:    Deque[str]            = deque(maxlen=500)
        self.openrb_log:  Deque[str]            = deque(maxlen=500)

        self._xiao_reader = _ReaderThread(
            self.xiao_ser, self.sensor_dq, self.xiao_evts, self.xiao_log, 'xiao')
        self._openrb_reader = _ReaderThread(
            self.openrb_ser, self.motor_dq, self.openrb_evts, self.openrb_log, 'openrb')
        self._xiao_reader.start()
        self._openrb_reader.start()

    # ---- command senders ----
    def send_xiao(self, cmd: str):
        self.xiao_ser.write((cmd + '\n').encode())
        self.xiao_ser.flush()

    def send_openrb(self, cmd: str):
        self.openrb_ser.write((cmd + '\n').encode())
        self.openrb_ser.flush()

    # ---- buffer access ----
    def clear_buffers(self):
        self.sensor_dq.clear()
        self.motor_dq.clear()
        self.xiao_evts.clear()
        self.openrb_evts.clear()

    def wait_for_event(self, source: str, name: str, timeout_s: float):
        """Block until an event named `name` arrives on `source` ('xiao' or 'openrb').

        Returns the Event, or None on timeout.
        """
        dq = self.xiao_evts if source == 'xiao' else self.openrb_evts
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            while dq:
                evt = dq.popleft()
                if evt.name == name:
                    return evt
            time.sleep(0.005)
        return None

    def ping_both(self) -> Tuple[bool, bool]:
        """Send 'p' to each board, return (xiao_alive, openrb_alive)."""
        self.xiao_evts.clear()
        self.openrb_evts.clear()
        self.send_xiao('p')
        self.send_openrb('p')
        xiao_ok   = self.wait_for_event('xiao',   'pong', 2.0) is not None
        openrb_ok = self.wait_for_event('openrb', 'pong', 2.0) is not None
        return xiao_ok, openrb_ok

    def enum_xiao(self) -> Optional[P.Event]:
        """Ask the XIAO for its enumeration string. Returns the Event or None."""
        self.xiao_evts.clear()
        self.send_xiao('e')
        return self.wait_for_event('xiao', 'ENUM', 2.0)

    def enum_openrb(self) -> Optional[P.Event]:
        self.openrb_evts.clear()
        self.send_openrb('e')
        return self.wait_for_event('openrb', 'MOTOR', 2.0)

    def streams_on(self):
        self.send_xiao('s')
        self.send_openrb('s')

    def streams_off(self):
        self.send_xiao('x')
        self.send_openrb('x')

    def close(self):
        try:
            self.streams_off()
        except Exception:
            pass
        self._xiao_reader.stop()
        self._openrb_reader.stop()
        time.sleep(0.1)
        try:
            self.xiao_ser.close()
        except Exception:
            pass
        try:
            self.openrb_ser.close()
        except Exception:
            pass

    # context manager
    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.close()
