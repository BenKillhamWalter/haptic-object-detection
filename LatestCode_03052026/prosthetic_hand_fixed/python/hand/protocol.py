"""Serial-line parsing.

Both firmwares emit three kinds of lines:
    sensor sample:  "S,<ms>,<12 floats>"      (XIAO)
    motor sample:   "M,<ms>,<enc>,<load>,<current>"  (OpenRB)
    event/comment:  "# ..."                   (either board)

Anything else is treated as a free-text log line and forwarded as a
LogLine record for visibility in the console.
"""

from dataclasses import dataclass, field
from typing import Optional, List, Union


@dataclass
class SensorSample:
    """Raw sensor reading from XIAO. Values are MLX90393 raw counts."""
    device_ms: int                # board millis() at sample time
    recv_t: float                 # laptop time.monotonic() at line receipt
    values: List[float] = field(default_factory=list)  # length 12


@dataclass
class MotorSample:
    """Motor state from OpenRB."""
    device_ms: int
    recv_t: float
    enc_deg: float
    load: int
    current: int


@dataclass
class Event:
    """A '#'-prefixed marker line. `name` is the first token, `args` are the rest."""
    name: str
    args: List[str]
    raw: str
    recv_t: float


@dataclass
class LogLine:
    """Any other text line — usually a startup banner or free-form '# something'."""
    text: str
    recv_t: float


ParsedLine = Union[SensorSample, MotorSample, Event, LogLine, None]


def parse_line(line: str, recv_t: float) -> ParsedLine:
    """Parse one already-decoded line of serial text.

    Returns None for empty / whitespace-only input.
    """
    s = line.strip()
    if not s:
        return None

    # Event / comment line.
    if s.startswith('#'):
        body = s[1:].strip()
        tokens = body.split()
        if not tokens:
            return LogLine(text=s, recv_t=recv_t)
        # Treat the first token as the event name. Even ad-hoc comments
        # like "# pong" parse as Event(name="pong") which is convenient.
        return Event(name=tokens[0], args=tokens[1:], raw=s, recv_t=recv_t)

    parts = s.split(',')
    tag = parts[0]

    if tag == 'S' and len(parts) == 14:
        try:
            return SensorSample(
                device_ms=int(parts[1]),
                recv_t=recv_t,
                values=[float(v) for v in parts[2:14]],
            )
        except ValueError:
            return LogLine(text=s, recv_t=recv_t)

    if tag == 'M' and len(parts) == 5:
        try:
            return MotorSample(
                device_ms=int(parts[1]),
                recv_t=recv_t,
                enc_deg=float(parts[2]),
                load=int(parts[3]),
                current=int(parts[4]),
            )
        except ValueError:
            return LogLine(text=s, recv_t=recv_t)

    return LogLine(text=s, recv_t=recv_t)
