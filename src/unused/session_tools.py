import json
import math
import time
from datetime import datetime

MODULES = (
    "nav", "state", "collection", "pose", "imu",
    "tof", "ultrasound", "map", "motor", "colour"
)

CORNERS = {
    "sw": (300, 300),
    "se": (4600, 300),
    "nw": (300, 2100),
    "ne": (4600, 2100),
}


def parse_config(line):
    p = line.split(",")

    if len(p) != 10 or p[0] != "CONFIG":
        return None

    if p[1] not in ("blue", "green"):
        return None

    try:
        values = [float(v) for v in p[2:7]]

        if not all(math.isfinite(v) for v in values):
            return None

        hx, hy, x, y, heading = values

        if not (
            0 <= hx <= 4900
            and 0 <= x <= 4900
            and 0 <= hy <= 2400
            and 0 <= y <= 2400
            and 0 <= heading <= 360
        ):
            return None

        if any(v not in ("0", "1") for v in p[7:]):
            return None

        return dict(
            colour=p[1],
            home_x=hx,
            home_y=hy,
            x=x,
            y=y,
            heading=heading % 360,
            locked=p[7] == "1",
            started=p[8] == "1",
            pickup=p[9] == "1",
        )

    except ValueError:
        return None


def command_allowed(command, config, mode, locked=False):
    c = command.strip().lower()

    if "\n" in c or "\r" in c or "\x00" in c:
        return False

    if c in (
    "config", "mode", "gains", "help", "flags",
    "arms", "tof", "targets",
    "debug list", "debug status", "mark fault"
    ):
        return True

    if config is None:
        return False

    started = config["started"]

    if c.startswith(("debug on ", "debug off ")):
        return not started or mode == "BENCH"

    if c == "config lock" or c.startswith(
        ("base ", "home ", "homepos ", "startpose ")
    ):
        return not started and not locked and not config["locked"]
    if c == "targets clear" or c.startswith("target add "):
        return (
            not started
            and not locked
            and not config["locked"]
        )
    if c in ("auto", "roam") and not started:
        return not locked and not config["locked"]

    return mode == "BENCH" and started


def elapsed(ns):
    ms = max(0, ns // 1_000_000)
    hours, ms = divmod(ms, 3_600_000)
    minutes, ms = divmod(ms, 60_000)
    seconds, ms = divmod(ms, 1000)

    return f"{hours:02}:{minutes:02}:{seconds:02}.{ms:03}"


class SessionRecorder:
    def __init__(self):
        self.start_ns = None
        self.end_ns = None
        self.records = []
        self.metadata = {}
        self.saved = True
        self.faults = 0

    @property
    def active(self):
        return self.start_ns is not None and self.end_ns is None

    def start(self, metadata, now=None):
        self.start_ns = time.monotonic_ns() if now is None else now
        self.end_ns = None
        self.records = []
        self.faults = 0
        self.metadata = dict(metadata)
        self.metadata["recorded_at"] = (
            datetime.now().astimezone().isoformat()
        )
        self.saved = False

        self.add("GUI", "Start Test", self.start_ns)

    def add(self, kind, text, now=None):
        now = time.monotonic_ns() if now is None else now

        # Accept queued RX events received before Stop,
        # even when the GUI processes them after Stop.
        if self.start_ns is None or now < self.start_ns:
            return

        if self.end_ns is not None and now > self.end_ns:
            return

        self.records.append((now, len(self.records), kind, text))
        self.saved = False

    def fault(self, now: int | None = None) -> str | None:
        start_ns = self.start_ns

        if not self.active or start_ns is None:
            return None

        now = time.monotonic_ns() if now is None else now
        self.faults += 1

        marker = (
            f"[FAULT MARKER {elapsed(now - start_ns)}] "
            f"#{self.faults}"
        )

        self.add("FAULT", marker, now)
        return marker

    def stop(self, now=None):
        if self.active:
            self.end_ns = time.monotonic_ns() if now is None else now
            self.add("GUI", "Stop Test", self.end_ns)

    def export_text(self):
        if self.start_ns is None:
            raise ValueError("No recorded session")

        lines = [
            "RoboCup test session",
            json.dumps(self.metadata, indent=2, ensure_ascii=False),
            "Times are local receive/send times, elapsed from Start Test.",
            "RX is the raw decoded serial line; TX is a completed serial write.",
            "Firmware MARK,FAULT replies contain robot millis().",
            "--- CHRONOLOGICAL LOG ---",
        ]

        for ns, _, kind, text in sorted(self.records):
            lines.append(
                f"[{elapsed(ns - self.start_ns)}] {kind}: {text}"
            )

        return "\n".join(lines) + "\n"