#!/usr/bin/env python3
"""Read every controller input from the Black Pill through the ST-Link.

The board runs firmware/controller/controller.ino, which keeps the newest
readings in RAM. This script reads that RAM through the ST-Link (SWD) while
the board keeps running, so no USB cable to the board is needed. (With the
board's USB-C plugged in, any serial monitor shows the same stream.)

    python read_controller.py             print the stream
    python read_controller.py --rate 50   50 lines a second

Each line: time_ms, arm_roll, arm_twist, arm_v (degrees), joy_x, joy_y
(-1..+1), button (0/1), legacy trigger (0/1), trigger_percent (0..100).
Requires the updated CTL2 controller firmware; nan = not calibrated/no answer.

Use it from other code (keep read_trigger.py in the same folder):

    from read_controller import ControllerReader

    with ControllerReader() as controller:
        reading = controller.read()
        print(reading.joy_x, reading.button)

Needs Python 3.8+ and OpenOCD (found the same way as read_trigger.py).
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
import time
from dataclasses import dataclass

from read_trigger import OpenOcd, TriggerError, find_openocd, install_stop_signals

# Must match firmware/controller/controller.ino.
SHARE_MAGIC = 0x324C5443  # "CTL2"
SHARE_WORDS = 12
FLAG_BUTTON = 1 << 0
FLAG_TRIGGER = 1 << 1

RAM_START = 0x20000000
RAM_WORDS = 128 * 1024 // 4
SCAN_CHUNK_WORDS = 1024

# The board is taken as stopped if its counter does not move for this long.
STALL_AFTER_S = 1.0

HEADER = "time_ms,arm_roll,arm_twist,arm_v,joy_x,joy_y,button,trigger,trigger_percent"

ControllerError = TriggerError


@dataclass(frozen=True)
class ControllerReading:
    time_ms: int
    arm_roll: float  # degrees, nan if no answer
    arm_twist: float  # degrees, nan if no answer
    arm_v: float  # degrees 0..360, nan if no answer or no magnet
    joy_x: float  # -1..+1
    joy_y: float  # -1..+1
    joy_x_raw: int  # 0..4095
    joy_y_raw: int  # 0..4095
    button: int  # 1 = pressed
    trigger: int  # legacy 1 = >50% of trigger travel
    trigger_percent: float  # 0..100; nan until full-squeeze calibration


class ControllerReader:
    """Reads the controller through the ST-Link.

    openocd   path of the openocd program, or None to find it
    """

    def __init__(self, openocd: str | None = None) -> None:
        program, scripts = find_openocd(openocd)
        self._ocd = OpenOcd(program, scripts)

        try:
            self._address = self._find_block()
        except BaseException:
            self._ocd.close()
            raise

        self._last_count: int | None = None
        self._last_change = time.monotonic()

    def _find_block(self) -> int:
        for start in range(RAM_START, RAM_START + RAM_WORDS * 4, SCAN_CHUNK_WORDS * 4):
            words = self._ocd.read_words(start, SCAN_CHUNK_WORDS)
            if SHARE_MAGIC in words:
                return start + 4 * words.index(SHARE_MAGIC)

        raise ControllerError(
            "The controller firmware is not running on the board "
            "(flash the updated CTL2 firmware/controller/controller.ino)."
        )

    def __enter__(self) -> ControllerReader:
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.close()

    def read(self) -> ControllerReading:
        """The newest reading."""
        words = self._ocd.read_words(self._address, SHARE_WORDS)

        if words[0] != SHARE_MAGIC:
            raise ControllerError("The board was reset or reflashed: start again.")

        count, time_ms = words[1], words[2]
        now = time.monotonic()

        if count != self._last_count:
            self._last_count = count
            self._last_change = now
        elif now - self._last_change > STALL_AFTER_S:
            raise ControllerError("The board stopped updating (halted or not running).")

        floats = struct.unpack("<5f", struct.pack("<5I", *words[3:8]))
        packed, flags = words[8], words[9]

        return ControllerReading(
            time_ms=time_ms,
            arm_roll=floats[0],
            arm_twist=floats[1],
            arm_v=floats[2],
            joy_x=floats[3],
            joy_y=floats[4],
            joy_x_raw=packed & 0xFFFF,
            joy_y_raw=packed >> 16,
            button=1 if flags & FLAG_BUTTON else 0,
            trigger=1 if flags & FLAG_TRIGGER else 0,
            trigger_percent=struct.unpack("<f", struct.pack("<I", words[11]))[0],
        )

    def close(self) -> None:
        self._ocd.close()


def fmt(value: float) -> str:
    return "nan" if math.isnan(value) else f"{value:.3f}"


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--rate", type=float, default=20.0, help="lines per second (default 20)")
    parser.add_argument("--openocd", help="path of the openocd program")
    args = parser.parse_args()

    rate: float = args.rate
    if not (math.isfinite(rate) and rate > 0):
        print("Error: --rate must be above zero", file=sys.stderr)
        return 1

    install_stop_signals()

    try:
        with ControllerReader(args.openocd) as controller:
            print("Connected through the ST-Link.", file=sys.stderr)
            print(HEADER, flush=True)

            while True:
                r = controller.read()
                print(
                    f"{r.time_ms},{fmt(r.arm_roll)},{fmt(r.arm_twist)},{fmt(r.arm_v)},"
                    f"{fmt(r.joy_x)},{fmt(r.joy_y)},{r.button},{r.trigger},{fmt(r.trigger_percent)}",
                    flush=True,
                )
                time.sleep(1.0 / rate)
    except KeyboardInterrupt:
        print(file=sys.stderr)
    except ControllerError as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
