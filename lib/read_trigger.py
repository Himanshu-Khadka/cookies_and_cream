#!/usr/bin/env python3
"""Read the PS5 trigger from the Black Pill through the ST-Link.

The board runs firmware/ps5_trigger/ps5_trigger.ino, which keeps the newest
analog readings in RAM. This script reads that RAM through the ST-Link (SWD)
with OpenOCD while the board keeps running, so no USB cable to the board is
needed.

    python read_trigger.py              print the trigger
    python read_trigger.py --pin PA1    the trigger is on PA1 (skip finding it)
    python read_trigger.py --all        print every analog pin

Finding the trigger: every candidate pin is read, and the pin that moves the
most once you squeeze the trigger is taken as the trigger. Squeeze it all the
way once: 0% is where it rested at start, 100% the furthest it has gone.

Output: time_ms, trigger percent, raw value (0..4095) and the pin.

Use it from other code:

    from read_trigger import TriggerReader

    with TriggerReader() as trigger:
        reading = trigger.read()
        print(reading.percent, reading.raw)

Needs Python 3.8+ and OpenOCD. OpenOCD comes with the STM32 Arduino core; it
is found there, on PATH, or given with --openocd.
"""

from __future__ import annotations

import argparse
import atexit
import glob
import math
import os
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from typing import IO, Sequence

# Must match firmware/ps5_trigger/ps5_trigger.ino.
PIN_NAMES: tuple[str, ...] = ("PA0", "PA1", "PA4", "PA5", "PA6", "PA7", "PB0", "PB1")
SHARE_MAGIC = 0x31475254  # "TRG1"
SHARE_WORDS = 3 + (len(PIN_NAMES) + 1) // 2  # magic, count, time_ms, packed raw

RAM_START = 0x20000000
RAM_WORDS = 128 * 1024 // 4
SCAN_CHUNK_WORDS = 1024

FULL_SCALE = 4095

# A pin must move at least this much (raw counts) to be taken as the trigger.
DETECT_THRESHOLD = 300

# OpenOCD start-up and replies.
CONNECT_TIMEOUT_S = 10.0
REPLY_TIMEOUT_S = 3.0

# The board is taken as stopped if its counter does not move for this long.
STALL_AFTER_S = 1.0


class TriggerError(Exception):
    """The trigger cannot be read."""


def install_stop_signals() -> None:
    """Turn Ctrl-C, a kill and a closed terminal into KeyboardInterrupt.

    Then the program's own clean-up runs, OpenOCD is closed and the ST-Link
    is free for the next program.
    """

    def stop(signum: int, frame: object) -> None:
        raise KeyboardInterrupt

    for name in ("SIGINT", "SIGTERM", "SIGHUP"):
        number = getattr(signal, name, None)
        if number is not None:
            signal.signal(number, stop)


@dataclass(frozen=True)
class TriggerReading:
    """One reading of the trigger.

    percent   0 at rest, 100 at the furthest point seen so far
              (nan until the trigger has been found)
    raw       the trigger pin's value, 0..4095 (nan until found)
    pin       the trigger pin's name, or None until found
    pins      every candidate pin's raw value
    """

    time_ms: int
    percent: float
    raw: float
    pin: str | None
    pins: dict[str, int]


# --------------------------------------------------------------------------
# OpenOCD
# --------------------------------------------------------------------------


def find_openocd(given: str | None) -> tuple[Path, Path | None]:
    """The openocd program, and its scripts folder when it is a bundled copy."""
    if given:
        path = Path(given)
        if not path.is_file():
            raise TriggerError(f"No openocd at {path}")
        return path, _scripts_next_to(path)

    on_path = shutil.which("openocd")
    if on_path:
        return Path(on_path), _scripts_next_to(Path(on_path))

    homes = [
        Path.home() / "Library" / "Arduino15",
        Path.home() / ".arduino15",
        Path(os.environ.get("LOCALAPPDATA", str(Path.home()))) / "Arduino15",
    ]
    exe = "openocd.exe" if sys.platform == "win32" else "openocd"

    for home in homes:
        pattern = str(
            home / "packages" / "STMicroelectronics" / "tools" / "xpack-openocd"
            / "*" / "bin" / exe
        )
        found = sorted(glob.glob(pattern))
        if found:
            path = Path(found[-1])
            return path, _scripts_next_to(path)

    raise TriggerError(
        "OpenOCD not found. Install the STM32 Arduino core, put openocd on "
        "PATH, or give its path with --openocd."
    )


def _scripts_next_to(program: Path) -> Path | None:
    scripts = program.resolve().parent.parent / "openocd" / "scripts"
    return scripts if scripts.is_dir() else None


def free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.bind(("127.0.0.1", 0))
        port: int = probe.getsockname()[1]
        return port


class OpenOcd:
    """An OpenOCD process attached to the board, driven over its Tcl port.

    It attaches without halting or resetting the chip, so the firmware
    keeps running while its memory is read.
    """

    def __init__(self, program: Path, scripts: Path | None) -> None:
        self._port = free_port()
        self._log: IO[bytes] = tempfile.TemporaryFile()
        command = [str(program)]
        if scripts is not None:
            command += ["-s", str(scripts)]
        command += [
            "-f", "interface/stlink.cfg",
            "-f", "target/stm32f4x.cfg",
            "-c", "gdb_port disabled",
            "-c", "telnet_port disabled",
            "-c", f"tcl_port {self._port}",
            "-c", "init",
        ]

        try:
            self._process = subprocess.Popen(
                command, stdout=self._log, stderr=subprocess.STDOUT
            )
        except OSError as error:
            raise TriggerError(f"Cannot start openocd: {error}") from error

        self._closed = False
        # Close OpenOCD even if the program ends without calling close().
        atexit.register(self.close)
        self._socket = self._connect()

    def _connect(self) -> socket.socket:
        deadline = time.monotonic() + CONNECT_TIMEOUT_S

        while time.monotonic() < deadline:
            if self._process.poll() is not None:
                raise TriggerError(
                    "OpenOCD could not reach the board through the ST-Link. "
                    "Check the ST-Link is plugged in and wired to the board, and "
                    "that no other program is using it (another reader, "
                    "STM32CubeProgrammer, or a leftover openocd).\n"
                    + self._log_tail()
                )
            try:
                connection = socket.create_connection(
                    ("127.0.0.1", self._port), timeout=REPLY_TIMEOUT_S
                )
            except OSError:
                time.sleep(0.1)
                continue
            return connection

        self.close()
        raise TriggerError("OpenOCD did not start in time:\n" + self._log_tail())

    def _log_tail(self) -> str:
        self._log.seek(0)
        lines = self._log.read().decode("utf-8", errors="replace").splitlines()
        useful = [line for line in lines if line.startswith(("Error", "Warn"))]
        return "\n".join((useful or lines)[-6:])

    def command(self, text: str) -> str:
        """Run one OpenOCD command and return its reply."""
        try:
            self._socket.sendall(text.encode("ascii") + b"\x1a")
            reply = b""
            while not reply.endswith(b"\x1a"):
                chunk = self._socket.recv(65536)
                if not chunk:
                    raise TriggerError("OpenOCD closed the connection.")
                reply += chunk
        except OSError as error:
            raise TriggerError(f"Lost the connection to OpenOCD: {error}") from error
        return reply[:-1].decode("ascii", errors="replace").strip()

    def read_words(self, address: int, count: int) -> list[int]:
        """count 32-bit words from the board's memory."""
        reply = self.command(f"read_memory 0x{address:08x} 32 {count}")
        try:
            words = [int(value, 16) for value in reply.split()]
        except ValueError:
            words = []
        if len(words) != count:
            raise TriggerError(f"Could not read the board's memory: {reply[:200]}")
        return words

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            if hasattr(self, "_socket"):
                self._socket.sendall(b"shutdown\x1a")
                self._socket.close()
        except OSError:
            pass
        try:
            self._process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self._process.kill()
            self._process.wait()
        self._log.close()


# --------------------------------------------------------------------------
# The trigger
# --------------------------------------------------------------------------


def unpack(words: Sequence[int]) -> tuple[int, int, dict[str, int]]:
    """(count, time_ms, raw value per pin) from the firmware's RAM block."""
    raw: list[int] = []
    for word in words[3:]:
        raw += [word & 0xFFFF, word >> 16]
    return words[1], words[2], dict(zip(PIN_NAMES, raw))


class TriggerReader:
    """Reads the trigger through the ST-Link.

    pin       the trigger's pin name (e.g. "PA1"), or None to find it
    openocd   path of the openocd program, or None to find it
    """

    def __init__(self, pin: str | None = None, openocd: str | None = None) -> None:
        if pin is not None and pin.upper() not in PIN_NAMES:
            raise TriggerError(
                f"{pin} is not read by the firmware; use one of {', '.join(PIN_NAMES)}"
            )

        self._fixed_pin = pin.upper() if pin else None
        program, scripts = find_openocd(openocd)
        self._ocd = OpenOcd(program, scripts)

        try:
            self._address = self._find_block()
        except BaseException:
            self._ocd.close()
            raise

        self._rest: dict[str, int] = {}
        self._furthest: dict[str, int] = {}
        self._last_count: int | None = None
        self._last_change = time.monotonic()

    def _find_block(self) -> int:
        """Address of the firmware's block in RAM (normally the very start)."""
        for start in range(RAM_START, RAM_START + RAM_WORDS * 4, SCAN_CHUNK_WORDS * 4):
            words = self._ocd.read_words(start, SCAN_CHUNK_WORDS)
            if SHARE_MAGIC in words:
                return start + 4 * words.index(SHARE_MAGIC)

        raise TriggerError(
            "The trigger firmware is not running on the board "
            "(flash firmware/ps5_trigger/ps5_trigger.ino)."
        )

    def __enter__(self) -> TriggerReader:
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.close()

    def read(self) -> TriggerReading:
        """The newest reading."""
        words = self._ocd.read_words(self._address, SHARE_WORDS)

        if words[0] != SHARE_MAGIC:
            raise TriggerError("The board was reset or reflashed: start again.")

        count, time_ms, pins = unpack(words)
        now = time.monotonic()

        if count != self._last_count:
            self._last_count = count
            self._last_change = now
        elif now - self._last_change > STALL_AFTER_S:
            raise TriggerError("The board stopped updating (halted or not running).")

        for name, value in pins.items():
            self._rest.setdefault(name, value)
            rest = self._rest[name]
            if abs(value - rest) > abs(self._furthest.get(name, rest) - rest):
                self._furthest[name] = value

        pin = self._fixed_pin or self._detected_pin()

        if pin is None:
            return TriggerReading(time_ms, math.nan, math.nan, None, pins)

        rest = self._rest[pin]
        span = abs(self._furthest.get(pin, rest) - rest)
        moved = abs(pins[pin] - rest)

        # Until the trigger has been squeezed, the span is only noise.
        percent = 100.0 * moved / span if span >= DETECT_THRESHOLD else math.nan

        return TriggerReading(time_ms, min(percent, 100.0), float(pins[pin]), pin, pins)

    def _detected_pin(self) -> str | None:
        """The pin that has moved the most, once it has moved enough."""
        best: str | None = None
        best_move = DETECT_THRESHOLD - 1

        for name, rest in self._rest.items():
            move = abs(self._furthest.get(name, rest) - rest)
            if move > best_move:
                best, best_move = name, move

        return best

    def close(self) -> None:
        self._ocd.close()


# --------------------------------------------------------------------------
# Command line
# --------------------------------------------------------------------------


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--pin", help=f"the trigger's pin: {', '.join(PIN_NAMES)}")
    parser.add_argument("--all", action="store_true", help="print every analog pin")
    parser.add_argument("--rate", type=float, default=20.0, help="lines per second (default 20)")
    parser.add_argument("--openocd", help="path of the openocd program")
    args = parser.parse_args()

    rate: float = args.rate
    if not (math.isfinite(rate) and rate > 0):
        print("Error: --rate must be above zero", file=sys.stderr)
        return 1

    install_stop_signals()

    try:
        with TriggerReader(args.pin, args.openocd) as trigger:
            print("Connected through the ST-Link.", file=sys.stderr)
            if not args.pin:
                print("Squeeze the trigger all the way once to find its pin.", file=sys.stderr)
            else:
                print(
                    "Squeeze the trigger all the way once: the % starts after that.",
                    file=sys.stderr,
                )

            while True:
                reading = trigger.read()

                if args.all:
                    print(
                        f"time_ms={reading.time_ms:<8} "
                        + " ".join(f"{name}={value:4d}" for name, value in reading.pins.items()),
                        flush=True,
                    )
                elif reading.pin is None:
                    print(f"time_ms={reading.time_ms:<8} trigger=  (not found yet)", flush=True)
                else:
                    percent = (
                        "  --- " if math.isnan(reading.percent) else f"{reading.percent:5.1f}%"
                    )
                    volts = reading.raw * 3.3 / FULL_SCALE
                    print(
                        f"time_ms={reading.time_ms:<8} trigger={percent} "
                        f"raw={int(reading.raw):4d} volts={volts:.3f} pin={reading.pin}",
                        flush=True,
                    )

                time.sleep(1.0 / rate)
    except KeyboardInterrupt:
        print(file=sys.stderr)
    except TriggerError as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
