#!/usr/bin/env python3
"""Drive the SO-101 arm from the Black Pill controller. One file, all of it.

Reads the controller (firmware/controller: MKS 1, MKS 2, AS5600 rail,
joystick, button, trigger), turns it into arm motion with its own inverse
kinematics, and sends it to the arm through lerobot. Nothing else from this
repository is needed.

    python control_arm.py                     dry run: read the controller, print
                                              what would be sent, touch no arm
    python control_arm.py --stlink            same, controller read through the ST-Link
    python control_arm.py --fake              same, with made-up input (no board)
    python control_arm.py --live --arm-port /dev/ttyACM0      MOVE THE REAL ARM

WHAT MOVES WHAT

    button      clutch: hold it (1) to drive the arm, let go (0) to freeze
    arm_twist   shoulder_pan (base turns left / right)
    arm_roll    wrist_roll
    joy_y       forward / back, through the inverse kinematics (speed control:
                push further = faster; centred = stop)
    arm_v       up / down, through the inverse kinematics (the rail's position
                sets the height)
    trigger     gripper: 1 = closed, 0 = open

Every motion is relative: when you press the clutch, the arm's current pose
and the controller's current values become the zero. So no home file and no
saved encoder center are needed, and pressing the clutch never jumps.
The gripper tilt is kept as it was when the clutch was pressed.

SAFETY (deliberate; read before changing)

  - Dry run unless --live.
  - Nothing is sent until the clutch is held and the arm pose has been read.
  - Speed limits on the tool (--tip-speed), on pan/roll (--angle-speed), on
    the three flex joints (--joint-speed) and on the gripper.
  - A step that would make a flex joint jump (elbow flipping over), or bring
    the elbow, wrist or tool below --floor, is refused: the arm stops there.
  - A part that stops answering, or a stale stream, holds that motion where
    the arm is. The joystick only drives once it has been seen centred, so a
    disconnected (floating) joystick never moves the arm.
  - On exit the motors stay on until you press Enter, so the arm does not
    drop. Ctrl-C at that prompt does not switch them off. Support the arm.
  - lerobot's connect() switches the motors off and on once while it writes
    their settings: start with the arm resting or supported.

NOT TESTED ON THE REAL ARM YET. First --live run: low speeds, hand on the power.

Needs Python 3.9+, numpy and pyserial (both come with lerobot). --live also
needs lerobot. --stlink also needs OpenOCD (it comes with the STM32 Arduino
core; it is found there, on PATH, or given with --openocd).
"""

from __future__ import annotations

import argparse
import atexit
import glob
import json
import math
import os
import shutil
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import IO, Any, Optional, Protocol

import numpy as np
from numpy.typing import NDArray

Vector = NDArray[np.float64]

HERE = Path(__file__).resolve().parent
DEFAULT_CALIBRATION = HERE.parent / "calibration_file" / "my_follower.json"


class ControlError(Exception):
    """The controller cannot start or continue."""


# ==========================================================================
# 1. Inverse kinematics (SO-101, from so101_new_calib.urdf)
#
# The arm's three flex joints (shoulder_lift, elbow_flex, wrist_flex) move
# the tool in the vertical plane the arm points in. Coordinates: metres,
# (forward, up) from the shoulder_lift axis. Joint angles: radians, equal to
# lerobot degrees (0 = middle of the calibrated range).
# ==========================================================================

FLEX = ("shoulder_lift", "elbow_flex", "wrist_flex")
ALL_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")

V1 = np.array([0.0280, 0.1126])  # shoulder_lift axis -> elbow_flex axis
V2 = np.array([0.1349, 0.0052])  # elbow_flex axis -> wrist_flex axis
V3 = np.array([0.1592, -0.0079])  # wrist_flex axis -> tool point (between the jaws)
SHOULDER_HEIGHT = 0.1166  # shoulder_lift axis above the base plane

L1, L2, L3 = (float(np.hypot(*v)) for v in (V1, V2, V3))
A1, A2, A3 = (float(np.arctan2(v[1], v[0])) for v in (V1, V2, V3))

# The tool point sits this far from the wrist_roll axis, so rolling can
# lower it by up to twice this. The floor check allows for it.
ROLL_RADIUS_M = 0.0079

TICKS_PER_TURN = 4096
DEG_PER_TICK = 360.0 / (TICKS_PER_TURN - 1)


def forward_kinematics(q: Vector) -> tuple[Vector, float]:
    """Joint radians (3,) -> tool point (forward, up) in metres, and tilt in radians."""
    q1, q2, q3 = (float(v) for v in q)
    tip = (
        L1 * np.array([math.cos(A1 - q1), math.sin(A1 - q1)])
        + L2 * np.array([math.cos(A2 - q1 - q2), math.sin(A2 - q1 - q2)])
        + L3 * np.array([math.cos(A3 - q1 - q2 - q3), math.sin(A3 - q1 - q2 - q3)])
    )
    return tip, A3 - (q1 + q2 + q3)


def inverse_kinematics(target: Vector, tilt: float, low: Vector, high: Vector) -> list[Vector]:
    """Every joint solution (both elbow branches) that reaches the tool point
    at this tilt and stays inside [low, high]."""
    total = A3 - tilt
    wrist = target - L3 * np.array([math.cos(tilt), math.sin(tilt)])
    c = (float(wrist @ wrist) - L1 ** 2 - L2 ** 2) / (2 * L1 * L2)

    if abs(c) > 1.0:
        return []

    d = math.acos(c)
    solutions: list[Vector] = []

    for delta in (d, -d):
        q2 = A2 - A1 - delta
        q1 = A1 - (math.atan2(wrist[1], wrist[0]) - math.atan2(L2 * math.sin(delta), L1 + L2 * math.cos(delta)))
        q = np.array([q1, q2, total - q1 - q2])
        q = (q + math.pi) % (2 * math.pi) - math.pi
        if np.all(q >= low - 1e-9) and np.all(q <= high + 1e-9):
            solutions.append(q)

    return solutions


def lowest_point_height(q: Vector) -> float:
    """Height in metres above the base plane of the lowest of elbow, wrist and tool."""
    q1, q2, q3 = (float(v) for v in q)
    elbow = L1 * math.sin(A1 - q1)
    wrist = elbow + L2 * math.sin(A2 - q1 - q2)
    tool = wrist + L3 * math.sin(A3 - q1 - q2 - q3) - 2 * ROLL_RADIUS_M
    return SHOULDER_HEIGHT + min(elbow, wrist, tool)


@dataclass(frozen=True)
class JointLimits:
    """Limits in lerobot units: degrees (0 = middle of the range); gripper 0..100."""

    pan: tuple[float, float]
    roll: tuple[float, float]
    flex_low: Vector  # radians
    flex_high: Vector  # radians


def load_limits(calibration: Path, margin_deg: float) -> JointLimits:
    """Joint limits from a lerobot calibration file, kept margin_deg inside."""
    try:
        data: Any = json.loads(calibration.read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        raise ControlError(f"Cannot read the calibration file {calibration}: {error}") from error

    def half_range(joint: str) -> tuple[float, float]:
        try:
            low, high = float(data[joint]["range_min"]), float(data[joint]["range_max"])
        except (KeyError, TypeError, ValueError) as error:
            raise ControlError(f"{calibration}: no usable range for {joint}") from error
        half = (high - low) / 2 * DEG_PER_TICK
        if half - margin_deg <= 0:
            raise ControlError(f"--margin {margin_deg} is larger than the range of {joint}")
        return -half + margin_deg, half - margin_deg

    flex = [half_range(joint) for joint in FLEX]
    return JointLimits(
        pan=half_range("shoulder_pan"),
        roll=half_range("wrist_roll"),
        flex_low=np.radians([low for low, _ in flex]),
        flex_high=np.radians([high for _, high in flex]),
    )


# ==========================================================================
# 2. Reading the controller
# ==========================================================================

# USB identity of the Black Pill running the Arduino STM32 core.
BOARD_VID, BOARD_PID = 0x0483, 0x5740

# firmware/controller RAM block, read over SWD.
SHARE_MAGIC = 0x314C5443  # "CTL1"
SHARE_WORDS = 10
RAM_START, RAM_BYTES = 0x20000000, 128 * 1024


@dataclass(frozen=True)
class Sample:
    """One controller reading. nan / None = that part did not answer."""

    received: float  # time.monotonic() when it arrived
    arm_roll: float  # degrees (MKS 1)
    arm_twist: float  # degrees (MKS 2)
    rail: float  # rail gear degrees, counted across turns (nan if none)
    rail_trusted: bool  # False once the turn count may have slipped
    joy_x: float  # -1..+1
    joy_y: float  # -1..+1
    button: Optional[int]  # 1 = pressed
    trigger: Optional[int]  # 1 = on


def finite_or_nan(text: str) -> float:
    try:
        value = float(text)
    except ValueError:
        return math.nan
    return value if math.isfinite(value) else math.nan


def switch(text: str) -> Optional[int]:
    value = text.strip()
    return int(value) if value in ("0", "1") else None


class RailUnwrapper:
    """Turns the AS5600's 0..360 angle into one that keeps counting turns.

    Needs the gear to move less than half a turn between two readings; if it
    seems to jump more than a third of a turn, or readings stop for too long,
    the count is no longer trusted.
    """

    SUSPECT_STEP_DEG = 120.0
    SUSPECT_GAP_S = 0.25

    def __init__(self) -> None:
        self._last: Optional[float] = None
        self._last_time = 0.0
        self._total = 0.0
        self.trusted = True

    def update(self, angle: float, now: float) -> float:
        if math.isnan(angle):
            return math.nan
        if self._last is None:
            self._total = angle
        else:
            step = (angle - self._last + 180.0) % 360.0 - 180.0
            if abs(step) > self.SUSPECT_STEP_DEG or now - self._last_time > self.SUSPECT_GAP_S:
                self.trusted = False
            self._total += step
        self._last, self._last_time = angle, now
        return self._total


class Controller(Protocol):
    def latest(self) -> Optional[Sample]: ...
    def close(self) -> None: ...


class _ReaderThread:
    """Reads in the background; latest() returns the newest Sample."""

    def __init__(self, name: str) -> None:
        self._lock = threading.Lock()
        self._latest: Optional[Sample] = None
        self._error: Optional[str] = None
        self._stop = threading.Event()
        self._rail = RailUnwrapper()
        self._thread = threading.Thread(target=self._run_safely, name=name, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def _run_safely(self) -> None:
        try:
            self._run()
        except Exception as error:  # reported to the main loop by latest()
            with self._lock:
                self._error = str(error) or type(error).__name__

    def _run(self) -> None:
        raise NotImplementedError

    def _publish(self, roll: float, twist: float, gear: float, joy_x: float, joy_y: float,
                 button: Optional[int], trigger: Optional[int]) -> None:
        now = time.monotonic()
        rail = self._rail.update(gear, now)
        sample = Sample(now, roll, twist, rail, self._rail.trusted, joy_x, joy_y, button, trigger)
        with self._lock:
            self._latest = sample

    def latest(self) -> Optional[Sample]:
        with self._lock:
            if self._error is not None:
                raise ControlError(f"Controller: {self._error}")
            return self._latest

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)


class UsbController(_ReaderThread):
    """The controller firmware's CSV stream over the Black Pill's USB-C."""

    def __init__(self, port: Optional[str]) -> None:
        super().__init__("controller-usb")
        try:
            import serial
            import serial.tools.list_ports
        except ImportError as error:
            raise ControlError("pyserial is needed for USB (pip install pyserial), or use --stlink") from error

        if port is None:
            found = [p.device for p in serial.tools.list_ports.comports()
                     if p.vid == BOARD_VID and p.pid == BOARD_PID]
            if not found:
                raise ControlError("No Black Pill on USB. Plug in its USB-C, give --encoder-port, or use --stlink.")
            if len(found) > 1:
                raise ControlError(f"Several Black Pills found ({', '.join(found)}): choose one with --encoder-port.")
            port = found[0]

        try:
            self._port = serial.Serial(port, 115200, timeout=0.2)
        except (serial.SerialException, OSError) as error:
            raise ControlError(f"Cannot open {port}: {error}") from error

        self.where = port
        self.start()

    def _run(self) -> None:
        buffer = b""
        while not self._stop.is_set():
            chunk = self._port.read(256)
            if not chunk:
                continue
            buffer += chunk
            while b"\n" in buffer:
                raw, buffer = buffer.split(b"\n", 1)
                parts = raw.decode("ascii", errors="replace").strip().split(",")
                if len(parts) != 8 or not parts[0].strip().isdigit():
                    continue  # header, or another firmware's line
                self._publish(
                    finite_or_nan(parts[1]), finite_or_nan(parts[2]), finite_or_nan(parts[3]),
                    finite_or_nan(parts[4]), finite_or_nan(parts[5]),
                    switch(parts[6]), switch(parts[7]),
                )
            if len(buffer) > 4096:
                buffer = b""

    def close(self) -> None:
        super().close()
        self._port.close()


def find_openocd(given: Optional[str]) -> tuple[Path, Optional[Path]]:
    """The openocd program and, for a bundled copy, its scripts folder."""
    def scripts_for(program: Path) -> Optional[Path]:
        folder = program.resolve().parent.parent / "openocd" / "scripts"
        return folder if folder.is_dir() else None

    if given:
        if not Path(given).is_file():
            raise ControlError(f"No openocd at {given}")
        return Path(given), scripts_for(Path(given))

    on_path = shutil.which("openocd")
    if on_path:
        return Path(on_path), scripts_for(Path(on_path))

    exe = "openocd.exe" if sys.platform == "win32" else "openocd"
    for home in (Path.home() / "Library" / "Arduino15", Path.home() / ".arduino15",
                 Path(os.environ.get("LOCALAPPDATA", str(Path.home()))) / "Arduino15"):
        found = sorted(glob.glob(str(home / "packages" / "STMicroelectronics" / "tools"
                                     / "xpack-openocd" / "*" / "bin" / exe)))
        if found:
            return Path(found[-1]), scripts_for(Path(found[-1]))

    raise ControlError("OpenOCD not found: install the STM32 Arduino core, put openocd on PATH, or use --openocd.")


class StlinkController(_ReaderThread):
    """The controller firmware's RAM block, read through the ST-Link (SWD)
    while the board keeps running. No USB cable to the board is needed."""

    def __init__(self, openocd: Optional[str]) -> None:
        super().__init__("controller-stlink")
        program, scripts = find_openocd(openocd)

        with socket.socket() as probe:
            probe.bind(("127.0.0.1", 0))
            port = probe.getsockname()[1]

        self._log: IO[bytes] = tempfile.TemporaryFile()
        command = [str(program)] + (["-s", str(scripts)] if scripts else []) + [
            "-f", "interface/stlink.cfg", "-f", "target/stm32f4x.cfg",
            "-c", "gdb_port disabled", "-c", "telnet_port disabled",
            "-c", f"tcl_port {port}", "-c", "init",
        ]
        try:
            self._ocd = subprocess.Popen(command, stdout=self._log, stderr=subprocess.STDOUT)
        except OSError as error:
            raise ControlError(f"Cannot start openocd: {error}") from error

        self._closed = False
        atexit.register(self.close)

        try:
            self._socket = self._connect(port)
            self._address = self._find_block()
        except BaseException:
            self.close()
            raise

        self.where = "ST-Link"
        self.start()

    def _connect(self, port: int) -> socket.socket:
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            if self._ocd.poll() is not None:
                self._log.seek(0)
                tail = self._log.read().decode(errors="replace").strip().splitlines()[-3:]
                raise ControlError(
                    "OpenOCD could not reach the board through the ST-Link (is it plugged in "
                    "and wired, and no other program using it?): " + " | ".join(tail))
            try:
                return socket.create_connection(("127.0.0.1", port), timeout=3.0)
            except OSError:
                time.sleep(0.1)
        raise ControlError("OpenOCD did not start in time.")

    def _words(self, address: int, count: int) -> list[int]:
        self._socket.sendall(f"read_memory 0x{address:08x} 32 {count}".encode() + b"\x1a")
        reply = b""
        while not reply.endswith(b"\x1a"):
            chunk = self._socket.recv(65536)
            if not chunk:
                raise ControlError("OpenOCD closed the connection.")
            reply += chunk
        try:
            words = [int(v, 16) for v in reply[:-1].decode(errors="replace").split()]
        except ValueError:
            words = []
        if len(words) != count:
            raise ControlError("Could not read the board's memory (board unplugged or reset?).")
        return words

    def _find_block(self) -> int:
        for start in range(RAM_START, RAM_START + RAM_BYTES, 4096):
            words = self._words(start, 1024)
            if SHARE_MAGIC in words:
                return start + 4 * words.index(SHARE_MAGIC)
        raise ControlError("The controller firmware is not running on the board (flash firmware/controller).")

    def _run(self) -> None:
        last_count: Optional[int] = None
        while not self._stop.is_set():
            w = self._words(self._address, SHARE_WORDS)
            if w[0] != SHARE_MAGIC:
                raise ControlError("The board was reset or reflashed: start again.")
            if w[1] != last_count:
                last_count = w[1]
                roll, twist, gear, jx, jy = struct.unpack("<5f", struct.pack("<5I", *w[3:8]))
                self._publish(
                    roll if math.isfinite(roll) else math.nan,
                    twist if math.isfinite(twist) else math.nan,
                    gear if math.isfinite(gear) else math.nan,
                    jx, jy, w[9] & 1, (w[9] >> 1) & 1,
                )
            time.sleep(0.004)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        super().close() if self._thread.is_alive() else None
        try:
            if hasattr(self, "_socket"):
                self._socket.sendall(b"shutdown\x1a")
                self._socket.close()
        except OSError:
            pass
        try:
            self._ocd.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self._ocd.kill()
        self._log.close()


class FakeController(_ReaderThread):
    """Smooth made-up input, for trying everything with no board."""

    def __init__(self) -> None:
        super().__init__("controller-fake")
        self.where = "fake input"
        self.start()

    def _run(self) -> None:
        start = time.monotonic()
        while not self._stop.is_set():
            t = time.monotonic() - start
            self._publish(
                80 * math.sin(t * 0.5), 60 * math.sin(t * 0.4), 180 * math.sin(t * 0.3),
                0.0, 0.0 if t < 1 else 0.8 * math.sin(t * 0.6),
                1 if t > 0.5 else 0, 1 if math.sin(t * 0.25) > 0 else 0,
            )
            time.sleep(0.02)


# ==========================================================================
# 3. The arm
# ==========================================================================


class Arm(Protocol):
    calibration: Path

    def read(self) -> dict[str, float]: ...
    def send(self, goals: dict[str, float]) -> None: ...
    def release(self) -> None: ...


class DryRunArm:
    """Pretends to be an arm that reaches every goal at once. Sends nothing."""

    def __init__(self, calibration: Path) -> None:
        self.calibration = calibration
        # A typical bent working pose (lerobot degrees; gripper 0..100).
        self._pose = {"shoulder_pan": 0.0, "shoulder_lift": -45.0, "elbow_flex": 60.0,
                      "wrist_flex": 30.0, "wrist_roll": 0.0, "gripper": 20.0}

    def read(self) -> dict[str, float]:
        return dict(self._pose)

    def send(self, goals: dict[str, float]) -> None:
        self._pose.update(goals)

    def release(self) -> None:
        return None


class RealArm:
    """The SO-101 follower through lerobot (degrees; gripper 0..100)."""

    def __init__(self, port: str, robot_id: str, calibration_dir: Optional[Path]) -> None:
        try:
            from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig
            from lerobot.robots.so_follower.so_follower import SOFollower
        except ImportError as error:
            raise ControlError("--live needs lerobot (pip install 'lerobot[feetech]')") from error

        settings: dict[str, Any] = {"port": port, "id": robot_id, "use_degrees": True}
        if calibration_dir is not None:
            settings["calibration_dir"] = calibration_dir
        self._robot = SOFollower(SOFollowerRobotConfig(**settings))
        self.calibration = Path(self._robot.calibration_fpath)

        if not self.calibration.is_file():
            raise ControlError(
                f"No calibration file at {self.calibration}. Run lerobot-calibrate, or "
                "give --calibration-dir / --id.")

    def connect(self) -> None:
        # calibrate=False: never rewrite the servos' stored calibration from here.
        try:
            self._robot.connect(calibrate=False)
            calibrated = bool(self._robot.is_calibrated)
        except BaseException:
            if getattr(self._robot, "is_connected", False):
                wait_for_release()
                self._robot.disconnect()
            raise
        if not calibrated:
            wait_for_release()
            self._robot.disconnect()
            raise ControlError("The servos' stored calibration does not match the calibration "
                               "file. Fix that with lerobot-calibrate first.")

    def read(self) -> dict[str, float]:
        observation = self._robot.get_observation()
        return {key[:-4]: float(value) for key, value in observation.items() if key.endswith(".pos")}

    def send(self, goals: dict[str, float]) -> None:
        self._robot.send_action({f"{joint}.pos": value for joint, value in goals.items()})

    def release(self) -> None:
        self._robot.disconnect()


def discard_typed_ahead() -> None:
    """Throw away keys pressed while running, so only a fresh Enter counts."""
    try:
        if not sys.stdin.isatty():
            return
        if sys.platform == "win32":
            import msvcrt
            while msvcrt.kbhit():  # type: ignore[attr-defined]
                msvcrt.getwch()  # type: ignore[attr-defined]
        else:
            import termios
            termios.tcflush(sys.stdin.fileno(), termios.TCIFLUSH)
    except (OSError, ValueError):
        pass


def wait_for_release() -> None:
    """Keep the motors on until the person has hold of the arm."""
    print("\nThe motors are still ON. Support the arm, then press Enter to switch them off.", flush=True)
    discard_typed_ahead()
    while True:
        try:
            input()
            return
        except KeyboardInterrupt:
            print("\nStill holding. Support the arm, then press Enter.", flush=True)
            discard_typed_ahead()
        except EOFError:
            break
    print("No keyboard: switching the motors off in 5 seconds. Support the arm now.", flush=True)
    deadline = time.monotonic() + 5.0
    while (remaining := deadline - time.monotonic()) > 0:
        try:
            time.sleep(remaining)
        except KeyboardInterrupt:
            pass


# ==========================================================================
# 4. Turning controller readings into arm goals
# ==========================================================================


@dataclass
class Settings:
    roll_gain: float  # wrist_roll degrees per encoder degree
    twist_gain: float  # pan degrees per encoder degree
    rail_cm_per_deg: float  # height change per rail gear degree (0 = off)
    joy_speed: float  # cm/s at full joystick
    joy_deadzone: float
    invert_y: bool
    use_gripper: bool
    gripper_open: float  # 0..100
    gripper_closed: float  # 0..100
    tip_speed: float  # cm/s
    angle_speed: float  # deg/s for pan and roll
    joint_speed: float  # deg/s for each flex joint
    gripper_speed: float  # %/s
    max_joint_step: float  # deg: larger means an elbow flip: refused
    floor_m: float
    reach_limit_cm: float  # how far forward/back/up/down from the clutch pose
    clutch: bool  # False = always engaged (no button wired)


@dataclass
class Pose:
    pan: float
    roll: float
    tip: Vector  # metres (forward, up) from the shoulder axis
    gripper: float

    def copy(self) -> Pose:
        return Pose(self.pan, self.roll, self.tip.copy(), self.gripper)


@dataclass
class Channel:
    """Where an input was when it took over its motion."""

    anchor: Optional[float] = None  # input value at take-over
    base: float = 0.0  # arm value at take-over
    note: str = ""


@dataclass
class ArmController:
    settings: Settings
    limits: JointLimits
    engaged: bool = False
    joints: Vector = field(default_factory=lambda: np.zeros(3))  # flex, radians
    tilt: float = 0.0
    commanded: Optional[Pose] = None
    target: Optional[Pose] = None
    start_tip: Vector = field(default_factory=lambda: np.zeros(2))
    channels: dict[str, Channel] = field(default_factory=dict)
    joystick_armed: bool = False
    note: str = ""

    # ---- clutch ----------------------------------------------------------------

    def engage(self, observed: dict[str, float]) -> None:
        """Take the arm's measured pose as the start. Nothing jumps."""
        self.joints = np.radians([observed[j] for j in FLEX])
        tip, self.tilt = forward_kinematics(self.joints)
        self.start_tip = tip.copy()
        self.commanded = Pose(observed["shoulder_pan"], observed["wrist_roll"], tip, observed["gripper"])
        self.target = self.commanded.copy()
        self.channels = {name: Channel() for name in ("roll", "twist", "rail")}
        self.joystick_armed = False
        self.engaged = True

    def disengage(self) -> None:
        self.engaged = False

    # ---- one control tick ------------------------------------------------------

    def tick(self, s: Sample, fresh: bool, dt: float) -> None:
        assert self.commanded is not None and self.target is not None
        cfg, target, done = self.settings, self.target, self.commanded
        self.note = ""

        # Absolute motions: the encoder's change since take-over sets the arm's change.
        def follow(name: str, value: float, gain: float, current: float, valid: bool) -> Optional[float]:
            ch = self.channels[name]
            if not (fresh and valid and math.isfinite(value)):
                ch.anchor, ch.note = None, "held"
                return None
            if ch.anchor is None:  # take over from where the arm is now
                ch.anchor, ch.base, ch.note = value, current, ""
            return ch.base + gain * (value - ch.anchor)

        pan = follow("twist", s.arm_twist, cfg.twist_gain, done.pan, True)
        target.pan = done.pan if pan is None else clamp(pan, *self.limits.pan)

        roll = follow("roll", s.arm_roll, cfg.roll_gain, done.roll, True)
        target.roll = done.roll if roll is None else clamp(roll, *self.limits.roll)

        if cfg.rail_cm_per_deg:
            up = follow("rail", s.rail, cfg.rail_cm_per_deg / 100.0, float(done.tip[1]), s.rail_trusted)
            target.tip[1] = done.tip[1] if up is None else up
        else:
            target.tip[1] = done.tip[1]

        # Joystick: speed control, only once it has been seen centred.
        y = -s.joy_y if cfg.invert_y else s.joy_y
        if not (fresh and math.isfinite(y)):
            self.joystick_armed = False
        elif abs(y) <= cfg.joy_deadzone:
            self.joystick_armed = True
        speed = 0.0
        if self.joystick_armed and math.isfinite(y) and abs(y) > cfg.joy_deadzone:
            strength = (min(abs(y), 1.0) - cfg.joy_deadzone) / (1.0 - cfg.joy_deadzone)
            speed = math.copysign(cfg.joy_speed * strength ** 1.5, y) / 100.0
        target.tip[0] = (done.tip[0] if speed == 0.0 else target.tip[0]) + speed * dt

        # Keep the target within reach of the clutch pose (no wind-up).
        limit = cfg.reach_limit_cm / 100.0
        target.tip = np.clip(target.tip, self.start_tip - limit, self.start_tip + limit)

        if cfg.use_gripper and fresh and s.trigger in (0, 1):
            target.gripper = cfg.gripper_closed if s.trigger == 1 else cfg.gripper_open
        else:
            target.gripper = done.gripper

        self._step(dt)

    def _step(self, dt: float) -> None:
        assert self.commanded is not None and self.target is not None
        cfg, target, done = self.settings, self.target, self.commanded

        angle_step = cfg.angle_speed * dt
        done.pan += clamp(target.pan - done.pan, -angle_step, angle_step)
        done.roll += clamp(target.roll - done.roll, -angle_step, angle_step)
        grip_step = cfg.gripper_speed * dt
        done.gripper += clamp(target.gripper - done.gripper, -grip_step, grip_step)

        offset = target.tip - done.tip
        distance = float(np.hypot(*offset))
        tip_step = cfg.tip_speed / 100.0 * dt
        if distance > tip_step:
            offset = offset * (tip_step / distance)

        joint_step = min(cfg.max_joint_step, cfg.joint_speed * dt)
        fraction = 1.0

        for _ in range(8):  # shorten the step until every joint is within its speed
            candidate = done.tip + fraction * offset
            solution = self._solve(candidate, joint_step)
            if solution is not None:
                done.tip, self.joints = candidate, solution
                return
            fraction /= 2

        # Not safe or not reachable: stop the tool here, and stop asking for more.
        target.tip = done.tip.copy()

    def _solve(self, tip: Vector, max_step_deg: float) -> Optional[Vector]:
        # A joint already outside its margin (arm started there) may stay or
        # come back in, never go further out.
        low = np.minimum(self.limits.flex_low, self.joints)
        high = np.maximum(self.limits.flex_high, self.joints)
        candidates = inverse_kinematics(tip, self.tilt, low, high)
        if not candidates:
            self.note = "out of reach or at a joint limit"
            return None
        best = min(candidates, key=lambda q: float(np.max(np.abs(q - self.joints))))
        jump = math.degrees(float(np.max(np.abs(best - self.joints))))
        if jump > max_step_deg + 1e-9:
            self.note = f"refused: a joint would jump {jump:.0f} deg"
            return None
        height = lowest_point_height(best)
        if height < self.settings.floor_m and height < lowest_point_height(self.joints) - 1e-6:
            self.note = "refused: too close to the table"
            return None
        return best

    def goals(self) -> dict[str, float]:
        assert self.commanded is not None
        done = self.commanded
        flex = np.degrees(self.joints)
        goals = {"shoulder_pan": done.pan, "wrist_roll": done.roll,
                 **{joint: float(value) for joint, value in zip(FLEX, flex)}}
        if self.settings.use_gripper:
            goals["gripper"] = clamp(done.gripper, 0.0, 100.0)
        return goals


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


# ==========================================================================
# 5. Command line
# ==========================================================================


def positive(parser: argparse.ArgumentParser, name: str, value: float) -> float:
    if not (math.isfinite(value) and value > 0):
        parser.error(f"--{name} must be a number above zero")
    return value


def parse_args() -> tuple[argparse.Namespace, Settings]:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--live", action="store_true", help="move the real arm (default: dry run)")
    p.add_argument("--arm-port", help="the arm's serial port (needed with --live)")
    p.add_argument("--id", default="my_follower", help="robot id used at calibration")
    p.add_argument("--calibration-dir", type=Path, help="folder with <id>.json (default: lerobot's)")
    p.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION,
                   help="calibration file for the dry run (with --live, lerobot's own is used)")
    source = p.add_mutually_exclusive_group()
    source.add_argument("--encoder-port", help="the controller's USB serial port (found automatically)")
    source.add_argument("--stlink", action="store_true", help="read the controller through the ST-Link")
    source.add_argument("--fake", action="store_true", help="made-up input, no controller board")
    p.add_argument("--openocd", help="path of openocd (for --stlink)")

    p.add_argument("--no-clutch", action="store_true",
                   help="drive without holding the button (only if no button is wired)")
    p.add_argument("--roll-gain", type=float, default=0.36, help="wrist_roll deg per arm_roll deg (default 0.36)")
    p.add_argument("--twist-gain", type=float, default=0.2174, help="pan deg per arm_twist deg (default 0.2174)")
    p.add_argument("--rail-cm-per-turn", type=float, default=2.0,
                   help="height change in cm per turn of the rail gear (0 = rail off; default 2)")
    p.add_argument("--joy-speed", type=float, default=2.0, help="forward/back speed at full joystick, cm/s")
    p.add_argument("--joy-deadzone", type=float, default=0.12, help="joystick centre band (default 0.12)")
    p.add_argument("--invert-y", action="store_true", help="swap joystick forward and back")
    p.add_argument("--no-gripper", action="store_true", help="do not drive the gripper")
    p.add_argument("--gripper-open", type=float, default=0.0, help="gripper value for open, 0..100 (default 0)")
    p.add_argument("--gripper-closed", type=float, default=100.0, help="gripper value for closed, 0..100 (default 100)")

    p.add_argument("--rate", type=float, default=20.0, help="commands per second (10..50, default 20)")
    p.add_argument("--stale-after", type=float, default=0.5, help="hold if no new reading for this many s")
    p.add_argument("--tip-speed", type=float, default=3.0, help="tool speed limit, cm/s")
    p.add_argument("--angle-speed", type=float, default=30.0, help="pan / roll speed limit, deg/s")
    p.add_argument("--joint-speed", type=float, default=60.0, help="flex joint speed limit, deg/s")
    p.add_argument("--gripper-speed", type=float, default=100.0, help="gripper speed limit, percent/s")
    p.add_argument("--max-joint-step", type=float, default=6.0, help="refuse a step needing more (deg)")
    p.add_argument("--floor", type=float, default=3.0, help="keep the arm this many cm above the base plane")
    p.add_argument("--reach-limit", type=float, default=15.0,
                   help="max tool travel from the clutch pose, cm (default 15)")
    p.add_argument("--margin", type=float, default=2.0, help="degrees kept clear of each joint limit")
    p.add_argument("--seconds", type=float, help="stop after this long (default: until Ctrl-C)")
    args = p.parse_args()

    if args.live and not args.arm_port:
        p.error("--live needs --arm-port")
    rate = positive(p, "rate", args.rate)
    if not 10 <= rate <= 50:
        p.error("--rate must be 10..50 (slow the arm with the speed options instead)")
    for name in ("stale_after", "tip_speed", "angle_speed", "joint_speed", "gripper_speed",
                 "max_joint_step", "joy_speed", "reach_limit"):
        positive(p, name.replace("_", "-"), getattr(args, name))
    for name in ("roll_gain", "twist_gain", "rail_cm_per_turn", "floor", "margin"):
        if not math.isfinite(getattr(args, name)):
            p.error(f"--{name.replace('_', '-')} must be a number")
    if not 0 <= args.joy_deadzone < 0.9:
        p.error("--joy-deadzone must be 0..0.9")
    if args.margin < 0:
        p.error("--margin must be 0 or more")
    for name in ("gripper_open", "gripper_closed"):
        if not 0 <= getattr(args, name) <= 100:
            p.error(f"--{name.replace('_', '-')} must be 0..100")

    settings = Settings(
        roll_gain=args.roll_gain, twist_gain=args.twist_gain,
        rail_cm_per_deg=args.rail_cm_per_turn / 360.0,
        joy_speed=args.joy_speed, joy_deadzone=args.joy_deadzone, invert_y=args.invert_y,
        use_gripper=not args.no_gripper,
        gripper_open=args.gripper_open, gripper_closed=args.gripper_closed,
        tip_speed=args.tip_speed, angle_speed=args.angle_speed, joint_speed=args.joint_speed,
        gripper_speed=args.gripper_speed, max_joint_step=args.max_joint_step,
        floor_m=args.floor / 100.0, reach_limit_cm=args.reach_limit, clutch=not args.no_clutch,
    )
    return args, settings


def fmt(value: float, width: int = 7) -> str:
    return f"{'nan':>{width}}" if not math.isfinite(value) else f"{value:+{width}.1f}"


def install_stop_signals() -> None:
    """Ctrl-C, a kill or a closed terminal all end in the clean-up below."""
    def stop(signum: int, frame: object) -> None:
        raise KeyboardInterrupt
    for name in ("SIGINT", "SIGTERM", "SIGHUP"):
        number = getattr(signal, name, None)
        if number is not None:
            signal.signal(number, stop)


def run(args: argparse.Namespace, cfg: Settings) -> None:
    period = 1.0 / args.rate

    controller: Controller
    if args.fake:
        controller = FakeController()
    elif args.stlink:
        controller = StlinkController(args.openocd)
    else:
        controller = UsbController(args.encoder_port)

    arm: Optional[Arm] = None
    live = False

    try:
        if args.live:
            real = RealArm(args.arm_port, args.id, args.calibration_dir)
            limits = load_limits(real.calibration, args.margin)  # check before energising
            real.connect()
            arm, live = real, True
        else:
            arm = DryRunArm(args.calibration)
            limits = load_limits(args.calibration, args.margin)

        ctl = ArmController(cfg, limits)
        print(f"Controller: {getattr(controller, 'where', '?')}.  Calibration: {arm.calibration}")
        print("MOVING THE REAL ARM. Ctrl-C to stop." if live else
              "DRY RUN: nothing is sent to any arm. Add --live --arm-port PORT to move it.")
        print("Hold the button (clutch) to drive." if cfg.clutch else
              "No clutch: the arm follows the controller as soon as it reads it.")

        started = next_tick = last_status = time.monotonic()
        while True:
            now = time.monotonic()
            if args.seconds is not None and now - started >= args.seconds:
                break

            s = controller.latest()
            fresh = s is not None and now - s.received <= args.stale_after
            wants = s is not None and fresh and (not cfg.clutch or s.button == 1)

            if wants and not ctl.engaged:
                ctl.engage(arm.read())
                print("CLUTCH ON: driving from the current pose.", flush=True)
            elif not wants and ctl.engaged:
                ctl.disengage()
                print("CLUTCH OFF: frozen where it is." if fresh else
                      "HOLD: controller stopped answering; frozen where it is.", flush=True)

            if ctl.engaged and s is not None:
                ctl.tick(s, fresh, period)
                arm.send(ctl.goals())

            if now - last_status >= 0.5:
                last_status = now
                print(status_line(s, fresh, ctl), flush=True)

            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()
    finally:
        try:
            if live and arm is not None:
                wait_for_release()
        finally:
            if arm is not None:
                arm.release()
            controller.close()


def status_line(s: Optional[Sample], fresh: bool, ctl: ArmController) -> str:
    if s is None:
        return "waiting for the controller..."
    inputs = (f"in: roll{fmt(s.arm_roll)} twist{fmt(s.arm_twist)} rail{fmt(s.rail)} "
              f"joy_y{s.joy_y:+5.2f} btn {s.button} trig {s.trigger}"
              + ("" if fresh else " STALE") + ("" if s.rail_trusted else " RAIL-COUNT-LOST"))
    if not ctl.engaged or ctl.commanded is None:
        return inputs + " | clutch off"
    c = ctl.commanded
    move = (c.tip - ctl.start_tip) * 100
    flex = np.degrees(ctl.joints)
    held = [name for name, ch in ctl.channels.items() if ch.note]
    notes = ([f"{', '.join(held)} held"] if held else []) + ([ctl.note] if ctl.note else []) \
        + ([] if ctl.joystick_armed else ["joystick not centred yet"])
    return (inputs + f" | pan{fmt(c.pan)} roll{fmt(c.roll)} fwd{move[0]:+5.1f} up{move[1]:+5.1f} cm "
            f"grip{c.gripper:5.0f} | lift{flex[0]:+6.1f} elbow{flex[1]:+6.1f} wrist{flex[2]:+6.1f}"
            + (" | " + "; ".join(notes) if notes else ""))


def main() -> int:
    args, settings = parse_args()
    install_stop_signals()
    try:
        run(args, settings)
    except KeyboardInterrupt:
        print()
    except ControlError as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
