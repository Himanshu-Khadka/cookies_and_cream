#!/usr/bin/env python3
"""Drive the SO-101 arm from the encoder stream (arm_roll, arm_twist, arm_v).

This is the bridge between lib/read_encoders.py (the input device) and
arm_ik.py (the inverse kinematics). It is DRY RUN by default: it reads the
encoders and prints what it would send, and does not touch the arm. Add
--execute to move the real arm.

    python arm_encoder_control.py                    dry run with the real encoders
    python arm_encoder_control.py --fake-encoders    dry run with made-up input (no board)
    python arm_encoder_control.py --execute --port /dev/ttyACM0     move the arm

HOW IT MAPS (defaults; change with the --*-drives and --*-gain options)

    arm_roll   -> wrist_roll      1 degree of the arm per degree of the encoder
    arm_twist  -> shoulder_pan    1 degree per degree
    arm_v      -> height (z)      0.1 cm per mm of rail travel

Reach (y) and gripper tilt (pitch) stay where the arm was when the program
started. Nothing here drives the gripper. Which encoder should drive which
motion has not been agreed by the team: these defaults are a starting point.

Everything is RELATIVE to the moment the program starts: the arm's pose then,
and the encoders' values then, are the two zeros. So no home file and no
saved encoder center are needed, and starting it never moves the arm.

SAFETY (each of these is deliberate; read before changing)

  - Dry run unless --execute is given.
  - The target moves at a limited speed (--tip-speed cm/s, --angle-speed
    deg/s); the arm follows it at about --rate commands a second.
  - A step that would need a joint to jump (elbow flipping over) is refused.
  - A step that would put the elbow, wrist or tool below --floor is refused.
  - A missing reading (nan), a stale stream, or a rail whose turn count can
    no longer be trusted, holds that motion where it is.
  - On exit the motors stay on until you press Enter, so the arm does not
    drop. Support it first.

NOT TESTED ON THE REAL ARM. It has only been run in dry-run mode. For the
first --execute run keep a hand on the power and use low speeds.

Needs numpy and arm_ik.py (same folder) and ../lib/read_encoders.py. Only
--execute needs lerobot.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Protocol

import numpy as np
from numpy.typing import NDArray

import arm_ik

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "lib"))

import read_encoders  # noqa: E402  (needs the path line above)

if TYPE_CHECKING:
    from read_encoders import Angles

Vector = NDArray[np.float64]

# The motions an encoder can drive, and the unit of each.
AXES = ("pan", "roll", "y", "z", "pitch")
ANGLE_AXES = ("pan", "roll", "pitch")
ROTATION_JOINTS = {"pan": "shoulder_pan", "roll": "wrist_roll"}
ALL_JOINTS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)
RESOLUTION = 4096  # STS3215 counts per turn

DEFAULT_CALIBRATION = HERE.parent / "calibration_file" / "my_follower.json"

# A reading older than this is not acted on.
STALE_AFTER_S = 0.5

# How long to wait for the first encoder reading at start-up.
FIRST_READING_TIMEOUT_S = 5.0

STATUS_PERIOD_S = 0.5


class ControlError(Exception):
    """The bridge cannot start or continue."""


# --------------------------------------------------------------------------
# Typed wrappers around arm_ik (which has no type hints)
# --------------------------------------------------------------------------


def to_model(q_deg: Vector) -> Vector:
    """LeRobot degrees (3,) -> the solver's radians."""
    result: Vector = arm_ik.to_model(q_deg)  # type: ignore[no-untyped-call]
    return result


def from_model(q_rad: Vector) -> Vector:
    """The solver's radians (3,) -> LeRobot degrees."""
    result: Vector = arm_ik.from_model(q_rad)  # type: ignore[no-untyped-call]
    return result


def forward(q_rad: Vector) -> tuple[Vector, float]:
    """Tool point (forward, up) in metres from the shoulder axis, and pitch in radians."""
    tip, pitch = arm_ik.fk(q_rad)  # type: ignore[no-untyped-call]
    return np.asarray(tip, dtype=np.float64), float(pitch)


def flex_limits(calibration: Path, margin_deg: float) -> tuple[Vector, Vector]:
    """Lower and upper limits of the three flex joints, in the solver's radians."""
    limits_deg = arm_ik.load_limits(calibration, margin_deg)  # type: ignore[no-untyped-call]
    low, high = arm_ik.model_limits(limits_deg)  # type: ignore[no-untyped-call]
    return np.asarray(low, dtype=np.float64), np.asarray(high, dtype=np.float64)


def branch_solutions(
    target: Vector, pitch_rad: float, low: Vector, high: Vector
) -> list[Vector]:
    """Every joint solution for this tool point and pitch that is inside the limits."""
    solutions, valid = arm_ik.ik(  # type: ignore[no-untyped-call]
        target, np.array([pitch_rad]), low, high
    )
    return [
        np.asarray(solutions[0, branch], dtype=np.float64)
        for branch in range(2)
        if bool(valid[0, branch])
    ]


def lowest_point_height(q_rad: Vector) -> float:
    """Height in metres above the base plane of the lowest of elbow, wrist and tool."""
    q1, q2, q3 = (float(value) for value in q_rad)
    elbow = arm_ik.L1 * math.sin(arm_ik.A1 - q1)
    wrist = elbow + arm_ik.L2 * math.sin(arm_ik.A2 - q1 - q2)
    tool = wrist + arm_ik.L3 * math.sin(arm_ik.A3 - q1 - q2 - q3)
    return float(arm_ik.SHOULDER_HEIGHT) + min(elbow, wrist, tool)


def rotation_limits(calibration: Path, margin_deg: float) -> dict[str, tuple[float, float]]:
    """Degree limits of shoulder_pan and wrist_roll (0 = middle of the calibrated range)."""
    try:
        data: object = json.loads(calibration.read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        raise ControlError(f"Cannot read the calibration file {calibration}: {error}") from error

    if not isinstance(data, dict):
        raise ControlError(f"{calibration} is not a calibration file")

    limits: dict[str, tuple[float, float]] = {}

    for axis, joint in ROTATION_JOINTS.items():
        entry = data.get(joint)
        if not isinstance(entry, dict):
            raise ControlError(f"{calibration} has no entry for {joint}")

        low_raw, high_raw = entry.get("range_min"), entry.get("range_max")
        if not isinstance(low_raw, (int, float)) or not isinstance(high_raw, (int, float)):
            raise ControlError(f"{calibration}: {joint} has no range_min / range_max")

        half_span = (high_raw - low_raw) / 2 * 360.0 / (RESOLUTION - 1)
        limits[axis] = (-half_span + margin_deg, half_span - margin_deg)

    return limits


# --------------------------------------------------------------------------
# Where the encoder readings come from
# --------------------------------------------------------------------------


class EncoderSource(Protocol):
    """What the control loop needs from the encoders."""

    def latest(self) -> Angles | None:
        """The newest reading, or None before the first one."""
        ...

    @property
    def rail_uncertain(self) -> bool:
        """True once arm_v may be off by whole gear turns."""
        ...

    @property
    def arm_v_unit(self) -> str:
        """'mm' or 'deg'."""
        ...

    def close(self) -> None:
        """Stop reading."""
        ...


class FakeEncoders:
    """Smooth made-up input, for trying the bridge with no board attached."""

    def __init__(self) -> None:
        self._start = time.monotonic()

    def latest(self) -> Angles:
        elapsed = time.monotonic() - self._start
        return read_encoders.Angles(
            elapsed,
            30.0 * math.sin(elapsed * 0.7),
            25.0 * math.sin(elapsed * 0.5),
            30.0 * math.sin(elapsed * 0.4),
        )

    @property
    def rail_uncertain(self) -> bool:
        return False

    @property
    def arm_v_unit(self) -> str:
        return "mm"

    def close(self) -> None:
        return None


# --------------------------------------------------------------------------
# The arm: real, or simulated for a dry run
# --------------------------------------------------------------------------


class Arm(Protocol):
    def read(self) -> dict[str, float]:
        """Present position of every joint, keyed '<joint>.pos'."""
        ...

    def send(self, goals: dict[str, float]) -> None:
        """Command every joint."""
        ...

    def release(self) -> None:
        """Let go of the arm (motors off) and close the port."""
        ...


class DryRunArm:
    """Pretends to be an arm that reaches every goal at once. Sends nothing."""

    def __init__(self) -> None:
        self._joints = {f"{joint}.pos": 0.0 for joint in ALL_JOINTS}

    def read(self) -> dict[str, float]:
        return dict(self._joints)

    def send(self, goals: dict[str, float]) -> None:
        self._joints.update(goals)

    def release(self) -> None:
        return None


class RealArm:
    """The SO-101 follower through lerobot."""

    def __init__(self, port: str, robot_id: str, calibration_dir: Path | None) -> None:
        settings = argparse.Namespace(
            port=port,
            id=robot_id,
            calibration_dir=calibration_dir,
            max_relative_target=None,
        )
        self._robot = arm_ik.make_robot(settings)  # type: ignore[no-untyped-call]
        self.calibration = Path(self._robot.calibration_fpath)

    def connect(self) -> None:
        # calibrate=False: never rewrite the servos' stored calibration from here.
        self._robot.connect(calibrate=False)

        if not self._robot.is_calibrated:
            self._robot.disconnect()
            raise ControlError(
                "The servos' stored calibration does not match the calibration "
                "file. Sort that out with lerobot-calibrate first; nothing was "
                "changed."
            )

    def read(self) -> dict[str, float]:
        observation = self._robot.get_observation()
        return {
            key: float(value)
            for key, value in observation.items()
            if key.endswith(".pos")
        }

    def send(self, goals: dict[str, float]) -> None:
        self._robot.send_action(dict(goals))

    def release(self) -> None:
        self._robot.disconnect()


# --------------------------------------------------------------------------
# Mapping encoders to motions
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class Channel:
    """One encoder value driving one motion of the arm."""

    encoder: str  # attribute of Angles: arm_roll, arm_twist or arm_v
    axis: str  # one of AXES
    gain: float  # axis units per encoder unit


@dataclass
class ChannelState:
    """Where a channel's zero is, once its encoder has given a reading."""

    reference: float | None = None
    held_reason: str | None = None


@dataclass
class Target:
    """A pose in the units people type: degrees and centimetres."""

    pan: float
    roll: float
    y: float  # cm forward of where the tool was at start
    z: float  # cm above where the tool was at start
    pitch: float  # degrees above horizontal

    def copy(self) -> Target:
        return Target(self.pan, self.roll, self.y, self.z, self.pitch)


@dataclass
class Limits:
    tip_speed_cm_s: float
    angle_speed_deg_s: float
    max_joint_step_deg: float
    floor_m: float
    rotation: dict[str, tuple[float, float]]
    flex_low: Vector
    flex_high: Vector


@dataclass
class Controller:
    """Turns encoder readings into joint goals, one small safe step at a time."""

    channels: list[Channel]
    limits: Limits
    start_tip: Vector  # metres, from the shoulder axis
    goal: Target  # where the encoders say to be
    commanded: Target  # where the arm has been told to be (moves toward goal)
    joints: Vector  # the three flex joints last commanded, solver radians
    states: dict[str, ChannelState] = field(default_factory=dict)
    note: str = ""
    _start: Target = field(init=False)  # the pose at start-up: the zero of every motion

    def __post_init__(self) -> None:
        self._start = self.commanded.copy()

    def update_goal(self, angles: Angles | None, fresh: bool, rail_uncertain: bool) -> None:
        """Move the goal to where the encoders point. A bad reading holds it."""
        for channel in self.channels:
            state = self.states.setdefault(channel.encoder, ChannelState())

            if angles is None or not fresh:
                state.held_reason = "no fresh reading"
                continue

            if channel.encoder == "arm_v" and rail_uncertain:
                state.held_reason = "rail turn count not trusted"
                continue

            value = float(getattr(angles, channel.encoder))

            if not math.isfinite(value):
                state.held_reason = "no reading"
                continue

            if state.reference is None or state.held_reason is not None:
                # First reading, or first one after a gap: take this as the
                # zero, shifted so the motion carries on from where it is now
                # and does not jump.
                state.reference = value - (
                    getattr(self.goal, channel.axis) - self._start_value(channel.axis)
                ) / channel.gain
                state.held_reason = None

            wanted = self._start_value(channel.axis) + channel.gain * (
                value - state.reference
            )

            if channel.axis in self.limits.rotation:
                low, high = self.limits.rotation[channel.axis]
                wanted = max(low, min(high, wanted))

            setattr(self.goal, channel.axis, wanted)

    def _start_value(self, axis: str) -> float:
        return float(getattr(self._start, axis))

    def step(self, dt: float) -> None:
        """Move the commanded pose one time step toward the goal, if that is safe."""
        self.note = ""
        previous = self.commanded.copy()
        angle_step = self.limits.angle_speed_deg_s * dt

        # Pan and roll are plain joint angles: limit their speed and range.
        for axis in ("pan", "roll"):
            current = getattr(self.commanded, axis)
            change = getattr(self.goal, axis) - current
            moved = current + max(-angle_step, min(angle_step, change))
            low, high = self.limits.rotation[axis]
            setattr(self.commanded, axis, max(low, min(high, moved)))

        # The tool point moves in a straight line at a limited speed.
        offset = np.array([self.goal.y - previous.y, self.goal.z - previous.z])
        distance = float(np.hypot(offset[0], offset[1]))
        tip_step = self.limits.tip_speed_cm_s * dt

        if distance > tip_step:
            offset = offset * (tip_step / distance)

        self.commanded.y = previous.y + float(offset[0])
        self.commanded.z = previous.z + float(offset[1])

        pitch_change = self.goal.pitch - previous.pitch
        self.commanded.pitch = previous.pitch + max(
            -angle_step, min(angle_step, pitch_change)
        )

        solution = self._solve(self.commanded)

        if solution is None:
            # Not safe: leave the tool point and tilt where they were.
            self.commanded.y, self.commanded.z = previous.y, previous.z
            self.commanded.pitch = previous.pitch
        else:
            self.joints = solution

    def _solve(self, pose: Target) -> Vector | None:
        """Joint angles for this pose, or None (with self.note set) if it is refused."""
        target = self.start_tip + np.array([pose.y, pose.z]) / 100.0
        candidates = branch_solutions(
            target, math.radians(pose.pitch), self.limits.flex_low, self.limits.flex_high
        )

        if not candidates:
            self.note = "out of reach or at a joint limit"
            return None

        # Stay on the elbow configuration the arm is already in.
        best = min(candidates, key=lambda q: float(np.max(np.abs(q - self.joints))))
        largest_step = math.degrees(float(np.max(np.abs(best - self.joints))))

        if largest_step > self.limits.max_joint_step_deg:
            self.note = f"refused: a joint would jump {largest_step:.0f} deg"
            return None

        # Refuse going below the floor margin. An arm that starts below it
        # (resting on the table) may still move, as long as it does not go lower.
        height = lowest_point_height(best)

        if height < self.limits.floor_m and height < lowest_point_height(self.joints) - 1e-6:
            self.note = "refused: too close to the table"
            return None

        return best

    def joint_goals(self, base: dict[str, float]) -> dict[str, float]:
        """The full command: every joint, with only the driven ones changed."""
        goals = dict(base)
        goals["shoulder_pan.pos"] = self.commanded.pan
        goals["wrist_roll.pos"] = self.commanded.roll

        for joint, value in zip(arm_ik.ARM, from_model(self.joints)):
            goals[f"{joint}.pos"] = float(value)

        return goals

    def held(self) -> list[str]:
        """Channels that are being held, with the reason."""
        return [
            f"{name} held ({state.held_reason})"
            for name, state in self.states.items()
            if state.held_reason is not None
        ]


# --------------------------------------------------------------------------
# Command line
# --------------------------------------------------------------------------


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--execute", action="store_true", help="move the real arm (default: dry run)"
    )
    parser.add_argument("--port", help="the arm's serial port (needed with --execute)")
    parser.add_argument("--id", default="my_follower", help="robot id used at calibration")
    parser.add_argument("--calibration-dir", type=Path, default=None)
    parser.add_argument(
        "--calibration",
        type=Path,
        default=DEFAULT_CALIBRATION,
        help="calibration file for the dry run (with --execute, lerobot's own is used)",
    )
    parser.add_argument("--encoder-port", help="the encoder board's serial port (found automatically)")
    parser.add_argument(
        "--fake-encoders", action="store_true", help="made-up input, no encoder board"
    )

    choices = AXES + ("none",)
    parser.add_argument("--roll-drives", choices=choices, default="roll", help="what arm_roll moves")
    parser.add_argument("--twist-drives", choices=choices, default="pan", help="what arm_twist moves")
    parser.add_argument("--v-drives", choices=choices, default="z", help="what arm_v moves")
    parser.add_argument("--roll-gain", type=float, default=1.0, help="arm units per degree of arm_roll")
    parser.add_argument("--twist-gain", type=float, default=1.0, help="arm units per degree of arm_twist")
    parser.add_argument(
        "--v-gain",
        type=float,
        default=None,
        help="arm units per unit of arm_v (default: 0.1 cm per mm; required "
        "when arm_v is in degrees)",
    )

    parser.add_argument("--rate", type=float, default=20.0, help="commands per second")
    parser.add_argument("--tip-speed", type=float, default=5.0, help="tool speed limit, cm/s")
    parser.add_argument("--angle-speed", type=float, default=30.0, help="pan / roll / tilt speed limit, deg/s")
    parser.add_argument(
        "--max-joint-step",
        type=float,
        default=6.0,
        help="refuse a step needing more than this many degrees from any flex joint",
    )
    parser.add_argument("--floor", type=float, default=3.0, help="keep the arm this many cm above the base plane")
    parser.add_argument("--margin", type=float, default=2.0, help="degrees kept clear of each joint limit")
    parser.add_argument("--seconds", type=float, default=None, help="stop after this long (default: until Ctrl-C)")
    return parser.parse_args()


def positive(name: str, value: float) -> float:
    """A command-line number that must be a real number above zero."""
    if not (math.isfinite(value) and value > 0):
        raise ControlError(f"--{name} must be a number above zero")
    return value


def build_channels(args: argparse.Namespace, v_unit: str) -> list[Channel]:
    """The encoder-to-motion table from the command line."""
    requested = [
        ("arm_roll", str(args.roll_drives), float(args.roll_gain)),
        ("arm_twist", str(args.twist_drives), float(args.twist_gain)),
    ]

    v_axis = str(args.v_drives)
    v_gain: float | None = args.v_gain

    if v_axis != "none":
        if v_gain is None and v_unit == "mm" and v_axis in ("y", "z"):
            v_gain = 0.1  # centimetres per millimetre
        if v_gain is None:
            print(
                "arm_v is not used: it is in "
                + ("degrees of the rail's gear" if v_unit == "deg" else "millimetres")
                + ", so give --v-gain (arm units per unit of arm_v), or save the "
                "rail's travel per gear turn with read_encoders.py --rail-mm-per-rev.",
                file=sys.stderr,
            )
        else:
            requested.append(("arm_v", v_axis, v_gain))

    channels: list[Channel] = []
    used: dict[str, str] = {}

    for encoder, axis, gain in requested:
        if axis == "none":
            continue
        if not math.isfinite(gain) or gain == 0:
            raise ControlError(f"The gain for {encoder} must be a non-zero number")
        if axis in used:
            raise ControlError(f"{encoder} and {used[axis]} both drive {axis}")
        used[axis] = encoder
        channels.append(Channel(encoder, axis, gain))

    return channels


def describe(channels: list[Channel]) -> None:
    units = {"pan": "deg", "roll": "deg", "pitch": "deg", "y": "cm", "z": "cm"}
    names = {
        "pan": "shoulder_pan",
        "roll": "wrist_roll",
        "pitch": "gripper tilt",
        "y": "reach (forward)",
        "z": "height",
    }

    for channel in channels:
        print(
            f"  {channel.encoder:10s} -> {names[channel.axis]:16s} "
            f"{channel.gain:+g} {units[channel.axis]} per unit"
        )

    if not channels:
        print("  (no encoder drives anything)")


def wait_for_release() -> None:
    """Keep the motors on until the person has hold of the arm."""
    print("\nThe motors are still ON. Support the arm, then press Enter to switch them off.")

    try:
        input()
    except (EOFError, KeyboardInterrupt):
        print("No keyboard: switching the motors off in 5 seconds.")
        time.sleep(5.0)


def run(args: argparse.Namespace) -> None:
    rate = positive("rate", float(args.rate))
    period = 1.0 / rate

    if args.execute and not args.port:
        raise ControlError("--execute needs --port (the arm's serial port)")

    encoders: EncoderSource
    if args.fake_encoders:
        encoders = FakeEncoders()
    else:
        try:
            stream = read_encoders.EncoderStream(args.encoder_port)
        except (read_encoders.PortError, read_encoders.CenterError) as error:
            raise ControlError(f"Encoders: {error}") from error
        stream.start()
        encoders = stream

    arm: Arm | None = None
    executing = False

    try:
        channels = build_channels(args, encoders.arm_v_unit)

        if args.execute:
            real = RealArm(str(args.port), str(args.id), args.calibration_dir)
            calibration = real.calibration
            real.connect()
            arm = real
            executing = True
        else:
            calibration = Path(args.calibration)
            arm = DryRunArm()

        margin = float(args.margin)
        flex_low, flex_high = flex_limits(calibration, margin)
        limits = Limits(
            tip_speed_cm_s=positive("tip-speed", float(args.tip_speed)),
            angle_speed_deg_s=positive("angle-speed", float(args.angle_speed)),
            max_joint_step_deg=positive("max-joint-step", float(args.max_joint_step)),
            floor_m=float(args.floor) / 100.0,
            rotation=rotation_limits(calibration, margin),
            flex_low=flex_low,
            flex_high=flex_high,
        )

        base = arm.read()
        missing = [joint for joint in ALL_JOINTS if f"{joint}.pos" not in base]
        if missing:
            raise ControlError(f"The arm did not report: {', '.join(missing)}")

        joints = to_model(np.array([base[f"{joint}.pos"] for joint in arm_ik.ARM]))
        start_tip, start_pitch = forward(joints)
        start = Target(
            pan=base["shoulder_pan.pos"],
            roll=base["wrist_roll.pos"],
            y=0.0,
            z=0.0,
            pitch=math.degrees(start_pitch),
        )

        controller = Controller(
            channels=channels,
            limits=limits,
            start_tip=start_tip,
            goal=start.copy(),
            commanded=start.copy(),
            joints=joints,
        )

        print(f"Calibration: {calibration}")
        print("Encoders drive:")
        describe(channels)
        print(
            f"Limits: tool {limits.tip_speed_cm_s:g} cm/s, angles "
            f"{limits.angle_speed_deg_s:g} deg/s, floor {args.floor:g} cm, "
            f"{rate:g} commands/s."
        )
        print(
            "MOVING THE REAL ARM. Ctrl-C to stop."
            if executing
            else "DRY RUN: nothing is sent to the arm. Ctrl-C to stop."
        )

        # Hold the measured pose first, so starting never moves the arm.
        arm.send(controller.joint_goals(base))

        last_seen: float | None = None
        last_seen_at = time.monotonic()
        waited_since = time.monotonic()
        last_status = 0.0
        started = time.monotonic()
        next_tick = started

        while True:
            now = time.monotonic()

            if args.seconds is not None and now - started >= float(args.seconds):
                break

            try:
                angles = encoders.latest()
            except read_encoders.PortError as error:
                raise ControlError(f"Encoders: {error}") from error

            if angles is not None and angles.time_s != last_seen:
                last_seen = angles.time_s
                last_seen_at = now

            fresh = angles is not None and now - last_seen_at <= STALE_AFTER_S

            if angles is None and now - waited_since > FIRST_READING_TIMEOUT_S:
                raise ControlError("No reading from the encoders.")

            controller.update_goal(angles, fresh, encoders.rail_uncertain)
            controller.step(period)
            arm.send(controller.joint_goals(base))

            if now - last_status >= STATUS_PERIOD_S:
                last_status = now
                pose = controller.commanded
                flex = from_model(controller.joints)
                notes = controller.held() + ([controller.note] if controller.note else [])
                print(
                    f"pan {pose.pan:+7.1f} roll {pose.roll:+7.1f} | "
                    f"y {pose.y:+6.1f} z {pose.z:+6.1f} cm tilt {pose.pitch:+6.1f} | "
                    f"lift {flex[0]:+6.1f} elbow {flex[1]:+6.1f} wrist {flex[2]:+6.1f}"
                    + (" | " + "; ".join(notes) if notes else ""),
                    flush=True,
                )

            next_tick += period
            delay = next_tick - time.monotonic()

            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()
    finally:
        try:
            if arm is not None and executing:
                wait_for_release()
        finally:
            if arm is not None:
                arm.release()
            encoders.close()


def main() -> int:
    args = parse_args()

    try:
        run(args)
    except KeyboardInterrupt:
        print()
    except ControlError as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
