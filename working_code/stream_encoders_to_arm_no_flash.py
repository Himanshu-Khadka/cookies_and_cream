#!/usr/bin/env python3
"""Stream one Black Pill controller into an SO-101 arm (arm_all.py semantics).

Project layout:
    project/lib/read_encoders.py      # UPDATED combined controller reader
    project/lib/read_controller.py    # UPDATED, for optional --stlink
    project/lib/read_trigger.py       # required only for --stlink
    project/working_code/arm_ik.py
    project/working_code/run_rotations.py
    project/working_code/arm_all.py   # existing; not run simultaneously
    project/working_code/stream_encoders_to_arm.py

Controls:
    arm_roll   -> wrist_roll (original -500..500 -> -180..180 mapping)
    arm_twist  -> shoulder_pan (original -500..500 -> -107.7..109.7)
    joy_y      -> Y (forward/back cm), velocity proportional to deflection^1.5
    arm_v      -> Z (up/down cm), velocity proportional to rail offset^1.5
    trigger (0/1) -> gripper fully open/closed, with existing firmware
    trigger_percent -> proportional gripper only when supported by firmware
    button     -> clutch (1 run, 0 freeze/hold)

    --no-rail     Z (height) is not driven; the rail (AS5600) is not needed
    --no-gripper  the gripper is not driven; the trigger is not needed
    So joystick forward/back + roll + twist only:
        python stream_encoders_to_arm_no_flash.py --no-rail --no-gripper

All motion stops receiving new targets when button=0, input is invalid or stale.
The program sends a one-time "hold current observed pose" command when an
active clutch disengages, but THIS IS NOT AN EMERGENCY STOP: the servos can
still move to their last command. Keep a hardware power disconnect accessible.

NO FLASH NEEDED in default --trigger-mode digital: uses existing CTL1
firmware and the 0/1 trigger on PB10. Proportional gripper movement is NOT
possible from a 1-bit signal. Optional --trigger-mode analog requires
firmware already supplying trigger_percent; never assumes a value.

Without --live this script is a dry run: it opens no SO-101 connection.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from collections import deque
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
LIB = HERE.parent / "lib"
if str(LIB) not in sys.path:
    sys.path.insert(0, str(LIB))


def clip(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize(value: float, in_min: float, in_max: float,
              out_min: float, out_max: float) -> float:
    """Linear mapping with hard clipping for approximate encoder end stops."""
    if not all(map(math.isfinite, (value, in_min, in_max, out_min, out_max))):
        raise ValueError("non-finite encoder value or mapping limit")
    if in_min >= in_max or out_min >= out_max:
        raise ValueError("mapping minimum must be below maximum")
    fraction = (clip(value, in_min, in_max) - in_min) / (in_max - in_min)
    return out_min + fraction * (out_max - out_min)


def shaped_velocity(value: float, deadzone: float, max_speed: float,
                    exponent: float = 1.5) -> float:
    """Normalized [-1, 1] axis -> cm/s, with quiet center and gentle ramp."""
    if not math.isfinite(value):
        raise ValueError("invalid velocity input")
    v = clip(value, -1.0, 1.0)
    if abs(v) <= deadzone:
        return 0.0
    strength = (abs(v) - deadzone) / (1.0 - deadzone)
    return math.copysign(max_speed * strength ** exponent, v)


def speed_limit(previous: float, target: float, max_change: float) -> float:
    return previous + clip(target - previous, -max_change, max_change)


def bounded_offset(current: float, movement: float, limit: float) -> float:
    """Limit position without snapping when the arm initially lies outside it."""
    proposed = current + movement
    if current > limit:
        return min(current, proposed)  # only permit movement back toward range
    if current < -limit:
        return max(current, proposed)
    return clip(proposed, -limit, limit)


def valid_input(sample: Any, *, rail_uncertain: bool = False,
                need_rail: bool = True, need_trigger: bool = True) -> str | None:
    """Conservative fail-closed checks; return why the arm must hold."""
    if sample.button not in (0, 1):
        return "clutch button missing (controller firmware not sending it)"
    if sample.button == 0:
        return None  # disabled: sensor validity is irrelevant while parked
    if need_rail and rail_uncertain:
        return "AS5600 rail has an uncertain turn count; stop and re-center it"
    names = ["arm_roll", "arm_twist", "joy_y"]
    if need_rail:
        names.append("arm_v")
    if need_trigger:
        names.append("trigger_percent")
    for name in names:
        value = getattr(sample, name, math.nan)
        if not isinstance(value, (float, int)) or not math.isfinite(value):
            if name == "trigger_percent":
                return "trigger percentage unavailable (use --trigger-mode digital for existing firmware)"
            return f"{name} missing/invalid (no commands sent)"
    if need_trigger and not 0 <= sample.trigger_percent <= 100:
        return "trigger_percent outside 0..100"
    if not -1.05 <= sample.joy_y <= 1.05:
        return "joystick outside expected -1..1 range"
    return None


class LowPass:
    """First-order exponential filter (one update per controller cycle)."""

    def __init__(self, alpha: float, initial: float | None = None) -> None:
        self.alpha = alpha
        self.value = initial

    def update(self, incoming: float) -> float:
        if self.value is None:
            self.value = incoming
        else:
            self.value += self.alpha * (incoming - self.value)
        return self.value


class TargetState:
    """Continuous target state, re-anchored whenever the clutch is engaged."""

    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.engaged = False
        self.pan_anchor = 0.0
        self.roll_anchor = 0.0
        self.pan_start = 0.0
        self.roll_start = 0.0
        self.y_cm = 0.0
        self.z_cm = 0.0
        self.pitch_deg = 0.0
        self.filters: dict[str, LowPass] = {}
        self.rail_history: deque[float] = deque(maxlen=5)
        self.pan_target = 0.0
        self.roll_target = 0.0
        self.trigger_pct = 0.0
        self.y_speed = 0.0
        self.z_speed = 0.0

    def mapped(self, sample: Any) -> tuple[float, float]:
        a = self.args
        roll = normalize(sample.arm_roll, a.roll_in_min, a.roll_in_max,
                         a.roll_out_min, a.roll_out_max)
        pan = normalize(sample.arm_twist, a.twist_in_min, a.twist_in_max,
                        a.pan_out_min, a.pan_out_max)
        return pan, roll

    def engage(self, sample: Any, *, pan: float, roll: float,
               y_cm: float, z_cm: float, pitch_deg: float) -> None:
        self.pan_anchor, self.roll_anchor = self.mapped(sample)
        self.pan_start, self.roll_start = pan, roll
        self.pan_target, self.roll_target = pan, roll
        self.y_cm, self.z_cm, self.pitch_deg = y_cm, z_cm, pitch_deg
        self.y_speed = self.z_speed = 0.0
        self.rail_history.clear()
        self.filters = {
            "pan": LowPass(self.args.angle_smoothing, pan),
            "roll": LowPass(self.args.angle_smoothing, roll),
            "y": LowPass(self.args.axis_smoothing, 0.0),
            "z": LowPass(self.args.z_smoothing, 0.0),
            "trigger": LowPass(self.args.trigger_smoothing,
                               0.0 if self.args.no_gripper else sample.trigger_percent),
        }
        self.trigger_pct = 0.0 if self.args.no_gripper else sample.trigger_percent
        self.engaged = True

    def pause(self) -> None:
        self.engaged = False
        self.y_speed = 0.0
        self.z_speed = 0.0

    def update(self, sample: Any, dt: float, rail_units: str) -> tuple[float, float]:
        """Compute desired pan/roll/gripper and integrate Y/Z in cm."""
        if not self.engaged:
            raise RuntimeError("cannot update while clutch is disengaged")
        a = self.args
        mapped_pan, mapped_roll = self.mapped(sample)
        self.pan_target = self.filters["pan"].update(
            self.pan_start + mapped_pan - self.pan_anchor)
        self.roll_target = self.filters["roll"].update(
            self.roll_start + mapped_roll - self.roll_anchor)
        if not a.no_gripper:
            self.trigger_pct = clip(self.filters["trigger"].update(sample.trigger_percent), 0, 100)

        raw_y = -sample.joy_y if a.invert_y else sample.joy_y
        # Stop promptly when the stick returns to center; otherwise a heavy
        # low-pass filter can make the arm creep long after release.
        if abs(raw_y) <= a.joy_deadzone:
            self.filters["y"].value = 0.0
        else:
            self.filters["y"].update(raw_y)
        self.y_speed = shaped_velocity(
            self.filters["y"].value, a.joy_deadzone, a.y_max_speed)

        if a.no_rail:
            # Height is not driven: Y moves alone, Z stays where it started.
            self.z_speed = 0.0
            self.y_cm = bounded_offset(self.y_cm, self.y_speed * dt, a.y_limit_cm)
            return self.y_cm, self.z_cm

        # arm_v already has a saved center and an unwrapped gear turn count.
        # Median protects against isolated AS5600 spikes before the EMA.
        rail_value = -sample.arm_v if a.invert_z else sample.arm_v
        self.rail_history.append(rail_value)
        median_rail = statistics.median(self.rail_history)
        full_scale = a.z_full_scale_mm if rail_units == "mm" else a.z_full_scale_deg
        normalized_z = clip(median_rail / full_scale, -1.0, 1.0)
        if abs(normalized_z) <= a.z_deadzone:
            self.filters["z"].value = 0.0
        else:
            self.filters["z"].update(normalized_z)
        self.z_speed = shaped_velocity(
            self.filters["z"].value, a.z_deadzone, a.z_max_speed)

        # Small capped increments prevent a stalled loop or USB delay from
        # turning into a sudden large move after it recovers.
        self.y_cm = bounded_offset(self.y_cm, self.y_speed * dt, a.y_limit_cm)
        self.z_cm = bounded_offset(self.z_cm, self.z_speed * dt, a.z_limit_cm)
        return self.y_cm, self.z_cm


def calibrated_range(calib: Path, name: str) -> tuple[float, float]:
    """LeRobot calibrated motor range (degrees around its midpoint)."""
    data = json.loads(calib.read_text())
    raw_min, raw_max = data[name]["range_min"], data[name]["range_max"]
    mid = (raw_min + raw_max) / 2.0
    scale = 360.0 / 4095.0
    return (raw_min - mid) * scale, (raw_max - mid) * scale


def gripper_target(pct: float, bounds: tuple[float, float], reverse: bool) -> float:
    """Map 0% open, 100% closed; flip mapping if gripper mechanics are reversed."""
    lo, hi = bounds
    fraction = clip(pct / 100.0, 0.0, 1.0)
    return hi - fraction * (hi - lo) if reverse else lo + fraction * (hi - lo)


def setup_arm(args: argparse.Namespace):
    """Connect only. No joint target is sent until valid data and clutch=1."""
    import numpy as np
    import arm_ik
    import run_rotations

    robot_args = argparse.Namespace(
        port=args.arm_port, id=args.id,
        calibration_dir=args.calibration_dir,
        home_file=args.home_file,
        max_relative_target=args.max_relative_target,
    )
    robot = arm_ik.make_robot(robot_args)
    calibration_path = Path(robot.calibration_fpath)
    pan_roll = run_rotations.load_limits(calibration_path)
    flex_deg = arm_ik.load_limits(calibration_path, args.margin)
    lo, hi = arm_ik.model_limits(flex_deg)
    gripper = calibrated_range(calibration_path, "gripper")
    home = arm_ik.load_home(arm_ik.home_path(robot, robot_args))
    home_tip, _ = arm_ik.fk(arm_ik.to_model(home))

    bounds: dict[str, tuple[float, float]] = dict(pan_roll)
    bounds.update({joint: tuple(map(float, pair)) for joint, pair in zip(arm_ik.ARM, flex_deg)})
    bounds["gripper"] = gripper

    try:
        robot.connect()
    except BaseException:
        try:
            robot.disconnect()
        except Exception:
            pass
        raise
    print(f"SO-101 connected on {args.arm_port}; calibration {calibration_path}")
    print("  Gripper calibrated range: {:.1f}..{:.1f} degrees; ".format(*gripper)
          + ("REVERSED" if args.gripper_reverse else "lower=end open (VERIFY ON HARDWARE)"))
    return robot, home_tip, (lo, hi), bounds


def observe(robot: Any) -> dict[str, float]:
    return {name: float(val) for name, val in robot.get_observation().items()
            if name.endswith(".pos")}


def hold_current(robot: Any) -> dict[str, float]:
    """Best-effort freeze at OBSERVED position, not the last requested target."""
    obs = observe(robot)
    robot.send_action(dict(obs))
    return obs


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--live", action="store_true", help="Send real SO-101 commands (default: DRY RUN)")
    p.add_argument("--encoder-port", help="Black Pill serial port, default auto-detect")
    p.add_argument("--stlink", action="store_true", help="Read Black Pill through ST-Link + OpenOCD")
    p.add_argument("--arm-port", help="SO-101 serial port (required for --live)")
    p.add_argument("--id", default="my_follower", help="SO-101 calibration ID")
    p.add_argument("--calibration-dir", type=Path)
    p.add_argument("--home-file", type=Path)
    p.add_argument("--rate", type=float, default=8.0, help="Maximum command rate in Hz (default 8)")
    p.add_argument("--stale-after", type=float, default=0.7, help="Hold if stream stops for this many seconds")
    p.add_argument("--max-speed", type=float, default=15.0, help="Joint target slew cap in degrees/second")
    p.add_argument("--max-relative-target", type=float, default=8.0,
                   help="LeRobot max per-command change in degrees")
    p.add_argument("--margin", type=float, default=2.0, help="Flex-joint calibration margin degrees")
    p.add_argument("--angle-smoothing", type=float, default=0.30)
    p.add_argument("--axis-smoothing", type=float, default=0.30)
    p.add_argument("--z-smoothing", type=float, default=0.20, help="Smaller = stronger AS5600 filtering")
    p.add_argument("--trigger-smoothing", type=float, default=0.30)
    p.add_argument("--joy-deadzone", type=float, default=0.10)
    p.add_argument("--z-deadzone", type=float, default=0.12)
    p.add_argument("--y-max-speed", type=float, default=2.0, help="Y tip speed cm/s at full joystick")
    p.add_argument("--z-max-speed", type=float, default=1.5, help="Z tip speed cm/s at full rail offset")
    p.add_argument("--z-full-scale-deg", type=float, default=360.0,
                   help="Rail displacement (gear degrees) for full Z speed")
    p.add_argument("--z-full-scale-mm", type=float, default=40.0,
                   help="Rail displacement (mm) for full Z speed if rail mm/rev is configured")
    p.add_argument("--y-limit-cm", type=float, default=15.0,
                   help="Y offset limit relative to saved arm home")
    p.add_argument("--z-limit-cm", type=float, default=15.0,
                   help="Z offset limit relative to saved arm home")
    p.add_argument("--invert-y", action="store_true")
    p.add_argument("--invert-z", action="store_true")
    p.add_argument("--trigger-mode", choices=("digital", "analog"), default="digital",
                   help="digital: existing firmware 0/1 open/close, no flashing; analog: percentage if firmware provides it")
    p.add_argument("--no-rail", action="store_true",
                   help="Do not drive Z (height) from the rail; the AS5600 is not needed")
    p.add_argument("--no-gripper", action="store_true",
                   help="Do not drive the gripper; the trigger is not needed")
    p.add_argument("--gripper-reverse", action="store_true",
                   help="Swap open and closed motor positions if needed")
    p.add_argument("--pitch", type=float, default=None,
                   help="Fixed pitch in degrees (default: capture current pitch when clutch engages)")
    p.add_argument("--auto-pitch", action="store_true", help="IK may vary pitch to reach a point")
    p.add_argument("--roll-in-min", type=float, default=-500.0)
    p.add_argument("--roll-in-max", type=float, default=500.0)
    p.add_argument("--twist-in-min", type=float, default=-500.0)
    p.add_argument("--twist-in-max", type=float, default=500.0)
    p.add_argument("--roll-out-min", type=float, default=-180.0)
    p.add_argument("--roll-out-max", type=float, default=180.0)
    p.add_argument("--pan-out-min", type=float, default=-107.7)
    p.add_argument("--pan-out-max", type=float, default=109.7)
    return p


def validate(args: argparse.Namespace, p: argparse.ArgumentParser) -> None:
    if args.live and not args.arm_port:
        p.error("--live requires --arm-port")
    if args.stlink and args.encoder_port:
        p.error("choose --stlink OR --encoder-port, not both")
    for name in ("rate", "stale_after", "max_speed", "max_relative_target", "z_full_scale_deg",
                 "z_full_scale_mm", "y_limit_cm", "z_limit_cm", "y_max_speed", "z_max_speed"):
        v = getattr(args, name)
        if not math.isfinite(v) or v <= 0:
            p.error(f"--{name.replace('_', '-')} must be positive and finite")
    for name in ("angle_smoothing", "axis_smoothing", "z_smoothing", "trigger_smoothing"):
        v = getattr(args, name)
        if not math.isfinite(v) or not 0 < v <= 1:
            p.error(f"--{name.replace('_', '-')} must be in (0,1]")
    for name in ("joy_deadzone", "z_deadzone"):
        v = getattr(args, name)
        if not math.isfinite(v) or not 0 <= v < 1:
            p.error(f"--{name.replace('_', '-')} must be in [0,1)")
    if not math.isfinite(args.margin) or args.margin < 0:
        p.error("--margin must be nonnegative")
    if args.pitch is not None and not math.isfinite(args.pitch):
        p.error("--pitch must be finite")
    for name in ("roll", "twist"):
        if getattr(args, f"{name}_in_min") >= getattr(args, f"{name}_in_max"):
            p.error(f"invalid {name} input range")
    for name in ("roll", "pan"):
        if getattr(args, f"{name}_out_min") >= getattr(args, f"{name}_out_max"):
            p.error(f"invalid {name} output range")


def main() -> int:
    p = parser()
    args = p.parse_args()
    validate(args, p)

    try:
        from read_encoders import CenterError, EncoderStream, PortError
    except ImportError as exc:
        print(f"Cannot import updated lib/read_encoders.py from {LIB}: {exc}", file=sys.stderr)
        return 1

    robot = None
    encoders = None
    state = None
    try:
        encoders = EncoderStream(port=args.encoder_port, stlink=args.stlink)
        if not args.no_rail and (encoders.center is None or encoders.center.rail is None):
            raise RuntimeError(
                "Saved center for BOTH MKS motors AND AS5600 is required. "
                "Put all controls at neutral and run: python lib/read_encoders.py --center")
        encoders.start()  # background reader is vital to unwrapping AS5600 turns reliably
        print(f"Reading controller on {encoders.port}; rail unit = {encoders.arm_v_unit}")
        print("Clutch: hold PB9 button (1) to control; release (0) to freeze.")
        if args.trigger_mode == "digital":
            print("NO-FLASH mode: PB10 trigger is DIGITAL (0% open / 100% closed).")
        else:
            print("Analog mode requires firmware already transmitting trigger_percent.")

        home_tip = None
        model_limits = None
        bounds = None
        last_sent: dict[str, float] = {}
        if args.live:
            robot, home_tip, model_limits, bounds = setup_arm(args)
            print("LIVE ENABLED: use a physical power disconnect for emergencies.")
        else:
            print("DRY RUN ONLY. No arm connection/commands; add --live --arm-port PORT.")

        import arm_ik
        import numpy as np

        state = TargetState(args)
        previous_sample = None
        freshness = time.monotonic()
        last_motion_at = time.monotonic()
        last_status: str | None = None
        cycle_period = 1.0 / args.rate

        def pause(message: str) -> None:
            nonlocal last_status, last_sent
            if state.engaged:
                state.pause()
                if robot is not None:
                    obs = hold_current(robot)
                    last_sent = {name[:-4]: val for name, val in obs.items() if name.endswith(".pos")}
            if message != last_status:
                print(message, flush=True)
                last_status = message

        while True:
            began = time.monotonic()
            sample = encoders.latest()  # re-raises board disconnect from background thread

            if began - freshness > args.stale_after:
                pause("HOLD: controller stream is stale; no new targets")

            if sample is not None and sample is not previous_sample:
                previous_sample = sample
                freshness = began
                if args.trigger_mode == "digital":
                    # Existing CTL1 firmware has a binary PB10 trigger. Do not
                    # falsely claim to read intermediate squeeze percentages.
                    from types import SimpleNamespace
                    values = vars(sample).copy()
                    values["trigger_percent"] = (100.0 * sample.trigger
                                                 if sample.trigger in (0, 1)
                                                 else math.nan)
                    sample = SimpleNamespace(**values)
                reason = valid_input(sample, rail_uncertain=encoders.rail_uncertain,
                                     need_rail=not args.no_rail,
                                     need_trigger=not args.no_gripper)
                if reason:
                    pause("HOLD: " + reason)
                elif sample.button == 0:
                    pause("CLUTCH OFF: frozen at current position")
                else:
                    if not state.engaged:
                        if robot is not None:
                            obs = observe(robot)
                            q_deg = [obs[f"{name}.pos"] for name in arm_ik.ARM]
                            tip, actual_pitch = arm_ik.fk(arm_ik.to_model(q_deg))
                            yy, zz = (tip - home_tip) * 100
                            pan = obs["shoulder_pan.pos"]
                            roll = obs["wrist_roll.pos"]
                            last_sent = {k[:-4]: v for k, v in obs.items() if k.endswith(".pos")}
                        else:
                            yy, zz, pan, roll, actual_pitch = 0.0, 0.0, 0.0, 0.0, 0.0
                        pitch_deg = (args.pitch if args.pitch is not None else
                                     math.degrees(actual_pitch))
                        state.engage(sample, pan=pan, roll=roll, y_cm=float(yy),
                                     z_cm=float(zz), pitch_deg=pitch_deg)
                        last_motion_at = began
                        last_status = "CLUTCH ON"
                        print("CLUTCH ON: re-anchored pan/roll and Y/Z to current pose; "
                              "no accumulated jump", flush=True)
                        # No target is sent on the engagement sample.
                    else:
                        dt = max(0.0, min(began - last_motion_at, cycle_period))
                        last_motion_at = began
                        old_y, old_z = state.y_cm, state.z_cm
                        state.update(sample, dt, encoders.arm_v_unit)

                        if robot is None:
                            print(
                                f"src roll={sample.arm_roll:+7.1f} twist={sample.arm_twist:+7.1f} "
                                f"joy_y={sample.joy_y:+.2f} rail={sample.arm_v:+7.1f}{encoders.arm_v_unit} "
                                f"trigger={sample.trigger_percent:5.1f}% | "
                                f"pan={state.pan_target:+6.1f} roll={state.roll_target:+6.1f} "
                                f"Y={state.y_cm:+6.2f}cm ({state.y_speed:+.2f}cm/s) "
                                f"Z={state.z_cm:+6.2f}cm ({state.z_speed:+.2f}cm/s)",
                                flush=True)
                        else:
                            obs = observe(robot)
                            current = np.array([obs[f"{joint}.pos"] for joint in arm_ik.ARM])
                            q_now = arm_ik.to_model(current)
                            target_tip = home_tip + np.array([state.y_cm, state.z_cm]) / 100.0
                            result = arm_ik.solve(target_tip, math.radians(state.pitch_deg),
                                                  q_now, *model_limits, args.auto_pitch)

                            desired: dict[str, float] = {
                                "shoulder_pan": state.pan_target,
                                "wrist_roll": state.roll_target,
                            }
                            if not args.no_gripper:
                                desired["gripper"] = gripper_target(
                                    state.trigger_pct, bounds["gripper"], args.gripper_reverse)
                            if result is None:
                                # Keep the last accepted tip target; do NOT accumulate
                                # an unreachable Y/Z target and later jump toward it.
                                state.y_cm, state.z_cm = old_y, old_z
                                if last_status != "IK_UNREACHABLE":
                                    print("IK limit reached: holding Y/Z, other joints still respond")
                                    last_status = "IK_UNREACHABLE"
                            else:
                                q, _ = result
                                desired.update(zip(arm_ik.ARM, map(float, arm_ik.from_model(q))))
                                last_status = "CLUTCH ON"

                            action = dict(obs)  # preserve unspecified axes
                            max_step = args.max_speed * dt
                            for name, target in desired.items():
                                lo, hi = bounds[name]
                                goal = clip(target, lo, hi)
                                # Limit from the OBSERVED position so a slow/stalled
                                # servo cannot get far behind an accumulating target.
                                previous = obs[f"{name}.pos"]
                                value = speed_limit(previous, goal, max_step)
                                action[f"{name}.pos"] = clip(value, lo, hi)
                                last_sent[name] = action[f"{name}.pos"]
                            robot.send_action(action)
                            print(
                                f"CMD pan={action['shoulder_pan.pos']:+6.1f} "
                                f"roll={action['wrist_roll.pos']:+6.1f} "
                                f"Y={state.y_cm:+5.2f}cm Z={state.z_cm:+5.2f}cm "
                                f"grip={state.trigger_pct:5.1f}%", flush=True)

            time.sleep(max(0, cycle_period - (time.monotonic() - began)))

    except KeyboardInterrupt:
        print("\nStopped (servos may continue holding their last commanded pose).")
        return 0
    except (PortError, CenterError, RuntimeError, OSError, KeyError, ValueError, ImportError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    finally:
        if robot is not None:
            # A final measured-position hold is best effort, not a physical E-stop.
            # Do not issue any action if the clutch never engaged.
            if state is not None and state.engaged:
                try:
                    hold_current(robot)
                except Exception as exc:
                    print(f"Warning: could not hold arm on shutdown: {exc}", file=sys.stderr)
            robot.disconnect()
        if encoders is not None:
            encoders.close()


if __name__ == "__main__":
    sys.exit(main())
