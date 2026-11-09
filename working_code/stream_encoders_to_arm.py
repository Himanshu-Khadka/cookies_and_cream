#!/usr/bin/env python3
"""Control the SO-101 shoulder_pan and wrist_roll from two external encoders.

Project layout (the existing files do not need to be edited):
    project/
      lib/
        read_encoders.py
        center.json                 # created by read_encoders.py --center
      working_code/
        arm_all.py
        arm_ik.py
        run_rotations.py
        stream_encoders_to_arm.py   # this file

Encoder motor1 ("roll")  -500..500  -> wrist_roll  -180..180 degrees
Encoder motor2 ("twist") -500..500  -> shoulder_pan -107.7..109.7 degrees
Other arm_all.py inputs: y=0 cm, z=0 cm, pitch=0 degrees (by default).

The new program calls the *same helpers* used by arm_all.py. It does not pipe
text into arm_all.py because that program waits on input() and could accumulate
outdated commands in a pipe.

IMPORTANT: Default mode only prints values. Nothing drives the arm until --live
is supplied. Keep the arm clear of obstacles, and be ready to disconnect power.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
LIB = HERE.parent / "lib"
if str(LIB) not in sys.path:
    sys.path.insert(0, str(LIB))


def normalize(value: float, input_min: float, input_max: float,
              output_min: float, output_max: float) -> float:
    """Linearly scale a measurement, clipping beyond the *estimated* input range."""
    if not math.isfinite(value):
        raise ValueError("invalid encoder value (NaN or infinity)")
    if input_min >= input_max or output_min >= output_max:
        raise ValueError("each min must be less than its max")
    clipped = min(max(value, input_min), input_max)
    fraction = (clipped - input_min) / (input_max - input_min)
    return output_min + fraction * (output_max - output_min)


def limit_speed(previous: float, desired: float, max_change: float) -> float:
    """Never move a requested joint target more than max_change per update."""
    return previous + min(max(desired - previous, -max_change), max_change)


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--live", action="store_true", help="Actually send commands to SO-101 (otherwise print only)")
    p.add_argument("--encoder-port", default=None, help="Encoder board port (default: auto-detect Black Pill)")
    p.add_argument("--arm-port", default=None, help="SO-101 serial port; REQUIRED with --live")
    p.add_argument("--id", default="my_follower", help="SO-101 calibration ID")
    p.add_argument("--calibration-dir", type=Path, default=None)
    p.add_argument("--home-file", type=Path, default=None)
    p.add_argument("--rate", type=float, default=8.0, help="Maximum command/update frequency, Hz (default: 8)")
    p.add_argument("--smoothing", type=float, default=0.30,
                   help="Exponential filter fraction, (0, 1]; smaller = smoother (default: 0.30)")
    p.add_argument("--deadband", type=float, default=0.30,
                   help="Ignore target changes smaller than this many output degrees")
    p.add_argument("--max-speed", type=float, default=30.0,
                   help="Maximum target change per second, deg/s for all joints (default: 30)")
    p.add_argument("--max-relative-target", type=float, default=8.0,
                   help="LeRobot's extra per-command movement limit in degrees (default: 8)")
    p.add_argument("--stale-after", type=float, default=1.0,
                   help="Warn if no valid, new encoder data for this many seconds")
    p.add_argument("--roll-in-min", type=float, default=-500.0)
    p.add_argument("--roll-in-max", type=float, default=500.0)
    p.add_argument("--twist-in-min", type=float, default=-500.0)
    p.add_argument("--twist-in-max", type=float, default=500.0)
    p.add_argument("--roll-out-min", type=float, default=-180.0)
    p.add_argument("--roll-out-max", type=float, default=180.0)
    p.add_argument("--pan-out-min", type=float, default=-107.7)
    p.add_argument("--pan-out-max", type=float, default=109.7)
    p.add_argument("--pitch", type=float, default=0.0,
                   help="Fixed gripper pitch in degrees (default: 0, as requested)")
    p.add_argument("--use-home-pitch", action="store_true",
                   help="Use saved home pitch instead of 0, if pitch=0 is unreachable")
    p.add_argument("--auto-pitch", action="store_true",
                   help="Permit IK to find a different reachable pitch if needed")
    return p


def validate(args: argparse.Namespace, p: argparse.ArgumentParser) -> None:
    if args.live and not args.arm_port:
        p.error("--arm-port is required when using --live")
    for name in ("rate", "max_speed", "max_relative_target", "stale_after"):
        if getattr(args, name) <= 0 or not math.isfinite(getattr(args, name)):
            p.error(f"--{name.replace('_', '-')} must be a positive finite number")
    if not 0 < args.smoothing <= 1:
        p.error("--smoothing must be greater than 0 and at most 1")
    if args.deadband < 0 or not math.isfinite(args.deadband):
        p.error("--deadband must be nonnegative and finite")
    for prefix in ("roll_in", "twist_in", "roll_out", "pan_out"):
        lo, hi = getattr(args, prefix + "_min"), getattr(args, prefix + "_max")
        if not (math.isfinite(lo) and math.isfinite(hi) and lo < hi):
            p.error(f"--{prefix.replace('_', '-')} min must be below max (both finite)")
    if not math.isfinite(args.pitch):
        p.error("--pitch must be finite")


def map_encoders(angles, args: argparse.Namespace) -> tuple[float, float]:
    """Return (pan, roll), matching arm_all.py's input order."""
    roll = normalize(angles.motor1, args.roll_in_min, args.roll_in_max,
                     args.roll_out_min, args.roll_out_max)
    pan = normalize(angles.motor2, args.twist_in_min, args.twist_in_max,
                    args.pan_out_min, args.pan_out_max)
    return pan, roll


def start_arm(args: argparse.Namespace):
    """Prepare the identical IK target that arm_all.py receives as 'pan roll 0 0 0'.

    Returns (robot, calibration limits, fixed flex-joint targets, initial positions).
    All actuation still happens later in the encoder loop.
    """
    import numpy as np
    import arm_ik
    import run_rotations

    robot_args = argparse.Namespace(
        port=args.arm_port, id=args.id,
        calibration_dir=args.calibration_dir, home_file=args.home_file,
        max_relative_target=args.max_relative_target,
    )
    robot = arm_ik.make_robot(robot_args)
    calibration_path = Path(robot.calibration_fpath)
    pan_roll_limits = run_rotations.load_limits(calibration_path)
    joint_limits_deg = arm_ik.load_limits(calibration_path, margin_deg=2.0)
    lo, hi = arm_ik.model_limits(joint_limits_deg)
    home = arm_ik.load_home(arm_ik.home_path(robot, robot_args))
    home_tip, home_pitch = arm_ik.fk(arm_ik.to_model(home))
    pitch_deg = float(np.rad2deg(home_pitch)) if args.use_home_pitch else args.pitch

    robot.connect()
    try:
        current_angles, base = arm_ik.read_q(robot)
        # y=0 and z=0 mean the home tip location; pitch is ABSOLUTE, not
        # relative to home. This is the arm_all.py IK calculation.
        result = arm_ik.solve(home_tip, np.deg2rad(pitch_deg),
                              arm_ik.to_model(current_angles), lo, hi,
                              args.auto_pitch)
        if result is None:
            raise RuntimeError(
                f"Home y=0, z=0 at pitch={pitch_deg:.1f} degrees is unreachable. "
                "No commands sent. Try --use-home-pitch (or --auto-pitch); "
                "confirm SO-101 calibration and arm_ik.py sethome first."
            )
        target_angles, chosen_pitch = result
        flex_targets = {
            name: float(deg)
            for name, deg in zip(arm_ik.ARM, arm_ik.from_model(target_angles))
        }
        initial_positions = {
            name: float(base[f"{name}.pos"])
            for name in ("shoulder_pan", "wrist_roll", *arm_ik.ARM)
        }
        print(f"SO-101 connected: {args.arm_port}; calibration: {calibration_path}")
        print(f"  Fixed y/z: 0/0 cm; requested pitch: {pitch_deg:.1f}°, "
              f"IK pitch: {float(np.rad2deg(chosen_pitch)):.1f}°")
        print("  Arm calibration limits (pan/roll): " + ", ".join(
            f"{name} {bounds[0]:.1f}..{bounds[1]:.1f}°"
            for name, bounds in pan_roll_limits.items()))
        return robot, pan_roll_limits, flex_targets, initial_positions
    except BaseException:
        robot.disconnect()
        raise


def main() -> int:
    p = parser()
    args = p.parse_args()
    validate(args, p)

    try:
        from read_encoders import CenterError, EncoderStream, PortError
    except ImportError as error:
        print(f"Cannot import lib/read_encoders.py from {LIB}: {error}", file=sys.stderr)
        print("Place this script in working_code/ beside arm_all.py, "
              "with read_encoders.py in the sibling lib/ folder.", file=sys.stderr)
        return 1

    encoders = None
    robot = None
    try:
        encoders = EncoderStream(port=args.encoder_port)
        if encoders.center is None:
            raise RuntimeError(
                "No encoder center has been saved. With the controls physically "
                "at their neutral pose, first run: python lib/read_encoders.py --center"
            )
        encoders.start()
        print(f"Encoder board connected: {encoders.port}")
        print("  motor1 roll -> wrist_roll; motor2 twist -> shoulder_pan; AS5600 ignored")
        print(f"  Encoder estimated ranges: roll {args.roll_in_min}..{args.roll_in_max}; "
              f"twist {args.twist_in_min}..{args.twist_in_max}")
        print("  Clipping, filtering, deadband, and speed limits are enabled.")

        if args.live:
            robot, pan_roll_limits, flex_targets, last_sent = start_arm(args)
            print("LIVE CONTROL ENABLED. Ctrl+C stops commands/disconnects (arm may hold pose).")
        else:
            print("DRY RUN ONLY: arm will not move. Add --live --arm-port <PORT> to enable.")

        # The background reader always provides the most recent sample. Reading
        # it once each loop prevents a queue of old motion commands building up.
        last_sample = None
        last_good_at = time.monotonic()
        stale_reported = False
        filtered: dict[str, float] = {}
        period = 1.0 / args.rate
        max_step = args.max_speed / args.rate

        while True:
            cycle_started = time.monotonic()
            sample = encoders.latest()  # may raise PortError if cable is unplugged

            if sample is not None and sample is not last_sample:
                last_sample = sample
                if not (math.isfinite(sample.motor1) and math.isfinite(sample.motor2)):
                    # AS5600 may be NaN; it is NOT used.
                    print("Skipping invalid roll/twist encoder reading")
                else:
                    last_good_at = cycle_started
                    stale_reported = False
                    target_pan, target_roll = map_encoders(sample, args)
                    requested = {
                        "shoulder_pan": target_pan,
                        "wrist_roll": target_roll,
                    }

                    # Robot calibration is an additional, independent physical
                    # limit; it can be narrower than the requested mapping.
                    if args.live:
                        for name, (lo, hi) in pan_roll_limits.items():
                            requested[name] = max(lo, min(requested[name], hi))

                    # Low-pass filter on the target. For live control initialize
                    # from the robot's observed angles so the first valid
                    # encoder packet cannot immediately issue a large jump.
                    for name in ("shoulder_pan", "wrist_roll"):
                        if name not in filtered:
                            filtered[name] = last_sent[name] if args.live else requested[name]
                        if abs(requested[name] - filtered[name]) >= args.deadband:
                            filtered[name] += args.smoothing * (requested[name] - filtered[name])

                    if not args.live:
                        print(f"encoder roll={sample.motor1:+8.2f}  "
                              f"twist={sample.motor2:+8.2f}  | "
                              f"mapped pan={target_pan:+7.2f}°  "
                              f"roll={target_roll:+7.2f}°  y=0 z=0 pitch={args.pitch:g}")
                    else:
                        import arm_ik
                        # All FIVE joints use speed limiting, including joints
                        # controlled by IK. Preserve gripper's observed value.
                        _, observed = arm_ik.read_q(robot)
                        command = dict(observed)
                        all_targets = {**filtered, **flex_targets}
                        for name, desired in all_targets.items():
                            new_value = limit_speed(last_sent[name], desired, max_step)
                            command[f"{name}.pos"] = new_value
                            last_sent[name] = new_value
                        robot.send_action(command)
                        print(f"encoder roll={sample.motor1:+8.2f} "
                              f"twist={sample.motor2:+8.2f} | "
                              f"command pan={last_sent['shoulder_pan']:+7.2f}° "
                              f"roll={last_sent['wrist_roll']:+7.2f}°")

            if not stale_reported and time.monotonic() - last_good_at > args.stale_after:
                print(f"WARNING: no fresh valid encoder data for >{args.stale_after:g}s; "
                      "no new movement commands sent (servos may hold their last target).")
                stale_reported = True

            time.sleep(max(0.0, period - (time.monotonic() - cycle_started)))

    except KeyboardInterrupt:
        print("\nStopped. No further commands will be sent.")
        return 0
    except (PortError, CenterError, RuntimeError, OSError, ImportError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1
    finally:
        if robot is not None:
            robot.disconnect()
        if encoders is not None:
            encoders.close()


if __name__ == "__main__":
    sys.exit(main())
