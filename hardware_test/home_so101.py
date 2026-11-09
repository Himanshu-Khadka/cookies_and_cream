#!/usr/bin/env python3
"""Move every joint to a home pose read from the calibration file, hold, then release.

--pose range (default): the midpoint of each joint's saved range_min..range_max, i.e. 0 degrees.
--pose calibration: raw count 2047 on every joint, the "middle" pose you held while calibrating.

The gripper keeps its current opening. Press Enter (or Ctrl+C) to release torque and exit;
support the arm first, because it goes limp.
"""
import argparse
import math
import sys
import time

from record_telemetry import JOINTS, RealArm, SimulatedArm, calibrated_positions, command_limits

BODY = JOINTS[:-1]


def read_pose(arm):
    started = time.monotonic()
    pose = calibrated_positions(arm.read("present_position"), arm.calibration)
    if time.monotonic() - started > 0.5:
        raise RuntimeError("Position read took over 500 ms; stopping")
    return pose


def home_step(requested, measured, calibration, held, dt, speed, max_lag):
    """Slew targets toward the request; a target may lead the measured joint by max_lag.

    The shared bounded_targets() allows only 5 deg of lead, which at the configured servo
    gain is too little torque for shoulder_lift to raise the arm against gravity.
    """
    step = speed * min(dt, 0.1)
    targets = {}
    for joint in JOINTS:
        low, high = command_limits(joint, calibration[joint])
        target = held[joint] + max(-step, min(step, requested[joint] - held[joint]))
        target = max(measured[joint] - max_lag, min(measured[joint] + max_lag, target))
        targets[joint] = max(low, min(high, target))
    return targets


def home_pose(calibration, pose):
    """Home angle per body joint, from the saved calibration."""
    if pose == "range":
        return {joint: 0.0 for joint in BODY}
    return calibrated_positions({joint: 2047 for joint in JOINTS}, calibration)


def go_home(arm, home, speed, tolerance, max_lag):
    """Slew all body joints to home together, starting from a fresh measurement.

    Under gravity the servos settle a few degrees short of their goal, so once a target has
    arrived its goal is nudged past home in proportion to the remaining error.
    """
    held = read_pose(arm)
    requested = dict(home)
    requested["gripper"] = held["gripper"]
    correction = {joint: 0.0 for joint in BODY}
    previous_tick = time.monotonic() - 1 / 30
    # Stop if no joint gets closer to home for 3 s (blocked, or too weak to lift).
    best = sum(abs(held[joint] - home[joint]) for joint in BODY)
    last_progress = time.monotonic()
    while True:
        measured = read_pose(arm)
        now = time.monotonic()
        dt = now - previous_tick
        distance = sum(abs(measured[joint] - home[joint]) for joint in BODY)
        if distance < best - 0.5:
            best, last_progress = distance, now
        elif now - last_progress > 3:
            far = {j: round(measured[j] - home[j], 1) for j in BODY
                   if abs(measured[j] - home[j]) > tolerance}
            raise RuntimeError(f"No progress toward home for 3 s; still off: {far}")
        for joint in BODY:
            if abs(held[joint] - requested[joint]) < 0.01 or correction[joint]:
                error = home[joint] - measured[joint]
                correction[joint] = max(-max_lag, min(max_lag, correction[joint] + 2 * error * dt))
                requested[joint] = home[joint] + correction[joint]
        targets = home_step(requested, measured, arm.calibration, held, dt, speed, max_lag)
        arm.send_action(targets)
        held, previous_tick = targets, now
        if all(abs(measured[j] - home[j]) <= tolerance for j in BODY):
            print("Home: " + " | ".join(f"{j} {measured[j]:.2f} (target {home[j]:.2f})" for j in BODY))
            return
        time.sleep(max(0, 1 / 30 - (time.monotonic() - now)))



def run(args):
    arm = SimulatedArm("follower") if args.simulate else RealArm(
        "follower", args.port, args.id
    )

    homed = False

    try:
        home = home_pose(arm.calibration, args.pose)

        print(
            f"{'SIMULATION | ' if args.simulate else ''}"
            f"Moving to '{args.pose}' home at {args.speed:g} deg/s: "
            + ", ".join(f"{j} {home[j]:.1f}" for j in BODY),
            flush=True
        )

        arm.prepare_teleoperation()

        go_home(arm, home, args.speed, args.tolerance, args.max_lag)

        # Only set this after go_home successfully finishes.
        homed = True

        if not args.simulate:
            if args.keep_torque:
                print("Home reached. Keeping motor torque enabled.")
            else:
                input(
                    "Holding home. Support the arm, then press Enter "
                    "to release torque> "
                )

    finally:
        if not args.simulate and args.keep_torque and homed:
            # RealArm.close() normally disables torque when
            # self.teleoperating is True. Setting it False makes
            # close() disconnect without releasing motor torque.
            arm.teleoperating = False

        arm.close()



def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--id", default="my_follower", help="Saved follower calibration ID")
    parser.add_argument("--pose", choices=("range", "calibration"), default="range",
                        help="range: midpoint of saved limits; calibration: pose held while calibrating")
    parser.add_argument("--speed", type=float, default=20, help="Command slew in degrees/s")
    parser.add_argument("--tolerance", type=float, default=1.5,
                        help="Degrees from home that count as arrived (servos settle slightly short)")
    parser.add_argument("--max-lag", type=float, default=12,
                        help="Degrees a target may lead the measured joint (more = more lifting torque)")
    parser.add_argument("--simulate", action="store_true", help="Try without connecting hardware")
    parser.add_argument(
    "--keep-torque",
    action="store_true",
    help="Keep the motors energized after successful homing for the next script"
)
    args = parser.parse_args(argv)
    if not math.isfinite(args.speed) or args.speed <= 0:
        parser.error("--speed must be finite and positive")
    if not math.isfinite(args.max_lag) or not 0 < args.max_lag <= 20:
        parser.error("--max-lag must be between 0 and 20 degrees")
    try:
        run(args)
    except (KeyboardInterrupt, EOFError):
        print("\nStopped.")
    except Exception as exc:
        print(f"Stopped: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
