#!/usr/bin/env python3
"""Type relative joint rotations in degrees; q or Ctrl+C releases torque and exits."""
import argparse
import math
import sys
import time

from record_telemetry import (
    JOINTS, RealArm, SimulatedArm, bounded_targets, calibrated_positions, command_limits,
)


def read_pose(arm):
    started = time.monotonic()
    pose = calibrated_positions(arm.read("present_position"), arm.calibration)
    if time.monotonic() - started > 0.5:
        raise RuntimeError("Position read took over 500 ms; stopping")
    return pose


def rotate(arm, joint, degrees, held, speed):
    """Move relative to a fresh measurement, retaining the other joints' targets."""
    measured = read_pose(arm)
    target = measured[joint] + degrees
    low, high = command_limits(joint, arm.calibration[joint])
    if not math.isfinite(degrees) or not low <= target <= high:
        print(f"Rejected: enter a rotation between {low - measured[joint]:.2f} "
              f"and {high - measured[joint]:.2f} degrees from here.")
        return held
    requested = held.copy()
    requested[joint] = target
    started = time.monotonic()
    previous_tick = started - 1 / 30
    # Allow settling, but stop if a motor cannot reach its goal.
    timeout = 3 + 2 * abs(target - held[joint]) / speed
    while True:
        measured = read_pose(arm)
        now = time.monotonic()
        if now - started > timeout:
            raise RuntimeError("Target was not reached in time; stopping")
        targets = bounded_targets(requested, measured, arm.calibration, held,
                                  now - previous_tick, speed, 30)
        arm.send_action(targets)
        held, previous_tick = targets, now
        if abs(held[joint] - target) < 0.01 and abs(measured[joint] - target) <= 1.0:
            print(f"{joint}: {measured[joint]:.2f} degrees (target {target:.2f})")
            return held
        time.sleep(max(0, 1 / 30 - (time.monotonic() - now)))


def run(args):
    arm = SimulatedArm("follower") if args.simulate else RealArm(
        "follower", args.port, args.id)
    try:
        print(f"{'SIMULATION | ' if args.simulate else ''}Controlling {args.joint}. "
              "Other joints hold their starting positions.\n"
              "Enter relative degrees, e.g. 30 or -10. q / Ctrl+C exits and releases torque.",
              flush=True)
        arm.prepare_teleoperation()
        held = read_pose(arm)
        while True:
            text = input("Rotate by degrees> ").strip()
            if text.lower() in ("q", "quit", "exit"):
                break
            try:
                degrees = float(text)
                if not math.isfinite(degrees):
                    raise ValueError
            except ValueError:
                print("Enter a finite number, such as 30 or -10, or q to quit.")
                continue
            held = rotate(arm, args.joint, degrees, held, args.speed)
    finally:
        arm.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/tty.usbmodem5C4C1241631")
    parser.add_argument("--id", default="my_follower", help="Saved follower calibration ID")
    parser.add_argument("--joint", choices=JOINTS[:-1], default="wrist_roll")
    parser.add_argument("--speed", type=float, default=20, help="Command slew in degrees/s")
    parser.add_argument("--simulate", action="store_true", help="Try without connecting hardware")
    args = parser.parse_args(argv)
    if not math.isfinite(args.speed) or args.speed <= 0:
        parser.error("--speed must be finite and positive")
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
