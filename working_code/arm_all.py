#!/usr/bin/env python3
"""Drive the whole SO-101 arm from one prompt using run_rotations.py and arm_ik.py (unchanged).

Keep run_rotations.py, arm_ik.py and this file in the same folder.

Input per line:  pan roll y z [pitch]
  pan, roll : degrees, 0 = calibration midpoint (same as run_rotations.py)
  y, z      : cm relative to the HOME tip you saved with `arm_ik.py sethome` (same as arm_ik.py)
  pitch     : optional, degrees above horizontal (default: home pitch, or --pitch)
Ctrl+C to stop.

  python arm_all.py --port /dev/ttyACM0 [--id my_follower] [--auto-pitch] [--clamp]
"""
import argparse
from pathlib import Path

import numpy as np

import arm_ik
import run_rotations


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyACM0, COM3")
    p.add_argument("--id", default="my_follower")
    p.add_argument("--calibration-dir", type=Path, default=None)
    p.add_argument("--home-file", type=Path, default=None)
    p.add_argument("--max-relative-target", type=float, default=None, help="Per-command movement cap in degrees")
    p.add_argument("--pitch", type=float, default=None, help="Default pitch in deg (default: home pitch)")
    p.add_argument("--auto-pitch", action="store_true")
    p.add_argument("--clamp", action="store_true")
    p.add_argument("--margin", type=float, default=2.0)
    args = p.parse_args()

    robot = arm_ik.make_robot(args)          # ONE robot object shared by both
    calib = Path(robot.calibration_fpath)

    pr_lim = run_rotations.load_limits(calib)     # shoulder_pan / wrist_roll limits (deg)
    lim_deg = arm_ik.load_limits(calib, args.margin)
    lo, hi = arm_ik.model_limits(lim_deg)
    home = arm_ik.load_home(arm_ik.home_path(robot, args))
    home_tip, home_pitch = arm_ik.fk(arm_ik.to_model(home))
    default_pitch = args.pitch if args.pitch is not None else float(np.rad2deg(home_pitch))

    print(f"Calibration: {calib}")
    for m, (a, b) in pr_lim.items():
        print(f"  {m:14s} {a:+7.1f} .. {b:+7.1f} deg")
    for m, (a, b), h in zip(arm_ik.ARM, lim_deg, home):
        print(f"  {m:14s} {a - h:+7.1f} .. {b - h:+7.1f} deg (relative to home)")
    print(f"Default pitch {default_pitch:+.1f} deg. y/z in cm from home tip.\n")

    robot.connect()
    try:
        while True:
            raw = input("pan roll y z [pitch]: ").replace(",", " ").split()
            try:
                if len(raw) not in (4, 5):
                    raise ValueError
                pan, roll, y, z = map(float, raw[:4])
                pitch = float(raw[4]) if len(raw) == 5 else default_pitch
            except ValueError:
                print("Enter 4 or 5 numbers, e.g.  20 -30 5 3   or   20 -30 5 3 -45")
                continue

            q_deg, base = arm_ik.read_q(robot)
            q_now = arm_ik.to_model(q_deg)
            target = home_tip + np.array([y, z]) / 100.0
            p_rad = np.deg2rad(pitch)

            # Solve the IK first; if it fails, move nothing (including pan/roll).
            if args.clamp:
                res = arm_ik.solve_clamped(target, p_rad, q_now, lo, hi, args.auto_pitch)
                if res is None:
                    print("  -> unreachable, nothing moved")
                    continue
                (q, p_used), tgt_used, clamped = res
            else:
                res = arm_ik.solve(target, p_rad, q_now, lo, hi, args.auto_pitch)
                if res is None:
                    print("  -> unreachable (workspace or joint limits for that pitch), nothing moved"
                          + ("" if args.auto_pitch else "; try --auto-pitch"))
                    continue
                (q, p_used), tgt_used, clamped = res, target, False

            action = dict(base)              # keeps the gripper where it is
            for name, val in (("shoulder_pan", pan), ("wrist_roll", roll)):
                c = run_rotations.clamp(val, *pr_lim[name])
                if c != val:
                    print(f"  {name}: {val}° outside range, clamped to {c:.1f}°")
                action[f"{name}.pos"] = c
            for name, val in zip(arm_ik.ARM, arm_ik.from_model(q)):
                action[f"{name}.pos"] = float(val)
            robot.send_action(action)        # single send for all five joints

            note = ""
            if clamped:
                yy, zz = (tgt_used - home_tip) * 100
                note += f" [clamped to y={yy:+.1f} z={zz:+.1f}]"
            if abs(np.rad2deg(p_used) - pitch) > 1e-6:
                note += f" [pitch changed to {np.rad2deg(p_used):+.1f}]"
            print(f"  -> pan {action['shoulder_pan.pos']:+.1f}  roll {action['wrist_roll.pos']:+.1f}  "
                  f"lift/elbow/wrist (rel home) "
                  + "/".join(f"{d:+.1f}" for d in arm_ik.from_model(q) - home) + note)
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()