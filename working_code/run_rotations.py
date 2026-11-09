#!/usr/bin/env python3
"""Move SO-101 shoulder_pan and wrist_roll by angle (degrees, 0 = calibration midpoint).

Examples:
  python move_arm.py --port /dev/ttyACM0
  python move_arm.py --port COM3 --id alice_follower
  python move_arm.py --port /dev/ttyACM0 --mode udp --udp-port 5005
"""
import argparse
import json
import socket
import sys
from pathlib import Path

from lerobot.robots.so_follower.so_follower import SOFollower
from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig

MOTORS = ["shoulder_pan", "wrist_roll"]
RESOLUTION = 4096  # STS3215 ticks per revolution


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyACM0, COM3, /dev/tty.usbmodem123")
    p.add_argument("--id", default="my_follower", help="Robot id used when you calibrated (default: my_follower)")
    p.add_argument("--calibration-dir", type=Path, default=None,
                   help="Folder containing <id>.json (default: LeRobot's cache folder)")
    p.add_argument("--max-relative-target", type=float, default=None,
                   help="Max per-command movement in degrees (safety limit). Default: LeRobot's setting")
    p.add_argument("--mode", choices=["interactive", "udp"], default="interactive")
    p.add_argument("--host", default="127.0.0.1", help="UDP bind address (use 0.0.0.0 for other machines)")
    p.add_argument("--udp-port", type=int, default=5005)
    return p.parse_args()


def load_limits(path: Path) -> dict:
    calib = json.loads(path.read_text())
    limits = {}
    for m in MOTORS:
        rmin, rmax = calib[m]["range_min"], calib[m]["range_max"]
        mid = (rmin + rmax) / 2
        to_deg = lambda raw: (raw - mid) * 360 / (RESOLUTION - 1)
        limits[m] = (to_deg(rmin), to_deg(rmax))
    return limits


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def parse_angles(text):
    pan, roll = map(float, text.split())
    return {"shoulder_pan": pan, "wrist_roll": roll}


def build_action(base, targets, limits, verbose=False):
    action = dict(base)
    for m, v in targets.items():
        c = clamp(v, *limits[m])
        if verbose and c != v:
            print(f"  {m}: {v}° outside range, clamped to {c:.1f}°")
        action[f"{m}.pos"] = c
    return action


def run_interactive(robot, limits):
    while True:
        raw = input("Angles 'pan roll' in degrees (q to quit): ").strip()
        if raw.lower() == "q":
            break
        try:
            targets = parse_angles(raw)
        except ValueError:
            print("Enter two numbers, e.g. 30 -45")
            continue
        obs = robot.get_observation()
        base = {k: v for k, v in obs.items() if k.endswith(".pos")}
        robot.send_action(build_action(base, targets, limits, verbose=True))


def run_udp(robot, limits, host, port):
    obs = robot.get_observation()
    base = {k: v for k, v in obs.items() if k.endswith(".pos")}
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((host, port))
    print(f"Listening on UDP {host}:{port}. Send 'pan roll' in degrees. Ctrl+C to stop.")
    while True:
        data, _ = sock.recvfrom(1024)
        sock.setblocking(False)  # drain the queue, keep only the newest message
        try:
            while True:
                data, _ = sock.recvfrom(1024)
        except BlockingIOError:
            pass
        sock.setblocking(True)
        try:
            targets = parse_angles(data.decode())
        except ValueError:
            continue
        base.update(build_action(base, targets, limits))
        robot.send_action(base)


def main():
    args = parse_args()

    cfg_kwargs = dict(port=args.port, id=args.id, use_degrees=True)
    if args.calibration_dir:
        cfg_kwargs["calibration_dir"] = args.calibration_dir
    if args.max_relative_target is not None:
        cfg_kwargs["max_relative_target"] = args.max_relative_target

    robot = SOFollower(SOFollowerRobotConfig(**cfg_kwargs))

    calib_path = Path(robot.calibration_fpath)
    if not calib_path.is_file():
        sys.exit(
            f"No calibration file at {calib_path}\n"
            f"Calibrate first:  lerobot-calibrate --robot.type=so101_follower "
            f"--robot.port={args.port} --robot.id={args.id}\n"
            f"or copy your <id>.json there, or pass --id / --calibration-dir."
        )

    limits = load_limits(calib_path)
    print(f"Calibration: {calib_path}")
    for m, (lo, hi) in limits.items():
        print(f"  {m}: {lo:.1f}° to {hi:.1f}°")

    robot.connect()
    try:
        if args.mode == "udp":
            run_udp(robot, limits, args.host, args.udp_port)
        else:
            run_interactive(robot, limits)
    except KeyboardInterrupt:
        pass
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()