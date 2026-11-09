#!/usr/bin/env python3
"""SO-101 planar (2D) inverse kinematics for shoulder_lift, elbow_flex, wrist_flex.

Coordinates you type are in CENTIMETRES, relative to the HOME tip position you set:
    y = forward(+) / back(-),   z = up(+) / down(-),   pitch = gripper angle above horizontal (deg, +up)

Steps:
  1) python arm_ik.py sethome --port /dev/ttyACM0     # move arm by hand, press Enter
  2) python arm_ik.py check   --port /dev/ttyACM0     # (optional) compare FK numbers with a ruler
  3) python arm_ik.py run     --port /dev/ttyACM0     # type "y z [pitch]" targets
     python arm_ik.py run     --port /dev/ttyACM0 --mode udp   # stream "y z [pitch]" over UDP

Geometry comes from so101_new_calib.urdf (hard-coded below, no URDF parsing at runtime).
Joint angles are LeRobot "degrees" (0 = midpoint of the calibration range), which is what that URDF uses.
"""
import argparse
import json
import socket
import sys
import time
from pathlib import Path

import numpy as np

# ----------------------------------------------------------------------------------------------
# Geometry from so101_new_calib.urdf (metres). World frame: x forward, z up.
# All three flex axes point along world +Y, so positive joint angle tilts the arm DOWN/forward.
# V1/V2/V3 are the link vectors (x, z) with every joint at 0 deg.
# ----------------------------------------------------------------------------------------------
V1 = np.array([0.0280, 0.1126])    # shoulder_lift axis -> elbow_flex axis
V2 = np.array([0.1349, 0.0052])    # elbow_flex axis    -> wrist_flex axis
V3 = np.array([0.1592, -0.0079])   # wrist_flex axis    -> gripper_frame_link (tool point)
SHOULDER_FROM_PAN = np.array([0.0304, 0.0542])  # shoulder_lift axis relative to the pan axis (x, z)
SHOULDER_HEIGHT = 0.1166                        # shoulder_lift axis height above base_link origin

# Per-joint corrections if --check shows your arm disagrees with the model (degrees, sign +1/-1).
SIGN = {"shoulder_lift": 1.0, "elbow_flex": 1.0, "wrist_flex": 1.0}
OFFSET_DEG = {"shoulder_lift": 0.0, "elbow_flex": 0.0, "wrist_flex": 0.0}
# (model angle) = SIGN * (LeRobot degrees) + OFFSET_DEG

ARM = ["shoulder_lift", "elbow_flex", "wrist_flex"]
RESOLUTION = 4096
DEG_PER_TICK = 360.0 / (RESOLUTION - 1)

L1, L2, L3 = (float(np.hypot(*v)) for v in (V1, V2, V3))
A1, A2, A3 = (float(np.arctan2(v[1], v[0])) for v in (V1, V2, V3))
SIGN_V = np.array([SIGN[m] for m in ARM])
OFFS_V = np.array([OFFSET_DEG[m] for m in ARM])


# ----------------------------------------------------------------------------------------------
# Kinematics (pure numpy; microseconds)
# ----------------------------------------------------------------------------------------------
def _e(a):
    return np.stack([np.cos(a), np.sin(a)], -1)


def to_model(q_deg):
    """LeRobot degrees (3,) -> model radians."""
    return np.deg2rad(SIGN_V * np.asarray(q_deg, float) + OFFS_V)


def from_model(q_rad):
    """Model radians (...,3) -> LeRobot degrees."""
    return (np.rad2deg(q_rad) - OFFS_V) / SIGN_V


def fk(q_rad):
    """Model radians (3,) -> (tip (x, z) relative to shoulder_lift axis [m], pitch above horizontal [rad])."""
    q1, q2, q3 = q_rad
    tip = L1 * _e(A1 - q1) + L2 * _e(A2 - q1 - q2) + L3 * _e(A3 - q1 - q2 - q3)
    return tip, A3 - (q1 + q2 + q3)


def ik(target, pitch, lo, hi):
    """Closed-form IK for a batch of pitches.
    target: (2,) tip (x, z) relative to shoulder axis [m]; pitch: (N,) rad; lo/hi: (3,) model-rad limits.
    Returns q (N, 2, 3) for the two elbow branches and ok (N, 2) = reachable and within limits."""
    pitch = np.atleast_1d(pitch).astype(float)
    Q = A3 - pitch                                    # q1+q2+q3
    W = target[None, :] - L3 * _e(pitch)              # wrist_flex axis position
    c = ((W ** 2).sum(-1) - L1 ** 2 - L2 ** 2) / (2 * L1 * L2)
    reach = np.abs(c) <= 1.0
    d = np.arccos(np.clip(c, -1.0, 1.0))
    q = np.empty((len(pitch), 2, 3))
    for k, s in enumerate((1.0, -1.0)):
        delta = s * d
        q2 = A2 - A1 - delta
        alpha1 = np.arctan2(W[:, 1], W[:, 0]) - np.arctan2(L2 * np.sin(delta), L1 + L2 * np.cos(delta))
        q1 = A1 - alpha1
        q[:, k, :] = np.stack([q1, q2, Q - q1 - q2], -1)
    q = (q + np.pi) % (2 * np.pi) - np.pi
    ok = reach[:, None] & np.all((q >= lo) & (q <= hi), axis=-1)
    return q, ok


def solve(target, pitch, q_now, lo, hi, auto_pitch):
    """Pick the best solution. Returns (q_rad, pitch_used_rad) or None if unreachable.
    Prefers the requested pitch; with auto_pitch it searches other pitches (closest first).
    Among equals it picks the solution nearest the current joint angles (also picks the elbow branch)."""
    offs = np.deg2rad(np.arange(-180, 181)) if auto_pitch else np.array([0.0])
    q, ok = ik(target, pitch + offs, lo, hi)
    if not ok.any():
        return None
    dist = np.rad2deg(np.abs(q - q_now)).sum(-1)
    cost = np.abs(np.rad2deg(offs))[:, None] * 1000.0 + dist
    cost[~ok] = np.inf
    i, k = np.unravel_index(np.argmin(cost), cost.shape)
    return q[i, k], pitch + offs[i]


def solve_clamped(target, pitch, q_now, lo, hi, auto_pitch):
    """Like solve, but if unreachable, slide the target back toward the current tip until reachable."""
    r = solve(target, pitch, q_now, lo, hi, auto_pitch)
    if r:
        return r, target, False
    cur, _ = fk(q_now)
    a, b, best = 0.0, 1.0, None
    for _ in range(25):
        m = (a + b) / 2
        p = cur + m * (target - cur)
        r = solve(p, pitch, q_now, lo, hi, auto_pitch)
        if r:
            a, best = m, (r, p, True)
        else:
            b = m
    return best


# ----------------------------------------------------------------------------------------------
# Calibration + home
# ----------------------------------------------------------------------------------------------
def load_limits(calib_path, margin_deg):
    """LeRobot-degree limits per arm joint from the calibration file: (min_deg, max_deg), 0 = range midpoint."""
    calib = json.loads(Path(calib_path).read_text())
    lim = []
    for m in ARM:
        rmin, rmax = calib[m]["range_min"], calib[m]["range_max"]
        mid = (rmin + rmax) / 2
        lim.append(((rmin - mid) * DEG_PER_TICK + margin_deg, (rmax - mid) * DEG_PER_TICK - margin_deg))
    lim = np.array(lim)  # (3, 2) in LeRobot degrees
    return lim


def model_limits(lim_deg):
    """LeRobot-degree limits -> model-radian (lo, hi), handling SIGN/OFFSET."""
    a = np.deg2rad(SIGN_V * lim_deg[:, 0] + OFFS_V)
    b = np.deg2rad(SIGN_V * lim_deg[:, 1] + OFFS_V)
    return np.minimum(a, b), np.maximum(a, b)


def home_path(robot, args):
    if args.home_file:
        return Path(args.home_file)
    p = Path(robot.calibration_fpath)
    return p.with_name(f"{p.stem}_home.json")


def load_home(path):
    if not path.is_file():
        sys.exit(f"No home file at {path}.\nRun:  python arm_ik.py sethome --port <PORT> --id <ID>")
    return np.array([json.loads(path.read_text())["deg"][m] for m in ARM], float)


# ----------------------------------------------------------------------------------------------
# Robot helpers (lerobot imported lazily so the maths can be tested without it)
# ----------------------------------------------------------------------------------------------
def make_robot(args):
    from lerobot.robots.so_follower.so_follower import SOFollower
    from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig

    kw = dict(port=args.port, id=args.id, use_degrees=True)
    if args.calibration_dir:
        kw["calibration_dir"] = args.calibration_dir
    if args.max_relative_target is not None:
        kw["max_relative_target"] = args.max_relative_target
    robot = SOFollower(SOFollowerRobotConfig(**kw))
    cp = Path(robot.calibration_fpath)
    if not cp.is_file():
        sys.exit(
            f"No calibration file at {cp}\n"
            f"Run:  lerobot-calibrate --robot.type=so101_follower --robot.port={args.port} --robot.id={args.id}\n"
            f"or pass --id / --calibration-dir."
        )
    return robot


def read_q(robot):
    obs = robot.get_observation()
    base = {k: v for k, v in obs.items() if k.endswith(".pos")}
    return np.array([base[f"{m}.pos"] for m in ARM], float), base


def fmt_state(q_deg, home_deg):
    tip, pitch = fk(to_model(q_deg))
    home_tip, _ = fk(to_model(home_deg)) if home_deg is not None else (np.zeros(2), 0)
    rel = (tip - home_tip) * 100
    return tip, pitch, rel


# ----------------------------------------------------------------------------------------------
# Commands
# ----------------------------------------------------------------------------------------------
def cmd_sethome(args):
    robot = make_robot(args)
    robot.connect()
    try:
        robot.bus.disable_torque()
        print("\nTorque is OFF: the arm is limp. SUPPORT IT with your hand.")
        print("Move the arm to the pose you want as HOME, then press Enter (Ctrl+C to cancel).")
        input()
        q, _ = read_q(robot)
        tip, pitch = fk(to_model(q))
        path = home_path(robot, args)
        path.write_text(json.dumps({"deg": dict(zip(ARM, map(float, q))), "note": "LeRobot degrees, 0 = calibration range midpoint"}, indent=2))
        print(f"Saved home to {path}")
        for m, v in zip(ARM, q):
            print(f"  {m:14s} {v:+7.2f} deg (calibration frame)")
        print(f"  tip: {tip[0]*100:+.1f} cm forward / {tip[1]*100:+.1f} cm up of the shoulder_lift axis; pitch {np.rad2deg(pitch):+.1f} deg")
    finally:
        robot.disconnect()


def cmd_check(args):
    robot = make_robot(args)
    home = load_home(home_path(robot, args)) if not args.no_home else None
    robot.connect()
    try:
        robot.bus.disable_torque()
        print("Torque OFF. Move the arm by hand and compare the printed tip position with a ruler. Ctrl+C to stop.\n")
        while True:
            q, _ = read_q(robot)
            tip, pitch, rel = fmt_state(q, home)
            fwd = (tip[0] + SHOULDER_FROM_PAN[0]) * 100
            hgt = (tip[1] + SHOULDER_HEIGHT) * 100
            relq = q - home if home is not None else q
            print(
                f"\rjoints(rel home) lift {relq[0]:+6.1f} elbow {relq[1]:+6.1f} wrist {relq[2]:+6.1f} | "
                f"from pan axis: fwd {fwd:5.1f} cm, height above base {hgt:5.1f} cm | "
                f"rel home: y {rel[0]:+5.1f} z {rel[1]:+5.1f} cm | pitch {np.rad2deg(pitch):+5.1f}   ",
                end="", flush=True,
            )
            time.sleep(0.2)
    except KeyboardInterrupt:
        print()
    finally:
        robot.disconnect()


def parse_target(text, default_pitch_deg):
    parts = text.replace(",", " ").split()
    if len(parts) not in (2, 3):
        raise ValueError
    y, z = float(parts[0]), float(parts[1])
    pitch = float(parts[2]) if len(parts) == 3 else default_pitch_deg
    return y, z, pitch


def cmd_run(args):
    robot = make_robot(args)
    lim_deg = load_limits(robot.calibration_fpath, args.margin)
    lo, hi = model_limits(lim_deg)
    home = load_home(home_path(robot, args))
    home_tip, home_pitch = fk(to_model(home))
    home_pitch_deg = float(np.rad2deg(home_pitch))
    default_pitch = args.pitch if args.pitch is not None else home_pitch_deg

    print(f"Calibration: {robot.calibration_fpath}")
    print("Joint limits relative to home (0 = home), degrees:")
    for m, (a, b), h in zip(ARM, lim_deg, home):
        print(f"  {m:14s} {a - h:+7.1f} .. {b - h:+7.1f}")
    print(f"Home tip is the origin (0, 0). Default pitch {default_pitch:+.1f} deg. Units: cm.")

    robot.connect()
    q_now, base = read_q(robot)
    q_now_rad = to_model(q_now)

    def move(y, z, pitch_deg):
        nonlocal q_now_rad
        target = home_tip + np.array([y, z]) / 100.0
        p = np.deg2rad(pitch_deg)
        if args.clamp:
            res = solve_clamped(target, p, q_now_rad, lo, hi, args.auto_pitch)
            if res is None:
                return "unreachable (even after clamping)"
            (q, p_used), tgt_used, clamped = res
        else:
            res = solve(target, p, q_now_rad, lo, hi, args.auto_pitch)
            if res is None:
                return "unreachable: outside the workspace or joint limits for that pitch" + ("" if args.auto_pitch else " (try --auto-pitch)")
            (q, p_used), tgt_used, clamped = res, target, False
        deg = from_model(q)
        for m, v in zip(ARM, deg):
            base[f"{m}.pos"] = float(v)
        robot.send_action(base)
        q_now_rad = q
        note = ""
        if clamped:
            yy, zz = (tgt_used - home_tip) * 100
            note += f" [clamped to y={yy:+.1f} z={zz:+.1f}]"
        if abs(np.rad2deg(p_used) - pitch_deg) > 1e-6:
            note += f" [pitch changed to {np.rad2deg(p_used):+.1f}]"
        return "lift {:+.1f}  elbow {:+.1f}  wrist {:+.1f} (rel home){}".format(*(deg - home), note)

    try:
        if args.mode == "udp":
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind((args.host, args.udp_port))
            print(f"Listening on UDP {args.host}:{args.udp_port}. Send 'y z [pitch]' in cm. Ctrl+C to stop.")
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
                    y, z, pitch = parse_target(data.decode(), default_pitch)
                except ValueError:
                    continue
                msg = move(y, z, pitch)
                if msg.startswith("unreachable"):
                    print(msg)
        else:
            while True:
                raw = input("y z [pitch]  (cm, cm, deg; q to quit): ").strip()
                if raw.lower() == "q":
                    break
                try:
                    y, z, pitch = parse_target(raw, default_pitch)
                except ValueError:
                    print("Enter 'y z' or 'y z pitch', e.g.  5 10   or   5 10 -45")
                    continue
                print("  ->", move(y, z, pitch))
    except KeyboardInterrupt:
        pass
    finally:
        robot.disconnect()


def main():
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyACM0, COM3")
    common.add_argument("--id", default="my_follower", help="Robot id used at calibration (default: my_follower)")
    common.add_argument("--calibration-dir", type=Path, default=None)
    common.add_argument("--home-file", type=Path, default=None, help="Default: <calibration dir>/<id>_home.json")
    common.add_argument("--max-relative-target", type=float, default=None, help="Per-command movement cap in degrees")

    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("sethome", parents=[common], help="Move arm by hand, press Enter to save home").set_defaults(fn=cmd_sethome)
    c = sub.add_parser("check", parents=[common], help="Live forward-kinematics readout (torque off)")
    c.add_argument("--no-home", action="store_true", help="Don't require a home file")
    c.set_defaults(fn=cmd_check)
    r = sub.add_parser("run", parents=[common], help="Send tip targets to the arm")
    r.add_argument("--mode", choices=["interactive", "udp"], default="interactive")
    r.add_argument("--host", default="127.0.0.1")
    r.add_argument("--udp-port", type=int, default=5005)
    r.add_argument("--pitch", type=float, default=None, help="Default gripper pitch in deg above horizontal (default: home pitch)")
    r.add_argument("--auto-pitch", action="store_true", help="If a point is unreachable at the requested pitch, use the closest pitch that works")
    r.add_argument("--clamp", action="store_true", help="If still unreachable, move to the nearest reachable point toward the target")
    r.add_argument("--margin", type=float, default=2.0, help="Degrees kept clear of each calibrated joint limit (default 2)")
    r.set_defaults(fn=cmd_run)
    args = p.parse_args()
    args.fn(args)


if __name__ == "__main__":
    main()