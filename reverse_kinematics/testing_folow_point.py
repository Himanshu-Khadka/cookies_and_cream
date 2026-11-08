"""
Make the SO-101 gripper follow a Cartesian target point, continuously.

Input: UDP text messages to 127.0.0.1:<port>, format "x y z" in METERS,
in the URDF base frame (use --relative to treat them as offsets from the
gripper's starting position instead).

Based on src/lerobot/model/kinematics.py (lerobot 0.7.1): degrees in/out.

Default is a DRY RUN with a simulated arm. Add --execute for the real arm.

Examples:
  # no hardware, gripper follows a small demo circle
  python so101_follow_point.py --urdf so101_new_calib.urdf --demo

  # real arm, listening for points from your controller code
  python so101_follow_point.py --urdf so101_new_calib.urdf --execute --relative
"""

import argparse
import socket
import time

import numpy as np

from lerobot.model.kinematics import RobotKinematics

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
GRIPPER = "gripper"
NAMES = ARM_JOINTS + [GRIPPER]

MAX_JOINT_STEP_DEG = 3.0   # max joint change per control cycle
MAX_POS_ERROR_M = 0.01     # if IK misses by more than 1 cm, skip this cycle
POINT_TIMEOUT_S = 0.5      # no new point for this long -> hold position
MAX_LAG_DEG = 10.0         # commanded joints may lead measured joints by at most this


class FakeRobot:
    """Stand-in so you can test without hardware."""

    def __init__(self):
        self.q = dict(zip(NAMES, [0.0, -30.0, 30.0, 60.0, 0.0, 0.0]))

    def connect(self): pass
    def disconnect(self): pass
    def get_observation(self): return {f"{n}.pos": v for n, v in self.q.items()}
    def send_action(self, a): self.q.update({k[:-4]: v for k, v in a.items()})


class UdpPoints:
    def __init__(self, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", port))
        self.sock.setblocking(False)

    def latest(self):
        """Return the newest valid point received since last call, or None."""
        point = None
        while True:
            try:
                data, _ = self.sock.recvfrom(1024)
            except BlockingIOError:
                return point
            try:
                vals = [float(v) for v in data.decode().replace(",", " ").split()]
                if len(vals) == 3:
                    point = np.array(vals)
            except ValueError:
                pass


def make_robot(args):
    if not args.execute:
        return FakeRobot()
    # >>> VERIFY these names against src/lerobot/robots/so_follower/ <<<
    from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
    return SO101Follower(SO101FollowerConfig(port=args.port, id=args.id, use_degrees=True))


def read_q(robot):
    obs = robot.get_observation()
    return np.array([obs[f"{n}.pos"] for n in NAMES], dtype=float)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--urdf", required=True)
    p.add_argument("--port", default="/dev/ttyACM0", help="serial port of the arm")
    p.add_argument("--id", default="my_follower", help="calibration id")
    p.add_argument("--udp-port", type=int, default=5005)
    p.add_argument("--hz", type=float, default=30.0)
    p.add_argument("--vmax", type=float, default=0.08, help="max gripper speed, m/s")
    p.add_argument("--reach", type=float, default=0.10, help="stay within +/- this many m of start, per axis")
    p.add_argument("--relative", action="store_true", help="points are offsets from the start position")
    p.add_argument("--demo", action="store_true", help="ignore UDP, trace a small circle")
    p.add_argument("--execute", action="store_true", help="move the real arm")
    args = p.parse_args()

    kin = RobotKinematics(args.urdf, "gripper_frame_link", ARM_JOINTS)
    robot = make_robot(args)
    source = None if args.demo else UdpPoints(args.udp_port)
    dt = 1.0 / args.hz

    robot.connect()
    try:
        q = read_q(robot)
        T0 = kin.forward_kinematics(q)
        start = T0[:3, 3].copy()
        R_hold = T0[:3, :3].copy()          # keep the starting gripper orientation
        lo, hi = start - args.reach, start + args.reach
        q_cmd = q.copy()                    # last commanded joints (integrated, not re-read)
        p_cmd = start.copy()                # smoothed target we are chasing
        goal = start.copy()
        last_msg = time.time()
        print("start xyz (m):", np.round(start, 4))
        print("limits lo:", np.round(lo, 3), " hi:", np.round(hi, 3))
        if args.execute:
            input("Press Enter to start following (Ctrl+C to stop)...")

        t_start = time.time()
        n = 0
        while True:
            t0 = time.time()
            q = read_q(robot)

            # 1) get the point we should chase
            if args.demo:
                t = t0 - t_start
                goal = start + 0.03 * np.array([0.0, np.cos(t), np.sin(t)])
                last_msg = t0
            else:
                msg = source.latest()
                if msg is not None:
                    goal = start + msg if args.relative else msg
                    last_msg = t0
            goal = np.clip(goal, lo, hi)
            if t0 - last_msg > POINT_TIMEOUT_S:
                goal = p_cmd                 # no fresh input: hold still

            # 2) speed limit: move p_cmd toward goal, no faster than vmax
            d = goal - p_cmd
            dist = np.linalg.norm(d)
            step = args.vmax * dt
            if dist > step:
                d *= step / dist
            p_cmd = p_cmd + d

            # 3) inverse kinematics for that point
            T = np.eye(4)
            T[:3, :3] = R_hold
            T[:3, 3] = p_cmd
            # keep the commanded joints from running far ahead of the real arm
            q_cmd[:5] = np.clip(q_cmd[:5], q[:5] - MAX_LAG_DEG, q[:5] + MAX_LAG_DEG)
            q_cmd[5] = q[5]                  # gripper: leave where it is

            q_new = kin.inverse_kinematics(q_cmd, T)   # start from last COMMAND
            err = np.linalg.norm(kin.forward_kinematics(q_new)[:3, 3] - p_cmd)

            if err > MAX_POS_ERROR_M:
                p_cmd = kin.forward_kinematics(q_cmd)[:3, 3].copy()   # unreachable: don't wind up
                print(f"skip: IK error {err * 1000:.1f} mm (target out of reach?)")
            else:
                # 4) joint speed limit, integrate into q_cmd, then send
                dq = np.clip(q_new[:5] - q_cmd[:5], -MAX_JOINT_STEP_DEG, MAX_JOINT_STEP_DEG)
                q_cmd[:5] += dq
                robot.send_action({f"{nm}.pos": float(v) for nm, v in zip(NAMES, q_cmd)})

            n += 1
            if n % int(args.hz) == 0:
                print(f"target {np.round(p_cmd, 3)}  actual {np.round(kin.forward_kinematics(q)[:3, 3], 3)}")
                print(f"   cmd joints {np.round(q_cmd[:5], 1)}  meas joints {np.round(q[:5], 1)}")
            time.sleep(max(0.0, dt - (time.time() - t0)))
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
