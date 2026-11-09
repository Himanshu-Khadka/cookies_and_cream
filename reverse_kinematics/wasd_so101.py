#!/usr/bin/env python3
"""WASD Cartesian jogging: W/S = +/-X, A/D = +/-Y, fixed starting Z.

Default: read-only hardware preview. --move explicitly enables motor commands.
--simulate never opens a serial port. Keyboard events belong to a focused Tk
window, so no global keyboard/Accessibility permission is needed.

Uses NumPy damped-least-squares position IK. Orientation may change; wrist roll
and gripper opening retain their starting targets. No automatic homing.
See README.md for the required hardware-to-model reference mapping.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

from record_telemetry import (
    JOINTS, RealArm, SimulatedArm, bounded_targets, calibrated_positions,
    command_limits, raw_goal_positions,
)
from rotate_so101 import read_pose

ARM_JOINTS = JOINTS[:-1]
# Joint origins, RPY, and limits extracted from TheRobotStudio's official model:
# https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/so101_new_calib.urdf
# URDF convention: T_parent_child = T_origin * R_axis(q); distances are metres.
# These numerical kinematic parameters need no mesh assets or network access.
MODEL_ID = "so101_new_calib:2026-11-07"
GEOMETRY = (
    ((.0388353, -8.97657e-9, .0624), (3.14159, 4.18253e-17, -3.14159), (-1.91986, 1.91986)),
    ((-.0303992, -.0182778, -.0542), (-1.5708, -1.5708, 0), (-1.91986, 1.91986)),
    ((-.11257, -.028, 1.73763e-16), (-3.63608e-16, 8.74301e-16, 1.5708), (-1.69, 1.69)),
    ((-.1349, .0052, 3.62355e-17), (4.02456e-15, 8.67362e-16, -1.5708), (-1.65806, 1.65806)),
    ((5.55112e-17, -.0611, .0181), (1.5708, .0486795, 3.14159), (-2.74385, 2.84121)),
)
TIP_XYZ = (-.0079, -.000218121, -.0981274)


def transform(xyz, rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    result = np.eye(4)
    result[:3, :3] = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])
    result[:3, 3] = xyz
    return result


ORIGINS = tuple(transform(xyz, rpy) for xyz, rpy, _ in GEOMETRY)
MODEL_LIMITS = np.array([limits for _, _, limits in GEOMETRY])


def forward_kinematics(q, with_jacobian=False):
    """Five model joint angles in radians -> gripper_frame_link position, metres."""
    q = np.asarray(q, dtype=float)
    if q.shape != (5,) or not np.isfinite(q).all():
        raise ValueError("Need five finite model joint angles")
    pose = np.eye(4)
    origins, axes = [], []
    for origin, angle in zip(ORIGINS, q):
        pose = pose @ origin
        origins.append(pose[:3, 3].copy())
        axes.append(pose[:3, 2].copy())
        pose = pose @ transform((0, 0, 0), (0, 0, angle))
    tip = (pose @ np.array([*TIP_XYZ, 1.0]))[:3]
    if with_jacobian:
        return tip, np.column_stack([np.cross(axis, tip - pos) for axis, pos in zip(axes, origins)])
    return tip


def inverse_kinematics(seed, target, lower, upper, *, tolerance=.00005):
    """Bounded local position IK with a default residual tolerance of 0.05 mm.

    Joint 5 (wrist roll) is fixed by the caller. Small trust steps and a line
    search prevent large branch changes. No clipping an invalid Cartesian goal
    into a different, silently accepted pose.
    """
    target = np.asarray(target, dtype=float)
    if target.shape != (3,) or not np.isfinite(target).all():
        raise ValueError("Need a finite XYZ target")
    q = np.clip(seed, lower, upper).astype(float)
    for _ in range(60):
        pos, jacobian = forward_kinematics(q, True)
        error = target - pos
        if np.linalg.norm(error) <= tolerance:
            return q
        jacobian = jacobian[:, :4]  # Keep wrist roll unchanged.
        # Exclude active bounds pointing away from the feasible region.
        gradient = jacobian.T @ error
        free = ~(((q[:4] <= lower[:4] + 1e-10) & (gradient < 0)) |
                 ((q[:4] >= upper[:4] - 1e-10) & (gradient > 0)))
        jacobian = jacobian * free
        delta = jacobian.T @ np.linalg.solve(
            jacobian @ jacobian.T + 0.002 ** 2 * np.eye(3), error)
        delta *= min(1.0, 0.08 / max(np.max(np.abs(delta)), 1e-12))
        improved = False
        for scale in (1.0, .5, .25, .125):
            trial = q.copy()
            trial[:4] = np.clip(q[:4] + scale * delta, lower[:4], upper[:4])
            if np.linalg.norm(target - forward_kinematics(trial)) < np.linalg.norm(error) - 1e-12:
                q, improved = trial, True
                break
        if not improved:
            break
    return None


class JointMapping:
    """model_deg = signs * controller_deg + offsets_deg (one per arm joint)."""

    def __init__(self, signs=None, offsets=None):
        self.signs = np.array(signs if signs is not None else [1] * 5, dtype=float)
        self.offsets = np.array(offsets if offsets is not None else [0] * 5, dtype=float)
        if (self.signs.shape != (5,) or self.offsets.shape != (5,)
                or not np.isin(self.signs, [-1, 1]).all() or not np.isfinite(self.offsets).all()):
            raise ValueError("Mapping needs five signs (+1/-1) and five finite offsets")

    def to_model(self, pose):
        return np.deg2rad(self.signs * np.array([pose[j] for j in ARM_JOINTS]) + self.offsets)

    def from_model(self, q, gripper):
        values = (np.rad2deg(q) - self.offsets) / self.signs
        return dict(zip(JOINTS, [*values, gripper]))

    def bounds(self, lower, upper):
        a = np.deg2rad(self.signs * np.array(lower) + self.offsets)
        b = np.deg2rad(self.signs * np.array(upper) + self.offsets)
        return np.minimum(a, b), np.maximum(a, b)

    @classmethod
    def load(cls, path, calibration):
        if path is None:
            raise ValueError("Hardware needs --mapping FILE; see --capture-reference and README.md")
        data = json.loads(path.read_text(encoding="utf-8"))
        if (data.get("model") != MODEL_ID or data.get("joint_order") != list(ARM_JOINTS)
                or data.get("calibration") != calibration):
            raise ValueError("Reference mapping does not match this model/calibration; capture a new reference")
        return cls(data["signs"], data["offsets_deg"])


def capture_reference(arm, args):
    """Read a manually positioned, KNOWN model pose; never write motor registers."""
    reference = np.deg2rad(args.reference_deg)
    if np.any(reference < MODEL_LIMITS[:, 0]) or np.any(reference > MODEL_LIMITS[:, 1]):
        raise ValueError("Reference angles must be within the model limits")
    measured = read_pose(arm)
    try:
        raw_goal_positions(measured, arm.calibration)
    except ValueError as exc:
        raise ValueError(f"Reference pose must be within the saved travel limits: {exc}") from exc
    signs = np.array(args.signs)
    offsets = np.array(args.reference_deg) - signs * np.array([measured[j] for j in ARM_JOINTS])
    data = dict(model=MODEL_ID, joint_order=list(ARM_JOINTS), signs=signs.tolist(),
                offsets_deg=offsets.tolist(), reference_deg=args.reference_deg,
                measured_reference=measured, calibration=arm.calibration)
    # Exclusive create: never overwrite a previous reference accidentally.
    with args.capture_reference.open("x", encoding="utf-8") as stream:
        stream.write(json.dumps(data, indent=2) + "\n")
    print(f"Reference saved: {args.capture_reference.resolve()}\n"
          "No motor commands/settings were written. The physical reference and signs must be correct.")


def key_velocity(keys, speed):
    velocity = np.array([int("w" in keys) - int("s" in keys),
                         int("a" in keys) - int("d" in keys), 0.0])
    return velocity * speed / max(1.0, np.linalg.norm(velocity))


class XYController:
    def __init__(self, calibration, mapping, pose, speed=0.01, joint_speed=20, radius=0.05,
                 tracking_distance=.005, sideways_speed=None):
        self.calibration, self.mapping = calibration, mapping
        self.speed, self.joint_speed, self.radius = speed, joint_speed, radius
        self.tracking_distance = tracking_distance
        self.sideways_speed = speed if sideways_speed is None else sideways_speed
        self.held = pose.copy()
        self.start = self.target = forward_kinematics(mapping.to_model(pose))
        self.start = self.start.copy()
        self.z = self.start[2]
        self.limits = {j: command_limits(j, calibration[j]) for j in JOINTS}
        self.goal_ceilings = {}
        q = mapping.to_model(pose)
        raw_goal_positions(pose, calibration)
        outside = []
        for joint, angle, (lower, upper) in zip(ARM_JOINTS, q, MODEL_LIMITS):
            if not lower <= angle <= upper:
                outside.append(f"{joint} {math.degrees(angle):.1f}° outside "
                               f"{math.degrees(lower):.1f}° to {math.degrees(upper):.1f}°")
        if outside:
            raise ValueError("Starting pose is outside model limits: " + "; ".join(outside))
        self.status = "Hold"

    def joint_limits(self, joint):
        lower, upper = self.limits[joint]
        if joint in self.goal_ceilings:
            # A start already in the margin can hold or move inward. The
            # ceiling tightens as the commanded position leaves the margin.
            upper = min(upper, max(self.goal_ceilings[joint], self.held[joint]))
        return lower, upper

    def pace_for_height(self, measured, planned, previous, dt, *, catching_up=False, sideways=False):
        """Slow joint commands that would worsen the measured height error."""
        measured_q = self.mapping.to_model(measured)
        planned_q = self.mapping.to_model(planned)
        tip, jacobian = forward_kinematics(measured_q, with_jacobian=True)
        height_error = self.z - tip[2]
        # Leave the first millimeter alone, then reduce the adverse command
        # smoothly to zero at 3 mm. Joints correcting the error can catch up.
        scale = float(np.clip((.003 - abs(height_error)) / .002, 0., 1.))
        desired = planned.copy()
        slowed = []
        assisted = None
        contributions = jacobian * (planned_q - measured_q)
        if scale < 1.:
            for joint, dz in zip(ARM_JOINTS[:-1], contributions[2]):
                # The base rotates nearly horizontally. Don't slow it for
                # a sub-tenth-millimeter predicted contribution to height.
                if height_error * dz < 0 and abs(dz) >= .0001:
                    desired[joint] = measured[joint] + scale * (planned[joint] - measured[joint])
                    slowed.append(joint)
        side_error = self.target[1] - tip[1]
        if catching_up and (abs(height_error) >= .002 or (sideways and abs(side_error) >= .003)):
            # Correct height first. With the claw near its plane, sideways
            # jogging can assist a joint that is keeping Y behind the target.
            scores = (height_error * contributions[2] if abs(height_error) >= .002
                      else side_error * contributions[1])
            for index in np.argsort(scores[:4])[::-1]:
                joint = ARM_JOINTS[index]
                lag = planned[joint] - measured[joint]
                if scores[index] <= 0 or joint in slowed or not 1. <= abs(lag) < 5.:
                    continue
                lower, upper = self.joint_limits(joint)
                model_bounds = sorted(self.mapping.from_model(edge, 0)[joint] for edge in MODEL_LIMITS.T)
                lower, upper = max(lower, model_bounds[0]), min(upper, model_bounds[1])
                extra = math.copysign(min(1.5, 5. - abs(lag)), lag)
                goal = min(upper, max(lower, planned[joint] + extra))
                if (goal - planned[joint]) * lag > 1e-9:
                    desired[joint] = goal
                    assisted = joint
                    break
        for joint in self.goal_ceilings:
            desired[joint] = min(desired[joint], self.joint_limits(joint)[1])
        sent = bounded_targets(desired, measured, self.calibration, previous, dt, self.joint_speed, 30)
        q = self.mapping.to_model(sent)
        if np.any(q < MODEL_LIMITS[:, 0] - 1e-9) or np.any(q > MODEL_LIMITS[:, 1] + 1e-9):
            raise RuntimeError("Height pacing would exceed a model joint limit")
        raw_goal_positions(sent, self.calibration)
        return sent, slowed, assisted

    def near_joint_limit(self, seed):
        bounds = [self.joint_limits(joint) for joint in ARM_JOINTS]
        lower, upper = self.mapping.bounds(*zip(*bounds))
        lower = np.maximum(lower, MODEL_LIMITS[:, 0])
        upper = np.minimum(upper, MODEL_LIMITS[:, 1])
        # Wrist roll is held fixed by the solver and cannot provide more travel.
        clearance = np.minimum(seed - lower, upper - seed)[:4]
        index = int(np.argmin(clearance))
        if clearance[index] <= math.radians(1):
            return ("Base", "Shoulder lift", "Elbow", "Wrist pitch")[index] + " near travel limit"
        return ""

    def jog(self, measured, delta, lower, upper):
        seed = self.mapping.to_model(self.held)
        actual = forward_kinematics(self.mapping.to_model(measured))
        lo, hi = self.mapping.bounds(lower, upper)
        lo = np.maximum(lo, MODEL_LIMITS[:, 0])
        hi = np.minimum(hi, MODEL_LIMITS[:, 1])
        lo[4] = hi[4] = seed[4]
        step_length = np.linalg.norm(delta)
        first_block = None
        for scale in (1., .5, .25, .125):
            candidate = self.target + scale * delta
            candidate[2] = self.z
            if np.linalg.norm(candidate[:2] - self.start[:2]) > self.radius:
                self.status = "Local workspace boundary"
            elif np.linalg.norm(candidate - actual) > self.tracking_distance:
                self.status = "Waiting for motors to catch up"
            else:
                solution = None if np.any(lo > hi) else inverse_kinematics(
                    seed, candidate, lo, hi, tolerance=min(.00005, step_length * scale / 4))
                if solution is not None:
                    self.target = candidate
                    self.status = "Jogging" if scale == 1 else f"Jogging at {scale:.0%}: {first_block}"
                    return self.mapping.from_model(solution, self.held["gripper"])
                near_limit = self.near_joint_limit(seed)
                self.status = "Rejected: " + (near_limit or "no feasible step within joint limits")
            if first_block is None:
                first_block = self.status.removeprefix("Rejected: ")
        return self.held

    def jog_bounds(self, lead_pose, dt):
        step = self.joint_speed * min(dt, .1)
        bounds = [self.joint_limits(joint) for joint in ARM_JOINTS]
        lower = [max(lo, self.held[j] - step, lead_pose[j] - 5) for j, (lo, _) in zip(ARM_JOINTS, bounds)]
        upper = [min(hi, self.held[j] + step, lead_pose[j] + 5) for j, (_, hi) in zip(ARM_JOINTS, bounds)]
        if np.any(np.array(lower) > upper):
            raise RuntimeError("Follower moved too far from its previous command; stopping")
        return lower, upper

    def horizontal_velocity(self, keys):
        velocity = key_velocity(keys, self.speed)
        if velocity[0] == 0 and velocity[1] != 0:
            velocity[1] = math.copysign(self.sideways_speed, velocity[1])
        return velocity

    def step(self, measured, keys, dt, *, previous_motor_goal=None):
        if not 0 < dt <= .5:
            raise RuntimeError("Control loop stalled for over 500 ms; stopping")
        # In paced mode the IK target is a waypoint, which can differ from the
        # actual motor command. Bound waypoint lead in XYZ in jog(); enforce
        # the per-joint measured lead when sending the paced motor command.
        lead_pose = measured if previous_motor_goal is None else self.held
        lower, upper = self.jog_bounds(lead_pose, dt)
        velocity = self.horizontal_velocity(keys)
        requested = self.held.copy()
        self.status = "Hold"
        if np.any(velocity):
            requested = self.jog(measured, velocity * min(dt, .1), lower, upper)
        else:
            # Discard solver residual on release; no accumulated target to chase.
            self.target = forward_kinematics(self.mapping.to_model(self.held))
        sent = bounded_targets(requested, lead_pose, self.calibration, self.held,
                               dt, self.joint_speed, 30)
        if any(abs(sent[j] - requested[j]) > 1e-8 for j in JOINTS):
            raise RuntimeError("Command would require clipping after IK; stopping")
        raw_goal_positions(sent, self.calibration)
        self.held = sent
        if previous_motor_goal is not None:
            sideways = abs(velocity[1]) > 0 and abs(velocity[1]) >= abs(velocity[0])
            waiting = self.status == "Waiting for motors to catch up"
            command, slowed, assisted = self.pace_for_height(
                measured, sent, previous_motor_goal, dt,
                catching_up=waiting or (sideways and self.status.endswith(": Waiting for motors to catch up")),
                sideways=sideways,
            )
            if assisted:
                self.status = f"Helping {assisted.replace('_', ' ')} catch up"
            elif slowed and self.status != "Local workspace boundary":
                names = ", ".join(j.replace("_", " ") for j in slowed)
                self.status = f"Pacing {names} to hold height"
            return command
        return sent


class MotionWatchdog:
    def __init__(self, pose, now, settle_degrees=.5, joints=ARM_JOINTS):
        self.anchor, self.progress_time = pose.copy(), now
        self.settle_degrees = settle_degrees
        self.joints = joints

    def update(self, measured, commanded, now, *, settle_degrees=None):
        tolerance = self.settle_degrees if settle_degrees is None else settle_degrees
        progress = max(abs(measured[j] - self.anchor[j]) for j in self.joints)
        joint = max(self.joints, key=lambda j: abs(measured[j] - commanded[j]))
        error = abs(measured[joint] - commanded[joint])
        if progress >= .2 or error <= tolerance:
            self.anchor, self.progress_time = measured.copy(), now
        elif now - self.progress_time > 2:
            raise RuntimeError(f"No measured joint progress for 2 seconds; {joint} is {error:.2f}° from its goal "
                               f"(measured {measured[joint]:.2f}°, goal {commanded[joint]:.2f}°, "
                               f"settle tolerance {tolerance:g}°)")


class KeyboardWindow:
    def __init__(self):
        import tkinter as tk
        self.root = tk.Tk()
        self.root.title("SO-101 | WASD XY control")
        self.root.geometry("640x370")
        self.keys, self.releases = set(), {}
        self.closed, self.paused = False, False
        tk.Label(self.root, text="W / S: +X / −X     A / D: +Y / −Y", font=("Arial", 17)).pack(pady=16)
        tk.Label(self.root, text="Keep this window focused. Release keys to hold.\n"
                 "Space: pause/resume     Esc / Q: exit\n"
                 "Wrist roll and gripper opening stay fixed; wrist pitch can change.").pack()
        self.text = tk.StringVar(value="Starting…")
        tk.Label(self.root, textvariable=self.text, justify="left", font=("Courier", 12)).pack(pady=20)
        self.root.bind("<KeyPress>", self.press)
        self.root.bind("<KeyRelease>", self.release)
        self.root.bind("<FocusOut>", lambda event: self.clear())
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def clear(self):
        for pending in self.releases.values():
            self.root.after_cancel(pending)
        self.releases.clear()
        self.keys.clear()

    def press(self, event):
        key = event.keysym.lower()
        if key in self.releases:
            self.root.after_cancel(self.releases.pop(key))
        if key in self.keys:
            return
        self.keys.add(key)
        if key in ("escape", "q"):
            self.close()
        elif key == "space":
            self.paused = not self.paused
            self.keys.difference_update("wasd")

    def release(self, event):
        key = event.keysym.lower()
        # X11 can emit a synthetic release/press pair for key repeat. Defer
        # removal until idle so the paired press cancels it without a false stop.
        if key in self.releases:
            self.root.after_cancel(self.releases[key])
        def finish():
            self.releases.pop(key, None)
            self.keys.discard(key)
        self.releases[key] = self.root.after_idle(finish)

    def close(self):
        self.clear()
        self.closed = True

    def poll(self):
        self.root.update()
        return set() if self.paused or self.closed else self.keys.copy()

    def destroy(self):
        self.root.destroy()


def run(args):
    arm = window = None
    try:
        if args.simulate:
            arm = SimulatedArm("follower")
            pose = dict(zip(JOINTS, [0, -25, 30, 0, 0, 50]))
            arm.position = raw_goal_positions(pose, arm.calibration)
            arm.prepare_teleoperation()
            mapping = JointMapping()
        else:
            arm = RealArm("follower", args.port, args.id)
            if args.capture_reference:
                capture_reference(arm, args)
                return
            mapping = JointMapping.load(args.mapping, arm.calibration)
        pose = read_pose(arm)
        controller = XYController(arm.calibration, mapping, pose, args.speed_mm_s / 1000,
                                  args.joint_speed, args.radius_mm / 1000, args.tracking_mm / 1000)
        if not args.demo:
            window = KeyboardWindow()  # Validate GUI availability before enabling torque.
        if args.move:
            arm.prepare_teleoperation()
            # Startup can release torque briefly; re-anchor to the fresh pose.
            pose = read_pose(arm)
            controller = XYController(arm.calibration, mapping, pose, args.speed_mm_s / 1000,
                                      args.joint_speed, args.radius_mm / 1000, args.tracking_mm / 1000)
        mode = "SIMULATION" if args.simulate else "MOTOR CONTROL" if args.move else "READ-ONLY PREVIEW"
        print(f"{mode} | W/S +/-X, A/D +/-Y (robot base frame) | starting Z held\n"
              "Focus the control window. Space pauses; Q/Esc exits.\n"
              + ("Exit/fault releases torque: support the arm.\n" if args.move else ""), flush=True)
        origin = time.monotonic()
        previous = origin - 1 / args.hz
        watchdog = MotionWatchdog(pose, origin, args.settle_deg)
        next_print = origin
        while True:
            tick = time.monotonic()
            if args.duration and tick - origin >= args.duration:
                break
            if window:
                keys = window.poll()
                if window.closed:
                    break
            else:
                # Offline smoke demonstration; all four directions, then hold.
                keys = set(("w", "a", "s", "d", "")[min(4, int((tick - origin) / args.duration * 5))])
            observed = read_pose(arm)
            # Preview uses a virtual follower for planning; hardware stays read-only.
            measured = observed if args.move or args.simulate else controller.held.copy()
            now = time.monotonic()
            dt = now - previous
            command = controller.step(measured, keys, dt)
            if args.move:
                watchdog.update(observed, command, now)
            if time.monotonic() - now > .5 or time.monotonic() - tick > .5:
                raise RuntimeError("Position-to-command cycle exceeded 500 ms; stopping")
            if args.move or args.simulate:
                arm.send_action(command)
            previous = now
            predicted = forward_kinematics(mapping.to_model(command)) * 1000
            actual = forward_kinematics(mapping.to_model(observed)) * 1000
            display = (f"{mode}\nPredicted XYZ mm: {predicted[0]:7.2f} {predicted[1]:7.2f} {predicted[2]:7.2f}\n"
                       f"Measured  XYZ mm: {actual[0]:7.2f} {actual[1]:7.2f} {actual[2]:7.2f}\n"
                       f"Speed: {args.speed_mm_s:g} mm/s | radius: {args.radius_mm:g} mm\n"
                       f"{'Paused' if window and window.paused else controller.status}")
            if window:
                window.text.set(display)
            if now >= next_print:
                print(display.replace("\n", " | "), flush=True)
                next_print = now + .5
            time.sleep(max(0, 1 / args.hz - (time.monotonic() - tick)))
    finally:
        # Release motor resources even when GUI destruction fails.
        try:
            if arm is not None:
                arm.close()
        finally:
            if window is not None:
                window.destroy()


def five_numbers(text):
    try:
        values = [float(value) for value in text.split(",")]
        if len(values) != 5 or not all(math.isfinite(value) for value in values):
            raise ValueError
        return values
    except ValueError:
        raise argparse.ArgumentTypeError("Need five finite comma-separated numbers") from None


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--simulate", action="store_true", help="Virtual follower; no serial access")
    parser.add_argument("--move", action="store_true", help="Enable actual follower motor commands")
    parser.add_argument("--port", default="/dev/tty.usbmodem5C4C1241631")
    parser.add_argument("--id", default="my_follower")
    parser.add_argument("--mapping", type=Path, help="Reference mapping JSON required for hardware")
    parser.add_argument("--capture-reference", type=Path, help="Read current known model pose into a NEW mapping JSON")
    parser.add_argument("--reference-deg", type=five_numbers, help="Known model pose: pan,lift,elbow,wrist_flex,wrist_roll")
    parser.add_argument("--signs", type=five_numbers, default=[1] * 5, help="Model-positive vs motor-positive: five +/-1 values")
    parser.add_argument("--speed-mm-s", type=float, default=10)
    parser.add_argument("--joint-speed", type=float, default=20, help="Joint target slew, degrees/s")
    parser.add_argument("--radius-mm", type=float, default=50, help="Local XY workspace radius about starting tip")
    parser.add_argument("--tracking-mm", type=float, default=5,
                        help="Maximum model XYZ target lead over measured tip, mm")
    parser.add_argument("--settle-deg", type=float, default=.5,
                        help="Joint tracking error treated as settled by the no-progress watchdog")
    parser.add_argument("--hz", type=float, default=30)
    parser.add_argument("--duration", type=float, default=0, help="Auto-exit after seconds; default unlimited")
    parser.add_argument("--demo", action="store_true", help="Headless, finite simulation of WASD (requires --simulate)")
    args = parser.parse_args(argv)
    for name in ("speed_mm_s", "joint_speed", "radius_mm", "tracking_mm", "settle_deg", "hz", "duration"):
        value = getattr(args, name)
        if not math.isfinite(value) or value < 0 or (name != "duration" and value == 0):
            parser.error(f"--{name.replace('_', '-')} must be finite and {'nonnegative' if name == 'duration' else 'positive'}")
    if not 5 <= args.hz <= 100:
        parser.error("--hz must be between 5 and 100")
    if args.settle_deg >= 5:
        parser.error("--settle-deg must be below the fixed 5-degree joint command lead")
    if args.simulate and args.move:
        parser.error("Use --simulate or --move, not both")
    if args.demo and (not args.simulate or args.duration <= 0):
        parser.error("--demo requires --simulate and a positive --duration")
    if args.capture_reference and (args.move or args.simulate or args.reference_deg is None):
        parser.error("--capture-reference requires --reference-deg and cannot use --move/--simulate")
    if args.reference_deg is not None and not args.capture_reference:
        parser.error("--reference-deg is only used with --capture-reference")
    if not all(value in (-1, 1) for value in args.signs):
        parser.error("--signs values must be +1 or -1")
    try:
        run(args)
    except KeyboardInterrupt:
        print("\nStopped.")
    except FileExistsError as exc:
        print(f"Stopped: Reference already exists at {exc.filename}. Choose a new filename to recapture.",
              file=sys.stderr)
        return 1
    except Exception as exc:
        print(f"Stopped: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
