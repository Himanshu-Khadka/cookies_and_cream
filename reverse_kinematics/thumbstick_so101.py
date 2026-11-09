"""WASD thumbstick GUI for the real SO-101 follower.

--check reports calibration and starting-pose readiness without opening the GUI.
--calibrate-follower records new travel limits while you move the arm by hand.
The default GUI is a read-only preview. --move enables physical jogging.
In motor mode, support the arm when closing the GUI because torque releases.
"""

import argparse
import json
import math
import sys
import tempfile
import time
import tkinter as tk
from pathlib import Path
from tkinter import ttk

import numpy as np

from apply_follower_limits import dump_calibration, verify_registers
from record_telemetry import (
    JOINTS, RealArm, bounded_targets, calibrated_positions, encoder_to_units,
    raw_goal_positions, validate_calibration,
)
from thumbstick_simulator import ThumbstickSimulator
from wasd_so101 import MODEL_LIMITS, JointMapping, MotionWatchdog, XYController, forward_kinematics


# Editable speed settings. Change these values, then restart the script.
JOG_SPEED_MM_S = 10.0       # Requested claw travel speed on the horizontal plane.
SIDEWAYS_SPEED_MM_S = 15.0  # Requested speed when jogging with A or D alone.
JOINT_SPEED_DEG_S = 10.0   # Maximum change in an individual joint command per second.

PORT = "/dev/tty.usbmodem5C4C1241631"
FOLLOWER_ID = "my_follower"
REFERENCE = Path(__file__).resolve().with_name("xy_reference_fixed.json")

# Bounds and watchdog tolerances from the project's existing WASD controller.
LOCAL_RADIUS_M = .05
TRACKING_DISTANCE_M = .01
SETTLE_DEGREES = 5.0
HOLD_SETTLE_DEGREES = 5.0  # Allow a small residual after releasing WASD.
TICK_SECONDS = .033
SHOULDER_ENCODER_MARGIN_TICKS = 8


def read_follower_pose(follower):
    start = time.monotonic()
    raw = follower.read("present_position")
    if time.monotonic() - start > .5:
        raise RuntimeError("Position read exceeded 500 ms")
    pose = calibrated_positions(raw, follower.calibration)
    outside = []
    for joint, calibration in follower.calibration.items():
        low = max(0, calibration["range_min"])
        high = min(4095, calibration["range_max"])
        if not low <= raw[joint] <= high:
            outside.append(f"{joint}: {raw[joint]} outside {low}..{high}")
    if outside:
        raise ValueError("Joints outside saved travel limits (encoder ticks):\n  " + "\n  ".join(outside))
    return pose


def make_controller(follower, mapping, pose, radius_m=LOCAL_RADIUS_M, *, controller_class=XYController):
    controller = controller_class(follower.calibration, mapping, pose, JOG_SPEED_MM_S / 1000,
                                  JOINT_SPEED_DEG_S, radius_m, TRACKING_DISTANCE_M,
                                  sideways_speed=SIDEWAYS_SPEED_MM_S / 1000)
    joint = "shoulder_lift"
    calibration = follower.calibration[joint]
    ceiling_ticks = 4095 - SHOULDER_ENCODER_MARGIN_TICKS
    if calibration["range_min"] < ceiling_ticks < calibration["range_max"]:
        controller.goal_ceilings[joint] = encoder_to_units(ceiling_ticks, joint, calibration)
    return controller


def verify_torque(follower):
    state = follower.read("Torque_Enable")
    disabled = [joint for joint in JOINTS if state.get(joint) != 1]
    if disabled:
        raise RuntimeError("Torque not enabled on " + ", ".join(disabled))


def calibrate_follower(follower):
    """Back up and record the follower's travel using the connected bus only."""
    input("Support the arm, then press Enter to release torque and begin calibration. Ctrl+C cancels. ")
    bus = follower.bus
    previous = bus.read_calibration()
    saved_path = Path(follower.calibration_path)
    original_file = saved_path.read_bytes()
    backup_root = Path(__file__).resolve().parent / "calibration_backups"
    backup_root.mkdir(exist_ok=True)
    backup = Path(tempfile.mkdtemp(prefix="follower_recalibration_", dir=backup_root))
    (backup / "original_saved_calibration.json").write_bytes(original_file)
    (backup / "original_motor_calibration.json").write_text(
        json.dumps(dump_calibration(previous), indent=2) + "\n", encoding="utf-8")
    print(f"Calibration backup saved to {backup}", flush=True)

    pending = saved_path.with_name(f"{saved_path.name}.{backup.name}.tmp")
    try:
        # LeRobot's hand calibration disables torque and leaves it off. Avoid
        # device.connect(), which configures the arm and enables torque.
        candidate = bus.record_calibration("position", full_turn_motors=["wrist_roll"])
        records = dump_calibration(candidate)
        for joint, record in records.items():
            validate_calibration(joint, record)
        bus.write_calibration(candidate)
        verify_registers(bus.read_calibration(), candidate)
        with pending.open("x", encoding="utf-8") as stream:
            json.dump(records, stream, indent=2)
            stream.write("\n")
        pending.replace(saved_path)
    except BaseException as exc:
        try:
            bus.disable_torque(num_retry=1)
            bus.write_calibration(previous, cache=False)
            verify_registers(bus.read_calibration(), previous)
            if saved_path.read_bytes() != original_file:
                pending.write_bytes(original_file)
                pending.replace(saved_path)
        except Exception as rollback_error:
            raise RuntimeError(f"Calibration failed ({exc}) and rollback failed ({rollback_error}). "
                               f"Calibration backup: {backup}") from exc
        print("Previous motor calibration restored. Torque is off.", file=sys.stderr)
        raise
    finally:
        pending.unlink(missing_ok=True)
    print(f"Follower calibration saved to {saved_path}. Torque is off.")
    print(f"Capture a fresh IK reference at a known model pose before jogging. Expected file: {REFERENCE}")


class FollowerThumbstick(ThumbstickSimulator):
    speed_label = "Jog speed"
    reset_key = None
    cartesian_label = "WASD"
    make_controller = staticmethod(make_controller)

    def __init__(self, root, follower, controller, pose, move):
        self.follower = follower
        self.controller = controller
        self.move = move
        self.fault = None
        self.joint_hold = None
        self.mode = tk.StringVar(master=root, value="xy")
        jaw = follower.calibration["gripper"]
        self.jaw_deg_per_percent = (jaw["range_max"] - jaw["range_min"]) * 360 / (4095 * 100)
        if jaw["drive_mode"]:
            self.jaw_deg_per_percent *= -1
        self.window_title = "SO-101 Thumbstick · " + ("MOTOR CONTROL" if move else "READ-ONLY PREVIEW")
        self.arm_hint = self.jog_hint(controller)
        self.arm_detail = ("View uses motor encoder readings. Support the arm before exiting." if move else
                           "Preview uses the live starting pose, calibrated limits, and motor-mode speed.")
        super().__init__(root)
        root.geometry("1120x790")
        root.minsize(900, 730)
        self.set_start(controller, pose)

    def jog_hint(self, controller):
        return (f"Travel cap: {controller.radius * 1000:g} mm  ·  "
                f"W/S: {JOG_SPEED_MM_S:g} mm/s  ·  A/D: {SIDEWAYS_SPEED_MM_S:g} mm/s")

    @property
    def jog_input(self):
        return self.held

    @property
    def jog_active(self):
        return any(self.axes)

    def build_motion_controls(self, frame):
        self.speed = tk.DoubleVar(value=JOG_SPEED_MM_S)
        modes = ttk.Frame(frame)
        modes.pack(pady=(8, 4))
        for name, value in ((self.cartesian_label, "xy"), ("Joint control", "joint")):
            ttk.Radiobutton(modes, text=name, value=value, variable=self.mode,
                            command=self.change_mode).pack(side="left", padx=4)
        if not self.move:
            ttk.Button(frame, text="Sync preview to arm", command=self.sync_preview).pack(pady=(2, 4))
        self.joint_controls = ttk.Frame(frame)
        self.hold_degrees = tk.DoubleVar(value=5)
        self.travel_label = ttk.Label(self.joint_controls, text="Travel per hold: 5°")
        self.travel_label.grid(row=0, column=0, columnspan=2)
        ttk.Scale(self.joint_controls, from_=1, to=30, variable=self.hold_degrees,
                  command=lambda value: self.travel_label.configure(text=f"Travel per hold: {round(float(value))}°"),
                  takefocus=False).grid(row=1, column=0, columnspan=2, sticky="ew")
        self.joint_angles = {}
        self.joint_names = {}
        for index, (joint, name) in enumerate(zip(JOINTS, (
            "Base", "Shoulder lift", "Elbow", "Wrist pitch", "Wrist roll", "Gripper motor"
        ))):
            row = 2 + 2 * index
            angle = self.joint_angles[joint] = tk.StringVar()
            ttk.Label(self.joint_controls, textvariable=angle).grid(row=row, column=0, columnspan=2, pady=(3, 0))
            self.joint_names[joint] = name
            for column, (label, direction) in enumerate((("Decrease angle", -1), ("Increase angle", 1))):
                button = ttk.Button(self.joint_controls, text=label, takefocus=False)
                button.grid(row=row+1, column=column, padx=2)
                button.bind("<ButtonPress-1>", lambda event, j=joint, d=direction: self.start_joint(event, j, d))
                button.bind("<ButtonRelease-1>", self.release_joint)
                button.bind("<Leave>", self.release_joint)

    def sync_preview(self):
        self.focus_out(None)
        try:
            pose = read_follower_pose(self.follower)
            controller = self.make_controller(self.follower, self.controller.mapping, pose, self.controller.radius)
        except (ValueError, RuntimeError) as exc:
            self.status.set(str(exc))
            return
        self.set_start(controller, pose)
        self.status.set("Preview synced to the arm's current pose.")

    def change_mode(self):
        self.focus_out(None)
        if self.mode.get() == "joint":
            self.controller.held = self.sent_pose.copy()
            self.canvas.configure(height=80)
            self.joint_controls.pack()
            self.status.set("Hold one button to jog its motor.")
        else:
            # A joint can settle short of its goal. Anchor the new WASD plane
            # to the measured pose, while slewing from the last motor command.
            try:
                pose = read_follower_pose(self.follower) if self.move else self.sent_pose
                controller = self.make_controller(self.follower, self.controller.mapping, pose, self.controller.radius)
            except (ValueError, RuntimeError, OSError) as exc:
                if self.joint_controls.winfo_manager():
                    self.mode.set("joint")
                self.status.set(f"Cannot start {self.cartesian_label}: {exc}")
                return
            previous = self.sent_pose
            self.set_start(controller, pose)
            self.sent_pose = previous
            self.joint_controls.pack_forget()
            self.canvas.configure(height=245)
            self.canvas.focus_set()
            self.status.set(self.ready_status)

    def press(self, event):
        if self.mode.get() == "xy" or event.keysym.lower() == "escape":
            super().press(event)

    def focus_in(self, event):
        if self.mode.get() == "joint":
            self.status.set("Hold one button to jog its motor.")
        else:
            super().focus_in(event)

    def focus_out(self, event):
        self.release_joint(None)
        super().focus_out(event)

    def start_joint(self, event, joint, direction):
        if self.mode.get() != "joint" or self.joint_hold is not None:
            return "break"
        step = direction * round(self.hold_degrees.get())
        low, high = self.controller.joint_limits(joint)
        if joint == "gripper":
            step /= self.jaw_deg_per_percent
        else:
            model = self.controller.mapping
            bounds = sorted(model.from_model(edge, 0)[joint] for edge in MODEL_LIMITS.T)
            low, high = max(low, bounds[0]), min(high, bounds[1])
        target = min(high, max(low, self.controller.held[joint] + step))
        event.widget.state(["pressed"])
        self.joint_hold = (event.widget, joint, target)
        return "break"

    def release_joint(self, event):
        if self.joint_hold is not None:
            button, _, _ = self.joint_hold
            if event is None or event.widget is button:
                button.state(["!pressed"])
                self.joint_hold = None
        return "break"

    def step_joint(self, measured, dt):
        held = self.controller.held
        requested = held.copy()
        self.controller.status = "Holding position"
        if self.joint_hold is not None:
            _, joint, target = self.joint_hold
            requested[joint] = target
            self.controller.status = f"Jogging {self.joint_names[joint]}"
        command = bounded_targets(requested, measured, self.follower.calibration, held, dt,
                                  JOINT_SPEED_DEG_S, JOINT_SPEED_DEG_S / abs(self.jaw_deg_per_percent))
        for joint in JOINTS:
            low, high = sorted((held[joint], requested[joint]))
            if not low - 1e-9 <= command[joint] <= high + 1e-9:
                raise RuntimeError(f"{joint} drifted too far from its held target")
        raw_goal_positions(command, self.follower.calibration)
        self.controller.held = command
        return command

    def motor_angles(self, pose):
        return pose | {"gripper": (pose["gripper"] - 50) * self.jaw_deg_per_percent}

    def set_start(self, controller, pose):
        self.controller = controller
        self.sent_pose = pose.copy()
        self.last_tick = time.monotonic()
        self.next_torque_check = self.last_tick + .5
        self.watchdog = MotionWatchdog(self.motor_angles(pose), self.last_tick, SETTLE_DEGREES, joints=JOINTS)
        self.arm.home = controller.start.copy()
        self.arm.trail = [controller.start.copy()]
        self.show_pose(pose, "Holding position" if self.move else "Read-only preview")

    def show_pose(self, pose, status):
        for joint, degrees in self.motor_angles(pose).items():
            self.joint_angles[joint].set(f"{self.joint_names[joint]}  {degrees:+.1f}°")
        view = self.arm
        view.q = self.controller.mapping.to_model(pose)
        view.tip = forward_kinematics(view.q)
        view.status = status
        if np.linalg.norm(view.tip - view.trail[-1]) >= .002:
            view.trail.append(view.tip.copy())
            view.trail = view.trail[-250:]
        self.update_arm_display()

    def tick(self):
        if self.closed:
            return
        started = time.monotonic()
        try:
            observed = read_follower_pose(self.follower)
            now = time.monotonic()
            measured = observed if self.move else self.controller.held
            dt = now - self.last_tick
            if self.mode.get() == "joint":
                command = self.step_joint(measured, dt)
                active = self.joint_hold is not None
            else:
                command = self.controller.step(measured, self.jog_input, dt,
                                               previous_motor_goal=self.sent_pose if self.move else None)
                active = self.jog_active
            if self.move:
                if now >= self.next_torque_check:
                    verify_torque(self.follower)
                    self.next_torque_check = now + .5
                self.watchdog.update(self.motor_angles(observed), self.motor_angles(command), now,
                                     settle_degrees=None if active else HOLD_SETTLE_DEGREES)
                if time.monotonic() - started > .5:
                    raise RuntimeError("Position-to-command cycle exceeded 500 ms")
                self.follower.send_action(command)
            self.sent_pose = command
            self.last_tick = now
            self.show_pose(observed if self.move else command, self.controller.status)
            self.timer = self.root.after(max(1, round((TICK_SECONDS - (time.monotonic() - started)) * 1000)),
                                         self.tick)
        except (Exception, KeyboardInterrupt) as exc:
            self.fault = exc
            self.close()


def preflight(follower, reference, radius_m=LOCAL_RADIUS_M, *, controller_factory=make_controller):
    problems = []
    pose = mapping = None
    try:
        pose = read_follower_pose(follower)
    except (ValueError, RuntimeError) as exc:
        problems.append(str(exc))
    try:
        mapping = JointMapping.load(reference, follower.calibration)
    except FileNotFoundError:
        problems.append(f"Missing current IK reference: {reference}")
    except ValueError as exc:
        problems.append(f"{reference.name}: {exc}")
    if problems:
        raise ValueError("\n".join(problems))
    try:
        return pose, controller_factory(follower, mapping, pose, radius_m)
    except ValueError as exc:
        raise ValueError(f"{reference.name}: {exc}") from exc


def main(argv=None, *, description=__doc__, window_class=FollowerThumbstick, allow_calibration=True):
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--port", default=PORT)
    parser.add_argument("--id", default=FOLLOWER_ID)
    parser.add_argument("--mapping", type=Path, default=REFERENCE, help="Current joint-to-model reference JSON")
    parser.add_argument("--radius-mm", type=float, default=LOCAL_RADIUS_M * 1000,
                        help="Maximum WASD travel from the starting position in mm (default: %(default)g)")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--check", action="store_true", help="Read calibration and pose, then exit")
    if allow_calibration:
        mode.add_argument("--calibrate-follower", action="store_true", help="Record the follower's travel by hand")
    mode.add_argument("--move", action="store_true", help="Enable physical jogging from the GUI")
    args = parser.parse_args(argv)
    if not math.isfinite(args.radius_mm) or args.radius_mm <= 0:
        parser.error("--radius-mm must be finite and positive")
    for name, value in [("JOG_SPEED_MM_S", JOG_SPEED_MM_S), ("SIDEWAYS_SPEED_MM_S", SIDEWAYS_SPEED_MM_S),
                        ("JOINT_SPEED_DEG_S", JOINT_SPEED_DEG_S)]:
        if not math.isfinite(value) or value <= 0:
            parser.error(f"{name} must be finite and positive")

    follower = root = window = None
    try:
        follower = RealArm("follower", args.port, args.id)
        print(f"Saved calibration matches motor readback for {args.id}.", flush=True)
        if getattr(args, "calibrate_follower", False):
            calibrate_follower(follower)
            return 0
        if args.check:
            state = follower.read("Torque_Enable")
            print(f"Torque enabled on {sum(state.get(joint) == 1 for joint in JOINTS)}/{len(JOINTS)} motors.")
        factory = window_class.make_controller
        pose, controller = preflight(follower, args.mapping, args.radius_mm / 1000, controller_factory=factory)
        print(f"IK reference: {args.mapping.resolve()}", flush=True)
        if args.check:
            x, y, z = controller.start * 1000
            print(f"Reference file and current joint limits passed. Model XYZ (mm): {x:.1f}, {y:.1f}, {z:.1f}")
            if z < 0:
                print("The model places the claw below the base. If the claw is above the base, "
                      "check the physical alignment used to capture the IK reference.")
            return 0
        root = tk.Tk()
        window = window_class(root, follower, controller, pose, args.move)
        if args.move:
            print("Enabling torque at the current pose. Support the arm before exiting.", flush=True)
            follower.prepare_teleoperation()
            verify_torque(follower)
            print("Torque confirmed on all six motors.", flush=True)
            pose = read_follower_pose(follower)
            window.set_start(factory(follower, controller.mapping, pose, controller.radius), pose)
        else:
            print("Read-only preview. Start with --move to enable torque.", flush=True)
        root.mainloop()
        if window.fault is not None:
            raise window.fault
    except KeyboardInterrupt:
        print("Stopped.")
    except Exception as exc:
        print(f"Stopped: {exc}", file=sys.stderr)
        return 1
    finally:
        try:
            if follower is not None:
                follower.close()
        finally:
            if window is not None and not window.closed:
                window.close()
            elif root is not None and window is None:
                root.destroy()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
