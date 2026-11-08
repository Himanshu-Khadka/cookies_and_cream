#!/usr/bin/env python3
"""SO-101 CSV logger with optional leader-to-follower teleoperation.

Default mode only reads. --teleoperate explicitly enables motor commands.
Run with Python 3.12 in the lerobot environment. --simulate never opens hardware.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import threading
import time
from dataclasses import asdict
from datetime import datetime, timezone
from importlib.metadata import version
from pathlib import Path

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
DIAGNOSTICS = (
    "present_speed", "present_load", "present_voltage", "present_temperature",
    "present_current", "status", "moving", "torque_enable", "goal_position",
    "goal_speed", "acceleration", "torque_limit",
)
POSITION_FIELDS = ["sample", "arm", "read_start_monotonic_ns", "read_end_monotonic_ns",
                   "host_unix_ns", "elapsed_s", "read_error"] + [
    f"{j}_{metric}" for j in JOINTS for metric in
    ("position_raw", "position_deg", "velocity_est_deg_s", "acceleration_est_deg_s2",
     "outside_calibrated_range")
] + ["gripper_open_pct"]
DIAGNOSTIC_FIELDS = ["sample", "arm", "register", "read_start_monotonic_ns",
                     "read_end_monotonic_ns", "read_error"] + [f"{j}_raw" for j in JOINTS]
CONTROLLER_FIELDS = ["host_receive_unix_ns", "host_receive_monotonic_ns", "seq", "device_time_us",
                     "joy1_x", "joy1_y", "joy2_x", "joy2_y", "yaw", "gripper", "enable",
                     "encoder_angles_deg_json", "payload_json", "read_error"]
ACTION_FIELDS = ["sample", "read_to_command_duration_ns", "command_start_monotonic_ns",
                 "command_end_monotonic_ns", "command_error"] + [
    f"{j}_{kind}_{'pct' if j == 'gripper' else 'deg'}"
    for j in JOINTS for kind in ("requested", "sent")
]


def validate_calibration(joint, cal):
    """Calibration endpoints are stored in unsigned 16-bit limit registers.

    Some existing LeRobot calibrations extend past one encoder revolution.
    Retain their coordinate origin but never use that as permission to send an
    out-of-turn target. No calibration or motor limits are rewritten here.
    """
    lower, upper = cal["range_min"], cal["range_max"]
    if (not isinstance(lower, int) or not isinstance(upper, int)
            or not 0 <= lower < upper <= 65535 or upper - lower > 4095
            or max(0, lower) >= min(4095, upper)):
        raise ValueError(f"Invalid calibration for {joint}: limits {lower!r}..{upper!r}; "
                         "need an ordered single-turn span intersecting 0..4095")
    return lower, upper


def encoder_to_units(value, joint, cal):
    lower, upper = validate_calibration(joint, cal)
    if joint == "gripper":
        opening = (min(upper, max(lower, value)) - lower) * 100 / (upper - lower)
        return 100 - opening if cal.get("drive_mode") else opening
    return (value - (lower + upper) / 2) * 360 / 4095


def command_limits(joint, cal):
    """Intersection of saved travel and the conservative 0..4095 goal window."""
    lower, upper = validate_calibration(joint, cal)
    endpoints = [encoder_to_units(value, joint, cal)
                 for value in (max(0, lower), min(4095, upper))]
    return min(endpoints), max(endpoints)


def raw_goal_positions(targets, calibration):
    """Match LeRobot's unit conversion and reject unsafe goals before any write."""
    if set(targets) != set(JOINTS):
        raise ValueError("A complete six-motor command is required")
    result = {}
    for joint in JOINTS:
        value, cal = targets[joint], calibration[joint]
        lower, upper = validate_calibration(joint, cal)
        minimum, maximum = command_limits(joint, cal)
        if not math.isfinite(value) or not minimum - 1e-9 <= value <= maximum + 1e-9:
            raise ValueError(f"Unsafe target for {joint}: {value!r}, permitted {minimum:g}..{maximum:g}")
        if joint == "gripper":
            opening = 100 - value if cal.get("drive_mode") else value
            raw = int(opening * (upper - lower) / 100 + lower)
        else:
            raw = int(value * 4095 / 360 + (lower + upper) / 2)
        if not max(0, lower) <= raw <= min(4095, upper):
            raise ValueError(f"Unsafe raw target for {joint}: {raw}")
        result[joint] = raw
    return result


def calibration_difference_report(saved, actual):
    """Compare hardware-backed SO/Feetech calibration fields, not software inversion."""
    differences = []
    for joint in sorted(set(saved) | set(actual)):
        if joint not in saved:
            differences.append(f"{joint}: missing from saved calibration")
        elif joint not in actual:
            differences.append(f"{joint}: missing from motor calibration readback")
        else:
            for field in ("homing_offset", "range_min", "range_max"):
                expected, found = saved[joint].get(field), actual[joint].get(field)
                if expected != found:
                    differences.append(f"{joint}.{field}: saved={expected!r}, motor={found!r}")
    return differences


def calibrated_positions(raw, calibration):
    """Convert one acquired sample to the units used by LeRobot's SO devices."""
    if set(raw) != set(JOINTS):
        raise ValueError("A complete six-motor position sample is required for teleoperation")
    result = {}
    for joint in JOINTS:
        value, cal = raw[joint], calibration[joint]
        validate_calibration(joint, cal)
        if not math.isfinite(value) or not 0 <= value <= 4095:
            raise ValueError(f"Invalid position for {joint}: measured {value!r}; "
                             f"permitted 0..4095, saved limits {cal['range_min']}..{cal['range_max']}")
        result[joint] = encoder_to_units(value, joint, cal)
    return result


def bounded_targets(requested, measured, calibration, previous, dt, joint_speed, gripper_speed):
    """Limit calibrated travel, command slew, and command distance from actual position."""
    if not 0 < dt <= 0.5:
        raise RuntimeError("Teleoperation position updates stalled for over 500 ms; stopping")
    result = {}
    for joint in JOINTS:
        cal = calibration[joint]
        lower, upper = command_limits(joint, cal)
        desired = min(upper, max(lower, requested[joint]))
        rate = gripper_speed if joint == "gripper" else joint_speed
        step = rate * min(dt, 0.1)
        base = previous[joint]
        command = min(base + step, max(base - step, desired))
        # Bound lag to 5 deg (or 5 percentage points), as well as bounding slew.
        # If external motion makes these constraints incompatible, stop rather than jump.
        low = max(lower, base - step, measured[joint] - 5)
        high = min(upper, base + step, measured[joint] + 5)
        if low > high:
            raise RuntimeError(f"Follower {joint} moved too far from its previous command; stopping")
        result[joint] = min(high, max(low, command))
    return result


class LiveDisplay:
    """Display cached data on its own timer; never perform extra motor reads."""

    def __init__(self, arms, origin_ns, hz, diagnostics_hz, interval_ms=100,
                 simulation=False, controller=False, output=None, teleoperation=False):
        self.arms = arms
        self.origin_ns = origin_ns
        self.hz, self.diagnostics_hz = hz, diagnostics_hz
        self.interval_ms = interval_ms
        self.simulation, self.controller_enabled = simulation, controller
        self.teleoperation = teleoperation
        self.output = output if output is not None else sys.stdout
        self.terminal = self.output.isatty()
        self.lock, self.stop_event = threading.Lock(), threading.Event()
        self.positions, self.diagnostics, self.controller = {}, {}, None
        self.samples, self.rows, self.errors = 0, 0, 0
        self.error = None
        self.thread = None

    def update_position(self, row):
        with self.lock:
            self.positions[row["arm"]] = row.copy()

    def update_diagnostic(self, row):
        with self.lock:
            self.diagnostics.setdefault(row["arm"], {})[row["register"]] = row.copy()

    def update_controller(self, row):
        with self.lock:
            self.controller = row.copy()

    def update_stats(self, samples, rows, errors):
        with self.lock:
            self.samples, self.rows, self.errors = samples, rows, errors

    @staticmethod
    def clean(value):
        # Serial errors/payloads must not inject terminal control sequences.
        return "".join(c if c.isprintable() else " " for c in str(value))

    @staticmethod
    def number(value):
        if value is None or value == "":
            return "--"
        if isinstance(value, float):
            text = f"{value:.2f}"
            return text if len(text) <= 8 else f"{value:.2g}"
        if isinstance(value, int) and len(str(value)) > 8:
            return f"{value:.2g}"
        return str(value)

    def render(self, now_ns=None):
        now_ns = time.monotonic_ns() if now_ns is None else now_ns
        with self.lock:
            positions = self.positions.copy()
            diagnostics = {arm: rows.copy() for arm, rows in self.diagnostics.items()}
            controller = self.controller.copy() if self.controller else None
            samples, rows, errors = self.samples, self.rows, self.errors
        elapsed = max(0, (now_ns - self.origin_ns) / 1e9)
        lines = [f"SO-101 LIVE{' | TELEOP' if self.teleoperation else ''}{' | SIMULATED DATA' if self.simulation else ''} | {elapsed:.1f} s | Ctrl+C stops",
                 f"Refresh {self.interval_ms:g} ms | Position {1000/self.hz:g} ms | Diagnostics {1000/self.diagnostics_hz:g} ms (targets)",
                 f"Actual positions: {samples/elapsed if elapsed else 0:.1f} Hz/arm | rows {rows} | read errors {errors}"]
        labels = ("ShldrPan", "ShldrLft", "Elbow", "WristFlx", "WristRol", "Gripper")
        diagnostic_labels = ("Motor speed raw", "Motor load raw", "Voltage raw", "Temperature raw",
                             "Current raw", "Status raw", "Moving raw", "Torque on raw",
                             "Goal position raw", "Goal speed raw", "Accel setting raw", "Torque limit raw")

        def age(row, field="read_end_monotonic_ns"):
            return f"{max(0, (now_ns-row[field])/1e6):.0f}" if row else "--"

        def table_row(label, values, age_ms):
            return f"{label:<18} {age_ms:>6} " + " ".join(f"{self.number(v):>8}" for v in values)

        for arm in self.arms:
            pos = positions.get(arm, {})
            diag = diagnostics.get(arm, {})
            lines += [f"{arm.upper()} | gripper opening: {self.number(pos.get('gripper_open_pct'))}%",
                      table_row("Metric / units", labels, "Age ms")]
            for label, metric in (("Position raw", "position_raw"),
                                  ("Motor angle deg", "position_deg"),
                                  ("Speed est deg/s", "velocity_est_deg_s"),
                                  ("Accel est deg/s2", "acceleration_est_deg_s2"),
                                  ("Outside limits 0/1", "outside_calibrated_range")):
                lines.append(table_row(label, [pos.get(f"{j}_{metric}") for j in JOINTS], age(pos)))
            for register, label in zip(DIAGNOSTICS, diagnostic_labels, strict=True):
                record = diag.get(register, {})
                lines.append(table_row(label,
                                       [record.get(f"{j}_raw") for j in JOINTS], age(record)))
            failed = (["position: " + pos["read_error"]] if pos.get("read_error") else [])
            failed += [f"{name}: {record['read_error']}" for name, record in diag.items()
                       if record.get("read_error")]
            if failed:
                lines.append("READ ERROR: " + self.clean(" | ".join(failed)))
        if self.controller_enabled:
            lines += ["", "ESP32 | " + (f"age {age(controller, 'host_receive_monotonic_ns')} ms"
                                        if controller else "waiting for data")]
            if controller:
                lines.append("Input: " + self.clean(controller.get("payload_json", "--")))
                if controller.get("read_error"):
                    lines.append("READ ERROR: " + self.clean(controller["read_error"]))
        lines.append("Age ms: latest read age | --: missing | Diagnostics: raw | Accel: estimate")
        return "\n".join(lines)

    def start(self):
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        try:
            if self.terminal:
                self.output.write("\033[?1049h\033[?25l")
            while not self.stop_event.is_set():
                frame = self.render()
                if self.terminal:
                    self.output.write("\033[H\033[2J" + frame + "\n")
                else:
                    self.output.write(frame + "\n\n")
                self.output.flush()
                self.stop_event.wait(self.interval_ms / 1000)
        except (OSError, ValueError) as exc:
            self.error = str(exc)
        finally:
            if self.terminal:
                try:
                    self.output.write("\033[?25h\033[?1049l")
                    self.output.flush()
                except (OSError, ValueError):
                    pass

    def close(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=2)


def csv_writer(path, fields):
    stream = path.open("x", newline="", encoding="utf-8")
    writer = csv.DictWriter(stream, fieldnames=fields)
    writer.writeheader()
    return stream, writer


def position_row(arm, raw, calibration, history, start_ns, end_ns, sample, origin_ns, error=""):
    """Derivatives use read midpoint times; missing reads reset their history."""
    midpoint_ns = (start_ns + end_ns) // 2
    row = dict(sample=sample, arm=arm, read_start_monotonic_ns=start_ns,
               read_end_monotonic_ns=end_ns, host_unix_ns=time.time_ns(),
               elapsed_s=(midpoint_ns - origin_ns) / 1e9, read_error=error)
    if error:
        history.clear()
        return row
    for joint in JOINTS:
        value = raw[joint]
        cal = calibration[joint]
        lower, upper = cal["range_min"], cal["range_max"]
        if upper <= lower:
            raise ValueError(f"Invalid calibration limits for {arm}/{joint}")
        # Matches the installed LeRobot degree convention (encoder range 0..4095).
        angle = (value - (lower + upper) / 2) * 360 / 4095
        row[f"{joint}_position_raw"] = value
        row[f"{joint}_position_deg"] = angle
        row[f"{joint}_outside_calibrated_range"] = int(not lower <= value <= upper)
        velocity = None
        if joint in history:
            previous_ns, previous_angle, previous_velocity = history[joint]
            dt = (midpoint_ns - previous_ns) / 1e9
            if dt > 0:
                # Do not silently differentiate a wraparound or a long acquisition gap.
                if abs(angle - previous_angle) < 180 and dt < 0.5:
                    velocity = (angle - previous_angle) / dt
                    row[f"{joint}_velocity_est_deg_s"] = velocity
                    if previous_velocity is not None:
                        previous_dt = history.get(joint + "_dt")
                        if previous_dt:
                            row[f"{joint}_acceleration_est_deg_s2"] = (
                                (velocity - previous_velocity) / ((dt + previous_dt) / 2)
                            )
                history[joint + "_dt"] = dt
        history[joint] = (midpoint_ns, angle, velocity)
        if joint == "gripper":
            opening = (min(upper, max(lower, value)) - lower) * 100 / (upper - lower)
            row["gripper_open_pct"] = 100 - opening if cal.get("drive_mode") else opening
    return row


class SimulatedArm:
    def __init__(self, name):
        self.name = name
        self.calibration = {j: dict(id=i + 1, range_min=500, range_max=3500,
                                    drive_mode=0, homing_offset=0) for i, j in enumerate(JOINTS)}
        self.t0 = time.monotonic()
        self.teleoperating = False
        self.position = None

    def read(self, register):
        if register == "present_position":
            if self.position is not None:
                return self.position.copy()
            return {j: int(2000 + 400 * math.sin(time.monotonic() - self.t0 + i / 5))
                    for i, j in enumerate(JOINTS)}
        return {j: 0 for j in JOINTS}

    def prepare_teleoperation(self):
        if self.name == "follower":
            self.position = self.read("present_position")
        self.teleoperating = True

    def send_action(self, targets):
        if not self.teleoperating or self.name != "follower":
            raise RuntimeError("Follower teleoperation was not enabled")
        raw_goal_positions(targets, self.calibration)
        for joint, value in targets.items():
            cal = self.calibration[joint]
            if joint == "gripper":
                opening = 100 - value if cal.get("drive_mode") else value
                self.position[joint] = round(cal["range_min"] + opening *
                                             (cal["range_max"] - cal["range_min"]) / 100)
            else:
                self.position[joint] = round((cal["range_min"] + cal["range_max"]) / 2 + value * 4095 / 360)

    def snapshot(self):
        return {"simulation": True, "calibration": self.calibration}

    def close(self):
        pass


class RealArm:
    def __init__(self, name, port, arm_id):
        from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
        from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig
        if name == "leader":
            device = SO101Leader(SO101LeaderConfig(port=port, id=arm_id))
        else:
            device = SO101Follower(SO101FollowerConfig(port=port, id=arm_id))
        self.name, self.port = name, port
        self.device, self.teleoperating = device, False
        self.bus = device.bus
        self.calibration = {j: asdict(c) for j, c in self.bus.calibration.items()}
        if set(self.calibration) != set(JOINTS):
            raise ValueError(f"Missing calibration for {name}: {device.calibration_fpath}")
        self.calibration_path = str(device.calibration_fpath)
        # Use the bus only. device.connect() would configure/write motor settings.
        self.bus.connect()
        try:
            actual = {j: asdict(c) for j, c in self.bus.read_calibration().items()}
            differences = calibration_difference_report(self.calibration, actual)
            if differences:
                helper = "apply_leader_limits.py" if name == "leader" else "apply_follower_limits.py"
                raise ValueError(
                    f"Saved and motor calibrations differ for {name}:\n"
                    + "\n".join("  " + entry for entry in differences)
                    + f"\nSaved file: {self.calibration_path}"
                    + f"\nTeleoperation has not started. To apply your provisional {name} limits and "
                    + f"save the current motor offsets, run: python {helper} --apply"
                    + "\nIf a helper refuses the pose, stop and resolve the reported joint; do not bypass the check."
                )
        except BaseException:
            self.close()
            raise

    def read(self, register):
        return self.bus.sync_read(register, normalize=False, num_retry=1)

    def prepare_teleoperation(self):
        # Mark this before any writes so partial setup also releases torque on close.
        self.teleoperating = True
        self.bus.disable_torque(num_retry=1)
        if self.name == "follower":
            self.bus.set_operating_mode("position")
            raw = self.read("present_position")
            calibrated_positions(raw, self.calibration)
            if any(not self.calibration[j]["range_min"] <= raw[j] <= self.calibration[j]["range_max"]
                   for j in JOINTS):
                raise ValueError("Place the follower within its calibrated range before teleoperation")
            # Seed the held goal with its actual pose before configure() enables torque.
            self.bus.sync_write("Goal_Position", raw, normalize=False)
        self.device.configure()

    def send_action(self, targets):
        if not self.teleoperating or self.name != "follower":
            raise RuntimeError("Follower teleoperation was not enabled")
        raw_goal_positions(targets, self.calibration)
        return self.device.send_action({f"{joint}.pos": value for joint, value in targets.items()})

    def snapshot(self):
        import rustypot
        _, definition = rustypot.find_model("sts3215")
        result = dict(port=self.port, calibration_path=self.calibration_path,
                      calibration=self.calibration, registers={}, register_definitions=[])
        seen = set()
        for register in definition.registers():
            result["register_definitions"].append(dict(
                name=register.name, address=register.addr, size=register.size,
                encoding=register.encoding, access=str(register.access)))
            key = (register.addr, register.size)
            if key in seen or register.access == rustypot.RegisterAccess.Write:
                continue
            seen.add(key)
            try:
                result["registers"][register.name] = {"values": self.read(register.name)}
            except Exception as exc:
                result["registers"][register.name] = {"read_error": str(exc)}
        return result

    def close(self):
        # Read-only mode leaves torque unchanged; active teleoperation releases it.
        self.bus.disconnect(disable_torque=getattr(self, "teleoperating", False))


def receive_controller(port, baud, path, stop, failure, display=None):
    """Optional ESP32 newline-delimited JSON; preserves unknown fields in payload_json."""
    stream = None
    try:
        import serial
        stream, writer = csv_writer(path, CONTROLLER_FIELDS)
        with serial.Serial(port, baud, timeout=0.2) as connection:
            buffer = bytearray()
            while not stop.is_set():
                buffer.extend(connection.read(connection.in_waiting or 1))
                if len(buffer) > 65536:
                    raise ValueError("Controller sent over 64 KB without a complete JSON line")
                while b"\n" in buffer:
                    line, _, rest = buffer.partition(b"\n")
                    buffer = bytearray(rest)
                    row = dict(host_receive_unix_ns=time.time_ns(),
                               host_receive_monotonic_ns=time.monotonic_ns())
                    payload = line.decode("utf-8", errors="replace").strip()
                    if not payload:
                        continue
                    row["payload_json"] = payload
                    try:
                        obj = json.loads(payload)
                        if not isinstance(obj, dict):
                            raise ValueError("Expected a JSON object")
                        for field in CONTROLLER_FIELDS[2:11]:
                            if field in obj:
                                row[field] = obj[field]
                        row["encoder_angles_deg_json"] = json.dumps(obj.get("encoder_angles_deg", []))
                    except (ValueError, TypeError) as exc:
                        row["read_error"] = str(exc)
                    if display:
                        display.update_controller(row)
                    writer.writerow(row)
                    stream.flush()
    except Exception as exc:
        failure.append(str(exc))
        stop.set()
    finally:
        if stream:
            stream.close()


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--follower-port", default="/dev/tty.usbmodem5C4C1241631")
    p.add_argument("--follower-id", default="my_follower")
    p.add_argument("--leader-port", help="Optional second arm, e.g. /dev/tty.usbmodem5C4C1237851")
    p.add_argument("--leader-id", default="my_leader")
    p.add_argument("--teleoperate", action="store_true",
                   help="Make the follower copy the SO-101 leader while recording (requires --leader-port)")
    p.add_argument("--joint-speed-deg-s", type=float, default=60,
                   help="Maximum commanded joint-angle slew during teleoperation (default: 60 deg/s)")
    p.add_argument("--gripper-speed-pct-s", type=float, default=100,
                   help="Maximum commanded gripper-opening slew (default: 100 percentage points/s)")
    p.add_argument("--hz", type=float, default=20, help="Requested position sampling rate, not guaranteed")
    p.add_argument("--diagnostics-hz", type=float, default=1)
    p.add_argument("--duration", type=float, default=0, help="Seconds; 0 records until Ctrl+C")
    p.add_argument("--out", type=Path)
    p.add_argument("--simulate", action="store_true", help="Synthetic data only, no hardware access")
    p.add_argument("--controller-port", help="Optional ESP32 USB serial port streaming JSON lines")
    p.add_argument("--controller-baud", type=int, default=115200)
    p.add_argument("--live", action=argparse.BooleanOptionalAction, default=None,
                   help="Live display (default: on in a terminal, off when redirected)")
    p.add_argument("--live-ms", type=float, default=100,
                   help="Display refresh interval in milliseconds; minimum 20 (default: 100)")
    return p


def main(argv=None):
    p = parser()
    args = p.parse_args(argv)
    for field in ("hz", "diagnostics_hz", "duration", "joint_speed_deg_s", "gripper_speed_pct_s"):
        number = getattr(args, field)
        if not math.isfinite(number) or number < 0 or (field != "duration" and number == 0):
            p.error(f"--{field.replace('_', '-')} must be finite and {'nonnegative' if field == 'duration' else 'positive'}")
    if not math.isfinite(args.live_ms) or args.live_ms < 20:
        p.error("--live-ms must be finite and at least 20 milliseconds")
    ports = [x for x in (args.follower_port, args.leader_port, args.controller_port) if x]
    if len(ports) != len(set(ports)):
        p.error("Each device must have a different serial port")
    if args.simulate and args.controller_port:
        p.error("--simulate cannot open a real controller port")
    if args.teleoperate and not args.leader_port:
        p.error("--teleoperate requires --leader-port for the SO-101 leader")
    if args.teleoperate and args.hz < 5:
        p.error("--teleoperate requires --hz of at least 5 to keep commands fresh")
    label = "SIMULATED_" if args.simulate else "session_"
    out = args.out or Path.cwd() / "recordings" / (label + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    out.mkdir(parents=True, exist_ok=False)
    stop, failures, devices, streams = threading.Event(), [], [], []
    thread, display = None, None
    metadata = dict(simulation=args.simulate, started_utc=datetime.now(timezone.utc).isoformat(),
                    requested_position_hz=args.hz, requested_diagnostics_hz=args.diagnostics_hz,
                    mode="leader_teleoperation" if args.teleoperate else "read_only",
                    motor_writes=args.teleoperate and not args.simulate,
                    arms={}, arguments={k: str(v) if isinstance(v, Path) else v
                                                           for k, v in vars(args).items()})
    exit_code, rows, read_errors = 0, 0, 0
    try:
        for name, port, arm_id in [("follower", args.follower_port, args.follower_id)] + (
            [("leader", args.leader_port, args.leader_id)] if args.leader_port else []
        ):
            device = SimulatedArm(name) if args.simulate else RealArm(name, port, arm_id)
            devices.append(device)
            print(f"Reading initial register snapshot: {name}", flush=True)
            metadata["arms"][name] = device.snapshot()
        if not args.simulate:
            metadata["versions"] = {name: version(name) for name in ("lerobot", "rustypot")}
        (out / "metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
        pos_stream, positions = csv_writer(out / "positions.csv", POSITION_FIELDS)
        diag_stream, diagnostics = csv_writer(out / "diagnostics.csv", DIAGNOSTIC_FIELDS)
        streams.extend([pos_stream, diag_stream])
        action_stream, actions, previous_command, last_command_ns = None, None, None, None
        pending_diagnostics = []
        if args.teleoperate:
            action_stream, actions = csv_writer(out / "actions.csv", ACTION_FIELDS)
            streams.append(action_stream)
            arm_map = {d.name: d for d in devices}
            print("TELEOPERATION: follower will copy leader. Keep both arms clear; Ctrl+C releases torque.", flush=True)
            # Free/configure leader first, then establish the follower's initial held pose.
            arm_map["leader"].prepare_teleoperation()
            arm_map["follower"].prepare_teleoperation()
        histories = {d.name: {} for d in devices}
        origin = time.monotonic_ns()
        next_position, next_diagnostics, next_status = time.monotonic(), time.monotonic(), time.monotonic() + 2
        sample, diagnostic_sample = 0, 0
        shutdown_note = "Teleoperation stops and torque is released." if args.teleoperate else "Motor torque is left unchanged."
        print(f"{'SIMULATED DATA. ' if args.simulate else ''}Recording to {out.resolve()}\nCtrl+C stops. {shutdown_note}", flush=True)
        live_enabled = sys.stdout.isatty() if args.live is None else args.live
        metadata["live_display"] = dict(enabled=live_enabled, interval_ms=args.live_ms)
        if live_enabled:
            display = LiveDisplay([d.name for d in devices], origin, args.hz, args.diagnostics_hz,
                                  args.live_ms, args.simulate, bool(args.controller_port),
                                  teleoperation=args.teleoperate)
            display.start()
        if args.controller_port:
            thread = threading.Thread(target=receive_controller, args=(
                args.controller_port, args.controller_baud, out / "controller.csv", stop, failures, display), daemon=True)
            thread.start()
        while not stop.is_set():
            now = time.monotonic()
            if args.duration and (time.monotonic_ns() - origin) / 1e9 >= args.duration:
                break
            if now >= next_position:
                acquired, position_errors, acquisition_starts = {}, [], []
                for device in devices:
                    start = time.monotonic_ns()
                    acquisition_starts.append(start)
                    raw, error = {}, ""
                    try:
                        raw = device.read("present_position")
                    except Exception as exc:
                        error = str(exc)
                        read_errors += 1
                        position_errors.append(f"{device.name}: {error}")
                    end = time.monotonic_ns()
                    position = position_row(device.name, raw, device.calibration,
                                            histories[device.name], start, end, sample, origin, error)
                    positions.writerow(position)
                    if display:
                        display.update_position(position)
                    rows += 1
                    acquired[device.name] = raw
                if args.teleoperate:
                    if position_errors:
                        raise RuntimeError("Teleoperation stopped after position read failure: " + "; ".join(position_errors))
                    if stop.is_set():
                        break
                    leader, follower = arm_map["leader"], arm_map["follower"]
                    requested = calibrated_positions(acquired["leader"], leader.calibration)
                    measured = calibrated_positions(acquired["follower"], follower.calibration)
                    command_ns = time.monotonic_ns()
                    if (command_ns - min(acquisition_starts)) / 1e9 > 0.5:
                        raise RuntimeError("Position acquisition took over 500 ms; stopping teleoperation")
                    dt = (command_ns - last_command_ns) / 1e9 if last_command_ns else 1 / args.hz
                    if previous_command is None:
                        previous_command = measured.copy()
                    sent = bounded_targets(requested, measured, follower.calibration, previous_command,
                                           dt, args.joint_speed_deg_s, args.gripper_speed_pct_s)
                    action = dict(sample=sample, command_start_monotonic_ns=command_ns)
                    for joint in JOINTS:
                        units = "pct" if joint == "gripper" else "deg"
                        action[f"{joint}_requested_{units}"] = requested[joint]
                    try:
                        follower.send_action(sent)
                        for joint in JOINTS:
                            units = "pct" if joint == "gripper" else "deg"
                            action[f"{joint}_sent_{units}"] = sent[joint]
                    except Exception as exc:
                        action["command_error"] = str(exc)
                        raise
                    finally:
                        action["command_end_monotonic_ns"] = time.monotonic_ns()
                        action["read_to_command_duration_ns"] = action["command_end_monotonic_ns"] - min(acquisition_starts)
                        actions.writerow(action)
                        action_stream.flush()
                    previous_command, last_command_ns = sent, command_ns
                sample += 1
                next_position += 1 / args.hz
                if next_position < time.monotonic():
                    next_position = time.monotonic() + 1 / args.hz
            if not pending_diagnostics and time.monotonic() >= next_diagnostics:
                pending_diagnostics = [(register, device) for register in DIAGNOSTICS for device in devices]
                next_diagnostics = time.monotonic() + 1 / args.diagnostics_hz
            # Active motion reads diagnostics one register at a time, checking for
            # due position/command updates between reads instead of blocking on a batch.
            if pending_diagnostics:
                count = 1 if args.teleoperate else len(pending_diagnostics)
                for _ in range(count):
                    register, device = pending_diagnostics.pop(0)
                    row = dict(sample=diagnostic_sample, arm=device.name, register=register,
                               read_start_monotonic_ns=time.monotonic_ns())
                    try:
                        row.update({f"{j}_raw": value for j, value in device.read(register).items()})
                    except Exception as exc:
                        row["read_error"] = str(exc)
                        read_errors += 1
                    row["read_end_monotonic_ns"] = time.monotonic_ns()
                    diagnostics.writerow(row)
                    if display:
                        display.update_diagnostic(row)
                if not pending_diagnostics:
                    diagnostic_sample += 1
            for stream in streams:
                stream.flush()
            if display:
                display.update_stats(sample, rows, read_errors)
            elif time.monotonic() >= next_status:
                elapsed = (time.monotonic_ns() - origin) / 1e9
                print(f"{elapsed:.1f}s | {rows} position rows | {sample / elapsed:.1f} samples/s per arm | {read_errors} read errors", flush=True)
                next_status = time.monotonic() + 2
            if not pending_diagnostics:
                stop.wait(max(0, min(next_position, next_diagnostics) - time.monotonic()))
        if failures:
            raise RuntimeError("Controller logging failed: " + failures[0])
    except KeyboardInterrupt:
        if display:
            display.close()
        print("\nRecording stopped.")
    except Exception as exc:
        if display:
            display.close()
        metadata["fatal_error"] = str(exc)
        print(f"Error: {exc}", file=sys.stderr)
        exit_code = 1
    finally:
        stop.set()
        if display:
            display.close()
            if display.error:
                metadata["live_display_error"] = display.error
        if thread:
            thread.join(timeout=2)
        for stream in streams:
            stream.close()
        for device in devices:
            try:
                device.close()
            except Exception as exc:
                metadata.setdefault("close_errors", []).append(str(exc))
                exit_code = 1
        metadata.update(ended_utc=datetime.now(timezone.utc).isoformat(),
                        position_rows=rows, read_errors=read_errors, controller_errors=failures)
        if "origin" in locals():
            elapsed = (time.monotonic_ns() - origin) / 1e9
            metadata.update(duration_s=elapsed, achieved_position_hz_per_arm=sample / elapsed if elapsed else 0)
        (out / "metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    print(f"Saved: {out.resolve()}")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
