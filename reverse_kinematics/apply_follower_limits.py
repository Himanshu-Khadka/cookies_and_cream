#!/usr/bin/env python3
"""Preview or apply provisional SO-101 FOLLOWER limits; never command motion.

The limits are an independent copy of apply_leader_limits.py at creation time,
requested as temporary values, not independently measured follower calibration.
The leader is never opened. Current follower homing offsets and wrist-roll
limits are preserved. --apply writes limits and the follower JSON; default
mode only reads and previews. Verify physical travel before using these values.
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import os
import sys
from dataclasses import asdict
from datetime import datetime
from pathlib import Path

LIMITS = {
    "shoulder_pan": (1966, 2094),
    "shoulder_lift": (2041, 3603),
    "elbow_flex": (0, 2800),
    "wrist_flex": (183, 3967),
    "gripper": (1169, 3349),
}
JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")


def proposed_calibration(current, saved):
    if set(current) != set(JOINTS):
        raise ValueError("All six follower motors must have readable calibration registers")
    candidate = copy.deepcopy(current)
    for joint, cal in candidate.items():
        # Feetech drive inversion is software-only, so retain the saved setting.
        if joint in saved:
            cal.drive_mode = saved[joint].drive_mode
        if joint in LIMITS:
            cal.range_min, cal.range_max = LIMITS[joint]
        if not 0 <= cal.range_min < cal.range_max <= 4095:
            raise ValueError(f"Invalid proposed limits for {joint}: {cal.range_min}..{cal.range_max}")
        if not -2047 <= cal.homing_offset <= 2047:
            raise ValueError(f"Invalid current homing offset for {joint}: {cal.homing_offset}")
    return candidate


def check_pose(positions, candidate):
    if set(positions) != set(candidate):
        raise ValueError("A complete six-motor follower position sample is required")
    outside = []
    for joint, cal in candidate.items():
        value = positions[joint]
        if not math.isfinite(value) or not cal.range_min <= value <= cal.range_max:
            outside.append(f"{joint}: current {value}, proposed {cal.range_min}..{cal.range_max}")
    if outside:
        raise ValueError("Follower is outside the proposed range; limits were not written. "
                         "With motors released, gently place it inside the listed ranges and retry:\n"
                         + "\n".join(outside))


def dump_calibration(calibration):
    return {joint: asdict(cal) for joint, cal in calibration.items()}


def verify_registers(found, expected):
    if set(found) != set(expected):
        raise RuntimeError("Motor calibration readback was incomplete")
    for joint in expected:
        for field in ("id", "homing_offset", "range_min", "range_max"):
            if getattr(found[joint], field) != getattr(expected[joint], field):
                raise RuntimeError(f"Calibration readback mismatch: {joint}.{field}")


def apply_limits(bus, before, candidate, calibration_path, backup_root):
    """Verify pose, back up, write only limits, verify, and atomically save JSON."""
    check_pose(bus.sync_read("Present_Position", normalize=False, num_retry=1), candidate)
    backup = backup_root / ("follower_limits_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    backup.mkdir(parents=True, exist_ok=False)
    if calibration_path.exists():
        (backup / "original_saved_calibration.json").write_bytes(calibration_path.read_bytes())
    (backup / "original_motor_calibration.json").write_text(
        json.dumps(dump_calibration(before), indent=2) + "\n", encoding="utf-8")
    (backup / "proposed_calibration.json").write_text(
        json.dumps(dump_calibration(candidate), indent=2) + "\n", encoding="utf-8")
    print(f"Backup: {backup.resolve()}", flush=True)
    bus.disable_torque(num_retry=1)
    # Check again after releasing holding torque; do not write around a shifted pose.
    check_pose(bus.sync_read("Present_Position", normalize=False, num_retry=1), candidate)
    temp = calibration_path.with_name(calibration_path.name + ".pending_" + backup.name)
    changed = False
    try:
        changed = True  # A partial first write also requires rollback.
        for joint, (lower, upper) in LIMITS.items():
            bus.write("Min_Position_Limit", joint, lower, normalize=False)
            bus.write("Max_Position_Limit", joint, upper, normalize=False)
        verify_registers(bus.read_calibration(), candidate)
        calibration_path.parent.mkdir(parents=True, exist_ok=True)
        with temp.open("x", encoding="utf-8") as stream:
            stream.write(json.dumps(dump_calibration(candidate), indent=2) + "\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp, calibration_path)
    except BaseException as exc:
        if changed:
            try:
                bus.write_calibration(before, cache=False)
                verify_registers(bus.read_calibration(), before)
                print("Original motor calibration restored; torque remains off.", file=sys.stderr)
            except Exception as rollback_error:
                raise RuntimeError(f"Apply failed ({exc}); rollback also failed ({rollback_error}). "
                                   f"Keep teleoperation stopped. Backup: {backup.resolve()}") from exc
        raise
    finally:
        temp.unlink(missing_ok=True)
    return backup


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--apply", action="store_true", help="Write the shown follower limits; torque is left off")
    parser.add_argument("--port", default="/dev/tty.usbmodem5C4C1241631")
    parser.add_argument("--id", default="my_follower")
    parser.add_argument("--backup-dir", type=Path, default=Path(__file__).resolve().parent / "calibration_backups")
    args = parser.parse_args(argv)
    from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

    follower = SO101Follower(SO101FollowerConfig(port=args.port, id=args.id))
    bus = follower.bus
    try:
        # Use the bus directly: device.connect() configures motors and may write.
        bus.connect()
        current = bus.read_calibration()
        candidate = proposed_calibration(current, follower.calibration)
        positions = bus.sync_read("Present_Position", normalize=False, num_retry=1)
        print(f"FOLLOWER ONLY | {args.port} | {follower.calibration_fpath}")
        print(f"{'Joint':<16} {'Min':>6} {'Position':>9} {'Max':>6} {'Active offset':>14}")
        for joint, cal in candidate.items():
            print(f"{joint:<16} {cal.range_min:>6} {positions[joint]:>9} {cal.range_max:>6} {cal.homing_offset:>14}")
        if not args.apply:
            print("Preview only. No settings changed. Add --apply to write these follower limits.")
            return 0
        apply_limits(bus, current, candidate, follower.calibration_fpath, args.backup_dir)
        print("Follower limits verified and saved. Follower torque is off; leader was not accessed.")
        print("Restart record_telemetry.py --teleoperate to use the new follower calibration.")
        return 0
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    finally:
        bus.disconnect(disable_torque=False)


if __name__ == "__main__":
    raise SystemExit(main())
