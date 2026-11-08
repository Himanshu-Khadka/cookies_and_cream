#!/bin/sh
# Real-arm IK follower with extra reach (gripper angle free to change). Run from anywhere.
cd "$(dirname "$0")/.." || exit 1
exec hardware_test/.venv/bin/python reverse_kinematics/testing_folow_point.py \
  --urdf hardware_test/SO-ARM100/Simulation/SO101/so101_new_calib.urdf \
  --port /dev/tty.usbmodem5C4C1241631 --execute --relative \
  --reach 0.30 --orientation-weight 0 "$@"
