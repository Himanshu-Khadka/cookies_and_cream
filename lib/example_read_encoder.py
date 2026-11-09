#!/usr/bin/env python3
"""Example: use the controller's readings in your own code.

Keep this file in the same folder as read_encoders.py, then run:

    python example_read_encoder.py            board on USB-C
    python example_read_encoder.py --stlink   board on the ST-Link only

Each reading has:
    arm_roll    wrist roll motor (MKS 1), degrees from the saved center
    arm_twist   wrist twist motor (MKS 2), degrees from the saved center
    arm_v       the linear rail, degrees of its gear from center (or
                millimetres once the rail's mm-per-turn has been set)
    joy_x       joystick X, -1.0 .. +1.0 (0 = middle)
    joy_y       joystick Y, -1.0 .. +1.0 (0 = middle)
    button      1 = pressed, 0 = not
    trigger     1 = on, 0 = off

A number is nan (math.isnan) when that part did not answer; button and
trigger are None when the board's firmware does not send them.
"""

from __future__ import annotations

import argparse
import math
import time

from read_encoders import (
    Angles,
    CenterError,
    EncoderStream,
    PortError,
    install_stop_signals,
)

# Ignore joystick movement smaller than this (it never rests exactly at 0).
DEADZONE = 0.08


def stick(value: float) -> float:
    """Joystick value with the small wobble around the middle removed."""
    if math.isnan(value) or abs(value) < DEADZONE:
        return 0.0
    return value


def describe(angles: Angles) -> str:
    return (
        f"roll {angles.arm_roll:8.2f}  twist {angles.arm_twist:8.2f}  "
        f"rail {angles.arm_v:8.2f}  |  joy x {stick(angles.joy_x):+.2f} "
        f"y {stick(angles.joy_y):+.2f}  button {angles.button}  "
        f"trigger {angles.trigger}"
    )


def simple_loop(stlink: bool) -> None:
    """Handle every reading as it arrives."""
    with EncoderStream(stlink=stlink) as encoders:
        was_pressed = False

        for angles in encoders:
            if math.isnan(angles.arm_roll) or math.isnan(angles.arm_twist):
                print("a motor encoder did not answer")

            # React once when the button goes down, not on every reading.
            pressed = angles.button == 1
            if pressed and not was_pressed:
                print("button pressed")
            was_pressed = pressed

            if angles.trigger == 1:
                print("trigger on")

            print(describe(angles))


def own_loop(stlink: bool) -> None:
    """Run a loop at your own pace and just look up the newest readings."""
    encoders = EncoderStream(stlink=stlink)
    encoders.start()

    try:
        while True:
            angles = encoders.latest()

            if angles is not None:
                print(f"{angles.time_s:7.2f} s  " + describe(angles))

            time.sleep(0.5)  # your own work goes here
    finally:
        encoders.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stlink", action="store_true", help="read through the ST-Link")
    parser.add_argument("--simple", action="store_true", help="use the simple loop")
    args = parser.parse_args()
    install_stop_signals()  # Ctrl-C, a kill or a closed terminal stop it cleanly

    try:
        if args.simple:
            simple_loop(args.stlink)
        else:
            own_loop(args.stlink)
    except KeyboardInterrupt:
        pass
    except (PortError, CenterError) as error:
        print(f"Error: {error}")
