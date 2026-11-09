#!/usr/bin/env python3
"""Example: use the arm's encoder readings in your own code.

Keep this file in the same folder as read_encoders.py, then run:

    python example_read_encoder.py

Each reading has:
    arm_roll    wrist roll motor (MKS 1), degrees from the saved center
    arm_twist   wrist twist motor (MKS 2), degrees from the saved center
    arm_v       the linear rail, degrees of its gear from center (or
                millimetres once the rail's mm-per-turn has been set)
"""

from __future__ import annotations

import math
import time

from read_encoders import CenterError, EncoderStream, PortError


def simple_loop() -> None:
    """Handle every reading as it arrives."""
    with EncoderStream() as encoders:
        for angles in encoders:
            if math.isnan(angles.arm_roll) or math.isnan(angles.arm_twist):
                print("a motor encoder did not answer")
                continue

            # arm_v is nan while the rail sensor sees no magnet.
            print(
                f"arm_roll {angles.arm_roll:8.2f}   "
                f"arm_twist {angles.arm_twist:8.2f}   arm_v {angles.arm_v:8.2f}"
            )


def own_loop() -> None:
    """Run a loop at your own pace and just look up the newest angles."""
    encoders = EncoderStream()
    encoders.start()

    try:
        while True:
            angles = encoders.latest()

            if angles is not None:
                print(
                    f"{angles.time_s:7.2f} s  arm_roll {angles.arm_roll:8.2f}  "
                    f"arm_twist {angles.arm_twist:8.2f}  arm_v {angles.arm_v:8.2f}"
                )

            time.sleep(0.5)  # your own work goes here
    finally:
        encoders.close()


if __name__ == "__main__":
    try:
        own_loop()  # or simple_loop()
    except KeyboardInterrupt:
        pass
    except (PortError, CenterError) as error:
        print(f"Error: {error}")

