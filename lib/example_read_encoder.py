#!/usr/bin/env python3
"""Example: use the encoder angles in your own code.

Keep this file in the same folder as read_encoders.py, then run:

    python example_use.py
"""

from __future__ import annotations

import math
import time

from read_encoders import CenterError, EncoderStream, PortError


def simple_loop() -> None:
    """Handle every reading as it arrives."""
    with EncoderStream() as encoders:
        for angles in encoders:
            if math.isnan(angles.motor1) or math.isnan(angles.motor2):
                print("an encoder did not answer")
                continue

            print(f"roll {angles.motor1:8.2f} deg   twist {angles.motor2:8.2f} deg")


def own_loop() -> None:
    """Run a loop at your own pace and just look up the newest angles."""
    encoders = EncoderStream()
    encoders.start()

    try:
        while True:
            angles = encoders.latest()

            if angles is not None:
                print(f"{angles.time_s:7.2f} s  roll {angles.motor1:8.2f}  "
                      f"twist {angles.motor2:8.2f}  as5600 {angles.as5600:8.2f}")

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

