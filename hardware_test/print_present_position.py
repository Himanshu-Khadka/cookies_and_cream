#!/usr/bin/env python3
# The line above selects Python 3 when this file is run as an executable.
"""Print labeled raw follower positions from one NumPy array until Ctrl+C."""  # Describe this script.
import time  # Provide sleep() to pause between reads.

import numpy as np  # Import NumPy and give it the short name np.


def main():  # Define the function that connects, reads, and prints positions.
    from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig  # Load the follower class and its configuration class.

    follower = SO101Follower(SO101FollowerConfig(  # Create the follower object with the configuration below.
        port="/dev/tty.usbmodem5C4C1241631", id="my_follower"))  # Set its USB port and saved calibration ID.
    bus = follower.bus  # Get the communication interface shared by the follower's motors.
    bus.connect()  # Connect directly to the bus without configuring motors or enabling torque.
    try:  # Ensure that the bus is disconnected when reading stops or an error occurs.
        motors = tuple(bus.motors)  # Use the configured motor names as labels and keep their array order fixed.
        positions = np.empty(len(motors), dtype=np.int32)  # Allocate one array of 32-bit integers, one per motor.
        while True:  
            values = bus.sync_read("present_position", normalize=False, num_retry=1)  # Read raw encoder counts for all motors, allowing one retry.
            for i, motor in enumerate(motors):  # Pair each motor name with its array index.
                positions[i] = values[motor]  # Replace that motor's entry with its latest position.
            print(" | ".join(f"{motor}: {positions[i]}" for i, motor in enumerate(motors)), flush=True)  # Print each motor name beside its raw position, separated by |.
            time.sleep(0.05)  # Pause for 50 milliseconds; read time adds to the total interval.
    except KeyboardInterrupt:  # Handle the interruption produced by pressing Ctrl+C.
        pass  # Exit quietly without printing a traceback for Ctrl+C.
    finally:  # Run cleanup whether the loop stops normally or because of an error.
        bus.disconnect(disable_torque=False)  # Close the connection while leaving motor torque unchanged.


if __name__ == "__main__": 
    main()  
