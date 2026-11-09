# SO-101 controller — no STM32 flashing required

Use the existing `controller.ino` firmware already running on your STM32 Black Pill (CTL1, 8-column CSV). The Python file `working_code/stream_encoders_to_arm_no_flash.py` maps the existing **digital** trigger (0 or 1) to gripper 0% or 100%. This does **not** provide proportional trigger control; that cannot be recovered from a 1-bit firmware signal. Do not move PB10 to PA1 for this version.

## File layout

```
project/
  lib/
    read_encoders.py                # use supplied updated reader, backward-compatible with CTL1
    read_controller.py              # only needed for --stlink
    read_trigger.py                 # only needed for --stlink
    center.json                     # created by --center
  working_code/
    arm_all.py
    arm_ik.py
    run_rotations.py
    stream_encoders_to_arm_no_flash.py
```

## Commands

1. Place both encoder motors and rail at neutral, with the clutch disengaged:

```
python3 lib/read_encoders.py /dev/ttyACM2 --center
```

2. Verify the stream includes valid `arm_roll`, `arm_twist`, `arm_v`, `joy_y`, `button`, and `trigger` fields:

```
python3 lib/read_encoders.py /dev/ttyACM2 --format labeled
```

3. Stop the separate reader with Ctrl+C before running the arm controller: only one program can own the same serial port at once.

4. DRY RUN (no SO-101 motion):

```
python3 working_code/stream_encoders_to_arm_no_flash.py --encoder-port /dev/ttyACM2
```

5. LIVE (slow initial speeds; replace SO-101 port with its actual port):

```
python3 working_code/stream_encoders_to_arm_no_flash.py \
  --encoder-port /dev/ttyACM2 \
  --arm-port /dev/ttyACM0 \
  --live --max-speed 10 --y-max-speed 0.5 --z-max-speed 0.5
```

Clutch `button=1` enables control; `button=0` freezes commands. Roll/twist are absolute encoder-derived offsets, joystick Y and AS5600 rail offset are velocity commands, and trigger `0/1` means gripper open/closed. `--gripper-reverse` reverses gripper travel; `--invert-y` and `--invert-z` reverse travel direction. Start with physical emergency power disconnect available, clear the arm workspace, and check servo/gripper directions with slow settings. The clutch is a software hold, not a safety-rated E-stop.

If the Black Pill is running a different firmware that does not transmit joystick/button/trigger/AS5600 data, Python cannot supply those missing signals without changing the firmware or using external input hardware. If you later want truly proportional gripping, you must either change the firmware/hardware input path, or read an independently connected gamepad through the host OS.
