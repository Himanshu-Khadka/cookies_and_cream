# Cookies and Cream

Cookies and Cream is an SO-101 robot arm teleoperation project. A handheld
controller built around an STM32F411 Black Pill reads two MKS SERVO42C encoders,
an AS5600 rail encoder, a joystick, a clutch button, and a PS5 trigger. The
repository contains the controller firmware, Python tools for reading its input,
and separate tools for exercising the arm and recording servo telemetry.

## Drive the arm (one command)

[`run_arm.py`](run_arm.py) sets up any computer (macOS, Linux, Windows) and
starts [`working_code/control_arm.py`](working_code/control_arm.py), which
drives the SO-101 from the controller. It gets the code if needed, makes a
`.venv` and installs the packages, fixes Linux serial permissions, finds the
arm's and the controller's ports, and uses the calibration stored in this repo.
Needs Python 3.10+ (3.12 recommended) and git.

Plug in the arm's USB and the controller's USB-C, then:

```sh
python3 run_arm.py              # set up, then dry run: prints, moves nothing
python3 run_arm.py --live       # set up, then MOVE THE REAL ARM
```

Other options:

```sh
python3 run_arm.py --stlink                 # read the controller through the ST-Link instead of USB-C
python3 run_arm.py --setup-only             # only set up
python3 run_arm.py --live -- --no-clutch    # anything after -- goes to control_arm.py
```

| Controller | Arm |
| --- | --- |
| Button | Clutch: hold to drive, release to freeze |
| `arm_twist` (MKS 2) | Base rotation (`shoulder_pan`) |
| `arm_roll` (MKS 1) | Wrist rotation (`wrist_roll`) |
| Joystick Y | Forward / back (inverse kinematics; push further = faster) |
| Rail (`arm_v`) | Up / down (inverse kinematics) |
| Trigger | Gripper close / open |

Motion starts from wherever the arm is when you press the clutch, so no home
file is needed. Start with the arm resting or supported, keep a hand on the
arm's power for the first live run, and press Enter at the end to release the
motors (hold the arm first). If the gripper works backwards, add
`-- --gripper-open 100 --gripper-closed 0`. If no button is wired, add
`-- --no-clutch`. See `python3 working_code/control_arm.py --help` for speeds
and limits.

## Try it without hardware

From the repository root, use Python 3.12 to run a simulated homing move, record
two seconds of synthetic telemetry, or run the controller package's offline tests.

```sh
python3.12 hardware_test/home_so101.py --simulate
python3.12 hardware_test/record_telemetry.py --simulate --duration 2 --no-live
python3.12 -m unittest discover -s maybe_final/so101_controller_update/tests -v
```

The telemetry example writes `positions.csv`, `diagnostics.csv`, and
`metadata.json` to a new session directory under `recordings/`. The offline tests
cover packet parsing, input mapping, clutch behavior, and gripper endpoints.

## Read the handheld controller

The main controller firmware uses the STM32 Arduino core with the BlackPill
F411CE board and USB CDC enabled. It streams eight-column CTL1 CSV over USB.
See [the controller sketch](firmware/controller/controller.ino).

Use the Python reader to find the port and inspect the stream. Replace
`/dev/ttyACM2` with your controller's port, such as `/dev/cu.usbmodem…` on macOS.

```sh
python3.12 lib/read_encoders.py --list
python3.12 lib/read_encoders.py /dev/ttyACM2 --center --format labeled
```

Put both MKS encoders and the AS5600 rail at neutral before running `--center`.
The reader saves the center locally in `lib/center.json`. Re-center after cycling
power to the encoder drivers. Press Ctrl+C to release the serial port before
starting another reader. For CSV, JSON, and rail calibration options, see
[the encoder reader](lib/read_encoders.py).

| Controller input | Arm control |
| --- | --- |
| MKS 1 (`arm_roll`) | Gripper rotation via `wrist_roll` |
| MKS 2 (`arm_twist`) | Base rotation via `shoulder_pan` |
| Joystick Y (`joy_y`) | Forward/back tip velocity |
| AS5600 rail (`arm_v`) | Up/down tip velocity |
| Clutch button (`button`) | Hold to enable controller input |
| PS5 trigger (`trigger`) | Gripper open/close with CTL1 |

## Work with the SO-101

Install the pinned dependencies in a Python 3.12 environment for the arm tools
and graphical simulator. See [requirements.txt](requirements.txt).

```sh
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

Use the LeRobot calibration for your connected arm and select its serial port
explicitly. Support the arm before releasing motor torque. The repository
includes [the saved calibration for `my_follower`](calibration_file/my_follower.json).

| Tool | Purpose |
| --- | --- |
| [Home the arm](hardware_test/home_so101.py) | Move the body joints to the saved calibration midpoint while holding the gripper opening. |
| [Rotate one joint](hardware_test/rotate_so101.py) | Enter relative rotations in degrees for a selected joint. |
| [Record telemetry](hardware_test/record_telemetry.py) | Log positions and servo diagnostics in read-only mode by default. `--teleoperate` enables leader-to-follower motion with a second SO-101. |
| [Simulate joint motion](hardware_test/hardware_test_simulator.py) | Explore joint motion in a Matplotlib visualization. |

## Full controller integration

**CTL1 over USB** maps the PB10 digital trigger to open and closed. With the
controller connected and the Python environment active, try joystick Y, wrist
roll, and base rotation in a dry run. `--no-rail` holds the starting height and
`--no-gripper` holds the gripper opening.

```sh
python working_code/stream_encoders_to_arm_no_flash.py \
  --encoder-port /dev/ttyACM2 --no-rail --no-gripper
```

Live motion uses the arm's LeRobot calibration and a saved Cartesian home. See
[the no-flash setup notes](NO_FLASH_SETUP.md) for the full controller workflow
and [the inverse kinematics tool](working_code/arm_ik.py) for saving home.
The clutch is a software hold. Keep a physical power disconnect within reach
during live tests.

**CTL2** adds proportional trigger position on PA1. The firmware, streaming
script, and offline tests are bundled as an overlay that uses the motion helpers
in `working_code/`. The ST-Link reader expects CTL2 and uses OpenOCD. For wiring,
trigger calibration, and where to place the overlay files, see the
[controller update guide](maybe_final/so101_controller_update/CONTROLLER_INTEGRATION_GUIDE.md).

The [firmware directory](firmware/) also contains a Black Pill motor bench test
and a standalone PS5 trigger reader. [Motor test sketches](tests/motors/) cover
the ESP32 encoder setup. [Collision detection](collision_detection/) contains
early notes and a model stub.

## Two-axis encoder bench test

If your checkout includes `hardware_test/encoder_so101.py`, you can test wrist
roll and shoulder pan independently of the full controller. Send one absolute
`roll,twist` pair per line. To try it without hardware, run the following command.

```sh
python hardware_test/encoder_so101.py --simulate
```

Type `0,0` for center, `15,-20` to move, or `181,0` to see an out-of-range
rejection. A bad value rejects the whole pair and holds the last command.
The other four joints hold their starting positions.

For the real arm, activate the project's Python 3.12 environment and pass the
SO-101 USB port.

```sh
python hardware_test/encoder_so101.py --port /dev/tty.usbmodemARM
```

Keep stdin open to feed positions from another program, or add
`--encoder-port /dev/tty.usbmodemESP32` to read directly from the controller
at 115200 baud. The script accepts both `roll,twist` lines and the existing
`time_ms,mks1_deg,mks2_deg,as5600_deg` firmware stream. It uses MKS 1 as roll
and MKS 2 as twist by default. Pass `--roll-mks 2` if they are wired the
other way. The stock multi-encoder sketch sends absolute angles. For readings
with an offset from zero, pass `--encoder-home ROLL TWIST` with the measured
readings at center.

The default arm calibration is `calibration_file/my_follower.json`.
Use `--calibration path/to/your_follower.json` to select the saved limits and
offsets that match the connected arm. Zero maps to the midpoint
of each joint's saved range. By default the input is in degrees, with travel
of ±180° for roll and about ±109.32° for twist with the bundled calibration.
If the hand controller has a different range or sends counts, set its measured
endpoints relative to home using `--roll-range LEFT RIGHT` and
`--twist-range LEFT RIGHT`. For example, `--roll-range -90 90` maps a 90°
hand roll to the calibrated wrist roll limit. Use `--reverse-roll` or
`--reverse-twist` if an axis moves in the opposite direction on your setup.

The arm stops advancing if the encoder stream goes quiet. Support the arm
before quitting with `q`, Ctrl+C, or stdin EOF, which releases motor torque.
