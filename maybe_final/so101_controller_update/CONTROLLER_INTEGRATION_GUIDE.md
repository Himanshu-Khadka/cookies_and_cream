# SO-101 controller integration — joystick Y, AS5600 Z, analog trigger, clutch

## Files in the update

Only replace/add these files in your project:

```
project/
├── firmware/
│   └── controller/
│       └── controller.ino                # NEW CTL2 Black Pill firmware
├── lib/
│   ├── read_encoders.py                  # REPLACE with updated combined reader
│   ├── read_controller.py                # REPLACE for optional ST-Link mode
│   ├── read_trigger.py                   # SAME helper as before (ST-Link only)
│   └── center.json                       # KEEP your saved center or re-center
└── working_code/
    ├── arm_all.py                        # KEEP unchanged; do not run concurrently
    ├── arm_ik.py                         # KEEP unchanged
    ├── run_rotations.py                  # KEEP unchanged
    └── stream_encoders_to_arm.py         # REPLACE with updated controller
```

`tests/test_controller_integration.py` contains optional offline checks.
The update does **not** require modifying `arm_all.py`, `arm_ik.py`, or
`run_rotations.py`. The streaming script calls the same IK and action helpers
used by `arm_all.py` rather than feeding it lines of text.

## Mandatory hardware change: proportional PS5 trigger

The **old** `controller.ino` reads a digital 0/1 trigger signal on PB10.
PB10 is **not** analog-capable, so a percentage is not available from that
old firmware. Rewire the trigger *signal* to **PA1 (A1)**, with a common
GND and a sensor supply of **3.3 V**. **Never apply more than 3.3 V to PA1.**
Check your actual PS5 trigger module's power and signal voltage before wiring.
PA1 is unused by the other pins in the supplied `controller.ino`.

Flash the new `firmware/controller/controller.ino` to the STM32F411 Black Pill
with the STM32 Arduino core. Board settings in the original firmware:
Generic STM32F4 / BlackPill F411CE and USB CDC enabled. The new firmware
outputs nine columns:

```
time_ms,arm_roll,arm_twist,arm_v,joy_x,joy_y,button,trigger,trigger_percent
```

`trigger` remains a **legacy 0/1 flag** based on whether the new analog
percentage is >=50%; the proportional value is **`trigger_percent`**, 0..100.
The separate `read_trigger.py`/`ps5_trigger.ino` setup is **not** required or
used while running the new combined controller firmware.

**Trigger calibration every STM32 boot:** Release the trigger before booting,
then, with the clutch/button **released (0)**, press the trigger completely
once. The firmware learns its resting value and maximum travel in RAM. It
reports `nan` until travel exceeds its ADC noise threshold; after a full
squeeze, release it again (roughly 0%) before engaging the clutch. If the
trigger starts pressed at boot or the sensor noise prevents stable readings,
reset the board with the trigger released. This is an approximate calibration,
not a mechanically certified gripper force limit.

## 1. Check serial devices

On Linux:

```bash
ls /dev/ttyACM*
python3 lib/read_encoders.py --list
```

On macOS:

```bash
ls /dev/cu.usb*
python3 lib/read_encoders.py --list
```

Example below uses **`/dev/ttyACM2` for the Black Pill** and **`/dev/ttyACM0` for the SO-101**. Replace with your actual ports. Close serial monitors before launching a reader.

## 2. Center the input encoders and AS5600 rail

Place **both MKS motors and the AS5600 rail at their neutral positions**.
Run once (and again after a stepper motor power cycle):

```bash
python3 lib/read_encoders.py /dev/ttyACM2 --center
```

The AS5600 must report a valid angle, not `nan`; otherwise the center will
not be saved for Z. The reader saves `center.json` in `lib/`. If using the
optional millimeter calibration, run, for example:

```bash
python3 lib/read_encoders.py /dev/ttyACM2 --rail-mm-per-rev 40
```

Only use that command if you have actually measured **40 mm of rail travel
per full gear revolution**. Otherwise leave the default unit as degrees.

Verify the stream without touching the SO-101:

```bash
python3 lib/read_encoders.py /dev/ttyACM2 --format labeled
```

Confirm `joy_y`, `arm_v`, `button`, and `trigger_percent` all update. The
button should read **1 only when pressed**. `trigger_percent` should report
~0 at rest and ~100 at full squeeze. Stop with Ctrl+C **before** launching
the integration script; only one process can own a serial port.

## 3. Set SO-101 home (if you haven't already)

From your project root, with arm supported by hand:

```bash
python3 working_code/arm_ik.py sethome --port /dev/ttyACM0
```

Torque turns off while setting home; **support the arm so it cannot drop**.
You must have calibrated the SO-101 via LeRobot already. Home is the origin
for Y and Z as in `arm_all.py`.

## 4. Dry run (no robot movement)

```bash
python3 working_code/stream_encoders_to_arm.py \
  --encoder-port /dev/ttyACM2
```

While the clutch/button is 0, output will say `CLUTCH OFF`. With all inputs
valid, press and hold the clutch and move the joystick/rail. The command
prints mapped pan/roll, integrated Y/Z, and trigger %. No SO-101 motor
commands are sent in this mode. The test returns to the zero Y/Z origin
on each new clutch engagement in dry-run mode (no arm pose is available).

## 5. First live test — slow settings

Use a clear workspace, hold the arm in a safe pose, remove objects from the
gripper, and keep a **physical power disconnect** within reach. **Test the
claw direction** before handling objects: the code assumes the lower end of
the gripper calibration range is **open** and the upper end is **closed**.
If the actual gripper goes the opposite way, add `--gripper-reverse`.

```bash
python3 working_code/stream_encoders_to_arm.py \
  --encoder-port /dev/ttyACM2 \
  --arm-port /dev/ttyACM0 \
  --live \
  --max-speed 10 \
  --y-max-speed 0.5 \
  --z-max-speed 0.5
```

**Keep clutch OFF when starting and calibrating.** Engage it only when the
robot is physically clear. The controller **does not send a movement target
on the first clutch-on sample** and re-anchors roll/pan plus Y/Z to the
arm's *observed* pose. When released, it sends **one** current-position hold
command and then no further commands until re-engagement. It does **not**
turn off torque or act as a hardware emergency stop; position-controlled
servos may still move briefly while stopping.

Stop with Ctrl+C. `arm_all.py` must NOT be running at the same time (same
SO-101 serial port).

### Default control behavior

| Input | Mapping | Defaults |
| --- | --- | --- |
| `arm_roll` | wrist roll, original normalized mapping | input -500..500° to output -180..180°; clamped to servo calibration |
| `arm_twist` | shoulder pan, original normalized mapping | input -500..500° to output -107.7..109.7°; clamped to servo calibration |
| `joy_y` | integrated Y velocity, positive Y = forward | 10% center deadzone; 2 cm/s maximum; curved response (exponent 1.5) |
| `arm_v` | integrated Z velocity, positive Z = up | 12% center deadzone; 1.5 cm/s maximum; 360° of gear offset (or 40 mm) for maximum speed |
| `trigger_percent` | proportional `gripper.pos` | 0% open, 100% closed using calibrated motor range |
| `button` | clutch | 0 frozen, 1 enabled; re-anchor whenever engaged |
| pitch | held at current actual pitch when clutch engages | `--pitch 0` for a fixed 0° pitch if desired |

The controller uses a 5-reading median for AS5600 noise, an exponential
low-pass filter, and deadzones. It forces velocities to zero when centered
so slow filters do not cause long-lasting creep. The rail turn counter can
become unreliable if the gear turns too far between sensor samples; in that
case the program refuses to command motion until the stream is restarted
and re-centered. Invalid/missing input or stale stream (>0.7 s by default)
also causes the system to hold.

The output Y/Z is bounded to **±15 cm relative to saved HOME**, but the
inverse-kinematics solver can still reject positions outside its real
workspace. The script then holds its last accepted Y/Z target while
letting other valid controls respond. All joint targets are additionally
limited to 15°/s by default (10°/s in the first-live-test command), subject
to calibrated limits and servo hardware behavior. The speed limit is a
**software target slew limit**, not a guaranteed measured motor speed.

### Useful command options

- `--invert-y` / `--invert-z` when the control direction is backward.
- `--gripper-reverse` if opening and closing are reversed on your robot.
- `--z-full-scale-deg 720` if you need **two** gear turns for maximum Z speed.
- `--z-smoothing 0.12` to smooth a noisy AS5600 more strongly (slower response).
- `--z-deadzone 0.2` to prevent Z drift when the rail is near neutral.
- `--max-speed 10` caps arm **joint target** changes to 10 degrees/second.
- `--y-max-speed 1` / `--z-max-speed 1` sets tip velocity caps (cm/s).
- `--pitch 0` requests fixed zero gripper pitch as in the previous script.
- `--stlink` instead of `--encoder-port`: requires `lib/read_controller.py`,
  `lib/read_trigger.py`, OpenOCD, and an ST-Link connected to the Black Pill.
- `--roll-in-min`, `--roll-in-max`, `--twist-in-min`, `--twist-in-max` override
  the approximate -500..500 input limits.

## Offline checks

From the root of the update package:

```bash
python3 -m unittest discover -s tests -v
python3 -m compileall -q lib working_code/stream_encoders_to_arm.py
```

These check Python parser, ST-Link RAM format, mappings, speed curve,
clutch re-anchoring, invalid-sensor handling, and gripper endpoints. They
do **not** compile STM32 firmware or establish actual robot motion/safety.

## Known issues / hardware checks

1. **PA1 wiring is mandatory** for proportional trigger data. PB10 only
   provides an on/off reading in the old combined firmware.
2. **Analog signal voltage must stay within 0..3.3 V** and share ground.
3. **Gripper direction is not knowable from calibration metadata alone**;
   verify and use `--gripper-reverse` if needed. Gripper calibration is a
   position range, not force control. Avoid pinching objects during testing.
4. **AS5600 must be centered and have a magnet**. Noise or lost multi-turn
   tracking pauses motion; re-center and restart in that case.
5. **Clutch is software only**, not a certified E-stop. Torque remains on.
6. **No hardware or MCU compiler was available** in the file-building
   environment. Firmware must be flashed and physically verified on your
   STM32F411 + SO-101 setup.
