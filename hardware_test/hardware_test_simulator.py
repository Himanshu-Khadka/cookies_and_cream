"""
Robotic arm test harness, live version.  Hold keys to move continuously.

    q / w       rotate counterclockwise / clockwise   (stepper 1)
    a / d       move left / right along x             (stepper 2)
    up / down   z up / down                           (handle rail motor)
    space       toggle claw (trigger), starts open

To use real hardware, replace read_inputs() with code that returns
[rotation_deg_per_s, x_units_per_s, z_units_per_s] from your motors.
"""
import matplotlib
try:
    matplotlib.use("TkAgg")      # interactive backend; remove if it errors
except Exception:
    pass
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

# ---------------------------------------------------------------- settings
ROT_SPEED = 90.0     # deg/s
MOVE_SPEED = 10.0    # units/s
LIMIT = 20
ARROW_LEN = 4.0
DT_MS = 30           # update interval

# key -> [rotation, x, z] direction
KEYMAP = {
    "q": [+1, 0, 0], "w": [-1, 0, 0],
    "a": [0, -1, 0], "d": [0, +1, 0],
    "up": [0, 0, +1], "down": [0, 0, -1],
}
SPEED = np.array([ROT_SPEED, MOVE_SPEED, MOVE_SPEED])

# ------------------------------------------------------------------- state
pos = np.array([10.0, 0.0, 5.0])   # x, y, z
theta = 0.0                         # degrees
claw_open = True
held = set()


def read_inputs():
    """Swap this for real motor readings. Returns [rot, x, z] per second."""
    d = sum((np.array(KEYMAP[k]) for k in held if k in KEYMAP), np.zeros(3))
    return d * SPEED


def step(rates, dt):
    global theta
    rot, dx, dz = rates * dt
    theta = (theta + rot) % 360      # spin in place, position unchanged
    pos[0] += dx
    pos[2] += dz
    pos[:] = np.clip(pos, [-LIMIT, -LIMIT, 0], [LIMIT, LIMIT, LIMIT])


# ---------------------------------------------------------------- plotting
for k in [k for k in plt.rcParams if k.startswith("keymap.")]:
    plt.rcParams[k] = []             # stop q from quitting, etc.

fig = plt.figure(figsize=(8, 7))
ax = fig.add_subplot(111, projection="3d")
ax.set(xlim=(-LIMIT, LIMIT), ylim=(-LIMIT, LIMIT), zlim=(0, LIMIT),
       xlabel="X", ylabel="Y", zlabel="Z")
point, = ax.plot([], [], [], "o", ms=12)
stem, = ax.plot([], [], [], "--", c="gray", lw=0.8)
heading, = ax.plot([], [], [], c="tab:blue", lw=2)
info = ax.text2D(0.02, 0.95, "", transform=ax.transAxes,
                 family="monospace", va="top")


def on_press(e):
    global claw_open
    if e.key == " " and " " not in held:   # toggle once per press, not repeat
        claw_open = not claw_open
    held.add(e.key)


def on_release(e):
    held.discard(e.key)


def update(_):
    step(read_inputs(), DT_MS / 1000)
    x, y, z = pos
    t = np.radians(theta)
    point.set_data_3d([x], [y], [z])
    point.set_color("tab:green" if claw_open else "tab:red")
    stem.set_data_3d([x, x], [y, y], [0, z])
    heading.set_data_3d([x, x + ARROW_LEN * np.cos(t)],
                        [y, y + ARROW_LEN * np.sin(t)], [z, z])
    info.set_text(f"x={x:6.1f}  y={y:6.1f}  z={z:6.1f}\n"
                  f"theta={theta:6.1f} deg\nclaw_open={claw_open}")


fig.canvas.mpl_connect("key_press_event", on_press)
fig.canvas.mpl_connect("key_release_event", on_release)
anim = FuncAnimation(fig, update, interval=DT_MS, cache_frame_data=False)
plt.show()