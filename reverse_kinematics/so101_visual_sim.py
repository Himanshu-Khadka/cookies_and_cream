"""Virtual SO-101 and Tk canvas renderer. No hardware objects are created.

Reuses the project's URDF-derived geometry and bounded position IK. This is a
kinematic preview, not a simulation of motor dynamics, loads, or collisions.
"""

import math

import numpy as np

from wasd_so101 import (
    MODEL_LIMITS, ORIGINS, TIP_XYZ, forward_kinematics,
    inverse_kinematics, transform,
)


class VirtualSO101:
    def __init__(self):
        self.reset()

    def reset(self):
        self.q = np.deg2rad([0, -45, 60, 0, 0])
        self.home = forward_kinematics(self.q)
        self.tip = self.home.copy()
        self.requested = self.tip.copy()
        self.trail = [self.tip.copy()]
        self.status = "Holding position"

    def frames(self):
        pose = np.eye(4)
        frames = []
        for origin, angle in zip(ORIGINS, self.q):
            pose = pose @ origin @ transform((0, 0, 0), (0, 0, angle))
            frames.append(pose.copy())
        return frames

    def step(self, x, y, dt, speed_mm_s=35):
        """Controller +X = right (-robot Y); +Y = forward (+robot X)."""
        self.requested = self.tip.copy()
        if not (x or y):
            self.status = "Holding position"
            return
        dt = min(max(dt, 0), .05)
        if not dt:
            return
        self.requested += np.array([y, -x, 0]) * speed_mm_s / 1000 * dt
        self.requested[2] = self.home[2]
        slew = math.radians(60) * dt
        lower = np.maximum(MODEL_LIMITS[:, 0], self.q - slew)
        upper = np.minimum(MODEL_LIMITS[:, 1], self.q + slew)
        lower[4] = upper[4] = self.q[4]
        solution = inverse_kinematics(self.q, self.requested, lower, upper)
        if solution is None:
            self.status = "Reach / joint limit — try another direction"
            return
        self.q = solution
        self.tip = forward_kinematics(self.q)
        self.status = "Moving"
        if np.linalg.norm(self.tip - self.trail[-1]) >= .002:
            self.trail.append(self.tip.copy())
            self.trail = self.trail[-250:]


class ArmView:
    def __init__(self, canvas, arm):
        self.canvas, self.arm = canvas, arm
        self.yaw, self.elevation = math.radians(24), math.radians(28)
        self.drag_origin = None
        canvas.bind("<Configure>", lambda event: self.draw())
        canvas.bind("<ButtonPress-1>", self.start_drag)
        canvas.bind("<B1-Motion>", self.drag)
        canvas.bind("<ButtonRelease-1>", lambda event: setattr(self, "drag_origin", None))

    def start_drag(self, event):
        self.canvas.focus_set()
        self.drag_origin = (event.x, event.y)

    def drag(self, event):
        if self.drag_origin is None:
            return
        x, y = self.drag_origin
        self.yaw += (event.x - x) * .008
        self.elevation = min(1.3, max(.15, self.elevation + (event.y - y) * .006))
        self.drag_origin = (event.x, event.y)
        self.draw()

    def project(self, point):
        # Robot X points forward; robot Y points left.
        forward, right, up = point[0] - .20, -point[1], point[2] - .12
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        screen_x = c * right + s * forward
        depth = c * forward - s * right
        screen_y = math.sin(self.elevation) * depth - math.cos(self.elevation) * up
        width, height = self.canvas.winfo_width(), self.canvas.winfo_height()
        scale = min(width / .88, height / .66)
        return width / 2 + screen_x * scale, height * .52 + screen_y * scale

    def line(self, points, **style):
        return self.canvas.create_line(*[v for p in points for v in self.project(p)], **style)

    def dot(self, point, radius, **style):
        x, y = self.project(point)
        return self.canvas.create_oval(x-radius, y-radius, x+radius, y+radius, **style)

    def draw(self):
        c, arm = self.canvas, self.arm
        c.delete("all")
        if min(c.winfo_width(), c.winfo_height()) < 10:
            return
        corners = [(-.05, -.30, 0), (.45, -.30, 0), (.45, .30, 0), (-.05, .30, 0)]
        c.create_polygon(*[v for p in corners for v in self.project(p)],
                         fill="#e7edf5", outline="#c6d2e1")
        for i in range(-1, 10):
            self.line([(i*.05, -.30, 0), (i*.05, .30, 0)], fill="#ced8e5")
        for i in range(-6, 7):
            self.line([(-.05, i*.05, 0), (.45, i*.05, 0)], fill="#ced8e5")

        for endpoint, label, color in [((.14, 0, .001), "Forward / W", "#2563eb"),
                                        ((0, -.14, .001), "Right / D", "#07866b")]:
            self.line([(0, 0, .001), endpoint], fill=color, width=2, arrow="last")
            x, y = self.project(endpoint)
            c.create_text(x, y+14, text=label, fill=color, font=("Helvetica", 10, "bold"))

        frames = arm.frames()
        points = [frame[:3, 3] for frame in frames] + [arm.tip]
        foot = (points[0][0], points[0][1], 0)
        x, y = self.project(foot)
        c.create_oval(x-30, y-12, x+30, y+12, fill="#69788d", outline="")
        self.line([foot, points[0]], fill="#69788d", width=18)

        if len(arm.trail) > 1:
            self.line(arm.trail, fill="#97b6e9", width=2)
        self.line([arm.tip, (arm.tip[0], arm.tip[1], 0)], fill="#96a5b8", dash=(3, 4))
        self.dot((arm.tip[0], arm.tip[1], 0), 5, fill="#96a5b8", outline="")
        for i, (a, b) in enumerate(zip(points, points[1:])):
            self.line([a, b], fill="#334761" if i < 3 else "#2563eb",
                      width=[17, 15, 13, 11, 7][i], capstyle="round")
        for p in points[:-1]:
            self.dot(p, 7, fill="#d9e4f3", outline="#334761", width=2)

        # A schematic open claw, oriented by the wrist's actual model frame.
        wrist = frames[-1]
        for sign in (-1, 1):
            jaw = [(wrist @ np.array([TIP_XYZ[0] + sign*.012, TIP_XYZ[1], z, 1]))[:3]
                   for z in (TIP_XYZ[2]+.025, TIP_XYZ[2]-.012)]
            self.line(jaw, fill="#e68a24", width=5, capstyle="round")
        self.dot(arm.tip, 5, fill="#e68a24", outline="")

        if arm.status.startswith("Reach"):
            self.dot(arm.requested, 8, fill="", outline="#c44141", width=2)
        for p, label in [(points[1], "Shoulder"), (points[2], "Elbow"), (points[3], "Wrist")]:
            x, y = self.project(p)
            c.create_text(x+13, y-13, text=label, anchor="w", fill="#34465f", font=("Helvetica", 10))
        c.create_text(16, 18, text="SO-101 · kinematic preview", anchor="w",
                      fill="#26364a", font=("Helvetica", 14, "bold"))
        c.create_text(16, 41, text="Drag to orbit  ·  Floor grid: 50 mm", anchor="w",
                      fill="#536780", font=("Helvetica", 11))
