"""Display WASD as a thumbstick and a virtual SO-101; no robot required.

Run: python3 thumbstick_simulator.py
Hold W/S for Y=+1/-1 and A/D for X=-1/+1. Opposite keys cancel.
"""

import math
import time
import tkinter as tk
from tkinter import ttk

from so101_visual_sim import ArmView, VirtualSO101


class ThumbstickSimulator:
    window_title = "WASD Thumbstick + SO-101 Simulator"
    arm_hint = "Height stays fixed. Release WASD to hold position."
    arm_detail = "Simplified links · Kinematic preview"
    speed_label = "Speed/axis"
    reset_key = "r"
    jog_keys = tuple("wasd")
    key_hint = "Hold WASD • Release to center • Esc to close"
    ready_status = "Ready — hold WASD."

    def __init__(self, root):
        self.root = root
        self.held = set()
        self.pending_releases = {}
        self.arm = VirtualSO101()
        self.last_tick = time.monotonic()
        self.closed = False
        self.timer = None
        root.title(self.window_title)
        root.geometry("1120x700")
        root.minsize(900, 620)

        outer = ttk.Frame(root, padding=16)
        outer.pack(fill="both", expand=True)
        frame = ttk.Frame(outer, width=320, padding=(0, 0, 16, 0))
        frame.pack(side="left", fill="y")
        frame.pack_propagate(False)
        ttk.Label(frame, text="Thumbstick simulator", font=("Helvetica", 20, "bold")).pack()
        ttk.Label(frame, text=self.key_hint).pack(pady=(6, 12))

        self.canvas = tk.Canvas(frame, background="#f3f6fa", highlightthickness=0, height=245)
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda event: self.draw())
        self.canvas.bind("<Button-1>", lambda event: self.canvas.focus_set())

        self.values = tk.StringVar()
        ttk.Label(frame, textvariable=self.values, font=("Courier", 16, "bold")).pack(pady=(14, 4))
        self.keys = tk.StringVar()
        ttk.Label(frame, textvariable=self.keys).pack()
        ttk.Label(frame, text="W = forward (+Y)    S = backward (−Y)").pack(pady=(12, 0))
        ttk.Label(frame, text="A = left (−X)    D = right (+X)").pack()
        ttk.Label(frame, text="Diagonals: both axes at full scale.\nOpposite keys cancel.", justify="center").pack(pady=(8, 0))
        self.build_motion_controls(frame)
        self.status = tk.StringVar(value="Click the window to use WASD.")
        ttk.Label(frame, textvariable=self.status, wraplength=295, justify="center").pack(pady=(10, 0))

        right = ttk.Frame(outer)
        right.pack(side="left", fill="both", expand=True)
        arm_canvas = tk.Canvas(right, background="#f3f6fa", highlightthickness=0)
        arm_canvas.pack(fill="both", expand=True)
        self.arm_view = ArmView(arm_canvas, self.arm)
        self.arm_info = tk.StringVar()
        ttk.Label(right, textvariable=self.arm_info, font=("Courier", 12), justify="left").pack(anchor="w", pady=(12, 4))
        ttk.Label(right, text=self.arm_hint).pack(anchor="w")
        ttk.Label(right, text=self.arm_detail).pack(anchor="w", pady=(4, 0))

        root.bind("<KeyPress>", self.press)
        root.bind("<KeyRelease>", self.release)
        root.bind("<FocusOut>", self.focus_out)
        root.bind("<Unmap>", self.focus_out)
        root.bind("<FocusIn>", self.focus_in)
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.refresh()
        self.canvas.focus_set()
        self.timer = root.after(33, self.tick)

    def build_motion_controls(self, frame):
        ttk.Label(frame, text="Speed per axis (mm/s)").pack(pady=(12, 0))
        self.speed = tk.DoubleVar(value=35)
        ttk.Scale(frame, from_=10, to=80, variable=self.speed, takefocus=False).pack(fill="x", padx=12)
        ttk.Button(frame, text="Reset arm (R)", command=self.reset_arm, takefocus=False).pack(pady=8)

    @property
    def axes(self):
        return int("d" in self.held) - int("a" in self.held), int("w" in self.held) - int("s" in self.held)

    def press(self, event):
        key = event.keysym.lower()
        if key == "escape":
            self.close()
            return
        if key == self.reset_key:
            self.reset_arm()
            return
        if key not in self.jog_keys:
            return
        pending = self.pending_releases.pop(key, None)
        if pending is not None:
            self.root.after_cancel(pending)
        self.held.add(key)
        self.refresh()

    def release(self, event):
        key = event.keysym.lower()
        if key not in self.held:
            return
        pending = self.pending_releases.pop(key, None)
        if pending is not None:
            self.root.after_cancel(pending)
        # Some systems send release/press pairs during keyboard auto-repeat.
        self.pending_releases[key] = self.root.after(25, lambda: self.finish_release(key))

    def finish_release(self, key):
        self.pending_releases.pop(key, None)
        self.held.discard(key)
        self.refresh()

    def focus_in(self, event):
        self.status.set(self.ready_status)

    def focus_out(self, event):
        for pending in self.pending_releases.values():
            self.root.after_cancel(pending)
        self.pending_releases.clear()
        self.held.clear()
        self.status.set("Unfocused — centered. Click here to resume.")
        self.refresh()

    def refresh(self):
        x, y = self.axes
        self.values.set(f"X: {x:+.2f}    Y: {y:+.2f}")
        active = " + ".join(key.upper() for key in self.jog_keys if key in self.held)
        self.keys.set(f"Held: {active or 'none'}")
        self.draw()

    def reset_arm(self):
        self.focus_out(None)
        self.arm.reset()
        self.status.set("Arm reset — hold WASD.")
        self.last_tick = time.monotonic()
        self.update_arm_display()

    def update_arm_display(self):
        arm = self.arm
        relative = (arm.tip - arm.home) * 1000
        degrees = [f"{math.degrees(q):.1f}" for q in arm.q]
        self.arm_info.set(
            f"{arm.status}\n"
            f"Right: {-relative[1]:+6.1f} mm   Forward: {relative[0]:+6.1f} mm\n"
            f"Height: {arm.tip[2]*1000:5.1f} mm   {self.speed_label}: {self.speed.get():g} mm/s\n"
            f"Joints (deg): {', '.join(degrees)}"
        )
        self.arm_view.draw()

    def tick(self):
        if self.closed:
            return
        now = time.monotonic()
        self.arm.step(*self.axes, now-self.last_tick, self.speed.get())
        self.last_tick = now
        self.update_arm_display()
        self.timer = self.root.after(33, self.tick)

    def close(self):
        self.closed = True
        if self.timer is not None:
            self.root.after_cancel(self.timer)
        for timer in self.pending_releases.values():
            self.root.after_cancel(timer)
        self.root.destroy()

    def draw(self):
        canvas = self.canvas
        canvas.delete("all")
        width, height = canvas.winfo_width(), canvas.winfo_height()
        if min(width, height) < 10:
            return
        cx, cy = width / 2, height / 2
        radius = max(10, min(width, height) / 2 - 40)
        canvas.create_rectangle(cx-radius, cy-radius, cx+radius, cy+radius,
                                fill="#e5ebf3", outline="#bac6d5")
        canvas.create_line(cx-radius, cy, cx+radius, cy, fill="#bac6d5")
        canvas.create_line(cx, cy-radius, cx, cy+radius, fill="#bac6d5")
        for label, px, py in [("W / +Y", cx, cy-radius-23), ("S / −Y", cx, cy+radius+23),
                              ("A", cx-radius-22, cy), ("D", cx+radius+22, cy)]:
            canvas.create_text(px, py, text=label, fill="#26364a", font=("Helvetica", 11))
        x, y = self.axes
        px, py = cx + x * radius, cy - y * radius
        canvas.create_line(cx, cy, px, py, fill="#2563eb", width=4)
        canvas.create_oval(px-13, py-13, px+13, py+13, fill="#2563eb", outline="")


def main():
    try:
        root = tk.Tk()
    except tk.TclError as exc:
        raise SystemExit(f"Cannot open the GUI. Run this on a desktop with Tk support: {exc}") from exc
    ThumbstickSimulator(root)
    root.mainloop()


if __name__ == "__main__":
    main()
