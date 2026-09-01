"""
03_rotation_ui.py -- HW1 Part 2, Task 1 (extra): interactive slider UI
with auto-playing preset sequences.

This is an OPTIONAL extension of 01_rotation_sandbox.py. It gives you
a live Tkinter control panel next to the MuJoCo viewer with:

  - one angle slider per axis (x, y, z), with a live numeric readout
  - a current/fixed radio-button toggle per axis
  - an "order slot" dropdown per axis (1st / 2nd / 3rd applied)
  - a live-updating printout of the resulting 3x3 rotation matrix R
  - TWO "Play Sequence" buttons that automatically animate a preset
    rotation sequence: the sliders visibly move on their own, the
    frame/order controls update to match, and the dart rotates live
    in the viewer. This is designed so you can hit one button, let
    it play, hit the other button, and have a clean single screen
    recording that shows both cases without any manual dragging.

Edit SEQUENCE_1 / SEQUENCE_2 below to match the exact two-rotation
example you used in Problem 3. By default:
  SEQUENCE_1 = same two rotations, composed about the CURRENT (body)
               frame, in order Z then X.
  SEQUENCE_2 = the identical two rotations, composed about the FIXED
               (space) frame, same nominal order.
These are picked so the end orientation visibly differs -- exactly
the non-commutativity result Problem 3 proves symbolically.

You can still use the sliders/toggles manually at any time; the
Play buttons just drive the same controls programmatically.

Requires: pip install mujoco numpy  (tkinter ships with most Python
installs already; on some Linux distros you may need:
    sudo apt install python3-tk
)
"""

import threading
import tkinter as tk
from tkinter import ttk

import numpy as np
import mujoco
import mujoco.viewer

from utils import ELEMENTARY_ROTATIONS, set_body_orientation

MODEL_PATH = "../model/asymmetric_body.xml"

AXES = ["x", "y", "z"]

# ---------------------------------------------------------------
# TODO(student): edit these to match your Problem 3 example.
# Each entry: (axis, target_angle_deg, frame, order_slot)
# ---------------------------------------------------------------
SEQUENCE_1 = [
    ("z", 90.0, "current", 1),
    ("x", 90.0, "current", 2),
    ("y", 0.0, "current", 3),
]

SEQUENCE_2 = [
    ("z", 90.0, "fixed", 1),
    ("x", 90.0, "fixed", 2),
    ("y", 0.0, "fixed", 3),
]

ANIM_DURATION_MS = 1200   # how long each axis takes to animate to its target
ANIM_STEP_MS = 16         # ~60 Hz


class RotationState:
    """Holds the current angle, frame choice, and application order
    for each axis, and knows how to compose them into a single
    rotation matrix R.

    `order[axis]` is an integer 1, 2, or 3 saying which "slot" that
    axis's rotation is applied in. The rotations are composed in
    increasing slot order. Each one independently uses its own
    'current' (right-multiply) or 'fixed' (left-multiply) rule,
    exactly like compose_sequence() in 01_rotation_sandbox.py.
    """

    def __init__(self):
        self.angles_deg = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.frames = {"x": "current", "y": "current", "z": "current"}
        self.order = {"x": 1, "y": 2, "z": 3}
        self.lock = threading.Lock()

    def set_angle(self, axis, deg):
        with self.lock:
            self.angles_deg[axis] = deg

    def set_frame(self, axis, frame):
        with self.lock:
            self.frames[axis] = frame

    def set_order(self, axis, slot):
        with self.lock:
            self.order[axis] = slot

    def sequence(self):
        with self.lock:
            angles = dict(self.angles_deg)
            frames = dict(self.frames)
            order = dict(self.order)
        axes_in_order = sorted(AXES, key=lambda a: order[a])
        return [(a, np.deg2rad(angles[a]), frames[a]) for a in axes_in_order]

    def compose(self):
        R = np.eye(3)
        for axis, angle, frame in self.sequence():
            R_step = ELEMENTARY_ROTATIONS[axis](angle)
            if frame == "current":
                R = R @ R_step
            else:  # "fixed"
                R = R_step @ R
        return R


def format_sequence(seq):
    parts = [f"{axis.upper()}({np.rad2deg(angle):.0f} deg, {frame})"
              for axis, angle, frame in seq]
    return "  ->  ".join(parts)


def format_matrix(R):
    rows = ["[" + "  ".join(f"{v:6.3f}" for v in row) + "]" for row in R]
    return "\n".join(rows)


class App:
    def __init__(self):
        self.model = mujoco.MjModel.from_xml_path(MODEL_PATH)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)

        self.state = RotationState()
        self.viewer_handle = mujoco.viewer.launch_passive(self.model, self.data)

        self.sliders = {}
        self.angle_labels = {}
        self.mode_vars = {}
        self.order_vars = {}
        self.playing = False

        self.root = tk.Tk()
        self.root.title("HW1 Rotation Sandbox -- live current vs fixed frame")
        self._build_axis_rows()
        self._build_play_buttons()
        self._build_readouts()

        self.on_change()  # populate initial state
        self.root.after(ANIM_STEP_MS, self._poll_viewer)

    # ---------------- UI construction ----------------

    def _build_axis_rows(self):
        for i, axis in enumerate(AXES):
            box = ttk.LabelFrame(self.root, text=f"{axis.upper()}-axis")
            box.grid(row=i, column=0, padx=10, pady=6, sticky="ew")

            angle_val_lbl = ttk.Label(box, text="0 deg", width=8)
            self.angle_labels[axis] = angle_val_lbl

            def on_slide(val, a=axis):
                deg = float(val)
                self.state.set_angle(a, deg)
                self.angle_labels[a].config(text=f"{deg:.0f} deg")
                self.on_change()

            slider = ttk.Scale(box, from_=-180, to=180, orient="horizontal",
                                length=260, command=on_slide)
            slider.set(0)
            slider.pack(side="left", padx=6, pady=6)
            angle_val_lbl.pack(side="left", padx=(0, 12))
            self.sliders[axis] = slider

            mode = tk.StringVar(value="current")
            self.mode_vars[axis] = mode

            def make_frame_setter(a, m):
                def _set():
                    self.state.set_frame(a, m.get())
                    self.on_change()
                return _set

            ttk.Radiobutton(box, text="Current", value="current", variable=mode,
                             command=make_frame_setter(axis, mode)).pack(side="left", padx=3)
            ttk.Radiobutton(box, text="Fixed", value="fixed", variable=mode,
                             command=make_frame_setter(axis, mode)).pack(side="left", padx=3)

            ttk.Label(box, text="  Order slot:").pack(side="left", padx=(12, 2))
            order_var = tk.StringVar(value=str(i + 1))
            self.order_vars[axis] = order_var

            def make_order_setter(a, v):
                def _set(*_):
                    self.state.set_order(a, int(v.get()))
                    self.on_change()
                return _set

            order_menu = ttk.Combobox(box, textvariable=order_var, values=["1", "2", "3"],
                                       width=3, state="readonly")
            order_menu.bind("<<ComboboxSelected>>", make_order_setter(axis, order_var))
            order_menu.pack(side="left", padx=2)

    def _build_play_buttons(self):
        row = ttk.Frame(self.root)
        row.grid(row=len(AXES), column=0, pady=8)

        ttk.Button(row, text="Reset all to 0",
                   command=self.reset_all).pack(side="left", padx=4)
        ttk.Button(row, text="Play Sequence 1 (current frame)",
                   command=lambda: self.play_sequence(SEQUENCE_1)).pack(side="left", padx=4)
        ttk.Button(row, text="Play Sequence 2 (fixed frame)",
                   command=lambda: self.play_sequence(SEQUENCE_2)).pack(side="left", padx=4)

        self.status_lbl = ttk.Label(self.root, text="")
        self.status_lbl.grid(row=len(AXES) + 1, column=0, padx=10, sticky="w")

    def _build_readouts(self):
        info = ttk.Label(self.root, text="Order slot = which step (1st/2nd/3rd) this axis's\n"
                                          "rotation is applied in. Give each axis a distinct slot.",
                          justify="left")
        info.grid(row=len(AXES) + 2, column=0, padx=10, pady=(4, 4), sticky="w")

        self.seq_lbl = ttk.Label(self.root, text="Sequence: ", justify="left", wraplength=480)
        self.seq_lbl.grid(row=len(AXES) + 3, column=0, padx=10, pady=(6, 2), sticky="w")

        self.matrix_lbl = ttk.Label(self.root, text="R =", font=("Courier", 10), justify="left")
        self.matrix_lbl.grid(row=len(AXES) + 4, column=0, padx=10, pady=(2, 10), sticky="w")

    # ---------------- state / viewer sync ----------------

    def on_change(self):
        seq = self.state.sequence()
        R = self.state.compose()
        set_body_orientation(self.data, R)
        mujoco.mj_forward(self.model, self.data)
        self.viewer_handle.sync()
        self.seq_lbl.config(text="Sequence: " + format_sequence(seq))
        self.matrix_lbl.config(text="R =\n" + format_matrix(R))

    def reset_all(self):
        for axis in AXES:
            self.sliders[axis].set(0)
            self.state.set_angle(axis, 0.0)
            self.angle_labels[axis].config(text="0 deg")
        self.status_lbl.config(text="")
        self.on_change()

    def _set_frame_ui(self, axis, frame):
        self.mode_vars[axis].set(frame)
        self.state.set_frame(axis, frame)

    def _set_order_ui(self, axis, slot):
        self.order_vars[axis].set(str(slot))
        self.state.set_order(axis, slot)

    # ---------------- animation ----------------

    def play_sequence(self, sequence, _step_index=0):
        """Animate the sliders (and thus the dart) through `sequence`,
        one axis at a time, each over ANIM_DURATION_MS. Non-blocking:
        schedules itself via root.after so the Tkinter/viewer loop
        keeps running smoothly."""
        if _step_index == 0:
            if self.playing:
                return  # ignore double-clicks mid-animation
            self.playing = True
            self.reset_all()
            self.status_lbl.config(text="Playing...")

        if _step_index >= len(sequence):
            self.playing = False
            self.status_lbl.config(text="Done. Final R shown below.")
            return

        axis, target_deg, frame, slot = sequence[_step_index]
        self._set_frame_ui(axis, frame)
        self._set_order_ui(axis, slot)
        start_deg = self.state.angles_deg[axis]
        n_steps = max(1, ANIM_DURATION_MS // ANIM_STEP_MS)

        def animate_step(k=0):
            frac = k / n_steps
            deg = start_deg + frac * (target_deg - start_deg)
            self.sliders[axis].set(deg)
            self.state.set_angle(axis, deg)
            self.angle_labels[axis].config(text=f"{deg:.0f} deg")
            self.on_change()
            if k < n_steps:
                self.root.after(ANIM_STEP_MS, lambda: animate_step(k + 1))
            else:
                self.root.after(150, lambda: self.play_sequence(sequence, _step_index + 1))

        animate_step()

    # ---------------- main loop glue ----------------

    def _poll_viewer(self):
        if self.viewer_handle.is_running():
            self.viewer_handle.sync()
            self.root.after(ANIM_STEP_MS, self._poll_viewer)
        else:
            self.root.destroy()

    def run(self):
        self.root.mainloop()
        if self.viewer_handle.is_running():
            self.viewer_handle.close()


def main():
    App().run()


if __name__ == "__main__":
    main()
