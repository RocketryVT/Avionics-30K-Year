r"""A window for driving the motor Pico: `py -m gs.gui`.

The same job as the `motor>` prompt, laid out so the state is visible at a
glance instead of having to be asked for. Both sit over `session.py`;
neither is privileged, and the prompt stays the better tool for a scripted
bench run.

This file is only the window. Everything that talks to the Pico is in
`gui_worker.py`, on its own thread, because a session call from a button
handler would freeze the window for up to two seconds - or a whole minute
during a move. The rule is strict and worth keeping: **this module never
touches the session, and the worker never touches a widget.** Events come
over a queue and are drained here by `after()`, on the Tk thread.

Tkinter rather than anything prettier because `pyproject.toml` means it:
pyserial is the only dependency, so a laptop at a test site needs one pip
install and nothing more. This will look plain and it will work with no
internet.

The three modes
---------------

The firmware knows nothing about "mode 2". It knows two separate things -
which gearing it is counting, and whether it is tracking - and the modes here
are those composed into the three ways this machine actually gets used:

    1 independent   gear off   each motor on its own, one axis at a time,
                               angles at the gearbox output shaft
    2 combined      gear on    both motors through the belt, both axes in
                               one command, angles at the antenna
    3 track         gear on    a target stream the firmware chases

The move panels for modes 1 and 2 each have two buttons for one box: "turn
by" reads the number as a change, the way the prompt's `move` does, and "go
to" reads it as a destination. The subtraction for "go to" happens on the
Pico, which is the only place that knows where the axis really is.

Mode 3 is the one where the motors move without anybody pressing anything,
so arming it is a separate click from entering it.

Choosing a mode changes the gearing, **which costs both axes their zero**,
so it asks first. Modes 2 and 3 are both `gear on`, so moving between them
costs nothing.

Typing commands
---------------

The commands panel lists what each command does *and when you would reach
for it* - the part that is hard to work out from a list. It comes from
`command_help.py`, which can be checked against the prompt's real command
table so the two do not drift apart.

Under it is a box to type them in. It runs the line through the prompt's own
handling (`_GuiConsole` in gui_worker.py), so a command means the same thing
in both places, and the things the buttons do not cover - `sweep`, `random`,
`true`, `note`, `rows`, `pin` - work here too. Up and down walk back through
what has been typed.

`stop` and `estop` typed in the box go the way the buttons send them,
jumping the job queue, because a stop waiting behind the move it is meant to
cut short would be no stop at all.


Four rules it holds to
----------------------

1. Nothing on screen is remembered. Every reading is what the Pico last
   said, for the same reason `scripted_runs.py` asks for the limits rather
   than keeping a copy of `motor_config.hpp`: a window showing a remembered
   position is a window that disagrees with the machine.
2. Every control is disabled while a command is outstanding, except the two
   stops and the command box. The prompt gets that for free by being a
   prompt; here it has to be done on purpose, or three fast clicks queue up
   three surprises. The box stays live so `stop` can always be typed.
3. The stops are always reachable, and they jump the queue in the worker. No
   limit switches, no encoder, and geared azimuth slews at 79 deg/s.
4. A move is refused until the axes are zeroed, so the window says so
   loudly rather than letting someone discover it from an error.
"""

from __future__ import annotations

import argparse
import queue
import sys
import time
import tkinter as tk
from tkinter import font as tkfont
from tkinter import messagebox
from tkinter import ttk

from gs import command_help
from gs import gui_worker as gw
from gs import motor
from gs import protocol as p
from gs import results_file

# How often the Tk thread looks for events from the worker. Well under the
# worker's own poll interval, so a fresh reading is shown as soon as it
# lands rather than up to a poll later.
DRAIN_MS = 100

# Lines kept in the log pane. A long bench session would otherwise grow it
# without limit, and nobody scrolls back past a few hundred.
LOG_LINES = 600

# Typed commands that must not wait their turn behind whatever is running.
# Everything else goes to the worker's job queue, where a long move is ahead
# of it; these two go to the flags the worker checks while it waits, which is
# the same path the buttons use.
TYPED_URGENT = {"stop": "stop", "estop": "estop"}

# Typed commands with nothing sensible to do in a window.
TYPED_IGNORED = {"quit", "exit"}

# How many typed lines the up arrow can walk back through.
HISTORY = 50

MODE_INDEPENDENT = 1
MODE_COMBINED = 2
MODE_TRACK = 3

# Which gearing each mode runs with. This is the whole mapping between the
# modes on screen and the one fact the firmware holds.
MODE_GEAR = {
    MODE_INDEPENDENT: False,
    MODE_COMBINED: True,
    MODE_TRACK: True,
}

MODE_NOTE = {
    MODE_INDEPENDENT: "each motor on its own - no 20:14 or 32:14 counted, "
                      "angles at the gearbox output shaft",
    MODE_COMBINED: "both motors through the belt - final pairs counted, "
                   "angles at the antenna",
    MODE_TRACK: "the Pico chases a target that keeps moving - pick a "
                "source, then arm it",
}

PAD = {"padx": 6, "pady": 3}


def _uptime(ms: int) -> str:
    if ms <= 0:
        return "-"
    seconds = ms // 1000
    if seconds < 60:
        return f"{seconds}s"
    if seconds < 3600:
        return f"{seconds // 60}m {seconds % 60:02d}s"
    return f"{seconds // 3600}h {(seconds % 3600) // 60:02d}m"


class App:
    """The window. Built once, then fed State objects by `_drain`."""

    def __init__(self, root: tk.Tk, worker: gw.MotorWorker,
                 clock=time.monotonic):
        self.root = root
        self.worker = worker
        self.clock = clock
        self._state = gw.State()

        # Every "are you sure?" goes through this, so the window can be
        # driven by a script without a modal nobody can click.
        self.confirm = messagebox.askokcancel

        # What was last written into each settings box, so an edit can be
        # told from a value that came from the Pico. See _fill.
        self._written: dict[ttk.Entry, str] = {}

        # The move in flight, for the progress bar: when it started and how
        # long the Pico predicted. None when nothing is running.
        self._move_from: float | None = None
        self._move_est_s = 0.0

        root.title("gs motor")
        root.minsize(820, 760)
        self.mono = tkfont.nametofont("TkFixedFont").copy()
        self.bold = tkfont.nametofont("TkDefaultFont").copy()
        self.bold.configure(weight="bold")

        root.columnconfigure(0, weight=3)
        root.columnconfigure(1, weight=2)
        root.rowconfigure(5, weight=1)

        self._build_connection(0)
        self._build_mode(1)
        self._build_position(2)
        self._build_settings(3)
        self._build_mode_panels(4)
        self._build_log_and_commands(5)
        self._build_stops(6)
        self._build_status(7)

        self.mode.set(MODE_INDEPENDENT)
        self._show_mode_panel(MODE_INDEPENDENT)
        self._applied_mode = MODE_INDEPENDENT

        # Escape is the ramped stop, deliberately not the emergency one: the
        # key that is easy to hit by accident should be the one that does not
        # forfeit the zero.
        root.bind("<Escape>", lambda _event: self._stop())
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(DRAIN_MS, self._drain)

    # --- building ----------------------------------------------------------

    def _frame(self, row: int, title: str) -> ttk.LabelFrame:
        frame = ttk.LabelFrame(self.root, text=title)
        frame.grid(row=row, column=0, columnspan=2, sticky="ew", **PAD)
        return frame

    def _value(self, parent, row: int, col: int, text: str = "-",
               font=None, **grid) -> ttk.Label:
        label = ttk.Label(parent, text=text, font=font or self.mono)
        label.grid(row=row, column=col, sticky="w", **PAD, **grid)
        return label

    def _entry(self, parent, row: int, col: int, value: str = "",
               width: int = 9) -> ttk.Entry:
        entry = ttk.Entry(parent, width=width, font=self.mono,
                          justify="right")
        entry.insert(0, value)
        entry.grid(row=row, column=col, sticky="w", **PAD)
        return entry

    def _build_connection(self, row: int) -> None:
        frame = self._frame(row, "connection")
        self.v_port = self._value(frame, 0, 0)
        self.v_who = self._value(frame, 0, 1)
        self.v_uptime = self._value(frame, 0, 2)
        self.v_heartbeat = self._value(frame, 0, 3)
        self.b_reconnect = ttk.Button(frame, text="reconnect",
                                      command=self._reconnect)
        self.b_reconnect.grid(row=0, column=4, sticky="e", **PAD)
        frame.columnconfigure(3, weight=1)

    def _build_mode(self, row: int) -> None:
        frame = self._frame(row, "mode")
        self.mode = tk.IntVar(value=MODE_INDEPENDENT)
        self.r_mode = {}
        for col, (value, text) in enumerate((
            (MODE_INDEPENDENT, "1  independent"),
            (MODE_COMBINED, "2  combined"),
            (MODE_TRACK, "3  track"),
        )):
            button = ttk.Radiobutton(frame, text=text, value=value,
                                     variable=self.mode,
                                     command=self._mode_chosen)
            button.grid(row=0, column=col, sticky="w", **PAD)
            self.r_mode[value] = button

        self.v_gear = ttk.Label(frame, text="-", font=self.bold)
        self.v_gear.grid(row=1, column=0, columnspan=3, sticky="w", **PAD)
        self.v_mode_note = self._value(frame, 2, 0, columnspan=3)
        self.v_steps = self._value(frame, 3, 0, columnspan=3)

    def _build_position(self, row: int) -> None:
        frame = self._frame(row, "position")
        self._value(frame, 0, 0, "az", font=self.bold)
        self._value(frame, 1, 0, "el", font=self.bold)
        self.v_az = self._value(frame, 0, 1)
        self.v_el = self._value(frame, 1, 1)
        self.v_zeroed = ttk.Label(frame, text="-", font=self.bold)
        self.v_zeroed.grid(row=0, column=2, rowspan=2, sticky="w", **PAD)

        zero = ttk.Frame(frame)
        zero.grid(row=0, column=3, rowspan=2, sticky="e", **PAD)
        ttk.Label(zero, text="is now").grid(row=0, column=0, padx=(0, 4))
        self.e_zero = self._entry(zero, 0, 1, "0")
        ttk.Label(zero, text="deg:").grid(row=0, column=2, padx=(0, 4))
        self.b_zero_all = ttk.Button(zero, text="zero both", width=10,
                                     command=lambda: self._zero("ALL"))
        self.b_zero_all.grid(row=0, column=3, padx=2)
        self.b_zero_az = ttk.Button(zero, text="az", width=4,
                                    command=lambda: self._zero("AZ"))
        self.b_zero_az.grid(row=0, column=4, padx=2)
        self.b_zero_el = ttk.Button(zero, text="el", width=4,
                                    command=lambda: self._zero("EL"))
        self.b_zero_el.grid(row=0, column=5, padx=2)
        frame.columnconfigure(3, weight=1)

    def _build_settings(self, row: int) -> None:
        frame = self._frame(row, "settings")
        for col, head in enumerate(
                ("", "accel deg/s/s", "travel min", "travel max",
                 "deadband deg", "speed deg/s")):
            self._value(frame, 0, col, head, font=self.bold)

        self.e_accel: dict[str, ttk.Entry] = {}
        self.e_min: dict[str, ttk.Entry] = {}
        self.e_max: dict[str, ttk.Entry] = {}
        self.e_dead: dict[str, ttk.Entry] = {}
        self.v_speed: dict[str, ttk.Label] = {}
        for line, axis in ((1, "az"), (2, "el")):
            self._value(frame, line, 0, axis, font=self.bold)
            self.e_accel[axis] = self._entry(frame, line, 1)
            self.e_min[axis] = self._entry(frame, line, 2)
            self.e_max[axis] = self._entry(frame, line, 3)
            # Only bites while tracking; harmless to set at any time.
            self.e_dead[axis] = self._entry(frame, line, 4)
            self.v_speed[axis] = self._value(frame, line, 5)

        self.v_saved = self._value(frame, 3, 0, "", columnspan=4)

        buttons = ttk.Frame(frame)
        buttons.grid(row=3, column=4, columnspan=2, sticky="e", **PAD)
        self.b_apply = ttk.Button(buttons, text="apply",
                                  command=self._apply_settings)
        self.b_apply.grid(row=0, column=0, padx=2)
        self.b_save = ttk.Button(buttons, text="save",
                                 command=self.worker.save)
        self.b_save.grid(row=0, column=1, padx=2)
        self.b_defaults = ttk.Button(buttons, text="defaults",
                                     command=self.worker.defaults)
        self.b_defaults.grid(row=0, column=2, padx=2)
        frame.columnconfigure(5, weight=1)

    def _build_mode_panels(self, row: int) -> None:
        holder = self._frame(row, "move")
        holder.columnconfigure(0, weight=1)
        self.panels: dict[int, ttk.Frame] = {}

        # --- mode 1: one axis at a time ---
        one = ttk.Frame(holder)
        self.panels[MODE_INDEPENDENT] = one
        self.m1_axis = tk.StringVar(value="AZ")
        for col, axis in enumerate(("AZ", "EL")):
            ttk.Radiobutton(one, text=axis.lower(), value=axis,
                            variable=self.m1_axis).grid(
                row=0, column=col, sticky="w", **PAD)
        ttk.Label(one, text="at").grid(row=0, column=2, padx=(10, 2))
        self.m1_speed = self._entry(one, 0, 3, "5")
        ttk.Label(one, text="deg/s,").grid(row=0, column=4, padx=2)
        self.m1_angle = self._entry(one, 0, 5, "45")
        ttk.Label(one, text="deg").grid(row=0, column=6, padx=2)
        # Two buttons, one box. "turn by" is the bench move that has always
        # been here; "go to" reads the same number as a destination.
        self.b_move_one = ttk.Button(one, text="turn by", width=9,
                                     command=self._move_one)
        self.b_move_one.grid(row=0, column=7, padx=(12, 2))
        self.b_goto_one = ttk.Button(one, text="go to", width=9,
                                     command=lambda: self._move_one(True))
        self.b_goto_one.grid(row=0, column=8, padx=2)

        # --- mode 2: both axes, each at its own speed ---
        both = ttk.Frame(holder)
        self.panels[MODE_COMBINED] = both
        self.m2_speed: dict[str, ttk.Entry] = {}
        self.m2_angle: dict[str, ttk.Entry] = {}
        for line, axis in ((0, "az"), (1, "el")):
            ttk.Label(both, text=axis).grid(row=line, column=0, **PAD)
            ttk.Label(both, text="at").grid(row=line, column=1)
            self.m2_speed[axis] = self._entry(both, line, 2, "5")
            ttk.Label(both, text="deg/s,").grid(row=line, column=3)
            self.m2_angle[axis] = self._entry(both, line, 4,
                                              "45" if axis == "az" else "20")
            ttk.Label(both, text="deg").grid(row=line, column=5)
        self.b_move_both = ttk.Button(both, text="turn by", width=9,
                                      command=self._move_both)
        self.b_move_both.grid(row=0, column=6, rowspan=2, padx=(12, 2))
        self.b_goto_both = ttk.Button(both, text="go to", width=9,
                                      command=lambda: self._move_both(True))
        self.b_goto_both.grid(row=0, column=7, rowspan=2, padx=2)

        # --- mode 3: a target stream the Pico chases ---
        track = ttk.Frame(holder)
        self.panels[MODE_TRACK] = track

        picker = ttk.Frame(track)
        picker.grid(row=0, column=0, columnspan=4, sticky="w")
        self.m3_source = tk.StringVar(value="manual")
        for col, (value, text) in enumerate((
            ("manual", "a direction I type"),
            ("sim", "a simulated flight"),
        )):
            ttk.Radiobutton(picker, text=text, value=value,
                            variable=self.m3_source,
                            command=self._source_chosen).grid(
                row=0, column=col, sticky="w", **PAD)
        ttk.Label(picker, text="at").grid(row=0, column=2, padx=(12, 2))
        self.m3_hz = self._entry(picker, 0, 3, "5", width=5)
        ttk.Label(picker, text="targets/sec").grid(row=0, column=4, padx=2)

        aim = ttk.Frame(track)
        aim.grid(row=1, column=0, columnspan=4, sticky="w")
        ttk.Label(aim, text="point at  az").grid(row=0, column=0, **PAD)
        self.m3_az = self._entry(aim, 0, 1, "0")
        ttk.Label(aim, text="el").grid(row=0, column=2, padx=2)
        self.m3_el = self._entry(aim, 0, 3, "45")
        ttk.Label(aim, text="deg").grid(row=0, column=4, padx=2)
        self.b_point = ttk.Button(aim, text="point here", width=11,
                                  command=self._point)
        self.b_point.grid(row=0, column=5, padx=(12, 2))

        arm = ttk.Frame(track)
        arm.grid(row=2, column=0, columnspan=4, sticky="w")
        self.b_arm = ttk.Button(arm, text="ARM tracking", width=14,
                                command=self._arm)
        self.b_arm.grid(row=0, column=0, **PAD)
        self.b_disarm = ttk.Button(arm, text="disarm", width=10,
                                   command=self._disarm)
        self.b_disarm.grid(row=0, column=1, **PAD)
        self.v_arm_note = self._value(arm, 0, 2, "")

        # The readout. Three numbers per axis, because the interesting thing
        # while tracking is not where it is but how far behind it is.
        board = ttk.Frame(track)
        board.grid(row=3, column=0, columnspan=4, sticky="w", pady=(4, 0))
        for col, head in enumerate(("", "target", "actual", "off by",
                                    "turning at")):
            self._value(board, 0, col, head, font=self.bold)
        self.v_trk = {}
        for line, axis in ((1, "az"), (2, "el")):
            self._value(board, line, 0, axis, font=self.bold)
            self.v_trk[axis] = [self._value(board, line, col)
                                for col in (1, 2, 3, 4)]

        self.progress = ttk.Progressbar(holder, mode="determinate",
                                        maximum=1000)
        self.progress.grid(row=1, column=0, sticky="ew", **PAD)
        self.v_progress = self._value(holder, 2, 0, "")

    def _build_log_and_commands(self, row: int) -> None:
        log = ttk.LabelFrame(self.root, text="conversation")
        log.grid(row=row, column=0, sticky="nsew", **PAD)
        log.rowconfigure(0, weight=1)
        log.columnconfigure(0, weight=1)

        self.log = tk.Text(log, height=12, width=52, font=self.mono,
                           wrap="none", state="disabled")
        self.log.grid(row=0, column=0, sticky="nsew")
        bar = ttk.Scrollbar(log, orient="vertical", command=self.log.yview)
        bar.grid(row=0, column=1, sticky="ns")
        self.log.configure(yscrollcommand=bar.set)
        # Sent and received are told apart by colour as well as by the
        # marker, because at a bench the pane is read at a glance.
        self.log.tag_configure("out", foreground="#0a5")
        self.log.tag_configure("in", foreground="#222")
        self.log.tag_configure("note", foreground="#06c")
        # A line somebody typed, and the prompt's answer to it.
        self.log.tag_configure("typed", foreground="#a0a")
        self.log.tag_configure("said", foreground="#555")

        commands = ttk.LabelFrame(self.root,
                                  text="commands, and when to use them")
        commands.grid(row=row, column=1, sticky="nsew", **PAD)
        commands.rowconfigure(0, weight=1)
        commands.columnconfigure(0, weight=1)

        # From command_help.py, which can be checked against the prompt's
        # real command table, so this panel lists the commands that exist.
        text = tk.Text(commands, height=12, width=52, font=self.mono,
                       wrap="word", state="normal")
        text.insert("1.0", command_help.text())
        text.configure(state="disabled")
        text.grid(row=0, column=0, sticky="nsew")
        cbar = ttk.Scrollbar(commands, orient="vertical", command=text.yview)
        cbar.grid(row=0, column=1, sticky="ns")
        text.configure(yscrollcommand=cbar.set)

        # And somewhere to type them. Under the reference rather than beside
        # it, so it reads as "here is what they do, here is where they go".
        typing = ttk.Frame(commands)
        typing.grid(row=1, column=0, columnspan=2, sticky="ew", **PAD)
        typing.columnconfigure(1, weight=1)
        ttk.Label(typing, text="motor>", font=self.mono).grid(
            row=0, column=0, padx=(0, 4))
        self.e_cmd = ttk.Entry(typing, font=self.mono)
        self.e_cmd.grid(row=0, column=1, sticky="ew")
        self.e_cmd.bind("<Return>", self._typed)
        self.e_cmd.bind("<Up>", lambda _e: self._recall(-1))
        self.e_cmd.bind("<Down>", lambda _e: self._recall(1))
        ttk.Label(typing, text="same as the prompt. The STOP button is the "
                               "one that jumps the queue.").grid(
            row=1, column=0, columnspan=2, sticky="w", pady=(2, 0))

        self._history: list[str] = []
        self._recall_at = 0

    def _build_stops(self, row: int) -> None:
        frame = ttk.Frame(self.root)
        frame.grid(row=row, column=0, columnspan=2, sticky="ew", padx=6)
        frame.columnconfigure(0, weight=1)
        frame.columnconfigure(1, weight=1)

        # Classic tk rather than ttk so the colour actually applies, and
        # never disabled: these two are the way out of anything the other
        # buttons started.
        self.b_stop = tk.Button(frame, text="STOP  (Esc)", height=2,
                                command=self._stop)
        self.b_stop.grid(row=0, column=0, sticky="ew", padx=(0, 3))
        self.b_estop = tk.Button(frame, text="E-STOP", height=2,
                                 bg="#c00", fg="white",
                                 activebackground="#e00",
                                 activeforeground="white",
                                 command=self._estop)
        self.b_estop.grid(row=0, column=1, sticky="ew", padx=(3, 0))

    def _build_status(self, row: int) -> None:
        self.v_status = ttk.Label(self.root, text="starting...", anchor="w",
                                  relief="sunken")
        self.v_status.grid(row=row, column=0, columnspan=2, sticky="ew",
                           padx=6, pady=(4, 6))

    # --- commands ----------------------------------------------------------

    def _typed(self, _event=None) -> None:
        """Run the line in the box, as the prompt would."""
        text = self.e_cmd.get().strip()
        self.e_cmd.delete(0, "end")
        if not text:
            return

        if not self._history or self._history[-1] != text:
            self._history.append(text)
            del self._history[:-HISTORY]
        self._recall_at = len(self._history)

        self.append("$", text)

        word = text.split()[0].lower()
        if word in TYPED_URGENT:
            # Sent the same way the buttons send it, so it is not stuck
            # behind a move that is still running.
            getattr(self.worker, TYPED_URGENT[word])()
            return
        if word in TYPED_IGNORED:
            self.note("close the window instead")
            return
        self.worker.run_line(text)

    def _recall(self, step: int) -> None:
        """Up and down through what has been typed."""
        if not self._history:
            return
        self._recall_at = max(0, min(len(self._history),
                                     self._recall_at + step))
        self.e_cmd.delete(0, "end")
        if self._recall_at < len(self._history):
            self.e_cmd.insert(0, self._history[self._recall_at])

    def _reconnect(self) -> None:
        self.note("reconnecting...")
        self.worker.reconnect()

    def _stop(self) -> None:
        self.worker.stop()

    def _estop(self) -> None:
        self.worker.estop()

    def _read(self, entry: ttk.Entry, what: str) -> float:
        # motor.py's own parser, so a number typed here means exactly what it
        # would mean at the prompt. It raises UsageError, caught by callers.
        return motor._number(entry.get().strip(), what)

    def _zero(self, axis: str) -> None:
        try:
            text = self.e_zero.get().strip()
            deg = self._read(self.e_zero, "angle") if text else None
        except motor.UsageError as exc:
            self.note(str(exc))
            return
        self.worker.zero(axis, deg)

    def _move_one(self, absolute: bool = False) -> None:
        try:
            speed = self._read(self.m1_speed, "speed")
            angle = self._read(self.m1_angle, "angle")
        except motor.UsageError as exc:
            self.note(str(exc))
            return
        self.worker.move(self.m1_axis.get(), speed, angle,
                         absolute=absolute)

    def _move_both(self, absolute: bool = False) -> None:
        try:
            values = {
                axis: (self._read(self.m2_speed[axis], f"{axis} speed"),
                       self._read(self.m2_angle[axis], f"{axis} angle"))
                for axis in ("az", "el")
            }
        except motor.UsageError as exc:
            self.note(str(exc))
            return
        self.worker.move_both(*values["az"], *values["el"],
                              absolute=absolute)

    # --- mode 3 ------------------------------------------------------------

    def _source_chosen(self) -> None:
        # Changing the source mid-flight would mean two of them fighting, so
        # it only takes effect next time tracking is armed.
        if self._state.tracking:
            self.note("the source changes next time you arm")
        self._refresh_enabled(self._state)

    def _arm(self) -> None:
        try:
            hz = self._read(self.m3_hz, "targets/sec")
            az = self._read(self.m3_az, "azimuth")
            el = self._read(self.m3_el, "elevation")
        except motor.UsageError as exc:
            self.note(str(exc))
            return
        if hz <= 0:
            self.note("targets/sec must be more than zero")
            return

        kind = self.m3_source.get()
        following = ("the direction you typed" if kind == "manual"
                     else "a simulated flight")
        if not self.confirm(
            "Arm tracking?",
            f"The motors will start moving on their own, following "
            f"{following}"
            f" at {hz:g} targets a second.\n\n"
            "STOP and E-STOP stay live. The Pico also gives up by itself if "
            "targets stop reaching it.",
        ):
            return
        self.worker.start_tracking(kind, hz, az, el)

    def _disarm(self) -> None:
        self.worker.stop_tracking()

    def _point(self) -> None:
        try:
            az = self._read(self.m3_az, "azimuth")
            el = self._read(self.m3_el, "elevation")
        except motor.UsageError as exc:
            self.note(str(exc))
            return
        self.worker.point_at(az, el)

    def _show_tracking(self, st: gw.State) -> None:
        sample = st.sample
        if not st.tracking and sample is None:
            self.v_arm_note.configure(text="not armed - nothing will move")
            for cells in self.v_trk.values():
                for cell in cells:
                    cell.configure(text="-")
            return

        trk = getattr(sample, "trk", None)
        if trk is None:
            self.v_arm_note.configure(text="armed, waiting for a report")
            return

        for axis, cells in self.v_trk.items():
            target = getattr(trk, f"{axis}_target_mdeg")
            actual = getattr(trk, f"{axis}_pos_mdeg")
            error = getattr(trk, f"{axis}_error_mdeg")
            rate = getattr(trk, f"{axis}_mdps")
            for cell, value, unit in zip(cells,
                                         (target, actual, error, rate),
                                         ("deg", "deg", "deg", "deg/s")):
                cell.configure(text=f"{p.format_mdeg(value):>10} {unit}")

        note = f"{st.targets_sent} targets sent"
        if getattr(sample, "clamped", False):
            # Said plainly, because the error will not close and the reason
            # is not the antenna's fault.
            note += "   OUT OF REACH - held at a travel limit"
        self.v_arm_note.configure(text=note)

    def _apply_settings(self) -> None:
        """Send only what differs from what the Pico reports.

        Comparing first keeps `apply` from sending six commands every time,
        and keeps it from marking the settings unsaved when nothing changed.
        """
        cfg = self._state.cfg
        if cfg is None:
            self.note("no settings read yet")
            return

        changes = []
        for axis in ("az", "el"):
            for name, entry, attr in (
                ("ACCEL", self.e_accel[axis], f"{axis}_accel_mdps2"),
                ("MIN", self.e_min[axis], f"{axis}_min_mdeg"),
                ("MAX", self.e_max[axis], f"{axis}_max_mdeg"),
                ("DEADBAND", self.e_dead[axis], f"{axis}_deadband_mdeg"),
            ):
                try:
                    value = self._read(entry, f"{axis} {name.lower()}")
                except motor.UsageError as exc:
                    self.note(str(exc))
                    return
                if p.deg_to_mdeg(value) != getattr(cfg, attr):
                    changes.append((axis.upper(), name, value))

        if not changes:
            self.note("settings already match - nothing to send")
            return
        for axis, name, value in changes:
            self.worker.set_setting(axis, name, value)

    # --- modes -------------------------------------------------------------

    def _mode_chosen(self) -> None:
        """A mode button was clicked. Change the gearing if it has to."""
        wanted = self.mode.get()
        gear_wanted = MODE_GEAR[wanted]
        state = self._state

        # Already in the right gearing - or we have not been told yet, in
        # which case asking the Pico is the only way to find out and the
        # worker no-ops if it already matches.
        known = state.gear is not None
        if known and state.gear.geared == gear_wanted:
            self._applied_mode = wanted
            self._show_mode_panel(wanted)
            self._refresh_enabled(state)
            return

        word = "on" if gear_wanted else "off"
        if not self.confirm(
            "Change the gearing?",
            f"Mode {wanted} runs with gear {word}.\n\n"
            "Changing the gearing costs BOTH axes their zero, because the "
            "drive train is physically different. You will have to zero "
            "again before anything will move.",
        ):
            # Put the button back where it was rather than leaving it
            # showing a mode that was not entered.
            self.mode.set(self._applied_mode)
            return

        self._applied_mode = wanted
        self._show_mode_panel(wanted)
        self.worker.set_gear(gear_wanted)

    def _show_mode_panel(self, mode: int) -> None:
        for value, panel in self.panels.items():
            if value == mode:
                panel.grid(row=0, column=0, sticky="ew")
            else:
                panel.grid_remove()
        self.v_mode_note.configure(text=MODE_NOTE[mode])

    # --- events ------------------------------------------------------------

    def _drain(self) -> None:
        """Take everything the worker has said and show it. Tk thread only."""
        try:
            while True:
                kind, payload = self.worker.events.get_nowait()
                if kind == gw.EVENT_STATE:
                    self.show(payload)
                elif kind == gw.EVENT_LINE:
                    self.append(*payload)
                elif kind == gw.EVENT_NOTE:
                    self.note(payload)
                elif kind == gw.EVENT_MOVE:
                    self._move_began(*payload)
                elif kind == gw.EVENT_DONE:
                    self._move_ended(*payload)
        except queue.Empty:
            pass  # everything the worker said has been shown

        # These two tick on even when no event arrives, which is exactly when
        # they matter: a Pico that stopped talking, and a move in progress.
        self._show_heartbeat(self._state)
        self._show_progress()
        self.root.after(DRAIN_MS, self._drain)

    def _move_began(self, plans, narrate: bool = True) -> None:
        self._move_from = self.clock()
        self._move_est_s = max(plan.est_s for plan in plans)
        # A typed move has already had its plan printed by the prompt's own
        # handling; only the progress bar is wanted here.
        if not narrate:
            return
        for plan in plans:
            short = ("  (too short to reach that speed)" if plan.triangular
                     else "")
            self.note(
                f"plan {plan.axis.lower()} {plan.steps:+d} steps  "
                f"{p.format_mdeg(plan.from_mdeg)} -> "
                f"{p.format_mdeg(plan.to_mdeg)} deg at "
                f"{p.format_mdeg(plan.speed_mdps)} deg/s{short}"
            )

    def _move_ended(self, outcomes, narrate: bool = True) -> None:
        self._move_from = None
        self.progress.configure(value=0)
        self.v_progress.configure(text="")
        if not narrate:
            return
        for outcome in outcomes:
            done = outcome.done
            self.note(
                f"done {done.axis.lower()} "
                f"{'finished' if done.ok else 'STOPPED SHORT'}  "
                f"{done.steps_done:+d} steps in {done.elapsed_s:.2f} s "
                f"(predicted {outcome.plan.est_s:.2f} s), now at "
                f"{p.format_mdeg(done.to_mdeg)} deg"
            )
        self.v_progress.configure(text="")

    def _show_progress(self) -> None:
        if self._move_from is None:
            return
        gone = self.clock() - self._move_from
        est = self._move_est_s
        fraction = 1.0 if est <= 0 else min(gone / est, 1.0)
        self.progress.configure(value=int(fraction * 1000))
        # Said to be an estimate, because it is the Pico's prediction rather
        # than anything measured - and a move that is stopping runs over it.
        self.v_progress.configure(
            text=f"{gone:4.1f} s of about {est:.1f} s predicted")

    def append(self, direction: str, text: str) -> None:
        tag = {">": "out", "<": "in", "$": "typed",
               "|": "said"}.get(direction, "note")
        self.log.configure(state="normal")
        self.log.insert("end", f"{direction} {text}\n", tag)
        extra = int(self.log.index("end-1c").split(".")[0]) - LOG_LINES
        if extra > 0:
            self.log.delete("1.0", f"{extra + 1}.0")
        self.log.see("end")
        self.log.configure(state="disabled")

    def note(self, text: str) -> None:
        self.append(" ", text)

    # --- showing state -----------------------------------------------------

    def _show_heartbeat(self, st: gw.State) -> None:
        if st.heartbeat_at is None:
            self.v_heartbeat.configure(text="heartbeat -")
            return
        age = self.clock() - st.heartbeat_at
        self.v_heartbeat.configure(text=f"heartbeat {age:.0f}s ago")

    def show(self, st: gw.State) -> None:
        """Put a State on the screen, and keep it as the current one.

        Displayed values are never remembered across calls - what is on
        screen is what the Pico last said. The State itself is kept because
        the controls need it: `apply` compares against it to work out what
        actually changed, and the mode buttons need to know the gearing.
        """
        self._state = st
        self.v_port.configure(text=st.port or "no port")
        self.v_who.configure(text=f"{st.target} {st.version}".strip() or "-")
        self.v_uptime.configure(text=f"up {_uptime(st.uptime_ms)}")
        self._show_heartbeat(st)

        if st.gear is None:
            self.v_gear.configure(text="gearing unknown")
            self.v_steps.configure(text="")
        else:
            on = st.gear.geared
            self.v_gear.configure(
                text="GEAR ON - final pair counted" if on
                else "GEAR OFF - gearbox only")
            self.v_steps.configure(
                text=f"az {st.gear.az_steps_per_deg:8.3f} steps/deg"
                     f"    el {st.gear.el_steps_per_deg:8.3f} steps/deg")

        if st.pos is None:
            self.v_az.configure(text="-")
            self.v_el.configure(text="-")
            self.v_zeroed.configure(text="")
        else:
            self.v_az.configure(
                text=f"{p.format_mdeg(st.pos.az_mdeg):>12} deg"
                     f"  {st.pos.az_steps:>8} steps")
            self.v_el.configure(
                text=f"{p.format_mdeg(st.pos.el_mdeg):>12} deg"
                     f"  {st.pos.el_steps:>8} steps")
            # Loud, because every move is refused until it is set and that
            # is the first thing anybody trips over.
            self.v_zeroed.configure(
                text="zeroed" if st.pos.zeroed else "NOT ZEROED",
                foreground="#0a5" if st.pos.zeroed else "#c00")

        self._show_settings(st)
        self._show_tracking(st)
        self._refresh_enabled(st)

        status = st.describe()
        if st.moving:
            status += " - readings above are from before the move started"
        self.v_status.configure(text=status)

    def _show_settings(self, st: gw.State) -> None:
        for axis in ("az", "el"):
            if st.cfg is not None:
                for boxes, name in ((self.e_accel, "accel_mdps2"),
                                    (self.e_min, "min_mdeg"),
                                    (self.e_max, "max_mdeg"),
                                    (self.e_dead, "deadband_mdeg")):
                    value = getattr(st.cfg, f"{axis}_{name}")
                    self._fill(boxes[axis], p.format_mdeg(value))
            speed = "-"
            if st.lim is not None:
                speed = (f"{p.format_mdeg(st.lim.speed_min_mdps)}"
                         f" .. {p.format_mdeg(st.lim.speed_max_for(axis))}")
            self.v_speed[axis].configure(text=speed)

        if st.cfg is None:
            self.v_saved.configure(text="")
        elif not st.cfg.saved:
            self.v_saved.configure(
                text="not saved - a restart would lose these")
        elif st.cfg.built_in:
            self.v_saved.configure(
                text="the built-in numbers for this gear mode - "
                     "a restart comes back here")
        else:
            self.v_saved.configure(text="saved - a restart comes back here")

    def _fill(self, entry: ttk.Entry, text: str) -> None:
        """Refresh a settings box without throwing away an edit.

        Polling twice a second would otherwise eat half a number as it was
        being typed. The rule:

          - if the Pico's value has moved since this box was last filled,
            show the new truth - an `apply` took effect, or `defaults`, or
            somebody changed it from the prompt;
          - otherwise, if the box no longer holds what was written into it,
            it is being edited, so leave it;
          - otherwise there is nothing to do.

        An abandoned edit therefore stays on screen until it is applied or
        the Pico's own value changes. Losing what someone typed is the worse
        of the two, and the box next to it still shows what is in force.
        """
        written = self._written.get(entry)
        if written is not None and written == text:
            return  # the Pico has not moved; leave any edit alone
        if written is not None and entry.get().strip() != written:
            # Being edited, and the Pico has moved: the edit still wins, but
            # record the new truth so this does not fire again every poll.
            self._written[entry] = text
            return
        self._written[entry] = text
        if entry.get().strip() == text:
            return
        entry.delete(0, "end")
        entry.insert(0, text)

    def _refresh_enabled(self, st: gw.State) -> None:
        """Rule 2: nothing but the stops while a command is outstanding."""
        settled = st.connected and not st.working and not st.moving
        can_move = settled and st.zeroed and not st.rebooted

        for widget in (self.b_zero_all, self.b_zero_az, self.b_zero_el,
                       self.b_apply, self.b_save, self.b_defaults):
            widget.state(["!disabled"] if settled else ["disabled"])

        for widget in (self.b_move_one, self.b_move_both,
                       self.b_goto_one, self.b_goto_both):
            widget.state(["!disabled"] if can_move else ["disabled"])

        # No switching modes mid-flight: `settled` is false while tracking,
        # which is what stops it.
        for button in self.r_mode.values():
            button.state(["!disabled"] if settled else ["disabled"])

        # Arming needs a zero and a settled machine; disarming must always be
        # available, like the stops.
        self.b_arm.state(["!disabled"] if can_move and not st.tracking
                         else ["disabled"])
        self.b_disarm.state(["!disabled"] if st.tracking else ["disabled"])
        # The direction boxes stay editable whenever the source is manual,
        # armed or not: you type where to point and THEN arm. (A disabled
        # ttk.Entry refuses even a programmatic write, so getting this the
        # wrong way round locks the operator out of their own target.)
        manual = self.m3_source.get() == "manual"
        for widget in (self.m3_az, self.m3_el):
            widget.state(["!disabled"] if manual else ["disabled"])
        # Sending one, though, needs something armed to send it to.
        self.b_point.state(["!disabled"] if manual and st.tracking
                           else ["disabled"])

        self.b_reconnect.state(
            ["disabled"] if (st.working or st.moving) else ["!disabled"])

    def close(self) -> None:
        self.worker.shutdown()
        self.worker.join(timeout=2.0)
        self.root.destroy()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="py -m gs.gui",
        description="Drive the ground station's motor Pico from a window.",
    )
    parser.add_argument(
        "--port",
        help="serial port, such as COM5. Left out, the Pico is found by "
        "itself",
    )
    parser.add_argument(
        "--log",
        nargs="?",
        const="",
        metavar="FILE",
        help="open a results file, so the typed `true`, `note` and `rows` "
        "have somewhere to write. Give --log with no name for a dated one "
        "in logs/",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    results = None
    if args.log is not None:
        path = args.log or results_file.default_path()
        results = results_file.ResultsFile(path=path)
        results.write()  # so the file exists from the start
        print(f"logging to {path}")

    # The worker opens the port itself, on its own thread, so a Pico that is
    # not plugged in leaves a window saying so rather than no window at all.
    worker = gw.MotorWorker(port=args.port, results=results)
    worker.start()

    root = tk.Tk()
    app = App(root, worker)
    try:
        root.mainloop()
    finally:
        worker.shutdown()
        worker.join(timeout=2.0)
    del app
    return 0


if __name__ == "__main__":
    sys.exit(main())
