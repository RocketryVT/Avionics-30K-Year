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

Always geared, always ready to track
------------------------------------

There are no modes to choose between. The firmware knows two separate
things - which gearing it is counting, and whether it is tracking - and this
window fixes the first: gear on, both motors through the belt, every angle
at the antenna. That is how the assembled antenna is driven, and tracking
needs it. A single motor on the bench with gear off is a job for the
`motor>` prompt.

So it is one screen: the position with its zero and a manual move under it,
then the tracking panel, then the settings.

The move row has two buttons for one set of boxes: "turn by" reads each
angle as a change, the way the prompt's `move` does, and "go to" reads it as
a destination. The subtraction for "go to" happens on the Pico, which is the
only place that knows where the axis really is. A blank angle keeps that
axis still. The Pico refuses a two-axis move outright if either half rounds
to no steps, so the window sends only the axes that are to move.

Tracking is where the motors move without anybody pressing anything, so
arming it is its own click, with an "are you sure?".

Its sources come in two kinds. Three are worked out here and sent as
targets. The other two are worked out on the Pico and nothing is sent: the
flight built into its firmware, and the rocket itself - GPS positions the
ground Pico receives over the radio and turns into pointing on its own.
For that one the window shows the radio link on a line of the connection
panel (from the Pico's once-a-second RX line, see `radio.py`), and when the
Pico's watchdog drops a radio session it offers a one-click re-arm, because
the Pico never re-arms by itself.

Keeping it geared is `_keep_geared`'s job. A gear change costs both axes
their zero, because the drive train is physically different. So whenever
the Pico reports gear off with nothing zeroed - it booted that way, or
somebody typed `gear off` - the window switches it straight back on, which
costs nothing. With the axes zeroed, gear off is left alone, because
spending somebody's zero unasked would undo their work. It is said in red
under the position instead, and ARM tracking offers the switch rather than
arming.

With no Pico plugged in
-----------------------

The window opens and stays usable without one: the boxes can be filled in
and the tracking source chosen, so it can be set up first. Readings show as
blank rather than remembered. Any button that needs the Pico says "not
connected" instead of doing anything (see `_offline`). The worker keeps
looking for the Pico every couple of seconds, so plugging it in is all it
takes, and its gearing is checked as soon as it answers.

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

The startup guide
-----------------

The "startup guide" button on the connection line opens the steps in
`startup_guide.py` in a window of their own: from plugging in the Pico to
arming on the rocket, with what this window should show after each one. It
needs no Pico, so it is never disabled - it is most wanted before anything
is connected.


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
from gs import radio
from gs import results_file
from gs import startup_guide

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

# Said beside the move buttons whenever no move is running.
MOVE_HINT = "a blank angle keeps that axis still"

# The tracking sources the Pico steers itself, so the laptop sends no targets
# and the targets/sec box does not apply.
PICO_SOURCES = {"radio", "replay"}

# How long radio tracking runs before the window will say packets are not
# steering. RX lines come once a second, so the first target to land needs
# a couple of them to show up as a rate.
RADIO_SETTLE_S = 3.0

PAD = {"padx": 6, "pady": 3}

WINDOW_TITLE = "gs motor"

# The palette. Plain is what a bench window is for, but plain need not mean
# grey on grey: colour is how a reading gets found without being read, which
# is the whole job of the zero, the stops and the status line.
#
# ttk only honours these under the "clam" theme, which is why _build_style
# leaves the native one behind. See the note there.
INK = "#1b2026"        # ordinary text
MUTED = "#6b7580"      # units, hints, anything read second
EDGE = "#ccd3da"       # panel borders
CANVAS = "#eef1f4"     # the window behind the panels
CARD = "#fbfcfd"       # the panels themselves
GOOD = "#0a7a46"       # zeroed, connected, finished
WARN = "#b45309"       # something is needed before anything will move
BAD = "#c0271a"        # not connected, not zeroed, stopped short
BUSY = "#1f6feb"       # something is happening right now
TYPED_INK = "#7a4bbf"  # a line somebody typed, in the conversation pane
QUOTE_BG = "#eef2f6"   # what the window should show, in the startup guide

# How far the startup guide's text sits in from its step titles, in pixels.
GUIDE_INDENT = 22

# The radio panel's headline colour for each level radio.py reports.
LINK_COLOUR = {
    radio.LEVEL_NONE: MUTED,
    radio.LEVEL_GOOD: GOOD,
    radio.LEVEL_WARN: WARN,
    radio.LEVEL_BAD: BAD,
}


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

        # The startup guide's window while it is open, so the button brings
        # it forward rather than opening a second one.
        self._guide: tk.Toplevel | None = None
        self._guide_title_font: tkfont.Font | None = None

        # True once gear on has been asked for, until the Pico reports it -
        # so one gear-off reading sends one GEAR, not one per poll while the
        # job waits its turn. See _keep_geared.
        self._gear_sent = False

        root.title(WINDOW_TITLE)
        # The smallest it can be dragged to, not the size it opens at - that
        # is worked out from the layout at the end of this method.
        root.minsize(820, 620)
        self.mono = tkfont.nametofont("TkFixedFont").copy()
        self.bold = tkfont.nametofont("TkDefaultFont").copy()
        self.bold.configure(weight="bold")
        self.heading = tkfont.nametofont("TkDefaultFont").copy()
        self.heading.configure(weight="bold")
        # The two position readings get the extra points, because they are
        # what gets read from the other side of a bench. A Tk font size is
        # points when positive and pixels when negative, and the two count in
        # opposite directions, hence the sign.
        self.big = self.mono.copy()
        size = self.mono.cget("size")
        self.big.configure(size=size + (3 if size > 0 else -3),
                           weight="bold")

        # What the title bar already says. Not a remembered reading - the
        # port comes from the State every time - it only keeps Tk from being
        # told twice a second what it already knows.
        self._title = WINDOW_TITLE

        self._build_style()

        root.columnconfigure(0, weight=3)
        root.columnconfigure(1, weight=2)
        root.rowconfigure(4, weight=1)

        self._build_connection(0)
        self._build_position(1)
        self._build_track(2)
        self._build_settings(3)
        self._build_log_and_commands(4)
        self._build_stops(5)
        self._build_status(6)
        self._show_source_rows()

        # Never bigger than the desktop. A window whose bottom edge is off
        # the screen is a window with the STOP buttons off the screen, and
        # rule 3 says those are always reachable. A ceiling rather than a
        # fixed size, so the window still sizes itself to its contents: a
        # reading that grows a column - six figures of steps, a longer
        # status - must not push a button off the right-hand edge.
        root.maxsize(root.winfo_screenwidth() - 40,
                     root.winfo_screenheight() - 80)

        # Escape is the ramped stop, deliberately not the emergency one: the
        # key that is easy to hit by accident should be the one that does not
        # forfeit the zero.
        root.bind("<Escape>", lambda _event: self._stop())
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(DRAIN_MS, self._drain)

    # --- building ----------------------------------------------------------

    def _build_style(self) -> None:
        """One place for the look. No behaviour belongs in here.

        The native Windows theme draws its own backgrounds and quietly
        ignores a colour asked for in a style, so the palette would do
        nothing under it; "clam" is the bundled theme that honours them, and
        it ships with every Tk. If some stripped-down Tk has not got it the
        window still works - it just comes up looking like stock Tk.
        """
        style = ttk.Style(self.root)
        if "clam" in style.theme_names():
            style.theme_use("clam")

        self.root.configure(bg=CANVAS)
        style.configure(".", background=CARD, foreground=INK,
                        fieldbackground="white", bordercolor=EDGE,
                        lightcolor=CARD, darkcolor=EDGE, focuscolor=BUSY)
        for name in ("TFrame", "TLabel", "TLabelframe", "TRadiobutton",
                     "TCheckbutton"):
            style.configure(name, background=CARD)
        # The panels read as cards on the window's own colour, which is what
        # the gaps between them are.
        style.configure("TLabelframe", bordercolor=EDGE, relief="solid",
                        borderwidth=1, padding=(8, 6))
        style.configure("TLabelframe.Label", foreground=MUTED,
                        font=self.heading)
        style.configure("Canvas.TFrame", background=CANVAS)
        style.configure("Hint.TLabel", foreground=MUTED)

        style.configure("TButton", padding=(7, 4), borderwidth=1,
                        background="#e7ebf0", bordercolor=EDGE)
        style.map("TButton",
                  background=[("disabled", "#eef1f4"),
                              ("active", "#dbe3ec")],
                  foreground=[("disabled", "#a7b0ba")])
        # The buttons that start a move, told apart from the ones that only
        # read something or set a number.
        style.configure("Go.TButton", background=BUSY, foreground="white",
                        bordercolor=BUSY)
        style.map("Go.TButton",
                  background=[("disabled", "#c3d2e8"),
                              ("active", "#3b82f6")],
                  foreground=[("disabled", "#eef1f4")])
        # Arming is the one button that sets the motors going with nobody
        # touching anything, so it does not look like its neighbours.
        style.configure("Arm.TButton", background=WARN, foreground="white",
                        bordercolor=WARN)
        style.map("Arm.TButton",
                  background=[("disabled", "#e3cdb2"),
                              ("active", "#d97706")],
                  foreground=[("disabled", "#f7f3ee")])

        # The status line, in four colours for the four things it says. The
        # style is swapped in `show`, where the State is.
        for name, colour in (("Status", INK), ("StatusGood", GOOD),
                             ("StatusWarn", WARN), ("StatusBad", BAD),
                             ("StatusBusy", BUSY)):
            style.configure(name + ".TLabel", background="white",
                            foreground=colour, bordercolor=EDGE,
                            relief="solid", borderwidth=1, padding=(8, 5))

        style.configure("Horizontal.TProgressbar", background=BUSY,
                        troughcolor="#e3e8ee", bordercolor=EDGE,
                        lightcolor=BUSY, darkcolor=BUSY)

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
        # Needs no Pico, so `_refresh_enabled` leaves it alone: it is most
        # wanted before anything is plugged in.
        self.b_guide = ttk.Button(frame, text="startup guide",
                                  command=self._open_guide)
        self.b_guide.grid(row=0, column=5, sticky="e", **PAD)
        frame.columnconfigure(3, weight=1)

        # The link from the rocket, as the ground Pico hears it. Here with
        # the connection rather than in the track panel, because whether
        # packets are arriving is worth knowing before anybody arms anything.
        # One line, not a panel of its own: the window is already as tall as
        # a laptop screen, and every row added up here pushes the STOP
        # buttons toward the bottom edge.
        link = ttk.Frame(frame)
        link.grid(row=1, column=0, columnspan=6, sticky="ew")
        link.columnconfigure(2, weight=1)
        ttk.Label(link, text="radio", font=self.bold).grid(
            row=0, column=0, sticky="w", **PAD)
        self.v_radio_head = ttk.Label(link, text="-", font=self.bold)
        self.v_radio_head.grid(row=0, column=1, sticky="w", **PAD)
        # width=1 asks for no width of its own, so the label takes whatever
        # the row has left and a long line is cut off at the right - loop
        # gap last, the number least needed - rather than widening the
        # window, which would shuffle every panel below it.
        self.v_radio_rates = ttk.Label(link, text="", font=self.mono,
                                       width=1)
        self.v_radio_rates.grid(row=0, column=2, sticky="ew", **PAD)

    def _build_position(self, row: int) -> None:
        frame = self._frame(row, "position")
        self._value(frame, 0, 0, "az", font=self.bold)
        self._value(frame, 1, 0, "el", font=self.bold)
        self.v_az = self._value(frame, 0, 1, font=self.big)
        self.v_el = self._value(frame, 1, 1, font=self.big)
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

        # Out and back on one axis, so it should end up exactly where it
        # started; if it does not, one of its lines is dropping pulses. Needs
        # a zero, like any other move, because the Pico refuses moves on an
        # axis that has not been zeroed.
        #
        # One button per axis, like the zeros above, so only the axis being
        # checked is swung out and back. The out leg is the negative one, so
        # an axis sitting within a sweep of its travel minimum is refused -
        # move it up and press again.
        cal = ttk.Frame(frame)
        cal.grid(row=2, column=3, sticky="e", **PAD)
        ttk.Label(
            cal,
            text="calibrate, -%g then +%g deg at %g deg/s:"
                 % (gw.CAL_SWEEP_DEG, gw.CAL_SWEEP_DEG, gw.CAL_SPEED_DPS),
        ).grid(row=0, column=0, padx=(0, 4))

        self.b_cal = {}
        for col, axis in enumerate(("AZ", "EL")):
            button = ttk.Button(
                cal, text=axis.lower(), width=4,
                command=lambda a=axis: self._calibrate(a))
            button.grid(row=0, column=1 + col, padx=2)
            self.b_cal[axis] = button
        frame.columnconfigure(3, weight=1)

        # Blank while the Pico is geared, which is always, unless it was
        # zeroed with gear off before this window opened or somebody typed
        # `gear off` after zeroing. Beside the calibrate row, which leaves
        # this corner empty, so it costs no height. See _keep_geared.
        self.v_gear = ttk.Label(frame, text="", font=self.bold,
                                foreground=BAD)
        self.v_gear.grid(row=2, column=0, columnspan=3, sticky="w", **PAD)

        # A manual move, both axes in one command, each at its own speed.
        # Here with the position and its zero rather than in a panel of its
        # own: those three are what gets used together, and a panel costs a
        # title and a border of height the window does not have.
        move = ttk.Frame(frame)
        move.grid(row=3, column=0, columnspan=4, sticky="w", pady=(6, 0))
        self.e_speed: dict[str, ttk.Entry] = {}
        self.e_angle: dict[str, ttk.Entry] = {}
        col = 0
        for axis, angle in (("az", "45"), ("el", "20")):
            ttk.Label(move, text=axis, font=self.bold).grid(
                row=0, column=col, padx=(6 if col == 0 else 14, 4))
            ttk.Label(move, text="at").grid(row=0, column=col + 1)
            self.e_speed[axis] = self._entry(move, 0, col + 2, "5",
                                             width=6)
            ttk.Label(move, text="deg/s,").grid(row=0, column=col + 3)
            self.e_angle[axis] = self._entry(move, 0, col + 4, angle,
                                             width=8)
            ttk.Label(move, text="deg").grid(row=0, column=col + 5)
            col += 6
        # Two buttons, one set of boxes: "turn by" reads each angle as a
        # change, "go to" as where to end up.
        self.b_turn = ttk.Button(move, text="turn by", width=9,
                                 style="Go.TButton",
                                 command=lambda: self._move(False))
        self.b_turn.grid(row=0, column=col, padx=(14, 2))
        self.b_goto = ttk.Button(move, text="go to", width=9,
                                 style="Go.TButton",
                                 command=lambda: self._move(True))
        self.b_goto.grid(row=0, column=col + 1, padx=2)

        # How far through the move it is. Says how to use the boxes while
        # nothing is moving, so that line earns its height either way.
        progress = ttk.Frame(frame)
        progress.grid(row=4, column=0, columnspan=4, sticky="ew")
        progress.columnconfigure(0, weight=1)
        self.progress = ttk.Progressbar(progress, mode="determinate",
                                        maximum=1000)
        self.progress.grid(row=0, column=0, sticky="ew", **PAD)
        self.v_progress = ttk.Label(progress, text=MOVE_HINT,
                                    style="Hint.TLabel")
        self.v_progress.grid(row=0, column=1, sticky="e", **PAD)

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
        self.b_save = ttk.Button(buttons, text="save", command=self._save)
        self.b_save.grid(row=0, column=1, padx=2)
        self.b_defaults = ttk.Button(buttons, text="defaults",
                                     command=self._defaults)
        self.b_defaults.grid(row=0, column=2, padx=2)
        frame.columnconfigure(5, weight=1)

    def _build_track(self, row: int) -> None:
        """A target the Pico chases. Always on screen: this window is for
        tracking, with the move row above for putting the antenna where it
        needs to be first."""
        track = self._frame(row, "track")

        picker = ttk.Frame(track)
        picker.grid(row=0, column=0, columnspan=4, sticky="w")
        self.m3_source = tk.StringVar(value="manual")
        # The first three are worked out here and sent as targets, at the
        # rate in the box. The last two are worked out on the Pico - the
        # rocket's position arriving over the radio, or the flight built
        # into its firmware - so nothing is sent from here and the rate box
        # does not apply. One row, because the window is already as tall as
        # a laptop screen.
        for col, (value, text) in enumerate((
            ("manual", "a direction I type"),
            ("moving", "a moving target"),
            ("sim", "a simulated flight"),
            ("radio", "the rocket (radio)"),
            ("replay", "the Pico's built-in flight"),
        )):
            ttk.Radiobutton(picker, text=text, value=value,
                            variable=self.m3_source,
                            command=self._source_chosen).grid(
                row=0, column=col, sticky="w", **PAD)
        ttk.Label(picker, text="at").grid(row=0, column=5, padx=(12, 2))
        self.m3_hz = self._entry(picker, 0, 6, "5", width=5)
        ttk.Label(picker, text="targets/sec").grid(row=0, column=7, padx=2)

        aim = ttk.Frame(track)
        aim.grid(row=1, column=0, columnspan=4, sticky="w")
        self.m3_aim_row = aim
        ttk.Label(aim, text="point at  az").grid(row=0, column=0, **PAD)
        self.m3_az = self._entry(aim, 0, 1, "0")
        ttk.Label(aim, text="el").grid(row=0, column=2, padx=2)
        self.m3_el = self._entry(aim, 0, 3, "45")
        ttk.Label(aim, text="deg").grid(row=0, column=4, padx=2)
        self.b_point = ttk.Button(aim, text="point here", width=11,
                                  command=self._point)
        self.b_point.grid(row=0, column=5, padx=(12, 2))

        # For the moving target, which sets off from the direction above.
        # A new target every time one is sent, so the chase can be watched
        # replacing one with the next without anybody typing them.
        moving = ttk.Frame(track)
        moving.grid(row=2, column=0, columnspan=4, sticky="w")
        self.m3_moving_row = moving
        ttk.Label(moving, text="moving at  az").grid(row=0, column=0, **PAD)
        self.m3_az_rate = self._entry(moving, 0, 1, "3")
        ttk.Label(moving, text="el").grid(row=0, column=2, padx=2)
        self.m3_el_rate = self._entry(moving, 0, 3, "2")
        ttk.Label(moving, text="deg/s, back and forth over").grid(
            row=0, column=4, padx=2)
        self.m3_span = self._entry(moving, 0, 5, "30", width=5)
        ttk.Label(moving, text="deg").grid(row=0, column=6, padx=2)

        arm = ttk.Frame(track)
        arm.grid(row=3, column=0, columnspan=4, sticky="w")
        self.b_arm = ttk.Button(arm, text="ARM tracking", width=14,
                                style="Arm.TButton", command=self._arm)
        self.b_arm.grid(row=0, column=0, **PAD)
        self.b_disarm = ttk.Button(arm, text="disarm", width=10,
                                   command=self._disarm)
        self.b_disarm.grid(row=0, column=1, **PAD)
        # Only live once the Pico has given up on the radio by itself. One
        # click, no "are you sure": the operator already said yes to radio
        # tracking, and a dialog costs seconds the rocket does not wait for.
        self.b_rearm = ttk.Button(arm, text="re-arm radio", width=13,
                                  style="Arm.TButton", command=self._rearm)
        self.b_rearm.grid(row=0, column=2, **PAD)
        self.v_arm_note = self._value(arm, 0, 3, "")

        # The readout. Three numbers per axis, because the interesting thing
        # while tracking is not where it is but how far behind it is.
        board = ttk.Frame(track)
        board.grid(row=4, column=0, columnspan=4, sticky="w", pady=(4, 0))
        for col, head in enumerate(("", "target", "actual", "off by",
                                    "turning at")):
            self._value(board, 0, col, head, font=self.bold)
        self.v_trk = {}
        for line, axis in ((1, "az"), (2, "el")):
            self._value(board, line, 0, axis, font=self.bold)
            self.v_trk[axis] = [self._value(board, line, col)
                                for col in (1, 2, 3, 4)]

    def _build_log_and_commands(self, row: int) -> None:
        log = ttk.LabelFrame(self.root, text="conversation")
        log.grid(row=row, column=0, sticky="nsew", **PAD)
        log.rowconfigure(0, weight=1)
        log.columnconfigure(0, weight=1)

        self.log = tk.Text(log, height=12, width=52, font=self.mono,
                           wrap="none", state="disabled",
                           background="white", foreground=INK,
                           relief="flat", borderwidth=0,
                           highlightthickness=0, padx=6, pady=4,
                           spacing1=1)
        self.log.grid(row=0, column=0, sticky="nsew")
        bar = ttk.Scrollbar(log, orient="vertical", command=self.log.yview)
        bar.grid(row=0, column=1, sticky="ns")
        self.log.configure(yscrollcommand=bar.set)
        # Sent and received are told apart by colour as well as by the
        # marker, because at a bench the pane is read at a glance.
        self.log.tag_configure("out", foreground=GOOD)
        self.log.tag_configure("in", foreground=INK)
        self.log.tag_configure("note", foreground=BUSY)
        # A line somebody typed, and the prompt's answer to it.
        self.log.tag_configure("typed", foreground=TYPED_INK)
        self.log.tag_configure("said", foreground=MUTED)

        commands = ttk.LabelFrame(self.root,
                                  text="commands, and when to use them")
        commands.grid(row=row, column=1, sticky="nsew", **PAD)
        commands.rowconfigure(0, weight=1)
        commands.columnconfigure(0, weight=1)

        # From command_help.py, which can be checked against the prompt's
        # real command table, so this panel lists the commands that exist.
        text = tk.Text(commands, height=12, width=52, font=self.mono,
                       wrap="word", state="normal", background="white",
                       foreground=INK, relief="flat", borderwidth=0,
                       highlightthickness=0, padx=6, pady=4)
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
        ttk.Label(typing, style="Hint.TLabel",
                  text="same as the prompt. The STOP button is the one that "
                       "jumps the queue.").grid(
            row=1, column=0, columnspan=2, sticky="w", pady=(2, 0))

        self._history: list[str] = []
        self._recall_at = 0

    def _build_stops(self, row: int) -> None:
        frame = ttk.Frame(self.root, style="Canvas.TFrame")
        frame.grid(row=row, column=0, columnspan=2, sticky="ew", padx=6)
        frame.columnconfigure(0, weight=1)
        frame.columnconfigure(1, weight=1)

        # Classic tk rather than ttk so the colour actually applies, and
        # never disabled: these two are the way out of anything the other
        # buttons started.
        #
        # Amber and red, not grey and red: the ramped stop is the one to
        # reach for first, and a grey button next to a red one does not look
        # like a stop at all.
        self.b_stop = tk.Button(frame, text="STOP  (Esc)", height=2,
                                bg="#e0a030", fg="#231700",
                                activebackground="#edb44e",
                                activeforeground="#231700",
                                font=self.bold, relief="flat",
                                borderwidth=0, highlightthickness=0,
                                command=self._stop)
        self.b_stop.grid(row=0, column=0, sticky="ew", padx=(0, 3))
        self.b_estop = tk.Button(frame, text="E-STOP", height=2,
                                 bg="#c0271a", fg="white",
                                 activebackground="#dc3a2c",
                                 activeforeground="white",
                                 font=self.bold, relief="flat",
                                 borderwidth=0, highlightthickness=0,
                                 command=self._estop)
        self.b_estop.grid(row=0, column=1, sticky="ew", padx=(3, 0))

    def _build_status(self, row: int) -> None:
        self.v_status = ttk.Label(self.root, text="starting...", anchor="w",
                                  style="Status.TLabel")
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

    def _open_guide(self) -> None:
        """The startup guide, in a window of its own beside this one.

        One at a time: pressing the button again brings the open one to the
        front. Not tied to this window, so it never sits on top of it - it
        would cover the STOP buttons, and rule 3 says nothing does. Esc is
        the ramped stop in it too, the same key in both windows.
        """
        if self._guide is not None and self._guide.winfo_exists():
            self._guide.deiconify()
            self._guide.lift()
            self._guide.focus_set()
            return

        top = tk.Toplevel(self.root)
        top.title(f"{WINDOW_TITLE} - {startup_guide.TITLE.lower()}")
        top.configure(bg=CANVAS)
        top.columnconfigure(0, weight=1)
        top.rowconfigure(0, weight=1)

        text = tk.Text(top, wrap="word", font="TkDefaultFont",
                       background="white", foreground=INK, relief="flat",
                       borderwidth=0, highlightthickness=0, padx=14,
                       pady=10, cursor="arrow")
        text.grid(row=0, column=0, sticky="nsew", padx=(6, 0), pady=(6, 0))
        bar = ttk.Scrollbar(top, orient="vertical", command=text.yview)
        bar.grid(row=0, column=1, sticky="ns", padx=(0, 6), pady=(6, 0))
        text.configure(yscrollcommand=bar.set)
        self._write_guide(text)
        text.configure(state="disabled")

        ttk.Button(top, text="close", command=self._close_guide).grid(
            row=1, column=0, columnspan=2, sticky="e", padx=6, pady=6)

        top.bind("<Escape>", lambda _event: self._stop())
        top.protocol("WM_DELETE_WINDOW", self._close_guide)
        self._place_guide(top)
        self._guide = top

    def _close_guide(self) -> None:
        if self._guide is not None and self._guide.winfo_exists():
            self._guide.destroy()
        self._guide = None

    def _place_guide(self, top: tk.Toplevel) -> None:
        """Beside the main window, on whichever side has more room.

        Narrower than its longest grey line if that is what it takes to
        fit - a wrapped line is a small price next to a guide sitting over
        E-STOP. Only a screen with no usable room either side gets it over
        the main window, at the right edge; a click on the main window
        brings that back in front.
        """
        expected = [line for step in startup_guide.steps()
                    for line in step.expect]
        margin = GUIDE_INDENT + 80  # quote indent, padding, scrollbar
        want = max(self.mono.measure(line) for line in expected) + margin
        least = self.mono.measure("0" * 40) + margin

        self.root.update_idletasks()
        screen_w = self.root.winfo_screenwidth()
        screen_h = self.root.winfo_screenheight()
        left = self.root.winfo_rootx()
        right = left + self.root.winfo_width()
        # The 24s leave room for the window frames Tk does not count.
        room_right = screen_w - right - 24
        room_left = left - 24

        if room_right >= room_left and room_right >= least:
            width = min(want, room_right)
            x = right + 12
        elif room_left >= least:
            width = min(want, room_left)
            x = left - width - 24
        else:
            width = min(want, screen_w - 40)
            x = max(0, screen_w - width - 12)

        y = max(0, self.root.winfo_rooty())
        height = max(320, min(self.root.winfo_height(), screen_h - y - 80))
        top.geometry(f"{width}x{height}+{x}+{y}")

    def _write_guide(self, text: tk.Text) -> None:
        if self._guide_title_font is None:
            self._guide_title_font = self.heading.copy()
            size = self._guide_title_font.cget("size")
            self._guide_title_font.configure(
                size=size + (3 if size > 0 else -3))
        quote_in = GUIDE_INDENT + 10
        text.tag_configure("title", font=self._guide_title_font, spacing3=4)
        text.tag_configure("intro", foreground=MUTED, spacing3=6)
        text.tag_configure("part", font=self.heading, foreground=BUSY,
                           spacing1=14, spacing3=2)
        text.tag_configure("step", font=self.bold, spacing1=10, spacing3=2)
        text.tag_configure("body", lmargin1=GUIDE_INDENT,
                           lmargin2=GUIDE_INDENT, spacing3=2)
        text.tag_configure("after", spacing1=4)
        text.tag_configure("label", foreground=MUTED, lmargin1=GUIDE_INDENT,
                           lmargin2=GUIDE_INDENT, spacing1=4, spacing3=2)
        # The newline is tagged too, so the grey runs to the right edge and
        # the lines read as one box. A line too long for the window wraps
        # with its second half indented, so it does not read as two lines
        # of output.
        wrapped = quote_in + 18
        text.tag_configure("quote", font=self.mono, background=QUOTE_BG,
                           lmargin1=quote_in, lmargin2=wrapped, rmargin=8)
        text.tag_configure("seen", font=self.mono, foreground=WARN,
                           lmargin1=quote_in, lmargin2=wrapped, spacing1=4)
        text.tag_configure("fix", lmargin1=quote_in + 14,
                           lmargin2=quote_in + 14)

        text.insert("end", startup_guide.TITLE + "\n", "title")
        text.insert("end", startup_guide.INTRO + "\n", "intro")
        number = 0
        for part in startup_guide.PARTS:
            text.insert("end", part.title + "\n", "part")
            if part.intro:
                text.insert("end", part.intro + "\n", "body")
            for step in part.steps:
                number += 1
                text.insert("end", f"{number}.  {step.title}\n", "step")
                if step.do:
                    text.insert("end", step.do + "\n", "body")
                text.insert("end", f"Look at {step.look}:\n", "label")
                for line in step.expect:
                    text.insert("end", line + "\n", "quote")
                if step.note:
                    text.insert("end", step.note + "\n", ("body", "after"))
                if step.otherwise:
                    text.insert("end", "If you see this instead:\n", "label")
                    for seen, fix in step.otherwise:
                        text.insert("end", seen + "\n", "seen")
                        text.insert("end", fix + "\n", "fix")

    def _stop(self) -> None:
        self.worker.stop()

    def _estop(self) -> None:
        self.worker.estop()

    def _offline(self) -> bool:
        """True, having said so, when there is no Pico to send anything to.

        Unplugged, the window stays usable - the boxes, the tracking source -
        so it can be set up before the Pico arrives. Every button that needs the
        Pico checks here first, so pressing one says why nothing happened
        instead of queueing a job, or asking "are you sure?" about something
        that cannot happen.
        """
        if self._state.connected:
            return False
        self.note(gw.NOT_CONNECTED)
        return True

    def _save(self) -> None:
        if not self._offline():
            self.worker.save()

    def _defaults(self) -> None:
        if not self._offline():
            self.worker.defaults()

    def _read(self, entry: ttk.Entry, what: str) -> float:
        # motor.py's own parser, so a number typed here means exactly what it
        # would mean at the prompt. It raises UsageError, caught by callers.
        return motor._number(entry.get().strip(), what)

    def _zero(self, axis: str) -> None:
        if self._offline():
            return
        try:
            text = self.e_zero.get().strip()
            deg = self._read(self.e_zero, "angle") if text else None
        except motor.UsageError as exc:
            self.note(str(exc))
            return
        self.worker.zero(axis, deg)

    def _calibrate(self, axis: str) -> None:
        if not self._offline():
            self.worker.calibrate(axis)

    def _move(self, absolute: bool) -> None:
        """The move row: "turn by" when `absolute` is false, "go to" when
        it is true.

        Only the axes that are to move are sent. A blank angle keeps its
        axis still, and so does 0 for "turn by" - the Pico refuses a
        two-axis move whose half rounds to no steps, and would refuse the
        whole thing rather than move the other axis alone.
        """
        if self._offline():
            return
        halves: dict[str, tuple[float, float]] = {}
        try:
            for axis in ("az", "el"):
                if not self.e_angle[axis].get().strip():
                    continue
                angle = self._read(self.e_angle[axis], f"{axis} angle")
                if angle == 0 and not absolute:
                    continue
                speed = self._read(self.e_speed[axis], f"{axis} speed")
                halves[axis] = (speed, angle)
        except motor.UsageError as exc:
            self.note(str(exc))
            return

        if not halves:
            self.note("nothing to move - give az or el an angle"
                      + ("" if absolute else " other than 0"))
            return
        if len(halves) == 2:
            self.worker.move_both(*halves["az"], *halves["el"],
                                  absolute=absolute)
            return
        (axis, (speed, angle)), = halves.items()
        self.worker.move(axis.upper(), speed, angle, absolute=absolute)

    # --- tracking ----------------------------------------------------------

    def _source_chosen(self) -> None:
        # Changing the source mid-flight would mean two of them fighting, so
        # it only takes effect next time tracking is armed.
        if self._state.tracking:
            self.note("the source changes next time you arm")
        self._show_source_rows()
        self._refresh_enabled(self._state)

    def _show_source_rows(self) -> None:
        """Hide the direction rows when the Pico steers itself.

        They do nothing for the radio or the built-in flight, and the window
        is short of height - the rows they free are what pay for the radio
        line up in the connection panel. Hidden rather than only disabled.
        """
        pico = self.m3_source.get() in PICO_SOURCES
        for row in (self.m3_aim_row, self.m3_moving_row):
            if pico:
                row.grid_remove()
            else:
                row.grid()

    def _arm(self) -> None:
        if self._offline():
            return
        # Pointing is worked out at the antenna, which only gear on counts.
        # _keep_geared fixes gear off by itself when it is free; this is the
        # case where it was not, because the axes were zeroed.
        gear = self._state.gear
        if gear is not None and not gear.geared:
            if self.confirm(
                "Switch to gear on?",
                "The Pico is counting gear off, so its angles are at the "
                "gearbox shaft, not the antenna. Tracking needs gear on.\n\n"
                "Switching costs BOTH axes their zero, because the drive "
                "train is physically different. Zero again, then arm.",
            ):
                self.note("switching to gear on - zero both axes again, "
                          "then arm")
                self.worker.set_gear(True)
            return
        kind = self.m3_source.get()
        try:
            hz = self._read(self.m3_hz, "targets/sec")
            az = self._read(self.m3_az, "azimuth")
            el = self._read(self.m3_el, "elevation")
            # Read only for the moving target. No other source uses them, so
            # a stray letter in one is no reason to refuse to arm.
            motion = ((self._read(self.m3_az_rate, "azimuth rate"),
                       self._read(self.m3_el_rate, "elevation rate"),
                       self._read(self.m3_span, "back-and-forth angle"))
                      if kind == "moving" else ())
        except motor.UsageError as exc:
            self.note(str(exc))
            return
        if hz <= 0:
            self.note("targets/sec must be more than zero")
            return
        if motion and motion[2] <= 0:
            self.note("back-and-forth angle must be more than zero")
            return

        if kind == "replay":
            message = (
                "The motors will start moving on their own, following the "
                "simulated flight built into the Pico's firmware (about "
                "2.5 minutes). Nothing is sent from the laptop.\n\n"
                "STOP, E-STOP and disarm stay live. When the flight ends the "
                "Pico stops tracking by itself.")
        elif kind == "radio":
            message = (
                "The motors will start moving on their own, pointing at each "
                "rocket position the Pico receives over the radio. Nothing "
                "is sent from the laptop. The Pico points from the ground "
                "station position built into its firmware "
                "(radio_config.hpp).\n\n"
                f"If no usable position arrives for {p.TRACK_TIMEOUT_S} s - "
                f"including in the first {p.TRACK_TIMEOUT_S} s after arming "
                "- the Pico stops tracking by itself, and 're-arm radio' "
                "starts it again.\n\n"
                "STOP, E-STOP and disarm stay live.")
            # Not a refusal: the operator may know the transmitter is about
            # to come on. But arming into silence is usually a mistake, so
            # it is said where they have to read it.
            link = self._state.link
            now = self.clock()
            if link is None or not link.hearing(now):
                head = ("no radio reports" if link is None
                        else link.headline(now)[0])
                message = (f"WARNING - the radio link says: {head}.\n\n"
                           + message)
        else:
            if kind == "manual":
                following = "the direction you typed"
            elif kind == "moving":
                following = ("a target that starts at az %g el %g and moves "
                             "az %g, el %g deg/s, back and forth over %g deg,"
                             % (az, el, *motion))
            else:
                following = "a simulated flight"
            message = (
                f"The motors will start moving on their own, following "
                f"{following}"
                f" at {hz:g} targets a second.\n\n"
                "STOP and E-STOP stay live. The Pico also gives up by itself "
                "if targets stop reaching it.")
        if not self.confirm("Arm tracking?", message):
            return
        self.worker.start_tracking(kind, hz, az, el, *motion)

    def _disarm(self) -> None:
        self.worker.stop_tracking()

    def _rearm(self) -> None:
        if self._offline():
            return
        self.note("re-arming radio tracking - waits for the axes to come to "
                  "rest first")
        self.worker.rearm_radio()

    def _point(self) -> None:
        if self._offline():
            return
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
            self.v_arm_note.configure(
                text=self._dropped_note() if st.radio_dropped
                else "not armed - nothing will move")
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

        if not st.tracking and st.radio_dropped:
            # The last readings stay on the board, but the note says what
            # happened rather than describing a session that has ended.
            self.v_arm_note.configure(text=self._dropped_note())
            return
        if st.pico_replay:
            note = "the Pico is flying its built-in flight"
        elif st.from_radio:
            note = self._radio_note(st)
        else:
            note = f"{st.targets_sent} targets sent"
        if getattr(sample, "clamped", False):
            # Said plainly, because the error will not close and the reason
            # is not the antenna's fault.
            note += "   OUT OF REACH - held at a travel limit"
        self.v_arm_note.configure(text=note)

    @staticmethod
    def _dropped_note() -> str:
        # Short: a note longer than the panel's other rows widens the window.
        return (f"STOPPED - radio quiet {p.TRACK_TIMEOUT_S} s. Re-arm when "
                "packets arrive")

    def _radio_note(self, st: gw.State) -> str:
        """While following the radio: is it actually steering?

        `ok` going up says packets are heard; `tgt` going up says they are
        turning into motor targets. The gap between the two is the thing to
        notice - packets that decode but give no pointing, from a station
        position that is wrong or a rocket too close overhead.
        """
        link = st.link
        now = self.clock()
        if link is None or not link.hearing(now):
            return "following the radio - NO PACKETS arriving"
        # The rate looks back over several seconds, most of them from before
        # arming, so give the first targets time to show before judging.
        armed_for = getattr(st.sample, "t", 0.0)
        if link.target_rate is None or armed_for < RADIO_SETTLE_S:
            return "following the radio"
        if not link.steering():
            return "following the radio - packets but NO targets (station?)"
        return f"following the radio - {link.target_rate:.1f} targets/s"

    def _apply_settings(self) -> None:
        """Send only what differs from what the Pico reports.

        Comparing first keeps `apply` from sending six commands every time,
        and keeps it from marking the settings unsaved when nothing changed.
        """
        if self._offline():
            return
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

    # --- gearing -----------------------------------------------------------

    def _keep_geared(self, st: gw.State) -> None:
        """Put the Pico back in gear on whenever that costs nothing.

        This window always runs geared (see the module notes). Gear off
        turns up when the Pico boots with it saved, or somebody types
        `gear off`. A gear change costs both axes their zero, so:

          - nothing zeroed: switch now, without asking, because there is no
            zero to lose. Not while a job or tracking is running - it waits
            for the next quiet reading rather than cut in front of a move.
          - zeroed: leave it. Spending somebody's zero unasked would undo
            their work. `show` says GEAR OFF in red, and ARM tracking offers
            the switch.
        """
        if not st.connected or st.gear is None or st.gear.geared:
            self._gear_sent = False
            return
        if (self._gear_sent or st.zeroed or st.working or st.moving
                or st.tracking):
            return
        self._gear_sent = True
        # ZEROED is one flag for both axes, so "not zeroed" can still mean
        # one axis was done. Said rather than guessed at.
        self.note("the Pico is counting gear off - switching to gear on, "
                  "which this window always runs. Nothing is zeroed, so it "
                  "costs nothing; if you had zeroed one axis, zero it again.")
        self.worker.set_gear(True)

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

        # These tick on even when no event arrives, which is exactly when
        # they matter: a Pico that stopped talking, a radio that went quiet,
        # and a move in progress.
        self._show_heartbeat(self._state)
        self._show_radio(self._state)
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
        self.v_progress.configure(text=MOVE_HINT, foreground=MUTED)
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
            text=f"{gone:4.1f} s of about {est:.1f} s predicted",
            foreground=INK)

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

    def _show_radio(self, st: gw.State) -> None:
        """The radio line. Ages are worked out against now, so they
        keep counting between the Pico's once-a-second reports."""
        link = st.link
        if link is None:
            self.v_radio_head.configure(text="-", foreground=MUTED)
            self.v_radio_rates.configure(text="")
            return
        now = self.clock()
        head, level = link.headline(now)
        self.v_radio_head.configure(text=head, foreground=LINK_COLOUR[level])
        self.v_radio_rates.configure(text=link.details(now))

    def show(self, st: gw.State) -> None:
        """Put a State on the screen, and keep it as the current one.

        Displayed values are never remembered across calls - what is on
        screen is what the Pico last said. The State itself is kept because
        the controls need it: `apply` compares against it to work out what
        actually changed, and arming needs to know the gearing.
        """
        self._state = st
        self._keep_geared(st)
        self._show_title(st)
        self.v_port.configure(text=st.port or "no port")
        self.v_who.configure(text=f"{st.target} {st.version}".strip() or "-")
        self.v_uptime.configure(text=f"up {_uptime(st.uptime_ms)}")
        self._show_heartbeat(st)
        self._show_radio(st)

        # Nothing to say while geared, which is the only way this window
        # runs. Gear off changes what every angle on screen means, so it is
        # said in red until it is fixed.
        if st.gear is not None and not st.gear.geared:
            self.v_gear.configure(
                text="GEAR OFF - angles at the gearbox shaft, not the "
                     "antenna" + ("  (ARM tracking offers gear on)"
                                  if st.zeroed else ""))
        else:
            self.v_gear.configure(text="")

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
                foreground=GOOD if st.pos.zeroed else BAD)

        self._show_settings(st)
        self._show_tracking(st)
        self._refresh_enabled(st)

        status = st.describe()
        if st.moving:
            status += " - readings above are from before the move started"
        # Colour as well as words: the status line gets read sideways, while
        # somebody's hands are on the antenna.
        if not st.connected:
            style = "StatusBad.TLabel"
        elif st.rebooted or not st.zeroed:
            style = "StatusWarn.TLabel"
        elif st.tracking or st.moving:
            style = "StatusBusy.TLabel"
        else:
            style = "StatusGood.TLabel"
        self.v_status.configure(text=status, style=style)

    def _show_title(self, st: gw.State) -> None:
        """Name the port in the title bar, for two windows side by side."""
        title = WINDOW_TITLE + (f"  -  {st.port}" if st.port else "")
        if title != self._title:
            self._title = title
            self.root.title(title)

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
                text="the built-in numbers - a restart comes back here")
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

        # Unplugged, nothing can be in progress, so everything stays
        # clickable: the window can be set up before the Pico arrives, and a
        # button that needs it says "not connected" - see _offline.
        offline = not st.connected
        usable = settled or offline
        movable = can_move or offline

        for widget in (self.b_zero_all, self.b_zero_az, self.b_zero_el,
                       self.b_apply, self.b_save, self.b_defaults):
            widget.state(["!disabled"] if usable else ["disabled"])

        # `settled` is false while tracking, so a manual move cannot be
        # thrown in while the Pico is chasing a target.
        for widget in (self.b_turn, self.b_goto, *self.b_cal.values()):
            widget.state(["!disabled"] if movable else ["disabled"])

        # Arming needs a zero and a settled machine; disarming must always be
        # available, like the stops.
        self.b_arm.state(["!disabled"] if movable and not st.tracking
                         else ["disabled"])
        self.b_disarm.state(["!disabled"] if st.tracking else ["disabled"])
        # Live while the axes are still braking from the session that ended
        # - pressing it then is the point - so `moving` does not close it.
        # The worker waits for rest before it arms.
        rearm = (st.connected and st.radio_dropped and not st.tracking
                 and not st.working and st.zeroed and not st.rebooted)
        self.b_rearm.state(["!disabled"] if rearm else ["disabled"])
        # The direction boxes stay editable whenever the source is manual,
        # armed or not: you type where to point and THEN arm. (A disabled
        # ttk.Entry refuses even a programmatic write, so getting this the
        # wrong way round locks the operator out of their own target.) The
        # moving target reads its boxes once, as it is armed, so they close
        # while it runs rather than take an edit that would do nothing.
        manual = self.m3_source.get() == "manual"
        moving = self.m3_source.get() == "moving" and not st.tracking
        for widget in (self.m3_az, self.m3_el):
            widget.state(["!disabled"] if manual or moving else ["disabled"])
        for widget in (self.m3_az_rate, self.m3_el_rate, self.m3_span):
            widget.state(["!disabled"] if moving else ["disabled"])
        # The Pico's built-in flight and the radio run at their own rates,
        # so there is no targets/sec to choose.
        self.m3_hz.state(["disabled"] if self.m3_source.get() in PICO_SOURCES
                         else ["!disabled"])
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
