r"""A window showing what the motor Pico is doing: `py -m gs.gui`.

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

Read-only for now
-----------------

Nothing here can command a motor: the only thing it sends are the five
questions the worker polls with, plus a reconnect. Zeroing, moving, settings
changes, the mode selector and the E-STOP come next, and E-STOP lands with
them - a window with buttons needs a way to stop what they started.
"""

from __future__ import annotations

import argparse
import sys
import time
import tkinter as tk
from tkinter import font as tkfont
from tkinter import ttk

from gs import gui_worker as gw
from gs import motor
from gs import protocol as p

# How often the Tk thread looks for events from the worker. Well under the
# worker's own poll interval, so a fresh reading is shown as soon as it
# lands rather than up to a poll later.
DRAIN_MS = 100

# Lines kept in the log pane. A long bench session would otherwise grow it
# without limit, and nobody scrolls back past a few hundred.
LOG_LINES = 600

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

        root.title("gs motor")
        root.minsize(760, 620)
        self.mono = tkfont.nametofont("TkFixedFont").copy()
        self.bold = tkfont.nametofont("TkDefaultFont").copy()
        self.bold.configure(weight="bold")

        root.columnconfigure(0, weight=3)
        root.columnconfigure(1, weight=2)
        root.rowconfigure(4, weight=1)

        self._build_connection(0)
        self._build_gear(1)
        self._build_position(2)
        self._build_settings(3)
        self._build_log_and_commands(4)
        self._build_status(5)

        root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(DRAIN_MS, self._drain)

    # --- building ----------------------------------------------------------

    def _frame(self, row: int, title: str, **grid) -> ttk.LabelFrame:
        frame = ttk.LabelFrame(self.root, text=title)
        frame.grid(row=row, column=0, columnspan=2, sticky="ew", **PAD,
                   **grid)
        return frame

    def _value(self, parent, row: int, col: int, text: str = "-",
               font=None) -> ttk.Label:
        label = ttk.Label(parent, text=text, font=font or self.mono)
        label.grid(row=row, column=col, sticky="w", **PAD)
        return label

    def _build_connection(self, row: int) -> None:
        frame = self._frame(row, "connection")
        self.v_port = self._value(frame, 0, 0)
        self.v_who = self._value(frame, 0, 1)
        self.v_uptime = self._value(frame, 0, 2)
        self.v_heartbeat = self._value(frame, 0, 3)
        ttk.Button(frame, text="reconnect", command=self._reconnect).grid(
            row=0, column=4, sticky="e", **PAD)
        frame.columnconfigure(3, weight=1)

    def _build_gear(self, row: int) -> None:
        frame = self._frame(row, "gearing")
        self.v_gear = ttk.Label(frame, text="-", font=self.bold)
        self.v_gear.grid(row=0, column=0, sticky="w", **PAD)
        self.v_gear_note = self._value(frame, 1, 0)
        self.v_steps = self._value(frame, 2, 0)

    def _build_position(self, row: int) -> None:
        frame = self._frame(row, "position")
        self._value(frame, 0, 0, "az", font=self.bold)
        self._value(frame, 1, 0, "el", font=self.bold)
        self.v_az = self._value(frame, 0, 1)
        self.v_el = self._value(frame, 1, 1)
        self.v_zeroed = ttk.Label(frame, text="-", font=self.bold)
        self.v_zeroed.grid(row=0, column=2, rowspan=2, sticky="w", **PAD)
        frame.columnconfigure(2, weight=1)

    def _build_settings(self, row: int) -> None:
        frame = self._frame(row, "settings")
        for col, head in enumerate(("", "accel", "travel", "speed")):
            self._value(frame, 0, col, head, font=self.bold)
        self._value(frame, 1, 0, "az", font=self.bold)
        self._value(frame, 2, 0, "el", font=self.bold)

        self.v_accel = {}
        self.v_travel = {}
        self.v_speed = {}
        for line, axis in ((1, "az"), (2, "el")):
            self.v_accel[axis] = self._value(frame, line, 1)
            self.v_travel[axis] = self._value(frame, line, 2)
            self.v_speed[axis] = self._value(frame, line, 3)

        self.v_saved = self._value(frame, 3, 0)
        self.v_saved.grid(columnspan=4)

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

        commands = ttk.LabelFrame(self.root, text="commands (at the prompt)")
        commands.grid(row=row, column=1, sticky="nsew", **PAD)
        commands.rowconfigure(0, weight=1)
        commands.columnconfigure(0, weight=1)

        # Rendered from motor.py's own help, so this panel cannot drift away
        # from the commands that actually exist.
        text = tk.Text(commands, height=12, width=46, font=self.mono,
                       wrap="none", state="normal")
        text.insert("1.0", motor.HELP_TEXT)
        text.configure(state="disabled")
        text.grid(row=0, column=0, sticky="nsew")
        cbar = ttk.Scrollbar(commands, orient="vertical",
                             command=text.yview)
        cbar.grid(row=0, column=1, sticky="ns")
        text.configure(yscrollcommand=cbar.set)

    def _build_status(self, row: int) -> None:
        self.v_status = ttk.Label(self.root, text="starting…",
                                  anchor="w", relief="sunken")
        self.v_status.grid(row=row, column=0, columnspan=2, sticky="ew",
                           padx=6, pady=(0, 6))

    # --- events ------------------------------------------------------------

    def _reconnect(self) -> None:
        self.append("  ", "reconnecting…")
        self.worker.reconnect()

    def _drain(self) -> None:
        """Take everything the worker has said and show it. Tk thread only."""
        try:
            while True:
                kind, payload = self.worker.events.get_nowait()
                if kind == gw.EVENT_STATE:
                    self._state = payload
                    self.show(payload)
                elif kind == gw.EVENT_LINE:
                    self.append(*payload)
                elif kind == gw.EVENT_NOTE:
                    self.append("  ", payload)
        except Exception:
            pass  # queue.Empty, or the window is going away

        # The heartbeat age ticks on even when no event arrives, which is
        # exactly when it matters - a Pico that stopped talking.
        self._show_heartbeat(self._state)
        self.root.after(DRAIN_MS, self._drain)

    def append(self, direction: str, text: str) -> None:
        tag = "out" if direction == ">" else "in"
        self.log.configure(state="normal")
        self.log.insert("end", f"{direction} {text}\n", tag)
        extra = int(self.log.index("end-1c").split(".")[0]) - LOG_LINES
        if extra > 0:
            self.log.delete("1.0", f"{extra + 1}.0")
        self.log.see("end")
        self.log.configure(state="disabled")

    def _show_heartbeat(self, st: gw.State) -> None:
        if st.heartbeat_at is None:
            self.v_heartbeat.configure(text="heartbeat -")
            return
        age = self.clock() - st.heartbeat_at
        self.v_heartbeat.configure(text=f"heartbeat {age:.0f}s ago")

    def show(self, st: gw.State) -> None:
        """Put a State on the screen. Nothing is remembered between calls -
        what is displayed is what the Pico last said."""
        self.v_port.configure(text=st.port or "no port")
        self.v_who.configure(
            text=f"{st.target} {st.version}".strip() or "-")
        self.v_uptime.configure(text=f"up {_uptime(st.uptime_ms)}")
        self._show_heartbeat(st)

        if st.gear is None:
            self.v_gear.configure(text="gearing unknown")
            self.v_gear_note.configure(text="")
            self.v_steps.configure(text="")
        else:
            on = st.gear.geared
            self.v_gear.configure(
                text="GEAR ON - final pair counted" if on
                else "GEAR OFF - gearbox only")
            self.v_gear_note.configure(text=st.gear.describe())
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

        status = st.describe()
        if st.moving:
            status += " - readings below are from before the move started"
        self.v_status.configure(text=status)

    def _show_settings(self, st: gw.State) -> None:
        for axis in ("az", "el"):
            accel = travel = speed = "-"
            if st.cfg is not None:
                accel = (
                    f"{p.format_mdeg(getattr(st.cfg, axis + '_accel_mdps2'))}"
                    f" deg/s/s")
                travel = (
                    f"{p.format_mdeg(getattr(st.cfg, axis + '_min_mdeg'))}"
                    f" … {p.format_mdeg(getattr(st.cfg, axis + '_max_mdeg'))}"
                    f" deg")
            if st.lim is not None:
                speed = (
                    f"{p.format_mdeg(st.lim.speed_min_mdps)}"
                    f" … {p.format_mdeg(st.lim.speed_max_for(axis))} deg/s")
            self.v_accel[axis].configure(text=accel)
            self.v_travel[axis].configure(text=travel)
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

    def close(self) -> None:
        self.worker.stop()
        self.worker.join(timeout=2.0)
        self.root.destroy()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="py -m gs.gui",
        description="Watch the ground station's motor Pico in a window.",
    )
    parser.add_argument(
        "--port",
        help="serial port, such as COM5. Left out, the Pico is found by "
        "itself",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    # The worker opens the port itself, on its own thread, so a Pico that is
    # not plugged in leaves a window saying so rather than no window at all.
    worker = gw.MotorWorker(port=args.port)
    worker.start()

    root = tk.Tk()
    app = App(root, worker)
    try:
        root.mainloop()
    finally:
        worker.stop()
        worker.join(timeout=2.0)
    del app
    return 0


if __name__ == "__main__":
    sys.exit(main())
