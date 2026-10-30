r"""The GUI's background thread: the one place that talks to the Pico.

`session.py` is synchronous and holds one command at a time. Called straight
from a button handler it would freeze the window for up to two seconds, or
for a whole minute during a move. So the session lives on this thread and
nothing else is allowed to touch it:

    Tk thread  --- jobs --->  MotorWorker  --- commands --->  Pico
               <-- events ---              <--- replies ----

While tracking it also drives a `tracking.TrackingLoop`, for the same reason
it owns the session: the loop has to be ticked from the thread that holds the
port. The window picks a source and a rate and says go; everything after that
happens here.

It also holds a `motor.Console` - the text prompt's own command handling - so
a line typed in the window means exactly what it would mean at the prompt,
with the same parsing, the same usage errors and the same results file. Only
one thing is overridden: the wait for a move to finish, which has to be the
interruptible one so the STOP button can reach a typed `move` the same way it
reaches a clicked one.

Both directions go through `queue.Queue`, which is the only thing shared.
The worker never imports tkinter and never touches a widget; the window
never touches the session. That split is what keeps the UI responsive, and
it is also why this module can be tested against a fake Pico with no
display attached.

Nothing here caches anything the Pico could be asked for. Every `State` is
built from what the Pico just said, because a GUI showing a remembered
position is a GUI that disagrees with the machine - the same reason
`scripted_runs.py` asks for the limits rather than keeping a copy.

What polling cannot see
-----------------------

While an axis is turning, the firmware answers everything except the two
stop commands with ERR busy, and holds back its heartbeat as well. So a
poll during a move learns exactly one thing: that it is moving. `State`
says so with `moving`, and the readings beside it are whatever they were
before the move started. That is a property of the firmware - nothing is
printed between the first pulse and the last - not something this module
can work around.
"""

from __future__ import annotations

import queue
import threading
import time
from dataclasses import dataclass, field, replace

from gs import motor
from gs import protocol as p
from gs import session as sn
from gs import targets as tg
from gs import tracking

# How often the worker asks the Pico what it looks like. Each poll is five
# short commands and the Pico answers in microseconds, so this is nowhere
# near a load; it is set by how soon the window should notice a move
# finishing.
POLL_INTERVAL_S = 0.5

# How long the worker waits on its job queue before looking at the clock
# again. Small enough that a quit or a reconnect feels immediate.
JOB_WAIT_S = 0.05

# How long each attempt to collect a move's DONE waits before the worker
# looks at the stop flags again. A move can run for a minute, and this is how
# soon a stop reaches it.
MOVE_SLICE_S = 0.2

# The calibration sweep, run on one axis at a time: out this far, then back
# the same amount.
#
# It is the priming move found at the bench on 27 Oct 2026. A driver whose
# DIR input cannot change state cleanly from rest will take the first move
# after a power-up in whichever direction it is stuck in, and only reverse
# properly once it has turned once. Sweeping out and back leaves the axis
# where it started, so it can be run before anything that matters.
#
# 45 degrees is far enough to see which way the axis actually went from
# across the room, and 10 deg/s is slow enough to watch it the whole way.
# The out leg goes NEGATIVE first, so the axis has to be at least this far
# above its travel minimum or the Pico refuses the sweep: with elevation
# zeroed on the horizon its -25 minimum means it has to be jogged up first.
#
# IT IS A WORKAROUND, NOT A FIX, and nothing in the CL57TE manual asks for
# it: a healthy drive does not care which way it is asked to turn first.
# Needing this is a sign that the axis's DIR input is damaged.
CAL_SWEEP_DEG = 45.0
CAL_SPEED_DPS = 10.0

# How long the worker spends reading the port on each pass while tracking.
#
# Needed because nothing else reads it then: the usual poll is what drives
# reading, and polling is pointless while tracking (every question is refused
# as busy). Without this the TRK stream would pile up unread in the OS
# buffer and the window would show nothing.
TRACK_READ_S = 0.02


@dataclass(frozen=True)
class State:
    """Everything the window shows, as of the last poll.

    `moving` and `rebooted` are the two that change what the rest means:
    while moving, the readings are stale by definition, and after a reboot
    the Pico has forgotten where the antenna is and refuses to move until
    something re-zeros it.
    """

    connected: bool = False
    port: str = ""
    target: str = ""
    version: str = ""
    uptime_ms: int = 0

    moving: bool = False
    rebooted: bool = False

    # True while the worker is inside a session call. Distinct from `moving`,
    # which is what the Pico reports: a zero or a settings change makes the
    # worker busy without turning a motor. The window disables controls on
    # either.
    working: bool = False

    # --- tracking ---------------------------------------------------------
    # True from the moment tracking is armed until it is disarmed, by the
    # operator or by the Pico's own watchdog. While it is true the readings
    # above go stale, because every question is refused as busy - `sample`
    # is what the window shows instead.
    tracking: bool = False
    source_name: str = ""
    targets_sent: int = 0
    sample: object | None = None  # a tracking.Sample

    pos: p.Pos | None = None
    gear: p.Gear | None = None
    cfg: p.Cfg | None = None
    lim: p.Lim | None = None

    # Monotonic time the last heartbeat arrived, so the window can show how
    # long ago. None until one has been seen.
    heartbeat_at: float | None = None

    # What went wrong with the last poll, or "" if nothing did.
    error: str = ""

    @property
    def zeroed(self) -> bool:
        return self.pos is not None and self.pos.zeroed

    def describe(self) -> str:
        """One line for a status bar."""
        if not self.connected:
            return f"not connected{': ' + self.error if self.error else ''}"
        if self.rebooted:
            return "the Pico restarted - re-zero before moving"
        if self.tracking:
            where = f" from {self.source_name}" if self.source_name else ""
            return (f"TRACKING{where} - {self.targets_sent} targets sent")
        if self.moving:
            return "moving"
        if self.working:
            return "working"
        if self.error:
            return self.error
        if not self.zeroed:
            return "idle, NOT ZEROED"
        return "idle"


# Event kinds put on the out queue. The payload shape is per kind:
#   STATE  a State
#   LINE   (">" or "<", text)
#   NOTE   text, for things worth saying that are not a line or a state
EVENT_STATE = "state"
EVENT_LINE = "line"
EVENT_NOTE = "note"

# MOVE carries the plans of a move that has just started, so the window can
# show how long it should take; DONE carries the outcomes once it has
# finished. Neither is needed to drive the move - they are for the display.
EVENT_MOVE = "move"
EVENT_DONE = "done"


@dataclass
class _Job:
    name: str
    args: tuple = field(default_factory=tuple)


class _Lines:
    """Catches what the prompt prints and hands it over a line at a time.

    `motor.Console` writes with `print(..., file=out)`, so all it needs is
    something with `write`. Buffered until a newline because print sends the
    text and the newline separately.
    """

    def __init__(self, emit):
        self._emit = emit
        self._held = ""

    def write(self, text: str) -> int:
        self._held += text
        while "\n" in self._held:
            line, _, self._held = self._held.partition("\n")
            self._emit(line)
        return len(text)

    def flush(self) -> None:
        if self._held:
            self._emit(self._held)
            self._held = ""


class _NoInput:
    """Stands in for stdin, and always says nothing was typed.

    `sweep pause` and `random pause` ask for a protractor reading between
    moves. There is no terminal behind a window, so reading real stdin would
    block the worker for ever. An empty answer is what the prompt already
    treats as "skip this one".
    """

    def readline(self) -> str:
        return ""

    def isatty(self) -> bool:
        return False


class _GuiConsole(motor.Console):
    """The prompt's own command handling, driven from the window.

    Subclassed for one reason: `_wait_out` is where the prompt sits waiting
    for a move to finish, and the window needs that wait to be the worker's
    interruptible one. Otherwise a typed `move az 1 360` would hold the
    worker for six minutes with the STOP button unable to reach it, which
    would break the rule that the stops always work.

    Everything else - parsing, usage errors, the results file, the scripted
    runs - is the prompt's, unchanged, so a typed command here behaves
    exactly as it does there.
    """

    def __init__(self, session, worker, out):
        super().__init__(session, out=out)
        self._worker = worker
        self.stdin = _NoInput()

    def _wait_out(self, plans):
        return self._worker.await_move_outcomes(plans)


class MotorWorker(threading.Thread):
    """Owns one Session and answers for it.

    Construct it, `start()` it, read `events` on the UI thread, and
    `shutdown()` it on the way out. `stop()` and `estop()` stop a *motor*,
    not this thread. `open_session` is injectable so a test can hand over a
    fake Pico instead of a serial port.
    """

    def __init__(self, port: str | None = None, open_session=None,
                 clock=time.monotonic, results=None):
        super().__init__(daemon=True, name="motor-worker")
        self.events: queue.Queue = queue.Queue()
        self._jobs: queue.Queue = queue.Queue()
        self._stopping = threading.Event()

        # Stop and estop do not queue. A move holds the worker for as long as
        # it runs, and a stop that waited its turn behind the move it is
        # meant to cut short would be no stop at all. They are flags the
        # worker checks while it waits, which is also why they are idempotent
        # - pressing twice is the same as pressing once.
        self._stop_req = threading.Event()
        self._estop_req = threading.Event()
        self._port = port
        self._open = open_session or sn.Session.open
        self._clock = clock
        self._session: sn.Session | None = None
        self._state = State(port=port or "")

        # The tracking loop, when there is one. It lives here rather than in
        # the window for the same reason the session does: it has to be
        # driven from the thread that owns the port.
        self._loop: tracking.TrackingLoop | None = None
        self._source: tg.TargetSource | None = None

        # The prompt's command handling, for lines typed in the window. Built
        # when the port opens, because it needs the session.
        self._console: _GuiConsole | None = None

        # Results file for `true`, `note` and `rows`, when the window was
        # started with --log. Handed to the console when it is built.
        self.results = results

    # --- the UI thread's side ----------------------------------------------

    def shutdown(self) -> None:
        """Close the port and finish the thread. Safe to call twice.

        Named apart from `stop`, which stops a motor.
        """
        self._stopping.set()

    def reconnect(self, port: str | None = None) -> None:
        self._jobs.put(_Job("reconnect", (port,)))

    def poll_now(self) -> None:
        """Ask for a fresh State without waiting for the next tick."""
        self._jobs.put(_Job("poll"))

    def stop(self) -> None:
        """Ramp both axes to a halt. Jumps the queue; safe to call any time."""
        self._stop_req.set()

    def estop(self) -> None:
        """Halt at once, forfeiting the zero. Jumps the queue."""
        self._estop_req.set()

    def zero(self, axis: str = "ALL", deg: float | None = None) -> None:
        self._jobs.put(_Job("zero", (axis, deg)))

    def move(self, axis: str, speed_dps: float, delta_deg: float,
             absolute: bool = False) -> None:
        """Turn an axis. `absolute` reads the angle as where to end up."""
        self._jobs.put(_Job("move", (axis, speed_dps, delta_deg, absolute)))

    def calibrate(self, axis: str) -> None:
        """Sweep one axis out and back, to prime a sticky DIR input.

        One job rather than two moves queued from the window, so that a leg
        which is refused or stopped takes the other one with it. Queued
        separately, the return leg would still run on its own, and an axis
        that only did half the sweep would be displaced by the rest of it.
        """
        self._jobs.put(_Job("calibrate", (axis,)))

    def move_both(self, az_speed: float, az_delta: float, el_speed: float,
                  el_delta: float, absolute: bool = False) -> None:
        self._jobs.put(_Job(
            "move_both",
            (az_speed, az_delta, el_speed, el_delta, absolute)))

    def set_gear(self, on: bool) -> None:
        self._jobs.put(_Job("gear", (on,)))

    def set_setting(self, axis: str, name: str, value: float) -> None:
        self._jobs.put(_Job("set", (axis, name, value)))

    def save(self) -> None:
        self._jobs.put(_Job("save"))

    def defaults(self) -> None:
        self._jobs.put(_Job("defaults"))

    # --- tracking ----------------------------------------------------------

    def start_tracking(self, kind: str, hz: float, az_deg: float = 0.0,
                       el_deg: float = 0.0) -> None:
        """Arm tracking and start sending targets.

        `kind` is "manual" or "sim". The source is built here, on the
        worker's thread, because that is the only thread allowed to touch it
        afterwards.
        """
        self._jobs.put(_Job("track_start", (kind, hz, az_deg, el_deg)))

    def stop_tracking(self) -> None:
        self._jobs.put(_Job("track_stop"))

    def point_at(self, az_deg: float, el_deg: float) -> None:
        """Move a manual source's target. Ignored by any other source."""
        self._jobs.put(_Job("track_point", (az_deg, el_deg)))

    # --- typed commands ----------------------------------------------------

    def run_line(self, text: str) -> None:
        """Run one line exactly as the motor> prompt would.

        `stop`, `estop` and `quit` are picked off before they get here - see
        TYPED_URGENT in gui.py. Everything else goes through the prompt's own
        handling, so what is typed means the same in both places.
        """
        self._jobs.put(_Job("line", (text,)))

    # --- the worker's side -------------------------------------------------

    def _emit(self, kind: str, payload) -> None:
        self.events.put((kind, payload))

    def _set_state(self, **changes) -> State:
        self._state = replace(self._state, **changes)
        self._emit(EVENT_STATE, self._state)
        return self._state

    def _connect(self, port: str | None) -> None:
        self._close()
        try:
            session = self._open(port)
        except Exception as exc:  # usually PortError, message already fit
            self._set_state(connected=False, port=port or "",
                            error=str(exc))
            return

        session.on_send = lambda line: self._emit(EVENT_LINE, (">", line))
        session.on_record = self._note_record
        self._session = session

        # The window's own copy of the prompt. Its output goes to the log
        # pane, prefixed so a reply to something typed is easy to pick out
        # from the raw wire lines around it.
        self._console = _GuiConsole(
            session, self, out=_Lines(lambda line: self._emit(
                EVENT_LINE, ("|", line))))
        self._console.results = self.results

        self._set_state(connected=True, port=port or "", error="",
                        rebooted=False)

        # Collect the banner before the first poll, the way motor.py does, so
        # what the Pico says about itself lands in the log in order.
        try:
            session.drain_until_quiet()
        except sn.SessionError as exc:
            self._emit(EVENT_NOTE, f"while reading the banner: {exc}")

    def _note_record(self, record: p.Reply) -> None:
        if self._loop is not None:
            self._loop.note(record)
        if isinstance(record, p.Mstat):
            # Tracked separately: a heartbeat is how the window knows the
            # Pico is alive while nothing is being asked of it.
            self._state = replace(self._state, heartbeat_at=self._clock())
        text = getattr(record, "text", None)
        if text is None:
            text = _record_text(record)
        self._emit(EVENT_LINE, ("<", text))

    def _close(self) -> None:
        self._console = None
        if self._session is not None:
            try:
                self._session.close()
            except Exception:
                pass  # going away anyway; nothing useful to do about it
            self._session = None

    def _poll(self) -> None:
        session = self._session
        if session is None:
            return

        try:
            pong = session.ping()
            self._state = replace(
                self._state, target=pong.target, version=pong.version,
                uptime_ms=pong.uptime_ms, moving=False, error="",
            )
            # Position first: it is the one that changes on its own.
            pos = session.position()
            gear = session.gear()
            cfg = session.settings()
            lim = session.limits()
        except sn.Refused as refused:
            # Everything but stop and estop is refused while an axis turns,
            # so this is how a move looks from out here.
            if refused.err.code == "busy":
                self._set_state(moving=True, error="")
            else:
                self._set_state(error=refused.err.message())
            return
        except sn.Rebooted as exc:
            self._set_state(rebooted=True, error=str(exc))
            return
        except sn.SessionError as exc:
            # A vanished port is not something polling can recover from.
            lost = "lost the connection" in str(exc)
            if lost:
                self._close()
            self._set_state(connected=not lost, error=str(exc))
            return

        self._set_state(pos=pos, gear=gear, cfg=cfg, lim=lim, moving=False,
                        error="")

    # --- carrying out a job ------------------------------------------------

    def _apply_urgent(self) -> bool:
        """Send a stop or estop if one has been asked for.

        Called both while idle and in the middle of waiting out a move, which
        is the whole reason these two are flags rather than queued jobs.
        Estop wins if both are set.
        """
        session = self._session
        if session is None:
            self._stop_req.clear()
            self._estop_req.clear()
            return False

        # The stop goes out first, and only then is the tracking loop let
        # go: disarming costs a round trip, and a stop should not wait.
        if self._estop_req.is_set():
            self._estop_req.clear()
            self._stop_req.clear()
            session.estop()
            self._stop_tracking("stopped")
            self._emit(EVENT_NOTE,
                       "emergency stop sent - re-zero before moving again")
            return True
        if self._stop_req.is_set():
            self._stop_req.clear()
            session.stop()
            self._stop_tracking("stopped")
            self._emit(EVENT_NOTE,
                       "stop sent - waiting for it to come to rest")
            return True
        return False

    def await_move_outcomes(self, plans):
        """The interruptible wait, for _GuiConsole._wait_out.

        Same contract as the prompt's own `_wait_out`: the outcomes in plan
        order, or None if it was given up on.
        """
        return self._await_move(plans, narrate=False)

    def _await_move(self, plans, narrate: bool = True):
        """Wait out every axis of a move, staying able to stop it.

        `await_done` is documented as safe to call again after a Timeout - the
        move keeps running on the Pico whatever this end does - so it is
        called with a short timeout in a loop and the urgent flags are checked
        between attempts. Same shape as the Ctrl-C handling in motor.py's
        `_wait_out`.
        """
        session = self._session
        if session is None:
            return None

        # `narrate` is false for a typed move: the prompt prints its own plan
        # and done lines, and saying the same thing twice in one log pane is
        # worse than saying it once. The progress bar still gets its events.
        self._emit(EVENT_MOVE, (list(plans), narrate))
        deadline = self._clock() + sn.move_timeout_s(*plans)
        outcomes = []

        for plan in plans:
            while True:
                self._apply_urgent()
                try:
                    done = session.await_done(plan, timeout_s=MOVE_SLICE_S)
                except sn.Timeout:
                    if self._stopping.is_set():
                        # The window is closing with an axis still turning.
                        # Leaving it running is worse than losing the zero.
                        session.estop()
                        return None
                    if self._clock() > deadline:
                        raise
                    continue
                outcomes.append(sn.MoveOutcome(plan, done))
                break

        self._emit(EVENT_DONE, (outcomes, narrate))
        return [o.done for o in outcomes]

    def _run_job(self, job: _Job) -> None:
        session = self._session
        if session is None:
            self._emit(EVENT_NOTE, "not connected")
            return

        if job.name == "zero":
            axis, deg = job.args
            session.zero(axis, deg)
            where = "0.000" if deg is None else str(deg)
            self._emit(EVENT_NOTE,
                       axis + " is now " + where + " deg. Nothing moved.")
        elif job.name == "move":
            axis, speed, delta, absolute = job.args
            self._await_move([session.begin_move(axis, speed, delta,
                                                 absolute=absolute)])
        elif job.name == "move_both":
            *halves, absolute = job.args
            self._await_move(
                list(session.begin_move_both(*halves, absolute=absolute)))
        elif job.name == "calibrate":
            self._calibrate(session, *job.args)
        elif job.name == "gear":
            gear = session.set_gear(job.args[0])
            word = "on" if gear.geared else "off"
            self._emit(EVENT_NOTE,
                       "gear " + word + ": " + gear.describe())
        elif job.name == "set":
            axis, name, value = job.args
            session.set_setting(axis, name, value)
            self._emit(EVENT_NOTE,
                       axis.lower() + " " + name.lower() + " set")
        elif job.name == "save":
            session.save()
            self._emit(EVENT_NOTE,
                       "saved - a restart comes back with these settings")
        elif job.name == "defaults":
            session.defaults()
            self._emit(EVENT_NOTE, "back to the built-in settings")
        elif job.name == "track_start":
            self._start_tracking(session, *job.args)
        elif job.name == "track_stop":
            self._stop_tracking("disarmed")
        elif job.name == "line":
            if self._console is None:
                self._emit(EVENT_NOTE, "not connected")
            else:
                # Straight through the prompt's own handling, which catches
                # its own usage errors and refusals and prints them.
                self._console.run_line(job.args[0])
        elif job.name == "track_point":
            if isinstance(self._source, tg.ManualSource):
                self._source.point(*job.args)
                self._emit(EVENT_NOTE,
                           "pointing at az %.3f el %.3f" % job.args)

    def _calibrate(self, session, axis: str) -> None:
        """Turn one axis out and back, stopping if a leg does not finish.

        One axis at a time, because only the axis whose DIR input is sticky
        needs priming, and sweeping an axis that does not need it is travel
        spent for nothing.

        A refusal - an axis with no zero, a sweep that would cross a travel
        limit - raises out of here and _handle_job reports it, so the return
        leg never runs. What has to be checked by hand is a leg that STARTS
        and is then cut short, by a stop or by the window closing: finishing
        the sweep from there would not put the axis back, it would carry it
        on past where it began.
        """
        name = axis.lower()
        for leg, delta in (("out", -CAL_SWEEP_DEG), ("back", CAL_SWEEP_DEG)):
            done = self._await_move(
                [session.begin_move(axis, CAL_SPEED_DPS, delta)])
            if done is None or not all(d.ok for d in done):
                self._emit(
                    EVENT_NOTE,
                    name + " calibration stopped on the " + leg + " leg - "
                    "the axis is NOT back where it started, check the "
                    "position")
                return
        self._emit(EVENT_NOTE,
                   name + " calibrated: swept " + str(CAL_SWEEP_DEG) +
                   " deg each way, back where it started")

    def _handle_job(self, job: _Job) -> None:
        """Run one job, reporting what it ran into rather than dying.

        A refusal is the Pico's answer, not a fault here, so it is shown and
        the worker carries on. Anything that is not a SessionError would be a
        bug in this file, and takes the thread down where it gets noticed.
        """
        self._set_state(working=True)
        try:
            self._run_job(job)
        except p.CommandError as bad:
            # The line was never sent: a builder would not make it. That is a
            # fault at this end rather than the Pico's answer, but reporting
            # it beats killing the thread and leaving a window that has
            # quietly stopped updating.
            self._emit(EVENT_NOTE, "cannot send that: " + str(bad))
        except sn.Refused as refused:
            self._emit(EVENT_NOTE, "refused: " + str(refused))
        except sn.Rebooted as exc:
            self._set_state(rebooted=True)
            self._emit(EVENT_NOTE, str(exc))
        except sn.SessionError as exc:
            if "lost the connection" in str(exc):
                self._close()
                self._set_state(connected=False, error=str(exc))
            self._emit(EVENT_NOTE, str(exc))
        finally:
            self._set_state(working=False)

    def _start_tracking(self, session, kind, hz, az_deg, el_deg) -> None:
        self._stop_tracking(None)

        if kind == "manual":
            source = tg.ManualSource(az_deg, el_deg, clock=self._clock)
        elif kind == "sim":
            source = tg.SimSource.rocket(hz=hz)
        else:
            self._emit(EVENT_NOTE, f"no source called {kind!r}")
            return

        loop = tracking.TrackingLoop(session, source, hz=hz,
                                     clock=self._clock)
        # Arming can be refused - an axis not zeroed - and _handle_job turns
        # that into a note. Only keep the loop if it went through.
        loop.arm()
        self._source = source
        self._loop = loop
        self._set_state(tracking=True, source_name=source.name,
                        targets_sent=0, sample=None)
        self._emit(EVENT_NOTE,
                   f"tracking armed from {source.name} at {hz:g} Hz")

    def _stop_tracking(self, why: str | None) -> None:
        """Disarm and let the loop go. Safe when there is nothing running."""
        if self._loop is None:
            self._source = None
            return
        self._loop.disarm()
        self._loop = None
        self._source = None
        self._set_state(tracking=False, sample=None)
        if why:
            self._emit(EVENT_NOTE, f"tracking {why}")

    def _tracking_pass(self) -> None:
        """One turn of the tracking loop: read the port, then send if due.

        Reading first, because nothing else is reading while tracking - the
        usual poll is skipped, every question being refused as busy - and the
        TRK stream is the only thing reporting where the axes are.
        """
        session = self._session
        loop = self._loop
        if session is None or loop is None:
            return

        try:
            session.drain_until_quiet(quiet_s=TRACK_READ_S,
                                      max_s=TRACK_READ_S * 2)
            sample = loop.tick()
        except sn.SessionError as exc:
            self._emit(EVENT_NOTE, f"tracking stopped: {exc}")
            if "lost the connection" in str(exc):
                self._close()
                self._set_state(connected=False, error=str(exc))
            self._loop = None
            self._source = None
            self._set_state(tracking=False, sample=None)
            return

        if sample is None:
            return

        # The Pico gives up on its own if targets stop arriving, and says so
        # in the stream. Following it rather than arguing keeps the window
        # showing what the machine is actually doing.
        if sample.trk is not None and not sample.trk.armed:
            self._loop = None
            self._source = None
            self._set_state(tracking=False, sample=sample)
            self._emit(EVENT_NOTE,
                       "the Pico stopped tracking - no targets reached it, "
                       "or something stopped it")
            return

        self._set_state(sample=sample, targets_sent=loop.sent,
                        tracking=True)

    def run(self) -> None:
        self._connect(self._port)
        next_poll = self._clock()

        try:
            while not self._stopping.is_set():
                try:
                    job = self._jobs.get(timeout=JOB_WAIT_S)
                except queue.Empty:
                    job = None

                # Before the job, so a stop that arrived while something was
                # queued behind it still goes first.
                if self._apply_urgent():
                    next_poll = self._clock()

                if job is not None:
                    if job.name == "reconnect":
                        self._connect(job.args[0] or self._port)
                    elif job.name != "poll":
                        self._handle_job(job)
                    next_poll = self._clock()

                # While tracking, the loop replaces the poll: every
                # question would be refused as busy, and the TRK stream
                # carries the readings instead.
                if self._loop is not None:
                    self._tracking_pass()
                    next_poll = self._clock()
                    continue

                now = self._clock()
                if self._session is not None and now >= next_poll:
                    next_poll = now + POLL_INTERVAL_S
                    self._poll()
        finally:
            self._stop_tracking(None)
            self._close()


def _record_text(record: p.Reply) -> str:
    """A record as one line for the log.

    The parsed record rather than the raw text, because by the time it gets
    here the line has been split up. Info keeps its own text, which is why
    that case is handled before this is called.
    """
    name = type(record).__name__.upper()
    fields = [
        f"{key}={value}"
        for key, value in vars(record).items()
        if not key.startswith("_")
    ]
    return f"{name} " + " ".join(fields) if fields else name
