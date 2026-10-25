r"""The GUI's background thread: the one place that talks to the Pico.

`session.py` is synchronous and holds one command at a time. Called straight
from a button handler it would freeze the window for up to two seconds, or
for a whole minute during a move. So the session lives on this thread and
nothing else is allowed to touch it:

    Tk thread  --- jobs --->  MotorWorker  --- commands --->  Pico
               <-- events ---              <--- replies ----

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

from gs import protocol as p
from gs import session as sn

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


class MotorWorker(threading.Thread):
    """Owns one Session and answers for it.

    Construct it, `start()` it, read `events` on the UI thread, and
    `shutdown()` it on the way out. `stop()` and `estop()` stop a *motor*,
    not this thread. `open_session` is injectable so a test can hand over a
    fake Pico instead of a serial port.
    """

    def __init__(self, port: str | None = None, open_session=None,
                 clock=time.monotonic):
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

    # --- the UI thread's side ----------------------------------------------

    def shutdown(self) -> None:
        """Close the port and finish the thread. Safe to call twice.

        Named apart from `stop`, which stops a motor. The two were briefly
        the same name and the lifecycle one lost, so the thread never exited.
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

        self._set_state(connected=True, port=port or "", error="",
                        rebooted=False)

        # Collect the banner before the first poll, the way motor.py does, so
        # what the Pico says about itself lands in the log in order.
        try:
            session.drain_until_quiet()
        except sn.SessionError as exc:
            self._emit(EVENT_NOTE, f"while reading the banner: {exc}")

    def _note_record(self, record: p.Reply) -> None:
        if isinstance(record, p.Mstat):
            # Tracked separately: a heartbeat is how the window knows the
            # Pico is alive while nothing is being asked of it.
            self._state = replace(self._state, heartbeat_at=self._clock())
        text = getattr(record, "text", None)
        self._emit(EVENT_LINE, ("<", text if text is not None else _record_text(record)))

    def _close(self) -> None:
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

        if self._estop_req.is_set():
            self._estop_req.clear()
            self._stop_req.clear()
            session.estop()
            self._emit(EVENT_NOTE,
                       "emergency stop sent - re-zero before moving again")
            return True
        if self._stop_req.is_set():
            self._stop_req.clear()
            session.stop()
            self._emit(EVENT_NOTE,
                       "stop sent - waiting for it to come to rest")
            return True
        return False

    def _await_move(self, plans) -> None:
        """Wait out every axis of a move, staying able to stop it.

        `await_done` is documented as safe to call again after a Timeout - the
        move keeps running on the Pico whatever this end does - so it is
        called with a short timeout in a loop and the urgent flags are checked
        between attempts. Same shape as the Ctrl-C handling in motor.py's
        `_wait_out`.
        """
        session = self._session
        if session is None:
            return

        self._emit(EVENT_MOVE, list(plans))
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
                        return
                    if self._clock() > deadline:
                        raise
                    continue
                outcomes.append(sn.MoveOutcome(plan, done))
                break

        self._emit(EVENT_DONE, outcomes)

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

                now = self._clock()
                if self._session is not None and now >= next_poll:
                    next_poll = now + POLL_INTERVAL_S
                    self._poll()
        finally:
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
