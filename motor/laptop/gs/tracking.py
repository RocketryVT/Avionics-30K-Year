r"""The tracking loop: keep telling the Pico where to point.

    source  ---Fix--->  TrackingLoop  ---TARGET--->  Pico
                                      <---TRK------

Thin on purpose. The Pico does the chasing - works out how fast to turn, when
to brake, when it has arrived - so what is left here is: ask the source where
to point, send it, and write down what came back.

Driveable two ways, which is why it is a class and not a function:

    from a terminal   `run()` below, or `py -m gs.track`, which owns the
                      waiting itself
    from the window   the GUI's worker calls `tick()` on its own loop and
                      `note()` from its record hook

Nothing in here blocks for longer than a command round trip, so either works.

**It does not wait for a target to be acknowledged**, because an accepted one
is answered with nothing at all. The TRK stream is the acknowledgement, and
`note()` is how it gets in. A loop that never sees a TRK still works - it just
logs blanks in the columns that come from one.

**Arming is separate from running.** `arm()` is a deliberate step, because
tracking is the one mode where the motors move without anybody pressing
anything. It needs both axes zeroed and the Pico refuses otherwise.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

from gs import protocol as p
from gs import session as sn
from gs import targets as tg

def wrap_azimuth(deg: float) -> float:
    """A compass bearing as an azimuth the Pico will accept: -180 .. +180.

    Sources speak in bearings, 0 to 360 clockwise from north. The Pico's
    azimuth runs -180 to +180 from its zero, with no wrap-around, so a bearing
    of 270 sent as-is would be held at the +180 limit - pointing 90 degrees
    the wrong way. 270 is -90; that is what goes on the wire.

    This assumes the azimuth zero was set facing north. If it was set facing
    somewhere else, every bearing is off by that much, and the place to fix
    it is the zero, not here.

    The seam: a target crossing south moves from +179.9 to -179.9, and the Pico
    turns a full circle the other way to follow it - correctly, by its own
    rules, because it will not cross +/-180. Put the seam where the rocket
    will not be.
    """
    return (float(deg) + 180.0) % 360.0 - 180.0


# How often a target goes out, unless told otherwise.
#
# Five a second against a target sweeping at 40 deg/s leaves the antenna up to
# 8 degrees behind just from the gap between fixes, which is comfortable
# inside a beam about 50 degrees wide. Slower than about 1 Hz and that gap
# starts to matter more than anything the chase loop does, and extrapolating
# forward becomes worth having.
DEFAULT_HZ = 5.0


@dataclass(frozen=True)
class Sample:
    """One pass of the loop: what was sent, and what the Pico last said."""

    t: float
    fix: tg.Fix | None
    trk: p.Trk | None

    @property
    def armed(self) -> bool:
        return self.trk is not None and self.trk.armed

    @property
    def error_deg(self) -> tuple[float, float] | None:
        """How far off each axis is, azimuth first. None without a TRK."""
        if self.trk is None:
            return None
        return (p.mdeg_to_deg(self.trk.az_error_mdeg),
                p.mdeg_to_deg(self.trk.el_error_mdeg))

    # True when the direction asked for was outside an axis's travel, so the
    # Pico is holding at the limit and doing the best it can. Worked out by
    # the loop from the fix and the limits it read when it armed - see
    # TrackingLoop._is_clamped. Worth showing: it means the rocket is out of
    # reach, rather than the error failing to close for some other reason.
    clamped: bool = False

    def describe(self) -> str:
        """One line fit to print while a flight is running."""
        if self.fix is None:
            return f"{self.t:7.2f}  no fix yet"
        head = (f"{self.t:7.2f}  want az {self.fix.az_deg:7.2f} "
                f"el {self.fix.el_deg:6.2f}")
        if self.trk is None:
            return head + ("   OUT OF REACH" if self.clamped
                           else "   (no report yet)")
        az_err, el_err = self.error_deg
        tail = (f"   at az {p.mdeg_to_deg(self.trk.az_pos_mdeg):7.2f} "
                f"el {p.mdeg_to_deg(self.trk.el_pos_mdeg):6.2f}"
                f"   off by {az_err:+6.2f} / {el_err:+5.2f}")
        if not self.trk.armed:
            tail += "   NOT ARMED"
        elif self.clamped:
            tail += "   OUT OF REACH (held at the limit)"
        return head + tail


class TrackingLoop:
    """Sends targets to one Pico from one source."""

    def __init__(self, session: sn.Session, source: tg.TargetSource, *,
                 hz: float = DEFAULT_HZ, log=None, clock=time.monotonic):
        if hz <= 0:
            raise ValueError("hz must be positive")
        self.session = session
        self.source = source
        self.period = 1.0 / hz
        self.log = log
        self.clock = clock

        self._began: float | None = None
        self._next_send: float | None = None
        self._trk: p.Trk | None = None
        self._last: Sample | None = None
        self._sent = 0

        # Read once when arming, and only used to notice a clamped target.
        # Asked for rather than assumed, like everything else that comes
        # from the Pico; None if it could not be read, and then clamping
        # simply goes unreported rather than being guessed at.
        self._lim: p.Lim | None = None

    # --- what the caller feeds in -----------------------------------------

    def note(self, record: p.Reply) -> None:
        """Hand it every record read. Only TRK is of interest.

        Called from whatever is reading the port - the GUI's worker thread,
        or `run()` below. Keeping the newest one is all that is needed: it is
        a stream of the same facts, not a queue of events.
        """
        if isinstance(record, p.Trk):
            self._trk = record

    @property
    def last(self) -> Sample | None:
        return self._last

    @property
    def sent(self) -> int:
        return self._sent

    @property
    def armed(self) -> bool:
        return self._trk is not None and self._trk.armed

    # --- arming ------------------------------------------------------------

    def arm(self) -> p.Trk:
        """Arm tracking. Raises Refused if the axes are not zeroed.

        A DeltaSource is re-anchored here, because this is the moment the
        Pico's idea of where it is becomes trustworthy again - see the note
        in targets.py.
        """
        trk = self.session.track(True)
        self._trk = trk
        try:
            self._lim = self.session.limits()
        except sn.SessionError:
            self._lim = None
        if isinstance(self.source, tg.DeltaSource):
            self.source.anchor(p.mdeg_to_deg(trk.az_pos_mdeg),
                               p.mdeg_to_deg(trk.el_pos_mdeg))
        return trk

    def disarm(self) -> p.Trk | None:
        """Stop tracking. Never raises - it is the way out."""
        try:
            trk = self.session.track(False)
            self._trk = trk
            return trk
        except sn.SessionError:
            # The port may already have gone, which is one of the reasons to
            # be disarming. The Pico's own watchdog covers it either way.
            return None

    # --- the loop ----------------------------------------------------------

    def _is_clamped(self, az_deg: float | None,
                    el_deg: float | None) -> bool:
        """Is the direction just asked for outside what the axes can reach?

        Worked out from the fix and the travel limits, not from what the Pico
        echoed back. Two reasons that is the right side to look at: the echo
        is always a round trip behind the fix, so comparing them flags every
        sample of a moving target; and an antenna legitimately resting at 0
        degrees is sitting on a travel limit, so looking for a target at a
        limit flags that too. Asking "did I just request something out of
        range" has neither problem, and the laptop already knows.
        """
        if az_deg is None or self._lim is None:
            return False
        az = p.deg_to_mdeg(az_deg)
        el = p.deg_to_mdeg(el_deg)
        return (az < self._lim.az_min_mdeg or az > self._lim.az_max_mdeg or
                el < self._lim.el_min_mdeg or el > self._lim.el_max_mdeg)

    def due(self, now: float | None = None) -> bool:
        now = self.clock() if now is None else now
        return self._next_send is None or now >= self._next_send

    def tick(self, now: float | None = None) -> Sample | None:
        """Send a target if one is due. Returns the Sample, or None if not.

        Safe to call as fast as the caller likes; it keeps its own cadence.
        """
        now = self.clock() if now is None else now
        if self._began is None:
            self._began = now
        if not self.due(now):
            return None
        self._next_send = now + self.period

        fix = self.source.latest(now)
        az = el = None
        if fix is not None:
            # The source's bearing, as the azimuth the Pico works in. The
            # flight log keeps the bearing in source_az and the Pico's own
            # number in target_az, so the conversion stays visible.
            az = wrap_azimuth(fix.az_deg)
            el = fix.el_deg
            self.session.set_target(az, el)
            self._sent += 1

        sample = Sample(t=now - self._began, fix=fix, trk=self._trk,
                        clamped=self._is_clamped(az, el))
        self._last = sample
        if self.log is not None:
            self.log.add(sample.t, fix, self._trk)
        return sample


def run(session: sn.Session, source: tg.TargetSource, *,
        hz: float = DEFAULT_HZ, seconds: float | None = None, log=None,
        on_sample=None, clock=time.monotonic, sleep=time.sleep) -> int:
    """Arm, send targets for a while, disarm. Returns how many were sent.

    For a terminal. The GUI drives `TrackingLoop` directly instead, because
    it has its own loop to fit into.

    `session.on_chatter` is borrowed for the duration so the TRK stream gets
    in, and handed back afterwards whatever happens - leaving it hooked would
    mean the prompt's own chatter printing stayed broken after a flight.
    """
    loop = TrackingLoop(session, source, hz=hz, log=log, clock=clock)
    was = session.on_chatter

    def listen(record):
        loop.note(record)
        if was is not None:
            was(record)

    session.on_chatter = listen
    try:
        # Inside the try: arming is refused if the axes are not zeroed, and
        # the hook has to be handed back even then.
        loop.arm()
        began = clock()
        while seconds is None or clock() - began < seconds:
            sample = loop.tick()
            if sample is not None and on_sample is not None:
                on_sample(sample)
            # A short sleep rather than a spin: the Pico is doing the work,
            # and a tenth of the send period keeps the cadence honest without
            # pinning a core.
            sleep(min(0.05, loop.period / 4.0))
    except KeyboardInterrupt:
        pass
    finally:
        loop.disarm()
        session.on_chatter = was
    return loop.sent
