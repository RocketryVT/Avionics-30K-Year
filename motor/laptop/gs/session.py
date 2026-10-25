r"""Holding a conversation with the Pico.

`protocol.py` turns commands into text and text into records. `ports.py`
opens the port. This is the piece in the middle that actually talks, and
the prompt in `motor.py` is written against it.

Four jobs, each of which bites if it is skipped:

**Gluing lines back together.** USB hands over whatever has arrived, split
wherever it likes: half a line now and the rest in a moment, or two lines in
one lump. The Pico's line_reader does the same job in the other direction.

**One command at a time.** Nothing here ever has two commands outstanding.
That keeps matching a reply to its command simple, and it is the only way
an error with no id (below) can be attributed at all.

**Matching replies to commands.** A move takes up to a minute and its
replies arrive long after the command, with heartbeats in between. Every
reply carries the id of the command that caused it.

**Noticing a Pico that restarted.** PING and the heartbeat both report how
long the Pico has been up. If that goes backwards it rebooted, which means
it has forgotten where the antenna is. Moving after that without a fresh
zero would drive the gimbal from an imaginary starting point, so everything
but ZERO and the two stops is refused until someone re-zeros.

Errors with no id
-----------------

A few refusals come back as `ERR,0,...`: a line too long to have been read,
an unknown command word, an id that is not a number. In each of those the
Pico could not read an id to quote back. Because only one command is ever
outstanding, such an error can only be about that one, so it is treated as
its answer.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

from gs import ports
from gs import protocol as p

# How long to wait for the answer to an ordinary command. Generous: the Pico
# answers in microseconds, so anything approaching this means the cable or
# the port is in trouble, not that the Pico is thinking.
DEFAULT_TIMEOUT_S = 2.0

# A move is given its own predicted time plus this much slack. The
# prediction is the Pico's own, so all that is left to cover is the serial
# round trip and a slow laptop.
MOVE_SLACK_FACTOR = 1.5
MOVE_SLACK_S = 5.0

# Most replies kept while waiting for a different one. Older ones fall off
# rather than piling up during a long session.
SPARE_KEPT = 200


class SessionError(Exception):
    """Anything that went wrong talking to the Pico."""


class Timeout(SessionError):
    """The Pico did not answer in time."""


class Refused(SessionError):
    """The Pico understood and said no.

    Carries the ERR record, so a caller can tell a soft limit refusal from a
    typo and print the operator-ready message.
    """

    def __init__(self, err: p.Err) -> None:
        super().__init__(err.message())
        self.err = err


class Rebooted(SessionError):
    """The Pico restarted and no longer knows where the antenna is."""


@dataclass(frozen=True)
class MoveOutcome:
    """A finished move: what the Pico planned before it started (`plan`),
    and what it reported afterwards (`done`)."""

    plan: p.Plan
    done: p.Done


class Session:
    """One conversation with one Pico, over an open serial port."""

    def __init__(self, conn) -> None:
        self.conn = conn
        self.ids = p.IdSource()
        self.clock = time.monotonic
        self.timeout_s = DEFAULT_TIMEOUT_S

        # Called with every line the Pico sends that is not a reply: the
        # heartbeat, the startup banner, `[fatal]` messages.
        self.on_chatter = None

        # Called with every line sent, and with every record read, whatever
        # it is. These are for a log of the conversation itself - the GUI's
        # log pane - where `on_chatter` only carries the lines nobody asked
        # for. All three are called on whichever thread is driving the
        # session, so a handler should hand the line off rather than do work
        # in it.
        self.on_send = None
        self.on_record = None

        self._buf = b""
        # Replies read while waiting for a different one.
        self._spare: list[p.Reply] = []

        self.rebooted = False
        self._last_uptime_s: int | None = None

    @classmethod
    def open(cls, port: str | None = None) -> Session:
        """Open a port and talk over it. `port` may be a device name such as
        COM5, a URL, or None to find the Pico automatically.

        The read timeout is short on purpose. It is how often this layer gets
        to check its own deadline, not how long it is prepared to wait, so a
        long move still works - it just checks in often while nothing is
        arriving.
        """
        return cls(ports.open_port(port, timeout=0.2))

    def close(self) -> None:
        self.conn.close()

    # --- reading ------------------------------------------------------------

    def _read_some(self) -> bytes:
        """Whatever has arrived, or empty if nothing has."""
        waiting = getattr(self.conn, "in_waiting", 0) or 0
        try:
            return self.conn.read(max(1, waiting))
        except Exception as exc:  # the port vanished under us
            raise SessionError(f"lost the connection: {exc}") from exc

    def _next_record(self, deadline: float) -> p.Reply | None:
        """One record, or None once the deadline has passed."""
        while True:
            # Either line ending finishes a line, matching the Pico's own
            # reader, and a \r\n pair leaves an empty line which is dropped.
            # Whichever comes first wins: splitting on "\n" alone would
            # swallow a "\r" in the middle and join two records into one.
            cut = min(
                (
                    i
                    for i in (self._buf.find(b"\n"), self._buf.find(b"\r"))
                    if i >= 0
                ),
                default=-1,
            )
            if cut >= 0:
                head, self._buf = self._buf[:cut], self._buf[cut + 1 :]
                text = head.decode("ascii", errors="replace").strip()
                if text:
                    return self._note(p.parse_reply(text))
                continue  # blank line, or the second half of a \r\n pair

            if self.clock() >= deadline:
                return None

            data = self._read_some()
            if data:
                self._buf += data

    def _note(self, record: p.Reply) -> p.Reply:
        """Watch every record go past, for a reboot and for chatter."""
        # Uptime is compared in whole seconds. The heartbeat reports seconds
        # and PONG reports milliseconds, so comparing the two directly would
        # make an ordinary heartbeat look like a reboot.
        uptime_s = None
        if isinstance(record, p.Pong):
            uptime_s = record.uptime_ms // 1000
        elif isinstance(record, p.Mstat):
            uptime_s = record.uptime_s

        if uptime_s is not None:
            if self._last_uptime_s is not None and uptime_s < self._last_uptime_s:
                self.rebooted = True
            self._last_uptime_s = uptime_s

        if isinstance(record, (p.Mstat, p.Info)) and self.on_chatter is not None:
            self.on_chatter(record)
        if self.on_record is not None:
            self.on_record(record)

        return record

    def drain_until_quiet(
        self, quiet_s: float = 0.05, max_s: float = 1.0
    ) -> None:
        """Read until the Pico stops talking, or until `max_s` is up.

        Used at startup to collect the banner. Better than waiting a fixed
        time: a Pico that answers at once is not kept waiting, and one that
        is still printing is not cut off part way.
        """
        hard_stop = self.clock() + max_s
        while True:
            until = min(self.clock() + quiet_s, hard_stop)
            record = self._next_record(until)
            if record is None:
                return
            # A heartbeat means the Pico has nothing else to say: it only
            # sends one when nothing is moving and no line is half-read, so
            # the banner must be finished.
            if isinstance(record, p.Mstat) or self.clock() >= hard_stop:
                return

    # --- sending ------------------------------------------------------------

    def _send(self, line: str) -> None:
        # Wrapped the same way _read_some() wraps a failed read: a port that
        # went away raises whatever pyserial feels like, and everything above
        # here is written to expect SessionError. Without this a cable pulled
        # between two commands comes out as a bare OSError and takes the
        # caller down with it.
        try:
            self.conn.write(p.encode(line))
            self.conn.flush()
        except p.CommandError:
            raise  # ours, and a fault in the caller rather than the port
        except Exception as exc:
            raise SessionError(f"lost the connection: {exc}") from exc

        if self.on_send is not None:
            self.on_send(line)

    def _await(
        self,
        cmd_id: int,
        kinds: tuple,
        deadline: float,
        what: str,
        axis: str | None = None,
    ):
        """Wait for this command's answer, keeping anything else for later.

        `axis` narrows it to one axis's record: a two-axis MOVE gets a PLAN
        and a DONE for each axis, all under the one id.
        """
        # Something read earlier while waiting for a different answer.
        for i, record in enumerate(self._spare):
            if _matches(record, cmd_id, kinds, axis):
                return self._spare.pop(i)
            if _is_error_for(record, cmd_id):
                raise Refused(self._spare.pop(i))

        while True:
            record = self._next_record(deadline)
            if record is None:
                raise Timeout(
                    f"the Pico did not answer {what} (id {cmd_id}) in time"
                )
            if _matches(record, cmd_id, kinds, axis):
                return record
            if _is_error_for(record, cmd_id):
                raise Refused(record)
            if not isinstance(record, (p.Mstat, p.Info)):
                # A reply to something else - most likely the DONE of a move
                # whose wait was given up. Kept rather than dropped, so
                # whoever is waiting for it can still find it.
                self._spare.append(record)
                del self._spare[:-SPARE_KEPT]

    def request(self, line: str, cmd_id: int, kinds: tuple, what: str):
        """Send one command and wait for its answer."""
        if self.rebooted:
            raise Rebooted(
                "the Pico restarted and has forgotten where the antenna is. "
                "Re-zero before moving."
            )
        self._send(line)
        return self._await(cmd_id, kinds, self.clock() + self.timeout_s, what)

    # --- the commands -------------------------------------------------------

    def ping(self) -> p.Pong:
        cmd_id = self.ids.next()
        return self.request(p.ping(cmd_id), cmd_id, (p.Pong,), "PING")

    def limits(self) -> p.Lim:
        cmd_id = self.ids.next()
        return self.request(p.limits(cmd_id), cmd_id, (p.Lim,), "LIMITS")

    def position(self) -> p.Pos:
        cmd_id = self.ids.next()
        return self.request(p.pos(cmd_id), cmd_id, (p.Pos,), "POS")

    def zero(self, axis: str = "ALL", deg: float | None = None) -> p.Ack:
        """Tell the Pico what angle an axis is sitting at. Nothing moves.

        This also clears a noticed reboot, because re-zeroing is exactly
        what makes moving safe again.
        """
        cmd_id = self.ids.next()
        # Sent directly rather than through request(), which refuses while
        # the reboot flag is up - and this is the cure for that.
        self._send(p.zero(cmd_id, axis, deg))
        ack = self._await(
            cmd_id, (p.Ack,), self.clock() + self.timeout_s, "ZERO"
        )
        self.rebooted = False
        return ack

    def set_pin(self, gpio: int, level: int) -> p.Pin:
        cmd_id = self.ids.next()
        return self.request(p.pin(cmd_id, gpio, level), cmd_id, (p.Pin,), "PIN")

    # --- settings -----------------------------------------------------------

    def set_setting(self, axis: str, name: str, value: float) -> p.Ack:
        """Change one setting on one axis, until the Pico restarts.

        Raises Refused with an ERR range if the Pico will not have it, which
        carries what would have been allowed.
        """
        cmd_id = self.ids.next()
        line = p.set_setting(cmd_id, axis, name, value)
        return self.request(line, cmd_id, (p.Ack,), "SET")

    def save(self) -> p.Ack:
        """Keep the settings in force, so a restart comes back with them.

        The Pico writes its flash and reads it back before it answers, so
        the ACK means it is really there.
        """
        cmd_id = self.ids.next()
        return self.request(p.save(cmd_id), cmd_id, (p.Ack,), "SAVE")

    def defaults(self) -> p.Ack:
        """Back to the built-in settings, until a restart unless saved."""
        cmd_id = self.ids.next()
        return self.request(p.defaults(cmd_id), cmd_id, (p.Ack,), "DEFAULTS")

    def settings(self) -> p.Cfg:
        """The settings in force, and whether they are saved."""
        cmd_id = self.ids.next()
        return self.request(p.settings(cmd_id), cmd_id, (p.Cfg,), "SETTINGS")

    def gear(self) -> p.Gear:
        """Which gearing the Pico is counting. Changes nothing."""
        cmd_id = self.ids.next()
        return self.request(p.gear(cmd_id), cmd_id, (p.Gear,), "GEAR")

    def set_gear(self, on: bool) -> p.Gear:
        """Switch the final gear pair in or out of the maths.

        Returns what the Pico is counting afterwards, so the caller can
        confirm rather than assume.

        Asking for the mode already in force changes nothing. A real switch
        costs both axes their zero, so re-zero before moving - the Pico will
        refuse a move until you do.
        """
        cmd_id = self.ids.next()
        return self.request(p.gear(cmd_id, on), cmd_id, (p.Gear,), "GEAR")

    # --- moving -------------------------------------------------------------

    def begin_move(
        self, axis: str, speed_dps: float, delta_deg: float,
        absolute: bool = False,
    ) -> p.Plan:
        """Start a move and return its plan. Does not wait for it to finish.

        `absolute` reads the angle as where to end up rather than how far to
        turn, which is MOVETO rather than MOVE.
        """
        cmd_id = self.ids.next()
        line = (p.move_to(cmd_id, axis, speed_dps, delta_deg) if absolute
                else p.move(cmd_id, axis, speed_dps, delta_deg))
        self.request(line, cmd_id, (p.Ack,), "MOVETO" if absolute else "MOVE")
        return self._await(
            cmd_id,
            (p.Plan,),
            self.clock() + self.timeout_s,
            "the plan for MOVETO" if absolute else "the plan for MOVE",
        )

    def begin_move_both(
        self,
        az_speed_dps: float,
        az_delta_deg: float,
        el_speed_dps: float,
        el_delta_deg: float,
        absolute: bool = False,
    ) -> tuple[p.Plan, p.Plan]:
        """Start both axes in one command. Returns their plans, azimuth first.

        Each axis runs at its own speed. Either half being refused refuses
        the whole command, and nothing moves. `absolute` picks MOVETO over
        MOVE, as in `begin_move`.
        """
        cmd_id = self.ids.next()
        builder = p.move_to_both if absolute else p.move_both
        line = builder(
            cmd_id, az_speed_dps, az_delta_deg, el_speed_dps, el_delta_deg
        )
        self.request(line, cmd_id, (p.Ack,),
                     "MOVETO" if absolute else "MOVE")
        deadline = self.clock() + self.timeout_s
        verb = "MOVETO" if absolute else "MOVE"
        az = self._await(
            cmd_id, (p.Plan,), deadline, f"the azimuth plan for {verb}", "AZ"
        )
        el = self._await(
            cmd_id, (p.Plan,), deadline, f"the elevation plan for {verb}",
            "EL"
        )
        return az, el

    def await_done(self, plan: p.Plan, timeout_s: float) -> p.Done:
        """Wait for the move described by `plan` to report that it finished.

        Safe to call again after it raises Timeout, or after a Ctrl-C: the
        move keeps running on the Pico whatever this end does, so its DONE
        is still coming.

        Take the timeout from `move_timeout_s`, given every plan of the move:
        in a two-axis move neither DONE arrives until the slower axis stops.
        """
        return self._await(
            plan.id,
            (p.Done,),
            self.clock() + timeout_s,
            "the end of the move",
            plan.axis,
        )

    def stop(self) -> None:
        """Ask for a controlled stop.

        No reply is waited for. While an axis is turning the Pico answers a
        stop with the move's own DONE and nothing else, because it prints
        nothing between two pulses - so wait for that with await_done.
        """
        self._send(p.stop(self.ids.next()))

    def estop(self) -> None:
        """Stop at once. The Pico gives up its zero, so re-zero afterwards."""
        self._send(p.estop(self.ids.next()))


def move_timeout_s(*plans: p.Plan) -> float:
    """How long to wait for the DONEs of a move made of these plans.

    The Pico reports finished axes only once nothing at all is moving, so in
    a two-axis move the quicker axis's DONE waits for the slower one.
    """
    slowest = max(plan.est_s for plan in plans)
    return slowest * MOVE_SLACK_FACTOR + MOVE_SLACK_S


def _matches(
    record: p.Reply, cmd_id: int, kinds: tuple, axis: str | None = None
) -> bool:
    if not isinstance(record, kinds) or getattr(record, "id", None) != cmd_id:
        return False
    return axis is None or getattr(record, "axis", None) == axis


def _is_error_for(record: p.Reply, cmd_id: int) -> bool:
    """Is this refusal ours?

    Id 0 counts, because the Pico uses it for lines it could not read an id
    from - and with only one command outstanding there is nothing else it
    could be about.
    """
    return isinstance(record, p.Err) and record.id in (cmd_id, 0)
