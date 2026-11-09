r"""What the ground Pico says about its radio, made readable.

Firmware with the receiver built in prints one line a second:

    RX,ON,ok=812,bad=3,seq=815,tgt=640,e=0,rx=412,gap=1180

Every number in it is a running total since the Pico started, or a worst
case since then. On their own they answer "has anything ever arrived".
What an operator needs is "is the rocket being heard right now, and is it
steering the antenna", and that takes two lines and a clock. `LinkWatch`
keeps the last few seconds of RX lines and turns the totals into rates and
ages; `LinkStatus` is what it hands the window.

Nothing here talks to the Pico or touches a widget, so all of it can be
checked with a list of lines and a pretend clock.

What it can and cannot say
--------------------------

The clock is the laptop's, read when a line is *read*, not when the Pico
sent it. Lines can sit in the serial buffer for up to a poll, so rates are
worked out over several seconds, where half a second of jitter does not
matter much. "Last good packet" is only as fine as the RX lines are: about
a second.

`bad` is everything the Pico threw away - a bad read, a wrong length, a
failed decode, an out-of-order packet. It is not a count of packets lost in
the air. A packet that never arrived leaves no trace on the ground at all;
the sign of that is `ok` going up slower than the transmitter sends.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

from gs import protocol as p

# Rates are worked out over this much history. Long enough that a line read
# half a second late moves a rate by about a tenth, short enough that a
# dropout shows within a few seconds.
WINDOW_S = 5.0

# A rate needs at least this much time between its first and last line, or it
# is two lines read in one gulp and means nothing.
MIN_SPAN_S = 1.5

# A packet older than this means the link has gone quiet. While packets flow,
# every RX line shows `ok` up on the last, so the age cycles from 0 to about
# a second, plus up to half a second of reading delay - and one RX line can
# be dropped, which makes 2.5. The Pico's own watchdog waits much longer
# (p.TRACK_TIMEOUT_S) before it gives up, so this warns well before then.
QUIET_S = 2.5

# No RX line for this long means the reports themselves have stopped. They
# come once a second and can be dropped one at a time when the USB buffer is
# full, so one missing is normal and three is not.
REPORTS_LATE_S = 3.5

# The three things the headline can say about the link, for the window to
# colour. NONE is "nothing to judge yet", not a fault.
LEVEL_NONE = "none"
LEVEL_GOOD = "good"
LEVEL_WARN = "warn"
LEVEL_BAD = "bad"


@dataclass(frozen=True)
class LinkStatus:
    """The radio link as of the last RX line, plus when that was.

    Ages are worked out from `now` when asked, so the window can keep a
    "last packet 4 s ago" ticking between lines without a new status.
    """

    # When watching began - the connection - so a firmware that never sends
    # an RX line can be told from one whose first line is still on its way.
    since: float = 0.0
    rx: p.Rx | None = None
    report_at: float | None = None
    # When `ok` last went up. None until it has been seen to go up: the
    # first line's total says packets arrived, not when.
    last_ok_at: float | None = None
    # Per second over the last WINDOW_S, or None until there is enough
    # history to say.
    ok_rate: float | None = None
    target_rate: float | None = None
    # Packets thrown away over the same window.
    bad_recent: int = 0

    @property
    def reported(self) -> bool:
        return self.rx is not None

    def ok_age(self, now: float) -> float | None:
        return None if self.last_ok_at is None else now - self.last_ok_at

    def report_age(self, now: float) -> float | None:
        return None if self.report_at is None else now - self.report_at

    def hearing(self, now: float) -> bool:
        """True while good packets are arriving."""
        age = self.ok_age(now)
        late = self.report_age(now)
        return (self.rx is not None and self.rx.enabled and age is not None
                and age <= QUIET_S and late is not None
                and late <= REPORTS_LATE_S)

    def steering(self) -> bool:
        """True while radio positions are turning into motor targets."""
        return self.target_rate is not None and self.target_rate > 0

    def headline(self, now: float) -> tuple[str, str]:
        """What to say first about the link, and how worried to look."""
        if self.rx is None:
            if now - self.since > REPORTS_LATE_S:
                return ("no radio reports - this firmware may not have the "
                        "receiver built in", LEVEL_NONE)
            return "waiting for the first radio report", LEVEL_NONE

        late = self.report_age(now)
        if late is not None and late > REPORTS_LATE_S:
            return (f"radio reports stopped {late:.0f} s ago", LEVEL_WARN)

        if not self.rx.enabled:
            return ("RADIO OFF - " + self.rx.error_message(), LEVEL_BAD)

        if self.hearing(now):
            return "receiving", LEVEL_GOOD

        age = self.ok_age(now)
        if age is None:
            return "no packets yet", LEVEL_WARN
        return f"no packets for {age:.0f} s", LEVEL_WARN

    def details(self, now: float) -> str:
        """One line of numbers beside the headline.

        Blank before any report, and while the radio is off - the numbers
        have stopped and the headline already says why. Kept to one short
        line: the window has no height to spare, and a line wider than the
        panels below it widens the whole window.

        "rejected 2/5 s (40)" is two thrown away in the last five seconds,
        forty since the Pico started.
        """
        if self.rx is None or not self.rx.enabled:
            return ""
        rx = self.rx
        rate = "-" if self.ok_rate is None else f"{self.ok_rate:.1f}"
        age = self.ok_age(now)
        last = "never" if age is None else f"{age:.0f} s ago"
        steer = ("-" if self.target_rate is None
                 else f"{self.target_rate:.1f}/s")
        return (f"heard {rate}/s, last {last}  "
                f"rejected {self.bad_recent}/{WINDOW_S:.0f} s ({rx.bad})  "
                f"steering {steer}  seq {rx.seq}  "
                f"loop gap {rx.loop_max_us} us")


class LinkWatch:
    """Keeps the last few RX lines, and works out the link from them.

    Feed it every RX line with the time it was read. Make a new one when the
    connection changes: a status from one Pico says nothing about the next.
    """

    def __init__(self, now: float) -> None:
        self._since = now
        self._history: deque[tuple[float, p.Rx]] = deque()
        self._last_ok_at: float | None = None
        self._status = LinkStatus(since=now)

    @property
    def status(self) -> LinkStatus:
        return self._status

    def note(self, rx: p.Rx, now: float) -> LinkStatus:
        newest = self._history[-1][1] if self._history else None

        # Totals only ever go up while the Pico runs. One that went down
        # means it restarted, so nothing before this line can be compared
        # with it.
        if newest is not None and (rx.ok < newest.ok or rx.bad < newest.bad
                                   or rx.targets < newest.targets):
            self._history.clear()
            self._last_ok_at = None
            newest = None

        if newest is not None and rx.ok > newest.ok:
            self._last_ok_at = now

        self._history.append((now, rx))
        # Keep the newest line that is at least WINDOW_S old, so the window
        # always spans the full period once there is that much history.
        while (len(self._history) > 2
               and self._history[1][0] <= now - WINDOW_S):
            self._history.popleft()

        first_at, first = self._history[0]
        span = now - first_at
        ok_rate = target_rate = None
        if span >= MIN_SPAN_S:
            ok_rate = (rx.ok - first.ok) / span
            target_rate = (rx.targets - first.targets) / span

        self._status = LinkStatus(
            since=self._since, rx=rx, report_at=now,
            last_ok_at=self._last_ok_at, ok_rate=ok_rate,
            target_rate=target_rate, bad_recent=rx.bad - first.bad,
        )
        return self._status
