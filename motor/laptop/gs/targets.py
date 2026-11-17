r"""Where a pointing direction comes from.

The tracking loop needs one thing: *where should the antenna point, now?*
This module is the boundary between that question and whatever answers it,
with several answers available and one interface over them.

    ManualSource   a direction you type. Nothing else involved, so it is how
                   the chase loop gets proved on the bench.
    MovingSource   a direction that will not keep still: it moves at a steady
                   rate and turns back every so many degrees, so there is
                   always a new target to chase without anybody typing one.
    SimSource      a rehearsal: a synthetic flight, or a recorded one
                   replayed from CSV. Takes its clock as a parameter, so a
                   two-minute flight can be run through in a test instantly.
    LiveSource     a direction worked out on the laptop from live data:
                   hand it a callable. The radio no longer needs it - see
                   RadioSource - but it stays for anything that does.

And two that hand over nothing, because the Pico steers itself:

    PicoReplaySource  the simulated flight built into its firmware.
    RadioSource       the rocket's GPS, received by the ground Pico over the
                      radio, turned into pointing on the Pico itself.

What a source hands over
------------------------

A `Fix`: an absolute azimuth and elevation, and the clock reading they were
worked out for.

**Absolute, not a change.** "Point at 45 degrees", never "turn 3 more". The
reason is what happens afterwards rather than taste: an `estop`, a re-zero, a
gear change or a reboot all throw away the Pico's idea of where it is, and an
absolute direction re-establishes everything on the next fix while a running
total has nothing left to add to. A target clamped at a travel limit has the
same problem - the next absolute fix recovers, a sum does not.

If the module upstream only produces changes, `DeltaSource` wraps it and does
the adding here, where it can at least be re-anchored when the zero moves.
That is a fallback, not the plan.

**With a time on it.** Nothing uses it yet beyond spotting a stale fix, but
it is the hook for extrapolating forward later if the LoRa rate turns out to
be slow enough that the antenna is always pointing where the rocket was.
"""

from __future__ import annotations

import csv
import math
import time
from dataclasses import dataclass

# Standard gravity, for the synthetic flight. Close enough: this is a
# rehearsal, not a trajectory prediction.
G_MS2 = 9.80665


@dataclass(frozen=True)
class Fix:
    """One absolute pointing direction, and the clock reading it belongs to."""

    az_deg: float
    el_deg: float
    at: float

    def age(self, now: float) -> float:
        return max(0.0, now - self.at)


class TargetSource:
    """What the tracking loop needs from anything that supplies directions.

    `latest` is asked at whatever rate the loop runs, which may be faster or
    slower than directions arrive, so it returns the newest one it has rather
    than queueing. None means it has nothing yet - the loop sends no target
    and the antenna stays where it is.
    """

    name = "source"

    # True when the Pico works out its own targets and nothing is sent from
    # here - the flight built into its firmware, or positions it receives
    # over the radio. The loop then never sends a TARGET: one from the
    # laptop would take over from whatever the Pico was following.
    pico_steers = False

    # True for the built-in flight alone - see PicoReplaySource. It changes
    # how tracking is armed (TRACK REPLAY instead of TRACK ON).
    pico_replay = False

    def latest(self, now: float) -> Fix | None:
        raise NotImplementedError

    def close(self) -> None:
        """Let go of anything held open. Safe to call twice."""


class ManualSource(TargetSource):
    """A direction somebody typed, held until they type another.

    The simplest possible source, and the useful one at a bench: it takes the
    rocket, the radio and the geometry out of the picture, so what is left
    under test is the chase loop itself.
    """

    name = "manual"

    def __init__(self, az_deg: float | None = None,
                 el_deg: float | None = None, clock=time.monotonic):
        self._clock = clock
        self._fix: Fix | None = None
        if az_deg is not None and el_deg is not None:
            self.point(az_deg, el_deg)

    def point(self, az_deg: float, el_deg: float) -> Fix:
        self._fix = Fix(float(az_deg), float(el_deg), self._clock())
        return self._fix

    def clear(self) -> None:
        self._fix = None

    def latest(self, now: float) -> Fix | None:
        return self._fix


def _out_and_back(travelled: float, span: float) -> float:
    """How far from the start, after `travelled` degrees of going out and back.

    Out to `span`, back to the start, out again: a triangle, so the speed is
    the same all the way and only the direction changes. The sign of
    `travelled` says which way is out.
    """
    lap = abs(travelled) % (2.0 * span)
    out = lap if lap <= span else 2.0 * span - lap
    return out if travelled >= 0 else -out


class MovingSource(TargetSource):
    """A direction that keeps moving, out at a steady rate and back again.

    The step between a typed direction and a whole flight. Each axis moves at
    its own rate from a starting direction, so the target changes every time
    one is sent - the same stream of replacements a live feed will be, with
    nobody typing them in. Each axis turns back after `span_deg` and returns
    to the start, over and over, so it can be left running without walking
    into a travel limit, and every turn is a reversal taken on the move.

    A negative rate goes out the other way; zero holds that axis still. Like
    SimSource it keeps no clock: `latest` is handed the time, and the first
    call is where the motion starts.
    """

    name = "moving"

    def __init__(self, az_deg: float, el_deg: float, az_rate_dps: float,
                 el_rate_dps: float, span_deg: float = 30.0):
        if span_deg <= 0:
            raise ValueError("span_deg must be positive")
        self._az = float(az_deg)
        self._el = float(el_deg)
        self._az_rate = float(az_rate_dps)
        self._el_rate = float(el_rate_dps)
        self._span = float(span_deg)
        self._began: float | None = None

    def latest(self, now: float) -> Fix:
        if self._began is None:
            self._began = now
        gone = now - self._began
        return Fix(self._az + _out_and_back(self._az_rate * gone, self._span),
                   self._el + _out_and_back(self._el_rate * gone, self._span),
                   now)


class SimSource(TargetSource):
    """A rehearsal. Either a flight worked out from kinematics, or a replay.

    It keeps no clock of its own: `latest` is handed the time, so a test can
    run a whole flight through in no time at all by passing its own - the
    same trick `motor_sim.py` uses in the repo this folder came from. The
    first call sets where zero is, so a flight starts when it is first asked
    rather than when it was built.
    """

    name = "sim"

    def __init__(self, fixes, loop: bool = False):
        # Sorted by time and held whole: a rehearsal is a few hundred points
        # at most, and being able to seek backwards is what makes `loop`
        # and a restart cheap.
        self._fixes = sorted(fixes, key=lambda f: f.at)
        self._loop = loop
        self._began: float | None = None
        if not self._fixes:
            raise ValueError("a simulated source needs at least one fix")

    @property
    def duration(self) -> float:
        return self._fixes[-1].at - self._fixes[0].at

    def latest(self, now: float) -> Fix | None:
        if self._began is None:
            self._began = now
        gone = now - self._began
        if self._loop and self.duration > 0:
            gone = gone % self.duration

        # The newest fix not in the future, which is what a receiver would
        # have in hand. Linear rather than bisect: the list is short and the
        # loop is slow, and a plain scan is easier to be sure of.
        found = self._fixes[0]
        for fix in self._fixes:
            if fix.at - self._fixes[0].at > gone:
                break
            found = fix
        return Fix(found.az_deg, found.el_deg, self._began + found.at -
                   self._fixes[0].at)

    # --- ways to build one -------------------------------------------------

    @classmethod
    def rocket(cls, *, bearing_deg: float = 0.0, downrange_m: float = 300.0,
               accel_ms2: float = 140.0, burn_s: float = 3.0,
               descent_ms: float = 20.0, drift_deg_s: float = 0.4,
               hz: float = 5.0) -> SimSource:
        """A plausible flight, as angles.

        Straight up from a pad `downrange_m` away, under power for `burn_s`,
        coasting to apogee, then down under a parachute while the wind walks
        the bearing round. The defaults reach about 9100 m - call it 30,000
        feet - from 300 m away.

        The point of it is the angular rates, not the altitudes. Being close
        to the pad early means elevation sweeps fastest in the first few
        seconds (20-40 deg/s with these numbers), which is the part that
        tests whether the axes can keep up at all.
        """
        v_burn = accel_ms2 * burn_s
        h_burn = 0.5 * accel_ms2 * burn_s * burn_s
        coast_s = v_burn / G_MS2
        apogee_m = h_burn + v_burn * coast_s - 0.5 * G_MS2 * coast_s * coast_s
        apogee_at = burn_s + coast_s
        fall_s = apogee_m / descent_ms
        total = apogee_at + fall_s

        def altitude(t: float) -> float:
            if t <= burn_s:
                return 0.5 * accel_ms2 * t * t
            if t <= apogee_at:
                dt = t - burn_s
                return h_burn + v_burn * dt - 0.5 * G_MS2 * dt * dt
            return max(0.0, apogee_m - descent_ms * (t - apogee_at))

        fixes = []
        step = 1.0 / hz
        t = 0.0
        while t <= total + 1e-9:
            el = math.degrees(math.atan2(altitude(t), downrange_m))
            # The bearing only moves once it is drifting under canopy.
            az = bearing_deg + drift_deg_s * max(0.0, t - apogee_at)
            fixes.append(Fix(az, el, t))
            t += step
        return cls(fixes)

    @classmethod
    def sweep(cls, *, az_from: float = 0.0, az_to: float = 90.0,
              el_from: float = 0.0, el_to: float = 80.0, seconds: float = 20.0,
              hz: float = 5.0) -> SimSource:
        """A straight line between two directions. For checking a rate."""
        fixes = []
        step = 1.0 / hz
        t = 0.0
        while t <= seconds + 1e-9:
            part = 1.0 if seconds <= 0 else min(1.0, t / seconds)
            fixes.append(Fix(az_from + (az_to - az_from) * part,
                             el_from + (el_to - el_from) * part, t))
            t += step
        return cls(fixes)

    @classmethod
    def from_csv(cls, path: str, loop: bool = False) -> SimSource:
        """Replay a recorded flight.

        Wants columns `t_s`, `az_deg`, `el_deg` - which is what a flight log
        from `flight_log.py` already has under `source_az`/`source_el`, so a
        real run can be replayed by renaming three headings.
        """
        fixes = []
        with open(path, newline="", encoding="utf-8") as handle:
            for row in csv.DictReader(handle):
                fixes.append(Fix(float(row["az_deg"]), float(row["el_deg"]),
                                 float(row["t_s"])))
        return cls(fixes, loop=loop)


class PicoReplaySource(TargetSource):
    """The simulated flight built into the Pico's firmware.

    Not a source of directions at all, from this side: arming with it sends
    TRACK REPLAY, and the Pico then feeds itself the flight in
    tracker/tests/bench_tests/replay. So `latest` never has anything and the
    loop sends no targets - it only reads the TRK stream back, which is what
    keeps the window's readout and its stop handling the same as for every
    other source. A target sent from here would end the replay, which is
    the other reason this one must never return a Fix.
    """

    name = "the Pico's built-in flight"
    pico_steers = True
    pico_replay = True

    def latest(self, now: float) -> Fix | None:
        return None


class RadioSource(TargetSource):
    """The rocket's own GPS, received by the ground Pico over the radio.

    Like PicoReplaySource, not a source of directions from this side. Arming
    with it sends TRACK ON, which on firmware with the receiver built in
    means "follow the radio": the ground Pico decodes each position, works
    out the pointing from the station position built into it, and sets the
    target itself. So `latest` never has anything and the loop only reads
    the TRK stream back. A target sent from here would take over from the
    radio, which is the other reason this one must never return a Fix.

    The packet ordering on the Pico is reset by TRACK OFF, so the worker
    sends one before arming with this source - see MotorWorker._arm_radio.
    """

    name = "the rocket, over the radio"
    pico_steers = True

    def latest(self, now: float) -> Fix | None:
        return None


class LiveSource(TargetSource):
    """Where the LoRa chain plugs in. Nothing behind it yet.

    Hand it a callable and it works: the callable is asked for the newest
    direction and should return `(az_deg, el_deg, fix_time)` or None. That is
    the whole contract, and it is deliberately the smallest thing that can be
    agreed with whoever writes the geometry.

    What is wanted from that module:

        az_deg, el_deg   ABSOLUTE, measured the same way the antenna's zero
                         is - degrees from true north, degrees above the
                         horizon. Not a change since the last one; see the
                         note at the top of this file.
        fix_time         the clock reading the fix is for, so staleness can
                         be spotted and so extrapolation can be added later
                         without changing anything here.

    If what arrives is a lat/lon/alt rather than angles, that is better still
    - wrap it in a callable that does the geometry and pass that. A position
    cannot be relative, so the question does not arise.
    """

    name = "live"

    CONTRACT = (
        "LiveSource has nothing behind it yet. Pass a callable returning "
        "(az_deg, el_deg, fix_time) with ABSOLUTE angles, or use "
        "SimSource/ManualSource until the LoRa chain is ready."
    )

    def __init__(self, read_fix=None):
        self._read = read_fix

    def latest(self, now: float) -> Fix | None:
        if self._read is None:
            raise NotImplementedError(self.CONTRACT)
        got = self._read()
        if got is None:
            return None
        az, el, at = got
        return Fix(float(az), float(el), float(at))


class DeltaSource(TargetSource):
    """Turns a stream of angle CHANGES into absolute directions.

    The fallback for a module that only produces "turn this much more". It
    keeps the running total here, which is the least bad place for it: the
    adding is at least visible, and it can be re-anchored.

    **Re-anchor whenever the Pico's zero moves.** An estop, a re-zero or a
    gear change throws away what the step count meant, so the total built on
    top of it is meaningless too. `anchor()` is how the tracking loop says
    so, and it calls it every time tracking is armed.

    Worth one message to whoever wrote the upstream module before settling
    for this: absolute costs them nothing and removes the whole problem.
    """

    name = "delta"

    def __init__(self, read_delta, az_deg: float = 0.0, el_deg: float = 0.0,
                 clock=time.monotonic):
        self._read = read_delta
        self._clock = clock
        self._az = float(az_deg)
        self._el = float(el_deg)

    def anchor(self, az_deg: float, el_deg: float) -> None:
        """Start the total again from here."""
        self._az = float(az_deg)
        self._el = float(el_deg)

    def latest(self, now: float) -> Fix | None:
        got = self._read()
        if got is not None:
            d_az, d_el = got
            self._az += float(d_az)
            self._el += float(d_el)
        return Fix(self._az, self._el, self._clock())
