r"""The two scripted runs: `sweep` and `random`.

Both are steps of the bench procedure that nobody wants to type out move by
move. They go through `Console.run_move`, so a scripted move and a typed one
use exactly the same code, are logged the same way, and stop the same way
on Ctrl-C.

**sweep** turns the same angle at a series of speeds, alternating direction
so it does not walk into a soft limit. It shows what the acceleration ramp
does to the fast rows: a short move commanded fast spends most of its time
speeding up and slowing down, so it averages well below the speed asked
for. That is the ramp, not a fault.

**random** sends the axis to unpredictable angles inside its limits. That
catches an error that only shows up at particular positions - a tight spot
in the gearing, or a step lost on one direction change.

Both ask the Pico for its limits rather than keeping a copy of
motor_config.hpp, so the two ends can never disagree about where the gimbal
may point. And both stop at the first move that does not finish: after a
refusal, the run's idea of where the antenna is no longer matches the
Pico's, and the moves after it would measure something nobody planned.
"""

from __future__ import annotations

import random

from gs import protocol as p

# The speeds a sweep uses unless told otherwise, in degrees per second.
# Spread wide on purpose: the slow end is where the bench procedure runs and
# the fast end is where the ramp starts to dominate.
DEFAULT_SWEEP_SPEEDS = (1.0, 2.0, 5.0, 10.0, 20.0, 45.0, 90.0)

DEFAULT_SWEEP_ANGLE_DEG = 45.0
DEFAULT_RANDOM_COUNT = 10
DEFAULT_RANDOM_SPEED_DPS = 5.0

# Kept clear of each soft limit when choosing a random target. A target
# landing exactly on the limit can round outward by half a step and be
# refused, which would end a run for no useful reason.
LIMIT_MARGIN_MDEG = 50  # 0.05 degrees

# A random target must be at least this far from where the axis is. Anything
# much closer rounds to no steps at all and the Pico refuses it.
MIN_RANDOM_MOVE_MDEG = 1000  # 1 degree


def _axis_limits(lim: p.Lim, axis: str) -> tuple[int, int]:
    if axis == "AZ":
        return lim.az_min_mdeg, lim.az_max_mdeg
    return lim.el_min_mdeg, lim.el_max_mdeg


def _axis_position(pos: p.Pos, axis: str) -> int:
    return pos.az_mdeg if axis == "AZ" else pos.el_mdeg


def _after_move(console, outcome, pause: bool, note: str) -> bool:
    """What both runs do after each move. False means stop the run."""
    if outcome is None:
        # run_move has already said why: refused, timed out, or Ctrl-C twice.
        return False

    results = console.results
    if results is None:
        return True

    row = results.rows[-1]
    results.annotate(note, row.row)
    if pause:
        return _take_reading(console, row)
    return True


def _take_reading(console, row) -> bool:
    """Ask for the protractor reading. False if the operator wants to stop.

    Asked between moves rather than at the end, because the antenna has to
    be still to be read and the next move would spoil it.
    """
    answer = console.ask(
        f"         row {row.row} - protractor reading in degrees "
        f"(enter to skip, q to stop): "
    )
    if answer.lower() in ("q", "quit", "stop"):
        return False
    if answer:
        try:
            entry = console.results.set_true(float(answer), row.row)
        except (ValueError, OverflowError):
            console.say(
                f"         {answer!r} is not a number - row left blank"
            )
        else:
            console.report_reading(entry, indent="         ")
    return True


# ============================================================================
# sweep
# ============================================================================


def sweep(
    console,
    axis: str = "AZ",
    angle_deg: float = DEFAULT_SWEEP_ANGLE_DEG,
    speeds=DEFAULT_SWEEP_SPEEDS,
    pause: bool = False,
) -> None:
    """The same angle at each speed in turn, alternating direction."""
    lim = console.session.limits()
    low, high = _axis_limits(lim, axis)

    usable = [s for s in speeds
              if lim.speed_min_mdps <= p.deg_to_mdeg(s) <= lim.speed_max_mdps]
    skipped = [s for s in speeds if s not in usable]
    if skipped:
        console.say(
            f"  skipping {', '.join(str(s) for s in skipped)} deg/s - "
            f"outside what this Pico allows"
        )
    if not usable:
        console.say("  nothing to do")
        return

    console.say(
        f"  sweep: {axis} {angle_deg} deg at "
        f"{', '.join(str(s) for s in usable)} deg/s"
    )

    step_mdeg = p.deg_to_mdeg(angle_deg)
    for speed in usable:
        here = _axis_position(console.session.position(), axis)

        # Go whichever way there is room for, so a sweep started near a limit
        # works its way back rather than stopping on the first move.
        if here + step_mdeg <= high:
            delta = angle_deg
        elif here - step_mdeg >= low:
            delta = -angle_deg
        else:
            console.say(
                f"  no room for {angle_deg} deg either way from "
                f"{p.format_mdeg(here)}. Move it clear and start again."
            )
            return

        outcome = console.run_move(axis, speed, delta, row_hint=not pause)
        if not _after_move(console, outcome, pause, f"sweep {speed}"):
            return

    console.say(f"  sweep done: {len(usable)} moves")


# ============================================================================
# random
# ============================================================================


def random_moves(
    console,
    axis: str = "AZ",
    count: int = DEFAULT_RANDOM_COUNT,
    speed_dps: float = DEFAULT_RANDOM_SPEED_DPS,
    seed: int | None = None,
    pause: bool = False,
) -> None:
    """Unpredictable angles inside the axis's limits."""
    lim = console.session.limits()
    low, high = _axis_limits(lim, axis)
    low += LIMIT_MARGIN_MDEG
    high -= LIMIT_MARGIN_MDEG

    if high <= low:
        console.say("  that axis has no room to move")
        return

    # Seeded on request, so a run that found something odd can be repeated
    # exactly. The seed is printed either way.
    if seed is None:
        seed = random.randrange(1_000_000)
    picker = random.Random(seed)

    console.say(
        f"  random: {count} moves on {axis} at {speed_dps} deg/s "
        f"within {p.format_mdeg(low)}..{p.format_mdeg(high)} deg, seed {seed}"
    )

    for _ in range(count):
        here = _axis_position(console.session.position(), axis)

        target = _pick_target(picker, here, low, high)
        if target is None:
            console.say("  could not find a target far enough away")
            return

        delta_deg = p.mdeg_to_deg(target - here)
        outcome = console.run_move(
            axis, speed_dps, delta_deg, row_hint=not pause
        )
        if not _after_move(console, outcome, pause, f"random {seed}"):
            return

    console.say(f"  random done: {count} moves, seed {seed}")


def _pick_target(picker, here: int, low: int, high: int) -> int | None:
    """An angle inside the limits that is actually somewhere else.

    Tried a handful of times rather than looped forever, because on a very
    narrow axis there may be nowhere to go.
    """
    for _ in range(20):
        target = picker.randint(low, high)
        if abs(target - here) >= MIN_RANDOM_MOVE_MDEG:
            return target
    return None
