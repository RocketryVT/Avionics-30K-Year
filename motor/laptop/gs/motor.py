r"""The motor prompt: `py -m gs.motor`.

This is the program an operator uses. `session.py` holds the conversation
with the Pico and `protocol.py` knows the wire format, so what is left here
is turning typed words into those calls and printing the answers in a form
worth reading at a bench.

    py -m gs.motor                  finds the Pico by itself
    py -m gs.motor --port COM5      when there is more than one
    py -m gs.motor --log            and write a results file as it goes

Type `help` at the prompt for the commands.

Why a prompt and not a pile of flags
------------------------------------

The bench procedure is a sequence: zero it, move it, read the protractor,
write the number down, move it again. A prompt keeps one connection and one
results file open for the whole sequence, so each reading lands on the row
of the move it belongs to.

Two rules it will not bend
--------------------------

**One command at a time.** Enforced in `session.py`. Nothing here can send a
second command while one is still being answered.

**Ctrl-C stops the motor, it does not abandon it.** Pressing it during a
move sends a stop and then goes on waiting for the move to report what it
did. Walking away from a running move would leave the Pico mid-turn with
the laptop no longer listening - and the next command would read the old
move's answer.
"""

from __future__ import annotations

import argparse
import math
import sys

from gs import ports
from gs import protocol as p
from gs import results_file
from gs import scripted_runs
from gs import session as sn

PROMPT = "motor> "

# A protractor reading further than this from the predicted angle is
# probably not a measurement of the same thing. The usual cause is typing
# how far it moved instead of where it ended up - the two are the same
# number on the first move of a run and nowhere near it after that.
READING_WARN_DEG = 5.0

HELP_TEXT = """\
  move <az|el> <deg/s> <degrees>   turn an axis, + or -
  move az <deg/s> <deg> el <deg/s> <deg>   both at once, each at its own speed
  goto <az|el> <deg/s> <degrees>   turn TO an angle, not by one
  goto az <deg/s> <deg> el <deg/s> <deg>   point both at once
  zero [az|el|all] [degrees]       say where it is now
  pos                              where it thinks it is
  stop                             slow to a stop
  estop                            stop at once (loses the zero)
  pin <gpio> <0|1>                 hold a pin high or low, for a meter
  ping                             is it there, and how long up
  limits                           the travel and speed it allows
  settings                         acceleration and travel, and if saved
  set <az|el> accel <deg/s/s>      change the acceleration, for now
  set <az|el> <min|max> <degrees>  narrow the travel, for now
  set <az|el> deadband <degrees>   how close to a target counts as there
  save                             keep the settings after a restart
  defaults                         back to the numbers in motor_config.hpp
  gear [on|off]                    count the final gear pair, or not
  track [on|off]                   arm or disarm chasing a target
  sweep [angle=45] [speeds=1,5,90] [pause]   one angle at each speed
  random [count=10] [speed=5] [seed=N] [pause]   angles within the limits
  true [row] <degrees>             what the protractor really reads
  note [row] <text>                write a note against a row
  rows [n]                         show the last n rows of the table
  verbose [on|off]                 show the heartbeat
  help                             this
  quit                             leave

  Angles are relative: 'move az 5 -45' turns back 45 degrees.
  'goto' is absolute: 'goto az 5 45' points at 45, wherever it started.
  Ctrl-C during a move stops the motor, then waits for its report.
  'gear on' means the belt is fitted and angles are at the antenna;
  'gear off' means each axis on its own, angles at the gearbox shaft.
  To track, close this prompt and run 'py -m gs.track' (or use the window):
  it arms tracking and sends the targets. 'track on' here only arms it, and
  with nothing sending targets the Pico gives up after a second or so."""


class UsageError(Exception):
    """The operator typed a command we know, with arguments we do not."""


def _axis(word: str, allow_all: bool = False) -> str:
    name = word.strip().upper()
    allowed = p.ZERO_AXES if allow_all else p.MOVE_AXES
    if name not in allowed:
        names = " or ".join(a.lower() for a in allowed)
        raise UsageError(f"axis must be {names}")
    return name


def _on_off(args: list[str], usage: str) -> bool | None:
    """The one optional on/off word `gear` and `track` take. None if absent."""
    if not args:
        return None
    if len(args) != 1:
        raise UsageError(usage)
    word = args[0].strip().lower()
    if word in ("on", "yes", "true", "1"):
        return True
    if word in ("off", "no", "false", "0"):
        return False
    raise UsageError(usage)


def _looks_like_a_number(word: str) -> bool:
    try:
        float(word)
    except ValueError:
        return False
    return True


def _number(word: str, what: str) -> float:
    try:
        value = float(word)
    except ValueError:
        raise UsageError(f"{what} must be a number, got {word!r}") from None
    # float() also accepts "inf" and "nan", which no motor can do anything
    # with and which cannot be turned into millidegrees.
    if not math.isfinite(value):
        raise UsageError(f"{what} must be a number, got {word!r}")
    return value


def _options(args: list[str], allowed: set[str], flags=("pause",)):
    """Read `name=value` arguments, plus bare words like `pause`.

    The scripted runs have too many knobs to remember in order, and getting
    the order wrong silently would mean running the wrong test.
    """
    values: dict[str, str] = {}
    marks: set[str] = set()

    for token in args:
        if "=" in token:
            key, _, value = token.partition("=")
            key = key.strip().lower()
            if key not in allowed:
                known = ", ".join(sorted(allowed))
                raise UsageError(f"no setting called {key!r}. Try: {known}")
            values[key] = value.strip()
        elif token.lower() in flags:
            marks.add(token.lower())
        else:
            raise UsageError(
                f"do not understand {token!r} - settings look like name=value"
            )
    return values, marks


class Console:
    """The prompt itself, with the session it drives.

    Everything it prints goes through `say`, to the stream it was given.
    """

    def __init__(
        self,
        session: sn.Session,
        out=None,
        verbose: bool = False,
        results: results_file.ResultsFile | None = None,
    ):
        self.session = session
        self.verbose = verbose
        self.results = results
        # Where typed lines, and the protractor readings that `pause` asks
        # for, are read from.
        self.stdin = sys.stdin
        self._out = out if out is not None else sys.stdout
        self.quit = False

    # --- output ------------------------------------------------------------

    def say(self, text: str = "") -> None:
        print(text, file=self._out)

    def ask(self, question: str) -> str:
        """Put a question to the operator and wait for the answer.

        Used by the scripted runs when they pause between moves. An empty
        answer, or no input at all, means skip it - a reading nobody took is
        better left blank than guessed at.
        """
        print(question, end="", file=self._out, flush=True)
        try:
            line = self.stdin.readline()
        except (KeyboardInterrupt, ValueError):
            return ""
        if not line:
            return ""
        answer = line.strip()
        if not self.stdin.isatty():
            # No terminal to echo it, so echo it ourselves and keep the
            # transcript readable.
            self.say(answer)
        return answer

    def show_chatter(self, record: p.Reply) -> None:
        """Lines the Pico sends unprompted.

        The heartbeat is hidden unless asked for - one line every five
        seconds would scroll everything else away during a long session.
        Anything else it says is shown, because the only other things it
        says unprompted are the startup banner and bad news.
        """
        if isinstance(record, p.Mstat):
            if self.verbose:
                self.say(
                    f"       heartbeat  az {p.format_mdeg(record.az_mdeg)}  "
                    f"el {p.format_mdeg(record.el_mdeg)}  "
                    f"up {record.uptime_s} s"
                )
        elif isinstance(record, p.Trk):
            # The tracking stream. Hidden like the heartbeat unless asked
            # for: at a few lines a second it would scroll everything else
            # away, and `py -m gs.track` is where it is meant to be watched.
            if self.verbose:
                self.say(
                    f"       tracking   az {p.format_mdeg(record.az_pos_mdeg)}"
                    f" of {p.format_mdeg(record.az_target_mdeg)}"
                    f"   el {p.format_mdeg(record.el_pos_mdeg)}"
                    f" of {p.format_mdeg(record.el_target_mdeg)}"
                )
        elif isinstance(record, p.Info) and record.text:
            mark = "  ??" if record.malformed else "    "
            self.say(f"{mark}   {record.text}")

    # --- the commands -------------------------------------------------------

    def do_help(self, args: list[str]) -> None:
        self.say(HELP_TEXT)

    def do_quit(self, args: list[str]) -> None:
        self.quit = True

    def do_ping(self, args: list[str]) -> None:
        pong = self.session.ping()
        up_s = pong.uptime_ms / 1000.0
        self.say(f"  {pong.target} {pong.version}, up {up_s:.1f} s")

    def do_limits(self, args: list[str]) -> None:
        lim = self.session.limits()
        self.say(
            f"  travel  az {p.format_mdeg(lim.az_min_mdeg)}"
            f"..{p.format_mdeg(lim.az_max_mdeg)} deg"
            f"   el {p.format_mdeg(lim.el_min_mdeg)}"
            f"..{p.format_mdeg(lim.el_max_mdeg)} deg"
        )
        # One ceiling per axis: geared mode scales each by its own gear
        # pair, so they are not the same number.
        self.say(
            f"  speed   az {p.format_mdeg(lim.speed_min_mdps)}"
            f"..{p.format_mdeg(lim.az_speed_max_mdps)} deg/s"
            f"   el {p.format_mdeg(lim.speed_min_mdps)}"
            f"..{p.format_mdeg(lim.el_speed_max_mdps)} deg/s"
        )

    def do_pos(self, args: list[str]) -> None:
        pos = self.session.position()
        state = "zeroed" if pos.zeroed else "NOT ZEROED - send zero first"
        self.say(
            f"  az {p.format_mdeg(pos.az_mdeg)} deg ({pos.az_steps} steps)"
            f"   el {p.format_mdeg(pos.el_mdeg)} deg ({pos.el_steps} steps)"
            f"   {state}"
        )

    def do_zero(self, args: list[str]) -> None:
        # "zero 90" means all axes at 90, not an axis called 90.
        if args and _looks_like_a_number(args[0]):
            args = ["ALL"] + args

        axis = _axis(args[0], allow_all=True) if args else "ALL"
        deg = _number(args[1], "angle") if len(args) > 1 else None
        if len(args) > 2:
            raise UsageError("zero takes at most an axis and an angle")

        self.session.zero(axis, deg)
        where = "0.000" if deg is None else f"{deg}"
        self.say(f"  {axis} is now {where} deg. Nothing moved.")

    def do_pin(self, args: list[str]) -> None:
        if len(args) != 2:
            raise UsageError("pin <gpio> <0|1>")
        gpio = int(_number(args[0], "gpio"))
        level = int(_number(args[1], "level"))
        reply = self.session.set_pin(gpio, level)
        self.say(f"  GP{reply.gpio} is now {'high' if reply.level else 'low'}")

    def do_stop(self, args: list[str]) -> None:
        self.session.stop()
        self.say("  stop sent. Nothing was moving, if nothing answers.")

    def do_estop(self, args: list[str]) -> None:
        self.session.estop()
        self.say("  emergency stop sent. Re-zero before moving again.")

    def do_verbose(self, args: list[str]) -> None:
        if args:
            self.verbose = args[0].lower() in ("on", "yes", "true", "1")
        self.say(f"  heartbeat {'shown' if self.verbose else 'hidden'}")

    # --- settings -----------------------------------------------------------

    def do_settings(self, args: list[str]) -> None:
        self.show_settings()

    def do_set(self, args: list[str]) -> None:
        if len(args) != 3:
            raise UsageError("set <az|el> <accel|min|max> <value>")
        axis = _axis(args[0])
        name = args[1].strip().upper()
        if name not in p.SETTING_NAMES:
            allowed = ", ".join(n.lower() for n in p.SETTING_NAMES)
            raise UsageError(f"setting must be {allowed}")
        value = _number(args[2], name.lower())

        self.session.set_setting(axis, name, value)
        unit = "deg/s/s" if name == "ACCEL" else "deg"
        # DEADBAND only bites while tracking, which is worth saying so nobody
        # wonders why a move ignored it.
        if name == "DEADBAND":
            unit = "deg (used when tracking)"
        self.say(
            f"  {axis.lower()} {name.lower()} set to "
            f"{p.format_mdeg(p.deg_to_mdeg(value))} {unit}"
        )
        self.show_settings()

    def do_save(self, args: list[str]) -> None:
        self.session.save()
        self.say("  saved. A restart will come back with these settings.")

    def do_defaults(self, args: list[str]) -> None:
        self.session.defaults()
        self.say("  back to the built-in settings")
        self.show_settings()

    def do_gear(self, args: list[str]) -> None:
        """Report or change which gearing the Pico counts."""
        wanted = _on_off(args, "gear [on|off]")
        if wanted is None:
            self.report_gear(self.session.gear())
            return

        # Asked first so we can say whether anything actually changed. The
        # Pico treats setting the mode already in force as a no-op, zero
        # included, and the operator should hear which happened.
        before = self.session.gear()
        after = self.session.set_gear(wanted)
        self.report_gear(after)

        if after.geared == before.geared:
            self.say("  already set that way - nothing changed")
            return
        self.say(
            "  both axes have lost their zero: the drive train just "
            "changed, so where they are is no longer known."
        )
        self.say("  send zero before the next move.")
        self.show_settings()

    def do_track(self, args: list[str]) -> None:
        """Arm or disarm chasing a target, or just report."""
        wanted = _on_off(args, "track [on|off]")
        if wanted is None:
            self.report_track(self.session.track())
            return

        trk = self.session.track(wanted)
        self.report_track(trk)
        if trk.armed:
            # Worth saying, because arming on its own looks like nothing
            # happened - and then the watchdog quietly undoes it.
            self.say(
                "  armed. Nothing will move until targets arrive, and the "
                "Pico gives up if they stop."
            )
            self.say(
                "  to send them, close this prompt and run: "
                "py -m gs.track --sim"
            )

    def report_track(self, trk: p.Trk) -> None:
        self.say(f"  tracking {'ARMED' if trk.armed else 'off'}")
        self.say(
            f"         az target {p.format_mdeg(trk.az_target_mdeg)}"
            f"  at {p.format_mdeg(trk.az_pos_mdeg)}"
            f"  off by {p.format_mdeg(trk.az_error_mdeg)} deg"
        )
        self.say(
            f"         el target {p.format_mdeg(trk.el_target_mdeg)}"
            f"  at {p.format_mdeg(trk.el_pos_mdeg)}"
            f"  off by {p.format_mdeg(trk.el_error_mdeg)} deg"
        )

    def report_gear(self, gear: p.Gear) -> None:
        # Every results row is marked with the mode it was taken in, so
        # keep that current whenever we learn what the mode is - on a bare
        # query as well as after a switch.
        if self.results is not None:
            self.results.gear = "on" if gear.geared else "off"
        self.say(f"  gear {'on' if gear.geared else 'off'}: {gear.describe()}")
        self.say(
            f"         az {gear.az_steps_per_deg:.3f} steps/deg"
            f"   el {gear.el_steps_per_deg:.3f} steps/deg"
        )

    def show_settings(self) -> None:
        cfg = self.session.settings()
        # First, because it decides what every number under it means.
        self.say(
            f"  gear {'on' if cfg.geared else 'off'} - angles at the "
            f"{'antenna' if cfg.geared else 'gearbox output shaft'}"
        )
        for name, accel, low, high, dead in (
            ("az", cfg.az_accel_mdps2, cfg.az_min_mdeg, cfg.az_max_mdeg,
             cfg.az_deadband_mdeg),
            ("el", cfg.el_accel_mdps2, cfg.el_min_mdeg, cfg.el_max_mdeg,
             cfg.el_deadband_mdeg),
        ):
            self.say(
                f"  {name}  accel {p.format_mdeg(accel)} deg/s/s"
                f"   travel {p.format_mdeg(low)}..{p.format_mdeg(high)} deg"
                f"   deadband {p.format_mdeg(dead)} deg"
            )
        if not cfg.saved:
            self.say(
                "  not saved - a restart would lose them. "
                "Type save to keep them."
            )
        elif cfg.built_in:
            # The gear mode is saved with the rest, so say that a restart
            # comes back here - it is the thing most worth confirming
            # straight after switching.
            self.say(
                "  the built-in numbers from motor_config.hpp for this "
                "gear mode - a restart comes back here"
            )
        else:
            self.say("  saved - a restart comes back with these")

    # --- moving -------------------------------------------------------------

    def do_move(self, args: list[str]) -> None:
        self._move_command(args, absolute=False)

    def do_goto(self, args: list[str]) -> None:
        """The same as move, with the angle read as where to end up."""
        self._move_command(args, absolute=True)

    def _move_command(self, args: list[str], absolute: bool) -> None:
        word = "goto" if absolute else "move"
        if len(args) == 3:
            axis = _axis(args[0])
            speed = _number(args[1], "speed")
            angle = _number(args[2], "angle")
            self.run_move(axis, speed, angle, absolute=absolute)
            return

        if len(args) != 6:
            raise UsageError(
                f"{word} <az|el> <deg/s> <degrees>, or both at once: "
                f"{word} az <deg/s> <degrees> el <deg/s> <degrees>"
            )

        # Both axes. Either may be typed first, but each only once.
        halves = {}
        for axis_word, speed_word, angle_word in (args[:3], args[3:]):
            axis = _axis(axis_word)
            if axis in halves:
                raise UsageError(
                    f"name each axis once: "
                    f"{word} az <deg/s> <degrees> el <deg/s> <degrees>"
                )
            halves[axis] = (
                _number(speed_word, "speed"),
                _number(angle_word, "angle"),
            )
        self.run_move_both(*halves["AZ"], *halves["EL"], absolute=absolute)

    def run_move(
        self,
        axis: str,
        speed_dps: float,
        delta_deg: float,
        row_hint: bool = True,
        absolute: bool = False,
    ):
        """One move, start to finish, with the waiting handled.

        Returns the outcome so the scripted runs can log it, or None if the
        move was refused or did not finish - that has already been reported
        here and is not worth raising again further up.
        """
        try:
            plan = self.session.begin_move(axis, speed_dps, delta_deg,
                                           absolute=absolute)
        except sn.Refused as refused:
            self.say(f"  refused: {refused}")
            return None

        self.report_plan(plan)

        dones = self._wait_out([plan])
        if dones is None:
            return None

        outcome = sn.MoveOutcome(plan, dones[0])
        self.report_move(outcome)

        if self.results is not None:
            # commanded_deg is how far it was asked to turn, so an absolute
            # move records the journey rather than the destination - the
            # column means the same thing in every row.
            commanded = (delta_deg if not absolute
                         else p.mdeg_to_deg(plan.to_mdeg - plan.from_mdeg))
            row = self.results.add(outcome, commanded_deg=commanded)
            # Left out when a scripted run is about to ask for the reading
            # itself, which would otherwise print two prompts for the same
            # number.
            if row_hint:
                self.say(f"         row {row.row} - type 'true <degrees>' "
                         f"once you have read the protractor")
        return outcome

    def run_move_both(
        self,
        az_speed_dps: float,
        az_delta_deg: float,
        el_speed_dps: float,
        el_delta_deg: float,
        absolute: bool = False,
    ):
        """Both axes in one command, each at its own speed.

        Returns the two outcomes, azimuth first, or None if the move was
        refused or the wait was given up. Each axis gets its own row in the
        results file; the shared id ties the pair together.
        """
        try:
            plans = self.session.begin_move_both(
                az_speed_dps, az_delta_deg, el_speed_dps, el_delta_deg,
                absolute=absolute,
            )
        except sn.Refused as refused:
            self.say(f"  refused: {refused}")
            return None

        for plan in plans:
            self.report_plan(plan, label=True)

        dones = self._wait_out(plans)
        if dones is None:
            return None

        outcomes = tuple(
            sn.MoveOutcome(plan, done) for plan, done in zip(plans, dones)
        )
        for outcome in outcomes:
            self.report_move(outcome, label=True)

        if self.results is not None:
            commanded = (
                (az_delta_deg, el_delta_deg) if not absolute
                else tuple(p.mdeg_to_deg(plan.to_mdeg - plan.from_mdeg)
                           for plan in plans)
            )
            rows = [
                self.results.add(outcome, commanded_deg=deg, note="both axes")
                for outcome, deg in zip(outcomes, commanded)
            ]
            self.say(
                f"         rows {rows[0].row} (az) and {rows[1].row} (el) - "
                f"type 'true <row> <degrees>' for each once you have read "
                f"the protractors"
            )
        return outcomes

    def _wait_out(self, plans: list[p.Plan]):
        """Wait for every axis of the move, turning Ctrl-C into a stop.

        Returns the DONEs in the same order as the plans, or None if the
        wait was given up. One stop covers every axis, so a second axis does
        not need a second Ctrl-C.
        """
        # Neither DONE of a two-axis move comes until both axes have
        # stopped, so every wait is sized by the slowest plan.
        timeout_s = sn.move_timeout_s(*plans)
        dones = []
        stopping = False
        for plan in plans:
            while True:
                try:
                    dones.append(
                        self.session.await_done(plan, timeout_s=timeout_s)
                    )
                    break
                except KeyboardInterrupt:
                    if not stopping:
                        stopping = True
                        self.say(
                            "  ^C - stopping. Waiting for it to come to rest."
                        )
                        self.session.stop()
                        continue
                    # Twice means they want out now. The move is still
                    # running on the Pico and its DONE is still coming, so
                    # say so rather than pretending the axis is parked.
                    self.say(
                        "  ^C again - no longer waiting. The axis may still "
                        "be turning, and its position here is now a guess. "
                        "Send estop, then pos, then zero."
                    )
                    return None
                except sn.Timeout as exc:
                    self.say(f"  {exc}")
                    self.say(
                        "  The move may still be running. Try pos, and estop "
                        "if it is not where it should be."
                    )
                    return None
        return dones

    def report_plan(self, plan: p.Plan, label: bool = False) -> None:
        """What the Pico is about to do.

        `label` names the axis, for when there are two plans to tell apart.
        """
        axis = f"{plan.axis.lower()}  " if label else ""
        short = "  (too short to reach that speed)" if plan.triangular else ""
        self.say(
            f"  plan   {axis}{plan.steps:+d} steps  "
            f"{p.format_mdeg(plan.from_mdeg)} -> "
            f"{p.format_mdeg(plan.to_mdeg)} deg"
            f"  at {p.format_mdeg(plan.speed_mdps)} deg/s{short}"
            f"  about {plan.est_s:.2f} s"
        )

    def report_move(
        self, outcome: sn.MoveOutcome, label: bool = False
    ) -> None:
        done = outcome.done
        axis = f"{done.axis.lower()}  " if label else ""
        verdict = "finished" if done.ok else "STOPPED SHORT"
        self.say(
            f"  done   {axis}{verdict}  {done.steps_done:+d} steps in "
            f"{done.elapsed_s:.2f} s  (predicted {outcome.plan.est_s:.2f} s)"
        )
        self.say(f"         {axis}now at {p.format_mdeg(done.to_mdeg)} deg")

    # --- the scripted runs --------------------------------------------------

    def do_sweep(self, args: list[str]) -> None:
        """The same angle at a series of speeds."""
        values, marks = _options(args, {"axis", "angle", "speeds"})

        axis = _axis(values.get("axis", "az"))
        angle = _number(
            str(values.get("angle", scripted_runs.DEFAULT_SWEEP_ANGLE_DEG)),
            "angle",
        )
        if "speeds" in values:
            speeds = [
                _number(word, "speed")
                for word in values["speeds"].split(",")
                if word
            ]
        else:
            speeds = list(scripted_runs.DEFAULT_SWEEP_SPEEDS)

        scripted_runs.sweep(
            self,
            axis=axis,
            angle_deg=angle,
            speeds=speeds,
            pause="pause" in marks,
        )

    def do_random(self, args: list[str]) -> None:
        """Unpredictable angles inside the soft limits."""
        values, marks = _options(args, {"axis", "count", "speed", "seed"})

        axis = _axis(values.get("axis", "az"))
        count = int(_number(
            str(values.get("count", scripted_runs.DEFAULT_RANDOM_COUNT)),
            "count",
        ))
        speed = _number(
            str(values.get("speed", scripted_runs.DEFAULT_RANDOM_SPEED_DPS)),
            "speed",
        )
        seed = None
        if "seed" in values:
            seed = int(_number(values["seed"], "seed"))

        if count < 1:
            raise UsageError("count must be at least 1")

        scripted_runs.random_moves(
            self,
            axis=axis,
            count=count,
            speed_dps=speed,
            seed=seed,
            pause="pause" in marks,
        )

    # --- the results file ---------------------------------------------------

    def _need_results(self) -> results_file.ResultsFile:
        if self.results is None:
            raise UsageError(
                "no results file is open. Start again with --log"
            )
        return self.results

    def do_true(self, args: list[str]) -> None:
        """Record what the protractor actually read."""
        results = self._need_results()
        if not args or len(args) > 2:
            raise UsageError("true [row] <degrees>")

        if len(args) == 2:
            row = int(_number(args[0], "row"))
            deg = _number(args[1], "angle")
        else:
            row, deg = None, _number(args[0], "angle")

        try:
            entry = results.set_true(deg, row)
        except results_file.ResultsError as exc:
            raise UsageError(str(exc)) from None

        self.report_reading(entry)

    def report_reading(self, entry, indent: str = "  ") -> None:
        """Say what was recorded, and query it if it looks unlikely.

        Recorded either way - a surprising measurement is still a
        measurement, and this is a question rather than a correction.
        """
        self.say(
            f"{indent}row {entry.row}: measured "
            f"{p.format_mdeg(entry.true_mdeg)}, predicted "
            f"{p.format_mdeg(entry.predicted_mdeg)}, out by "
            f"{p.format_mdeg(entry.error_mdeg)} deg"
        )
        if abs(entry.error_mdeg) > p.deg_to_mdeg(READING_WARN_DEG):
            self.say(
                f"{indent}that is a long way out. The reading wanted is "
                f"where the antenna is now, not how far it just moved. "
                f"Retype it with 'true {entry.row} <degrees>' if that was it."
            )

    def do_note(self, args: list[str]) -> None:
        results = self._need_results()
        if not args:
            raise UsageError("note [row] <text>")

        row = None
        if len(args) > 1 and _looks_like_a_number(args[0]):
            row = int(_number(args[0], "row"))
            args = args[1:]

        try:
            entry = results.annotate(" ".join(args), row)
        except results_file.ResultsError as exc:
            raise UsageError(str(exc)) from None
        self.say(f"  noted against row {entry.row}")

    def do_rows(self, args: list[str]) -> None:
        results = self._need_results()
        count = int(_number(args[0], "count")) if args else 10
        if not results.rows:
            self.say("  nothing logged yet")
            return
        for entry in results.rows[-count:]:
            self.say(entry.summary())
        missing = results.missing_readings()
        if missing:
            numbers = ", ".join(str(r.row) for r in missing)
            self.say(f"  still waiting on a reading for row {numbers}")

    def warn_about_blanks(self) -> None:
        """Say so before anyone packs up.

        A missing protractor reading cannot be filled in later from
        anywhere - the antenna has moved on.
        """
        if self.results is None:
            return
        missing = self.results.missing_readings()
        if missing:
            numbers = ", ".join(str(r.row) for r in missing)
            self.say("")
            self.say(
                f"{len(missing)} move(s) have no protractor reading: "
                f"row {numbers}"
            )
        self.say(f"results in {self.results.path}")

    # --- the loop -----------------------------------------------------------

    COMMANDS = {
        "move": do_move,
        "goto": do_goto,
        "zero": do_zero,
        "pos": do_pos,
        "stop": do_stop,
        "estop": do_estop,
        "pin": do_pin,
        "ping": do_ping,
        "limits": do_limits,
        "settings": do_settings,
        "set": do_set,
        "save": do_save,
        "defaults": do_defaults,
        "gear": do_gear,
        "track": do_track,
        "sweep": do_sweep,
        "random": do_random,
        "true": do_true,
        "note": do_note,
        "rows": do_rows,
        "verbose": do_verbose,
        "help": do_help,
        "?": do_help,
        "quit": do_quit,
        "exit": do_quit,
    }

    def run_line(self, text: str) -> None:
        """One typed line. Never raises for anything the operator can cause."""
        line = text.strip()
        if not line or line.startswith("#"):
            return

        word, _, rest = line.partition(" ")
        handler = self.COMMANDS.get(word.lower())
        if handler is None:
            self.say(f"  no such command: {word}. Type help.")
            return

        try:
            handler(self, rest.split())
        except UsageError as exc:
            self.say(f"  {exc}")
        except sn.Refused as exc:
            self.say(f"  refused: {exc}")
        except sn.SessionError as exc:  # Timeout, Rebooted, a lost port
            self.say(f"  {exc}")
        except KeyboardInterrupt:
            self.say("  ^C")

    def interact(self, prompt: str = PROMPT) -> None:
        """Read lines until they run out or someone types quit."""
        interactive = self.stdin.isatty()

        while not self.quit:
            if interactive:
                print(prompt, end="", file=self._out, flush=True)
            try:
                line = self.stdin.readline()
            except KeyboardInterrupt:
                self.say("")
                continue
            if not line:  # end of input, or Ctrl-D
                self.say("")
                return
            if not interactive:
                # Echo what came in, so a piped session reads as a
                # transcript rather than as answers to invisible questions.
                self.say(f"{prompt}{line.rstrip()}")
            self.run_line(line)


# ============================================================================
# Starting up
# ============================================================================


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="py -m gs.motor",
        description="Drive the ground station's two motors by hand.",
    )
    parser.add_argument(
        "--port",
        help="serial port, such as COM5. Left out, the Pico is found by "
        "itself",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="list the serial ports and exit",
    )
    parser.add_argument(
        "--log",
        nargs="?",
        const="",
        metavar="FILE",
        help="write the results to this CSV file as the test runs. "
        "Give --log with no name for a dated one in a logs folder",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="show the heartbeat as it arrives",
    )
    parser.add_argument(
        "-c",
        "--command",
        action="append",
        default=[],
        metavar="TEXT",
        help="run this command and exit. May be given more than once",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    if args.list:
        return ports.main()

    try:
        session = sn.Session.open(args.port)
    except Exception as exc:  # usually a PortError, with a message ready
        print(exc)
        return 2

    console = Console(session, verbose=args.verbose)
    session.on_chatter = console.show_chatter

    try:
        # Collect the banner before the first prompt, so the operator sees
        # what they are connected to rather than a bare cursor. The chatter
        # hook above prints it as it arrives.
        session.drain_until_quiet()

        # One PING before anything else. It proves the link works rather
        # than letting the first real command discover that it does not,
        # and it names what answered, which goes in every row of the results
        # file.
        who = ""
        try:
            pong = session.ping()
            who = f"{pong.target} {pong.version}"
            print(f"connected: {who}")
        except sn.SessionError as exc:
            print(f"warning: no answer to PING ({exc})")

        # Which gearing it is counting, every time. It decides what every
        # angle typed and reported means, and a Pico that was already
        # running has no banner left on screen to say.
        gear_word = ""
        try:
            gear = session.gear()
            gear_word = "on" if gear.geared else "off"
            print(f"gear {gear_word}: {gear.describe()}")
        except sn.SessionError as exc:
            print(f"warning: could not read the gear mode ({exc})")

        if args.log is not None:
            path = args.log or results_file.default_path()
            console.results = results_file.ResultsFile(
                path=path, pico=who, gear=gear_word
            )
            console.results.write()  # so the file exists from the start
            print(f"logging to {path}")

        if args.command:
            for text in args.command:
                console.say(f"{PROMPT}{text}")
                console.run_line(text)
        else:
            console.interact()

        console.warn_about_blanks()
    finally:
        session.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
