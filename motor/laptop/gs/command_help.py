r"""What each command does, and when you would reach for it.

The `motor>` prompt's own `help` is a one-line-per-command reminder for
somebody who already knows the machine. This is the longer form the window
shows: what it does, and - the part that is actually hard to work out from a
list - *when* to use it.

Kept as data rather than prose so it can be checked. `missing()` compares it
against the prompt's real command table, and a test fails if a command is
added without a line here. That is the same instinct as the window rendering
`HELP_TEXT`: a reference that can drift from the thing it describes is worse
than none.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Entry:
    """One command: how it is typed, what it does, when to use it."""

    usage: str
    what: str
    when: str

    # The prompt command this belongs to, for the drift check. Several
    # entries can share one - `set` has three, because the three settings
    # are reached for at completely different moments.
    command: str = ""

    def lines(self) -> list[str]:
        return [self.usage, f"    {self.what}", f"    When: {self.when}"]


ENTRIES = [
    # --- getting started ---------------------------------------------------
    Entry(
        command="ping",
        usage="ping",
        what="Asks the Pico what it is and how long it has been running.",
        when="First thing, and after any reconnect. If the uptime goes "
             "backwards it rebooted and has forgotten where the antenna is.",
    ),
    Entry(
        command="zero",
        usage="zero [az|el|all] [degrees]",
        what="Tells the Pico what angle the axes are sitting at. Nothing "
             "moves.",
        when="Before anything else will work - every move is refused until "
             "you do. Again after an emergency stop, after a gear change, "
             "and after the Pico reboots, because all three throw away what "
             "it believed.",
    ),
    Entry(
        command="pos",
        usage="pos",
        what="Where it thinks it is, in degrees and in steps.",
        when="To check a move ended where it said it would. Steps are what "
             "it actually counts; the degrees are worked out from them.",
    ),

    # --- the two ways to move ----------------------------------------------
    Entry(
        command="move",
        usage="move <az|el> <deg/s> <degrees>",
        what="Turns an axis BY that many degrees from wherever it is. "
             "Negative turns back.",
        when="The bench procedure: +45, read the protractor, -45, read it "
             "again. Use this when what you care about is the journey.",
    ),
    Entry(
        command="move",
        usage="move az <deg/s> <deg> el <deg/s> <deg>",
        what="Turns both axes at once, each at its own speed.",
        when="Testing the two motors together once the belt is on - mode 2 "
             "in the window.",
    ),
    Entry(
        command="goto",
        usage="goto <az|el> <deg/s> <degrees>",
        what="Turns an axis TO that angle, measured from the zero. The Pico "
             "works out how far that is.",
        when="When you know where you want it pointing rather than how far "
             "to turn - going back to a known reference, or lining up on "
             "something. Refused if it is already there.",
    ),
    Entry(
        command="stop",
        usage="stop",
        what="Slows to a halt. Keeps the zero.",
        when="A move is heading somewhere you did not intend and there is "
             "time. This is the one to reach for first.",
    ),
    Entry(
        command="estop",
        usage="estop",
        what="Stops at once, with no slowing down. Gives up the zero.",
        when="Something is about to hit something. It can lose steps, so "
             "where it thinks it is afterwards is a guess - re-zero.",
    ),

    # --- the two gearings --------------------------------------------------
    Entry(
        command="gear",
        usage="gear [on|off]",
        what="Whether the final 32:14 and 20:14 gear pairs are counted. Off "
             "means angles at the gearbox shaft; on means angles at the "
             "antenna.",
        when="Whenever you fit or remove the belt. Get it wrong and every "
             "move is out by the pair's ratio - 1.43x on elevation, 2.29x "
             "on azimuth. Switching costs both axes their zero.",
    ),

    # --- tracking ----------------------------------------------------------
    Entry(
        command="track",
        usage="track [on|off]",
        what="Arms or disarms chasing a target. Arming moves nothing on its "
             "own.",
        when="Before starting 'py -m gs.track', which supplies the targets. "
             "If none arrive the Pico gives up after about a second and a "
             "half, which is deliberate - it is what a radio dropout looks "
             "like.",
    ),

    # --- tuning ------------------------------------------------------------
    Entry(
        command="limits",
        usage="limits",
        what="The travel and speed each axis will allow.",
        when="Before a big or fast move, to see what it will refuse. The "
             "speed ceiling differs per axis in geared mode.",
    ),
    Entry(
        command="settings",
        usage="settings",
        what="Acceleration, travel and deadband per axis, and whether a "
             "restart would come back with them.",
        when="After changing anything, to confirm it took - and to see "
             "whether you still need to save.",
    ),
    Entry(
        command="set",
        usage="set <az|el> accel <deg/s/s>",
        what="How quickly an axis may change speed.",
        when="Finding how hard the axis can be driven. Raise it one step at "
             "a time, watching the driver's alarm light and whether it "
             "still returns to zero.",
    ),
    Entry(
        command="set",
        usage="set <az|el> <min|max> <degrees>",
        what="Narrows an axis's travel. It can never be widened past what "
             "the code allows.",
        when="The mechanism has less room than the code thinks. There are no "
             "limit switches, so these numbers are the only thing stopping a "
             "mistyped move reaching a hard stop.",
    ),
    Entry(
        command="set",
        usage="set <az|el> deadband <degrees>",
        what="How close to a target counts as being there. Only used while "
             "tracking.",
        when="The motors chatter or buzz while tracking - raise it. Pointing "
             "is looser than you want and it is quiet - lower it. Half a "
             "degree is about 2% of the antenna beam, so there is room.",
    ),
    Entry(
        command="save",
        usage="save",
        what="Writes the settings and the gear mode to the Pico's flash.",
        when="Once you have numbers you want back after a restart. Without "
             "it, a power cycle loses them.",
    ),
    Entry(
        command="defaults",
        usage="defaults",
        what="Back to the built-in numbers for the gear mode in force.",
        when="You have tuned yourself into a corner and want a known "
             "starting point. Does not change the gear mode.",
    ),

    # --- measuring ---------------------------------------------------------
    Entry(
        command="sweep",
        usage="sweep [angle=45] [speeds=1,5,90] [pause]",
        what="The same angle at each speed in turn, alternating direction.",
        when="Seeing what the acceleration ramp does to fast moves. A short "
             "move commanded fast spends most of its time speeding up and "
             "slowing down, so it averages well below the speed asked for. "
             "That is the ramp, not a fault.",
    ),
    Entry(
        command="random",
        usage="random [count=10] [speed=5] [seed=N] [pause]",
        what="Unpredictable angles inside the limits.",
        when="Hunting an error that only shows up at particular positions - "
             "backlash, or a tight spot in the gearing. Go out and come back "
             "and see whether it returns to the same reading.",
    ),
    Entry(
        command="true",
        usage="true [row] <degrees>",
        what="Records what the protractor actually reads, against a row of "
             "the results file.",
        when="Straight after a move, while the antenna is still. The reading "
             "wanted is where it is NOW, not how far it just turned.",
    ),
    Entry(
        command="note",
        usage="note [row] <text>",
        what="Writes a note against a row.",
        when="Something happened that the numbers will not show - a noise, a "
             "wobble, somebody knocking the tripod.",
    ),
    Entry(
        command="rows",
        usage="rows [n]",
        what="Shows the last n rows of the results table.",
        when="Before packing up, to check nothing is still missing a "
             "protractor reading. You cannot fill one in later.",
    ),

    # --- wiring and odds and ends ------------------------------------------
    Entry(
        command="pin",
        usage="pin <gpio> <0|1>",
        what="Holds one pin high or low. GP4-7 and GP12-15 only.",
        when="Checking wiring with a multimeter, before trusting any motor. "
             "Driving a step pin costs that axis its zero, because the "
             "driver takes a step nobody counted.",
    ),
    Entry(
        command="verbose",
        usage="verbose [on|off]",
        what="Shows or hides the heartbeat and the tracking stream.",
        when="You want to see the Pico is alive, or watch tracking close in. "
             "Hidden by default because a few lines a second scrolls "
             "everything else away.",
    ),
    Entry(
        command="help",
        usage="help",
        what="The short list of commands.",
        when="At the prompt. This panel is the longer version.",
    ),
    Entry(
        command="quit",
        usage="quit",
        what="Leaves the prompt. 'exit' works too.",
        when="Done. It warns about any move still missing a reading.",
    ),
]

# Commands that are only another spelling of one already covered.
ALIASES = {"?": "help", "exit": "quit"}


def text() -> str:
    """The whole reference, as lines for a plain text widget."""
    out = []
    for entry in ENTRIES:
        out.extend(entry.lines())
        out.append("")
    return "\n".join(out).rstrip() + "\n"


def missing(commands) -> list[str]:
    """Which of `commands` have no entry here.

    Point it at `motor.Console.COMMANDS`. A test asserts this is empty, so a
    command cannot be added to the prompt without a line saying when to use
    it.
    """
    covered = {e.command for e in ENTRIES} | set(ALIASES)
    return sorted(name for name in commands if name not in covered)


def extra(commands) -> list[str]:
    """Entries here for commands the prompt does not have. Also a drift
    check, in the other direction."""
    real = set(commands) | set(ALIASES.values())
    return sorted({e.command for e in ENTRIES} - real)
