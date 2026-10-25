r"""The laptop's half of the motor protocol: the text that goes over USB.

This is the mirror of two firmware files: `command_parser.cpp`, which reads
the lines we send, and `replies.cpp`, which writes the lines we read. The
wire format lives in both places, so this module and those two have to be
changed together.

The two directions, one line each, "\n" terminated:

    laptop -> Pico                      Pico -> laptop
    MOVE,<id>,<AZ|EL>,<mdps>,<mdeg>     ACK,<id>,<verb>
    STOP,<id>                           PLAN,<id>,<axis>,<steps>,...
    ESTOP,<id>                          DONE,<id>,<axis>,<OK|ABORT>,...
    ZERO,<id>,<AZ|EL|ALL>[,<mdeg>]      POS,<id>,<az>,<el>,...
    POS,<id>                            PIN,<id>,<gpio>,<level>
    PIN,<id>,<gpio>,<0|1>               LIM,<id>,<az_min>,...
    LIMITS,<id>                         PONG,<id>,<target>,...
    PING,<id>                           MSTAT,<uptime_s>,<az>,<el>,...
    SET,<id>,<AZ|EL>,<name>,<value>     ERR,<id>,<code>,<detail>
    SAVE,<id>                           CFG,<id>,<az_accel>,...
    DEFAULTS,<id>                       GEAR,<id>,<ON|OFF>,<az_num>,...
    SETTINGS,<id>
    GEAR,<id>[,<ON|OFF>]

A MOVE can also turn both axes at once, each at its own speed:
`MOVE,<id>,AZ,<mdps>,<mdeg>,EL,<mdps>,<mdeg>`. It is answered with one ACK,
a PLAN for each axis, and later a DONE for each - both DONEs only once the
slower axis has stopped.

SET changes one setting on one axis - ACCEL, or the MIN or MAX travel limit
- until the Pico restarts. SAVE keeps the settings in flash, DEFAULTS goes
back to the built-in ones from motor_config.hpp, and SETTINGS reports them.

GEAR asks which gearing the Pico is counting, or switches it. OFF is the
10:1 gearbox alone, with angles at the gearbox output shaft and each axis
driven on its own. ON adds the final gear pair, with angles at the antenna.
Switching costs both axes their zero, because the drive train has changed.
The mode is saved by SAVE along with everything else, and it changes what a
degree means, so every angle in a results file belongs to one mode or the
other.

Two rules run through all of it.

**Whole millidegrees on the wire, never decimals.** The Pico's parser takes
integers only: no floating point, no locale surprises, nothing that rounds
differently on two machines. Turning the operator's "45.5" into 45500 is
this module's job, and turning it back for display is too.

**Reading a reply never raises.** A line that does not make sense becomes an
Info record carrying the original text. The Pico prints banners, `[motor]`
notes and `[fatal]` lines on the same wire as the records, and a half-read
line at startup is normal - none of that should take down a program that is
in the middle of driving a motor. Building a command is the opposite: it
raises CommandError rather than send a line the Pico will refuse, because
there the mistake is ours and we would rather hear about it here.
"""

from __future__ import annotations

from dataclasses import dataclass

# ============================================================================
# Limits, copied from the firmware
# ============================================================================
# These mirror the guards in command_parser.cpp. They are only a nonsense
# filter - whether a move is actually allowed (speed in range, inside the soft
# limits, nothing already moving) is the Pico's decision, made with the
# settings it has. We deliberately do not second-guess that here: the laptop
# keeping its own copy of the soft limits is how the two ends come to disagree
# about where the gimbal may point. Ask with LIMITS instead.
MAX_LINE_LEN = 95  # line_reader.hpp's buffer; longer lines get ERR toolong

ID_MIN = 1  # 0 is reserved - see IdSource
ID_MAX = 65535

MDPS_MAX = 1000000  # 1000 deg/sec
MDEG_MAX = 1000000  # 1000 degrees
GPIO_MAX = 29

MOVE_AXES = ("AZ", "EL")
ZERO_AXES = ("AZ", "EL", "ALL")

# The two words GEAR takes and reports. ON means the final gear pair is in
# the maths; see the module docstring.
GEAR_ON = "ON"
GEAR_OFF = "OFF"

# What SET can change: acceleration in deg/sec/sec, and the travel limits in
# degrees. Both cross the wire in thousandths, like every other number.
SETTING_NAMES = ("ACCEL", "MIN", "MAX")

# Every axis word the firmware can put in a reply, including the two that
# only turn up in errors.
AXIS_NAMES = ("AZ", "EL", "ALL", "NONE")

# err_name() in replies.cpp, with what each one means to the operator.
ERR_MEANING = {
    "syntax": "the Pico could not read that line",
    "verb": "no such command",
    "axis": "axis must be AZ or EL",
    "gpio": "pin number or level out of range, or a pin we must not touch",
    "speed": "speed outside the range the Pico allows",
    "limit": "the move would finish outside the soft limits",
    "nozero": "that axis has not been zeroed yet - send ZERO first",
    "busy": "an axis is still moving",
    "zerosteps": "the angle rounds to no steps at all",
    "toolong": "the command line was too long and was thrown away",
    "setting": "no such setting - use accel, min or max",
    "range": "that value is outside what this build allows",
    "flash": "the Pico could not write its flash, so nothing was saved",
}


class CommandError(ValueError):
    """A command could not be built, with a message meant for the operator."""


# ============================================================================
# Degrees and millidegrees
# ============================================================================


def deg_to_mdeg(deg: float) -> int:
    """Degrees from a human into whole millidegrees for the wire.

    Rounds half away from zero, so 0.0005 becomes 1 and -0.0005 becomes -1.
    Python's built-in round() would send both to 0, and a rule that depends
    on whether the neighbouring digit is even is not one anybody wants to
    reason about while reading a row of results.
    """
    whole = int(abs(float(deg)) * 1000.0 + 0.5)
    return -whole if deg < 0 else whole


def mdeg_to_deg(mdeg: int) -> float:
    """Millidegrees back to degrees."""
    return mdeg / 1000.0


def format_mdeg(mdeg: int) -> str:
    """Millidegrees as a fixed three-decimal string: 45006 -> "45.006".

    Built from the integer rather than from mdeg/1000.0, so the text is
    exactly the number the Pico used with no float formatting in between.
    Rows of results get compared against these strings by eye.
    """
    sign = "-" if mdeg < 0 else ""
    whole, frac = divmod(abs(mdeg), 1000)
    return f"{sign}{whole}.{frac:03d}"


# ============================================================================
# Building commands
# ============================================================================


class IdSource:
    """Hands out the id that ties a reply back to the command that caused it.

    A move takes up to a minute and its replies arrive long after the
    command went out. The id is what connects them, and it is also what ties
    each row of the results file to the move that produced it.

    Ids run 1..65535 and then wrap. Zero is skipped because the firmware uses
    it for errors about lines it could not read the id from, such as a line
    too long to have been read at all.
    """

    def __init__(self) -> None:
        self._next = ID_MIN

    def next(self) -> int:
        value = self._next
        self._next = ID_MIN if value >= ID_MAX else value + 1
        return value


def _check_id(cmd_id: int) -> int:
    if not isinstance(cmd_id, int) or isinstance(cmd_id, bool):
        raise CommandError(f"id must be a whole number, got {cmd_id!r}")
    if not ID_MIN <= cmd_id <= ID_MAX:
        raise CommandError(f"id must be {ID_MIN}..{ID_MAX}, got {cmd_id}")
    return cmd_id


def _check_axis(axis: str, allowed: tuple[str, ...]) -> str:
    # Accepted in any case for the operator's sake, but always sent upper
    # case so the line on the wire looks the same however it was typed.
    name = str(axis).strip().upper()
    if name not in allowed:
        raise CommandError(f"axis must be {' or '.join(allowed)}, got {axis!r}")
    return name


def _check_mdeg(mdeg: int, deg: float, what: str) -> int:
    if not -MDEG_MAX <= mdeg <= MDEG_MAX:
        raise CommandError(
            f"{what} must be within +/-{MDEG_MAX // 1000} degrees, got {deg}"
        )
    return mdeg


def move(cmd_id: int, axis: str, speed_dps: float, delta_deg: float) -> str:
    """Turn an axis by delta_deg degrees at speed_dps degrees per second.

    The angle is a change from where the axis is now, not a position to go
    to, which is why it may be negative. The Pico works out the end angle,
    checks it against the soft limits, and reports both in its PLAN line.
    """
    _check_id(cmd_id)
    return f"MOVE,{cmd_id},{_move_half(axis, speed_dps, delta_deg)}"


def move_both(
    cmd_id: int,
    az_speed_dps: float,
    az_delta_deg: float,
    el_speed_dps: float,
    el_delta_deg: float,
) -> str:
    """Turn both axes in one command, each at its own speed.

    Each half means exactly what it would in a one-axis move. The Pico
    checks both before starting either, so if one half is refused nothing
    moves at all. Azimuth always goes first on the wire, though the Pico
    would take them either way round.
    """
    _check_id(cmd_id)
    az = _move_half("AZ", az_speed_dps, az_delta_deg)
    el = _move_half("EL", el_speed_dps, el_delta_deg)
    return f"MOVE,{cmd_id},{az},{el}"


def _move_half(axis: str, speed_dps: float, delta_deg: float) -> str:
    """One axis's share of a MOVE: `<AXIS>,<mdps>,<mdeg>`."""
    name = _check_axis(axis, MOVE_AXES)

    mdps = deg_to_mdeg(speed_dps)
    if not 0 <= mdps <= MDPS_MAX:
        raise CommandError(
            f"speed must be 0..{MDPS_MAX // 1000} deg/sec, got {speed_dps}"
        )
    mdeg = _check_mdeg(deg_to_mdeg(delta_deg), delta_deg, "angle")

    return f"{name},{mdps},{mdeg}"


def stop(cmd_id: int) -> str:
    """Bring both axes to a controlled stop."""
    return f"STOP,{_check_id(cmd_id)}"


def estop(cmd_id: int) -> str:
    """Stop both axes at once, without slowing down.

    A stepper told to stop dead can lose steps, so the position the Pico
    reports afterwards may not be where the gimbal really is. Re-zero after
    one of these.
    """
    return f"ESTOP,{_check_id(cmd_id)}"


def zero(cmd_id: int, axis: str = "ALL", deg: float | None = None) -> str:
    """Tell the Pico what angle an axis is sitting at right now.

    Nothing moves. There are no limit switches on this machine, so this is
    the only thing that gives the soft limits an origin, and a MOVE sent
    before it is refused with ERR nozero.
    """
    _check_id(cmd_id)
    name = _check_axis(axis, ZERO_AXES)

    if deg is None:
        return f"ZERO,{cmd_id},{name}"
    mdeg = _check_mdeg(deg_to_mdeg(deg), deg, "angle")
    return f"ZERO,{cmd_id},{name},{mdeg}"


def pos(cmd_id: int) -> str:
    """Ask where both axes are."""
    return f"POS,{_check_id(cmd_id)}"


def pin(cmd_id: int, gpio: int, level: int | bool) -> str:
    """Hold one GPIO high or low, for checking wiring with a multimeter.

    The Pico only allows the motor lines (GP4-7) and the spare pins
    (GP12-15). Driving a motor's step line makes that axis lose its zero.
    """
    _check_id(cmd_id)
    if not isinstance(gpio, int) or isinstance(gpio, bool):
        raise CommandError(f"gpio must be a whole number, got {gpio!r}")
    if not 0 <= gpio <= GPIO_MAX:
        raise CommandError(f"gpio must be 0..{GPIO_MAX}, got {gpio}")
    return f"PIN,{cmd_id},{gpio},{1 if level else 0}"


def limits(cmd_id: int) -> str:
    """Ask what soft limits and speed range this build was flashed with."""
    return f"LIMITS,{_check_id(cmd_id)}"


def ping(cmd_id: int) -> str:
    """Ask the Pico to identify itself and say how long it has been up."""
    return f"PING,{_check_id(cmd_id)}"


def set_setting(cmd_id: int, axis: str, name: str, value: float) -> str:
    """Change one setting on one axis, until the Pico restarts.

    `value` is in deg/sec/sec for ACCEL and in degrees for MIN and MAX.
    Whether it is allowed is the Pico's decision, made against what it was
    built with, and it says so with ERR range if not.
    """
    _check_id(cmd_id)
    which = _check_axis(axis, MOVE_AXES)
    setting = str(name).strip().upper()
    if setting not in SETTING_NAMES:
        allowed = ", ".join(n.lower() for n in SETTING_NAMES)
        raise CommandError(f"setting must be {allowed}, got {name!r}")
    milli = _check_mdeg(deg_to_mdeg(value), value, setting.lower())
    return f"SET,{cmd_id},{which},{setting},{milli}"


def save(cmd_id: int) -> str:
    """Keep the settings in force, so a restart comes back with them."""
    return f"SAVE,{_check_id(cmd_id)}"


def defaults(cmd_id: int) -> str:
    """Go back to the built-in settings, until a restart unless saved."""
    return f"DEFAULTS,{_check_id(cmd_id)}"


def settings(cmd_id: int) -> str:
    """Ask what settings are in force, and whether they are saved."""
    return f"SETTINGS,{_check_id(cmd_id)}"


def gear(cmd_id: int, on: bool | None = None) -> str:
    """Ask which gearing the Pico is counting, or switch it.

    `on` left out asks the question and changes nothing. True folds the
    final gear pair into the maths (angles at the antenna), False counts the
    10:1 gearbox alone (angles at the gearbox output shaft, each axis driven
    on its own).

    Switching is refused while anything is moving, and when it does go
    through it costs both axes their zero: the drive train is physically
    different, so the step count no longer describes a known angle.
    """
    _check_id(cmd_id)
    if on is None:
        return f"GEAR,{cmd_id}"
    return f"GEAR,{cmd_id},{GEAR_ON if on else GEAR_OFF}"


def encode(line: str) -> bytes:
    """A command line as the bytes to write to the port.

    The newline is added here rather than by each builder, so the builders
    return something printable that can go straight into a log, and the one
    piece of framing lives in one place.

    The length check cannot fire on anything the builders produce - the
    longest, a two-axis MOVE, is 50 characters against a 95 character
    buffer - but a hand-written line could reach it, and the Pico throws an
    overlong line away, so this is the kinder place to find out.
    """
    if "\n" in line or "\r" in line:
        raise CommandError("a command must be a single line")
    if len(line) > MAX_LINE_LEN:
        raise CommandError(
            f"command is {len(line)} characters; the Pico accepts "
            f"{MAX_LINE_LEN}"
        )
    return (line + "\n").encode("ascii", errors="strict")


# ============================================================================
# Reading replies
# ============================================================================


class Reply:
    """Base class, so a reader has one type to check against."""


@dataclass(frozen=True)
class Ack(Reply):
    """The command was understood and is being acted on."""

    id: int
    verb: str


@dataclass(frozen=True)
class Plan(Reply):
    """What the Pico worked out a MOVE would do, sent before it starts.

    `to_mdeg` is the end angle after rounding to whole steps, which is not
    simply start plus commanded: 10 degrees is 444 steps, or 9.990 degrees.
    That is the angle a protractor reading is compared against.
    """

    id: int
    axis: str
    steps: int
    from_mdeg: int
    to_mdeg: int
    speed_mdps: int
    triangular: bool  # True = too short a move to ever reach full speed
    est_us: int

    @property
    def est_s(self) -> float:
        return self.est_us / 1e6


@dataclass(frozen=True)
class Done(Reply):
    """A move finished. `ok` is False when a stop cut it short."""

    id: int
    axis: str
    ok: bool
    steps_done: int
    from_mdeg: int
    to_mdeg: int
    elapsed_us: int

    @property
    def elapsed_s(self) -> float:
        return self.elapsed_us / 1e6


@dataclass(frozen=True)
class Pos(Reply):
    """Where both axes are.

    Steps come with the angles because steps are what the firmware actually
    counts; the millidegrees are derived from them.
    """

    id: int
    az_mdeg: int
    el_mdeg: int
    az_steps: int
    el_steps: int
    zeroed: bool


@dataclass(frozen=True)
class Pin(Reply):
    """A GPIO was set."""

    id: int
    gpio: int
    level: int


@dataclass(frozen=True)
class Lim(Reply):
    """The soft limits and speed range in force on this Pico.

    The travel limits are the built-in ones unless settings have narrowed
    them.

    The speed ceiling is per axis, because geared mode scales each axis's
    down by its own gear pair - azimuth ends up slower than elevation. The
    floor is shared. Ask which axis you mean rather than assuming they
    match.
    """

    id: int
    az_min_mdeg: int
    az_max_mdeg: int
    az_speed_max_mdps: int
    el_min_mdeg: int
    el_max_mdeg: int
    el_speed_max_mdps: int
    speed_min_mdps: int

    def speed_max_for(self, axis: str) -> int:
        """That axis's ceiling, in millidegrees per second."""
        return (
            self.el_speed_max_mdps
            if str(axis).strip().upper() == "EL"
            else self.az_speed_max_mdps
        )


@dataclass(frozen=True)
class Cfg(Reply):
    """The settings in force: acceleration and travel, per axis.

    `geared` says which gearing these numbers belong to. It is here rather
    than only in Gear because switching gear is one of the things that makes
    the settings unsaved, and because the built-in numbers differ per mode -
    `built_in` means "the numbers in motor_config.hpp for this mode".

    `saved` means a restart would come back with all of this, mode included.
    """

    id: int
    az_accel_mdps2: int
    az_min_mdeg: int
    az_max_mdeg: int
    el_accel_mdps2: int
    el_min_mdeg: int
    el_max_mdeg: int
    geared: bool
    saved: bool
    built_in: bool


@dataclass(frozen=True)
class Gear(Reply):
    """Which gearing the Pico is counting, and what it makes a degree worth.

    The steps per degree come over as the exact fraction the Pico divides
    by, not a decimal, so what is shown is what is being applied:

        direct   400/9    = 44.444 steps per gearbox-output degree
        geared   12800/126 = 101.587 steps per antenna degree (azimuth)
                 8000/126  =  63.492 steps per antenna degree (elevation)
    """

    id: int
    geared: bool
    az_steps_num: int
    az_steps_den: int
    el_steps_num: int
    el_steps_den: int

    @property
    def az_steps_per_deg(self) -> float:
        return self.az_steps_num / self.az_steps_den

    @property
    def el_steps_per_deg(self) -> float:
        return self.el_steps_num / self.el_steps_den

    def describe(self) -> str:
        """One line fit to print to the operator."""
        if self.geared:
            return "geared - final pair counted, angles at the ANTENNA"
        return "direct - gearbox only, angles at the GEARBOX OUTPUT SHAFT"


@dataclass(frozen=True)
class Pong(Reply):
    """The answer to PING.

    Uptime is here so we can spot a Pico that rebooted: if it goes backwards
    the position was lost, and moving before re-zeroing would be unsafe.
    """

    id: int
    target: str
    version: str
    uptime_ms: int


@dataclass(frozen=True)
class Mstat(Reply):
    """The heartbeat. No id - this one is not a reply to anything.

    It only arrives while nothing is moving and nothing is half-typed, so a
    gap in the heartbeat during a move is normal rather than a sign of
    trouble.
    """

    uptime_s: int
    az_mdeg: int
    el_mdeg: int
    moving: bool


@dataclass(frozen=True)
class Err(Reply):
    """A command was refused.

    Two codes carry numbers instead of a detail string: `limit`, for a move
    that would leave the soft limits, and `range`, for a SET value outside
    what is allowed. Both say what would have been allowed. They are kept as
    the same class rather than separate ones so a caller asking "did my
    command fail?" has a single type to test for; the extra fields are None
    on every other code.
    """

    id: int
    code: str
    detail: str = ""
    axis: str | None = None
    target_mdeg: int | None = None
    min_mdeg: int | None = None
    max_mdeg: int | None = None

    @property
    def is_limit(self) -> bool:
        return self.code == "limit" and self.target_mdeg is not None

    @property
    def is_range(self) -> bool:
        return self.code == "range" and self.target_mdeg is not None

    def message(self) -> str:
        """One line fit to print to the operator."""
        meaning = ERR_MEANING.get(self.code, f"error '{self.code}'")
        if self.is_limit:
            return (
                f"{meaning}: {self.axis} would reach "
                f"{format_mdeg(self.target_mdeg)} deg, allowed "
                f"{format_mdeg(self.min_mdeg)}..{format_mdeg(self.max_mdeg)}"
            )
        if self.is_range:
            # No unit: the same reply serves acceleration and travel, and
            # the operator knows which they asked for.
            return (
                f"{meaning}: {self.axis} {format_mdeg(self.target_mdeg)}, "
                f"allowed {format_mdeg(self.min_mdeg)}"
                f"..{format_mdeg(self.max_mdeg)}"
            )
        if self.detail:
            return f"{meaning} ({self.detail})"
        return meaning


@dataclass(frozen=True)
class Info(Reply):
    """Anything that is not a record: banners, `[motor]` notes, `[fatal]`.

    `malformed` marks a line that began like a record we know but did not
    parse - a truncated DONE, a field that is not a number. It still reaches
    the operator as text, because dropping it would hide exactly the kind of
    fault worth noticing, but it is flagged so the reader can say so instead
    of printing it as ordinary chatter.
    """

    text: str
    malformed: bool = False


# Every record name parse_reply knows. A line starting with one of these but
# not parsing is flagged as malformed; anything else is the Pico talking to a
# human and is passed through untouched.
RECORD_NAMES = frozenset(
    {
        "ACK", "PLAN", "DONE", "POS", "PIN", "LIM", "CFG", "GEAR", "PONG",
        "MSTAT", "ERR",
    }
)


def _int(text: str) -> int:
    """Strict whole number: an optional sign, then digits, and nothing else.

    Python's int() is looser than the firmware's parser - it accepts
    surrounding whitespace and digit underscores, so "1_0" would come back as
    10. Being exactly as strict as the wire format keeps a corrupted line
    from parsing as a plausible wrong number.
    """
    body = text[1:] if text[:1] in ("+", "-") else text
    if not body or not body.isascii() or not body.isdigit():
        raise ValueError(f"not a whole number: {text!r}")
    return int(text)


def _axis(text: str) -> str:
    if text not in AXIS_NAMES:
        raise ValueError(f"not an axis: {text!r}")
    return text


def _word(text: str, true_word: str, false_word: str) -> bool:
    """One of two fixed words, as a bool. Anything else is a bad line."""
    if text == true_word:
        return True
    if text == false_word:
        return False
    raise ValueError(f"expected {true_word} or {false_word}, got {text!r}")


def _parse_err(f: list[str]) -> Err:
    # Two shapes share the ERR name: the usual four fields, and the seven of
    # a limit or range refusal, which reports numbers rather than text.
    if len(f) == 7 and f[2] in ("limit", "range"):
        return Err(
            id=_int(f[1]),
            code=f[2],
            axis=_axis(f[3]),
            target_mdeg=_int(f[4]),
            min_mdeg=_int(f[5]),
            max_mdeg=_int(f[6]),
        )
    if len(f) != 4:
        raise ValueError("wrong field count for ERR")
    return Err(id=_int(f[1]), code=f[2], detail=f[3])


def parse_reply(line: str) -> Reply:
    r"""One line from the Pico as a record. Never raises.

    The line may arrive with or without its newline, and with a stray "\r"
    if something along the way added one.
    """
    text = line.rstrip("\r\n")
    if not text.strip():
        return Info("")

    f = text.split(",")
    name = f[0]

    try:
        if name == "ACK" and len(f) == 3:
            return Ack(id=_int(f[1]), verb=f[2])

        if name == "PLAN" and len(f) == 9:
            return Plan(
                id=_int(f[1]),
                axis=_axis(f[2]),
                steps=_int(f[3]),
                from_mdeg=_int(f[4]),
                to_mdeg=_int(f[5]),
                speed_mdps=_int(f[6]),
                triangular=_word(f[7], "TRI", "TRAP"),
                est_us=_int(f[8]),
            )

        if name == "DONE" and len(f) == 8:
            return Done(
                id=_int(f[1]),
                axis=_axis(f[2]),
                ok=_word(f[3], "OK", "ABORT"),
                steps_done=_int(f[4]),
                from_mdeg=_int(f[5]),
                to_mdeg=_int(f[6]),
                elapsed_us=_int(f[7]),
            )

        if name == "POS" and len(f) == 7:
            return Pos(
                id=_int(f[1]),
                az_mdeg=_int(f[2]),
                el_mdeg=_int(f[3]),
                az_steps=_int(f[4]),
                el_steps=_int(f[5]),
                zeroed=_word(f[6], "ZEROED", "UNZEROED"),
            )

        if name == "PIN" and len(f) == 4:
            return Pin(id=_int(f[1]), gpio=_int(f[2]), level=_int(f[3]))

        if name == "LIM" and len(f) == 9:
            return Lim(
                id=_int(f[1]),
                az_min_mdeg=_int(f[2]),
                az_max_mdeg=_int(f[3]),
                az_speed_max_mdps=_int(f[4]),
                el_min_mdeg=_int(f[5]),
                el_max_mdeg=_int(f[6]),
                el_speed_max_mdps=_int(f[7]),
                speed_min_mdps=_int(f[8]),
            )

        if name == "CFG" and len(f) == 11:
            return Cfg(
                id=_int(f[1]),
                az_accel_mdps2=_int(f[2]),
                az_min_mdeg=_int(f[3]),
                az_max_mdeg=_int(f[4]),
                el_accel_mdps2=_int(f[5]),
                el_min_mdeg=_int(f[6]),
                el_max_mdeg=_int(f[7]),
                geared=_word(f[8], "GEARED", "DIRECT"),
                saved=_word(f[9], "SAVED", "UNSAVED"),
                built_in=_word(f[10], "BUILTIN", "CUSTOM"),
            )

        if name == "GEAR" and len(f) == 7:
            return Gear(
                id=_int(f[1]),
                geared=_word(f[2], GEAR_ON, GEAR_OFF),
                az_steps_num=_int(f[3]),
                az_steps_den=_int(f[4]),
                el_steps_num=_int(f[5]),
                el_steps_den=_int(f[6]),
            )

        if name == "PONG" and len(f) == 5:
            return Pong(
                id=_int(f[1]), target=f[2], version=f[3], uptime_ms=_int(f[4])
            )

        if name == "MSTAT" and len(f) == 5:
            return Mstat(
                uptime_s=_int(f[1]),
                az_mdeg=_int(f[2]),
                el_mdeg=_int(f[3]),
                moving=_word(f[4], "MOVING", "IDLE"),
            )

        if name == "ERR":
            return _parse_err(f)

    except ValueError:
        return Info(text, malformed=True)

    # A record name we know, with the wrong number of fields.
    if name in RECORD_NAMES:
        return Info(text, malformed=True)
    return Info(text)
