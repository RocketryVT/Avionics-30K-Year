#include "command_parser.hpp"

#include <stddef.h>

namespace {

// A command has at most 8 fields (MOVE with both axes). One spare so that a
// line with too many fields can be noticed rather than silently truncated.
constexpr size_t MAX_FIELDS = 9;

struct Field {
    const char *p;
    size_t n;
};

// Cut the line at every comma. Empty fields are kept, so ",,," is four empty
// fields and gets rejected later rather than looking like a short command.
size_t split(const char *line, Field *out)
{
    size_t count = 0;
    const char *start = line;

    for (const char *p = line;; p++) {
        if (*p == ',' || *p == '\0') {
            if (count < MAX_FIELDS) {
                out[count].p = start;
                out[count].n = static_cast<size_t>(p - start);
            }
            count++;
            if (*p == '\0') {
                break;
            }
            start = p + 1;
        }
    }
    return count;
}

char upper(char c)
{
    return (c >= 'a' && c <= 'z') ? static_cast<char>(c - 'a' + 'A') : c;
}

// Compare a field against a fixed word, ignoring case.
bool is_word(const Field &f, const char *word)
{
    size_t i = 0;
    for (; i < f.n; i++) {
        if (word[i] == '\0' || upper(f.p[i]) != word[i]) {
            return false;
        }
    }
    return word[i] == '\0';
}

// Read a whole number. Strict on purpose: an optional + or -, then at least
// one digit, then the end of the field. Anything else is a mistake worth
// reporting rather than guessing at - "45.5" from a confused sender should
// not quietly become 45.
//
// The running total is int64 so that a long run of digits cannot wrap round
// into a plausible-looking small number before the range check sees it.
bool parse_int(const Field &f, int64_t lo, int64_t hi, int64_t *out)
{
    if (f.n == 0) {
        return false;
    }

    size_t i      = 0;
    bool negative = false;
    if (f.p[0] == '+' || f.p[0] == '-') {
        negative = (f.p[0] == '-');
        i        = 1;
        if (f.n == 1) {
            return false;  // a sign on its own
        }
    }

    int64_t value = 0;
    for (; i < f.n; i++) {
        const char c = f.p[i];
        if (c < '0' || c > '9') {
            return false;
        }
        value = value * 10 + (c - '0');
        if (value > 4000000000LL) {
            return false;  // far past anything useful; stop before it wraps
        }
    }

    if (negative) {
        value = -value;
    }
    if (value < lo || value > hi) {
        return false;
    }
    *out = value;
    return true;
}

// Fill in err and leave the verb unset, so a caller that forgets to check
// err still does nothing rather than something wrong.
//
// The id is carried through whenever it was readable. It costs nothing and
// it is the only thing that ties the refusal to the command that caused it:
// without it the laptop gets ERR,0 for a mistyped axis and cannot tell which
// of its commands was turned down. Where the id itself could not be read -
// a line with no fields, an unknown verb, an id that is not a number - it
// stays 0, because there is nothing honest to put there.
Command fail(ParseError e, uint16_t id = 0)
{
    Command c;
    c.err = e;
    c.id  = id;
    return c;
}

bool read_axis(const Field &f, bool allow_all, AxisId *out)
{
    if (is_word(f, "AZ")) {
        *out = AxisId::Az;
        return true;
    }
    if (is_word(f, "EL")) {
        *out = AxisId::El;
        return true;
    }
    if (allow_all && is_word(f, "ALL")) {
        *out = AxisId::All;
        return true;
    }
    return false;
}

// One axis's share of a MOVE: three fields, the axis, its speed and how far.
//
// The range here is only a guard against nonsense; whether the speed and
// angle are actually acceptable is plan_move's decision.
ParseError read_move(const Field *f, AxisId *axis, int32_t *mdps,
                     int32_t *mdeg)
{
    if (!read_axis(f[0], false, axis)) {
        return ParseError::BadAxis;
    }
    int64_t speed = 0;
    int64_t angle = 0;
    if (!parse_int(f[1], 0, 1000000, &speed) ||
        !parse_int(f[2], -1000000, 1000000, &angle)) {
        return ParseError::Syntax;
    }
    *mdps = static_cast<int32_t>(speed);
    *mdeg = static_cast<int32_t>(angle);
    return ParseError::None;
}

bool read_setting(const Field &f, Setting *out)
{
    if (is_word(f, "ACCEL")) {
        *out = Setting::Accel;
        return true;
    }
    if (is_word(f, "MIN")) {
        *out = Setting::Min;
        return true;
    }
    if (is_word(f, "MAX")) {
        *out = Setting::Max;
        return true;
    }
    if (is_word(f, "DEADBAND")) {
        *out = Setting::Deadband;
        return true;
    }
    return false;
}

// ON or OFF, shared by GEAR and TRACK. False if it is neither, which is a
// malformed field rather than a command we do not know.
bool read_on_off(const Field &f, bool *out)
{
    if (is_word(f, "ON")) {
        *out = true;
        return true;
    }
    if (is_word(f, "OFF")) {
        *out = false;
        return true;
    }
    return false;
}

}  // namespace

Command parse_command(const char *line)
{
    Field f[MAX_FIELDS] = {};
    const size_t n      = split(line, f);

    if (n > MAX_FIELDS) {
        return fail(ParseError::Syntax);
    }

    // Work out which command this is first, so an unknown word is reported as
    // such rather than as a field-count problem.
    Verb verb          = Verb::None;
    size_t want_fields = 0;

    if (is_word(f[0], "MOVE")) {
        verb        = Verb::Move;
        want_fields = 5;  // or 8 for both axes, handled below
    } else if (is_word(f[0], "MOVETO")) {
        verb        = Verb::MoveTo;
        want_fields = 5;  // or 8 for both axes, handled below
    } else if (is_word(f[0], "STOP")) {
        verb        = Verb::Stop;
        want_fields = 2;
    } else if (is_word(f[0], "ESTOP")) {
        verb        = Verb::Estop;
        want_fields = 2;
    } else if (is_word(f[0], "ZERO")) {
        verb        = Verb::Zero;
        want_fields = 3;  // or 4, handled below
    } else if (is_word(f[0], "POS")) {
        verb        = Verb::Pos;
        want_fields = 2;
    } else if (is_word(f[0], "PIN")) {
        verb        = Verb::Pin;
        want_fields = 4;
    } else if (is_word(f[0], "LIMITS")) {
        verb        = Verb::Limits;
        want_fields = 2;
    } else if (is_word(f[0], "PING")) {
        verb        = Verb::Ping;
        want_fields = 2;
    } else if (is_word(f[0], "SET")) {
        verb        = Verb::Set;
        want_fields = 5;
    } else if (is_word(f[0], "SAVE")) {
        verb        = Verb::Save;
        want_fields = 2;
    } else if (is_word(f[0], "DEFAULTS")) {
        verb        = Verb::Defaults;
        want_fields = 2;
    } else if (is_word(f[0], "SETTINGS")) {
        verb        = Verb::Settings;
        want_fields = 2;
    } else if (is_word(f[0], "GEAR")) {
        verb        = Verb::Gear;
        want_fields = 2;  // or 3 with a mode, handled below
    } else if (is_word(f[0], "TRACK")) {
        verb        = Verb::Track;
        want_fields = 2;  // or 3 with a mode, handled below
    } else if (is_word(f[0], "TARGET")) {
        verb        = Verb::Target;
        want_fields = 4;
    } else {
        return fail(ParseError::UnknownVerb);
    }

    const bool zero_with_angle = (verb == Verb::Zero && n == 4);
    const bool move_both =
        ((verb == Verb::Move || verb == Verb::MoveTo) && n == 8);
    const bool gear_with_mode = (verb == Verb::Gear && n == 3);
    const bool track_with_mode = (verb == Verb::Track && n == 3);
    if (n != want_fields && !zero_with_angle && !move_both &&
        !gear_with_mode && !track_with_mode) {
        return fail(ParseError::Syntax);
    }

    // Every command carries an id.
    int64_t id = 0;
    if (!parse_int(f[1], 0, 65535, &id)) {
        return fail(ParseError::Syntax);
    }

    Command c;
    c.verb = verb;
    c.id   = static_cast<uint16_t>(id);

    switch (verb) {
    case Verb::Move:
    case Verb::MoveTo: {
        ParseError e = read_move(&f[2], &c.axis, &c.mdps, &c.mdeg);
        if (e == ParseError::None && move_both) {
            e = read_move(&f[5], &c.axis2, &c.mdps2, &c.mdeg2);
            // Each axis once. Two halves for the same axis would be two moves
            // fighting over one motor, and there is no right way to pick.
            if (e == ParseError::None && c.axis2 == c.axis) {
                e = ParseError::BadAxis;
            }
        }
        if (e != ParseError::None) {
            return fail(e, c.id);
        }
        break;
    }

    case Verb::Zero: {
        if (!read_axis(f[2], true, &c.axis)) {
            return fail(ParseError::BadAxis, c.id);
        }
        if (zero_with_angle) {
            int64_t mdeg = 0;
            if (!parse_int(f[3], -1000000, 1000000, &mdeg)) {
                return fail(ParseError::Syntax, c.id);
            }
            c.mdeg     = static_cast<int32_t>(mdeg);
            c.has_mdeg = true;
        }
        break;
    }

    case Verb::Pin: {
        // 0-29 is every GPIO the chip has. Whether a particular pin is safe
        // to poke is a hardware question, answered where the pin map lives.
        int64_t gpio  = 0;
        int64_t level = 0;
        if (!parse_int(f[2], 0, 29, &gpio) || !parse_int(f[3], 0, 1, &level)) {
            return fail(ParseError::BadGpio, c.id);
        }
        c.gpio  = static_cast<uint8_t>(gpio);
        c.level = static_cast<uint8_t>(level);
        break;
    }

    case Verb::Set: {
        if (!read_axis(f[2], false, &c.axis)) {
            return fail(ParseError::BadAxis, c.id);
        }
        if (!read_setting(f[3], &c.setting)) {
            return fail(ParseError::BadSetting, c.id);
        }
        // Only a guard against nonsense, as for MOVE. Whether the value is
        // allowed depends on the setting and the build, and is decided later.
        int64_t value = 0;
        if (!parse_int(f[4], -1000000, 1000000, &value)) {
            return fail(ParseError::Syntax, c.id);
        }
        c.value = static_cast<int32_t>(value);
        break;
    }

    case Verb::Gear: {
        if (gear_with_mode) {
            if (!read_on_off(f[2], &c.gear_on)) {
                return fail(ParseError::Syntax, c.id);
            }
            c.has_gear = true;
        }
        break;
    }

    case Verb::Track: {
        if (track_with_mode) {
            if (!read_on_off(f[2], &c.track_on)) {
                return fail(ParseError::Syntax, c.id);
            }
            c.has_track = true;
        }
        break;
    }

    case Verb::Target: {
        // Only a guard against nonsense, as for MOVE. Whether the angles are
        // reachable is decided later, and a target outside the travel limits
        // is clamped rather than refused - see do_target.
        int64_t az = 0;
        int64_t el = 0;
        if (!parse_int(f[2], -1000000, 1000000, &az) ||
            !parse_int(f[3], -1000000, 1000000, &el)) {
            return fail(ParseError::Syntax, c.id);
        }
        c.az_mdeg = static_cast<int32_t>(az);
        c.el_mdeg = static_cast<int32_t>(el);
        break;
    }

    default:
        break;  // the rest need nothing more than their id
    }

    return c;
}
