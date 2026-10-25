// Turning a line of text from the laptop into a command.
//
// This only reads the text. Whether the command is allowed - speed in range,
// inside the soft limits, axis not already moving - is decided afterwards, in
// command_rules.cpp.
//
// The commands, one per line:
//
//   MOVE,<id>,<AZ|EL>,<speed_mdps>,<delta_mdeg>   turn an axis
//   MOVETO,<id>,<AZ|EL>,<speed_mdps>,<mdeg>       turn TO an angle
//   MOVETO,<id>,AZ,<speed>,<mdeg>,EL,<speed>,<mdeg>
//   MOVE,<id>,AZ,<speed>,<delta>,EL,<speed>,<delta>
//                                                 turn both at once, each
//                                                 at its own speed (either
//                                                 axis may come first)
//   STOP,<id>                                     slow to a stop
//   ESTOP,<id>                                    stop at once
//   ZERO,<id>,<AZ|EL|ALL>[,<mdeg>]                say where the axis is now
//   POS,<id>                                      report both positions
//   PIN,<id>,<gpio>,<0|1>                         set a pin high or low
//   LIMITS,<id>                                   report the soft limits
//   PING,<id>                                     are you there?
//   SET,<id>,<AZ|EL>,<ACCEL|MIN|MAX>,<value>      change a setting, for now
//   SAVE,<id>                                     keep the settings in flash
//   DEFAULTS,<id>                                 back to the built-in ones
//   SETTINGS,<id>                                 report the settings
//   GEAR,<id>[,<ON|OFF>]                          report or change gear mode
//   TRACK,<id>[,<ON|OFF>]                         report or arm tracking
//   TARGET,<id>,<az_mdeg>,<el_mdeg>               point here (absolute)
//
// GEAR with no mode asks which one is in force. With ON or OFF it switches,
// which changes what a degree means on both axes - see motor_config.hpp.
//
// A SET value is millidegrees per second per second for ACCEL, and
// millidegrees for the MIN and MAX travel limits.
//
// Every command carries an id that the Pico echoes back in its replies. A
// move takes up to a minute and its replies arrive long after the command,
// so the id is how the laptop tells which reply belongs to which command, and
// which move each row of the results came from.
//
// MOVE's angle is a change from where the axis is now; MOVETO's is where to
// end up, measured from the zero. Everything else about them is the same,
// including both being able to name each axis once with its own speed.
//
// Angles and speeds arrive as whole millidegrees, never as decimals. The
// laptop turns "45.5" into 45500 before sending, which keeps this parser to
// integers only: no floating point, no locale surprises, nothing that rounds
// differently on two machines.
#pragma once

#include <stdint.h>

enum class Verb {
    None,
    Move,
    MoveTo,
    Stop,
    Estop,
    Zero,
    Pos,
    Pin,
    Limits,
    Ping,
    Set,
    Save,
    Defaults,
    Settings,
    Gear,
    Track,
    Target,
};

enum class AxisId {
    None,
    Az,
    El,
    All,  // ZERO only
};

// What SET can change. See settings.hpp for the rules on each.
enum class Setting {
    None,
    Accel,     // acceleration, millidegrees per second per second
    Min,       // lower travel limit, millidegrees
    Max,       // upper travel limit, millidegrees
    Deadband,  // how close to a target counts as there, millidegrees
};

// Why a line could not be understood. These are faults in the text itself.
// Whether a command can actually be carried out - speed in range, inside the
// soft limits, axis not already moving - is decided later, not here.
enum class ParseError {
    None,
    Syntax,       // wrong number of fields, or a field that is not a number
    UnknownVerb,  // first field is not a command we know
    BadAxis,      // not AZ, EL, or ALL where ALL is allowed
    BadGpio,      // pin number or level out of range
    BadSetting,   // SET named something other than ACCEL, MIN or MAX
};

struct Command {
    Verb verb       = Verb::None;
    ParseError err  = ParseError::None;
    uint16_t id     = 0;

    AxisId axis     = AxisId::None;  // MOVE, ZERO, SET
    int32_t mdps    = 0;             // MOVE: commanded speed
    int32_t mdeg    = 0;             // MOVE: how far. ZERO: where we are now
    bool has_mdeg   = false;         // ZERO: was the optional angle given?

    // MOVE with both axes: the second axis named, its speed and how far, in
    // the order they were written. axis2 is None on a one-axis MOVE.
    AxisId axis2    = AxisId::None;
    int32_t mdps2   = 0;
    int32_t mdeg2   = 0;

    Setting setting = Setting::None; // SET: which one
    int32_t value   = 0;             // SET: what to

    uint8_t gpio    = 0;             // PIN
    uint8_t level   = 0;             // PIN

    // GEAR and TRACK: was a mode given, and which. Without one the command
    // is a question rather than a change.
    bool has_gear   = false;
    bool gear_on    = false;
    bool has_track  = false;
    bool track_on   = false;

    // TARGET: where to point, absolute, both axes. Always both - tracking
    // has no use for one axis on its own, and a pair of numbers is simpler
    // than optional halves on a command that arrives several times a second.
    int32_t az_mdeg = 0;
    int32_t el_mdeg = 0;
};

// Read one line. On failure the returned command has err set and verb left as
// None, except for an unknown verb where there is nothing else to report.
//
// Command words and axis names may be upper or lower case, so that someone
// poking at the Pico from a plain serial monitor during bring-up does not
// have to hold shift. Numbers are strict: digits with an optional leading
// sign, nothing else - no spaces, no decimal point, no exponent.
Command parse_command(const char *line);
