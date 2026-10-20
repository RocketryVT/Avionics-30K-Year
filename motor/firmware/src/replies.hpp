// Building the lines the Pico sends back to the laptop.
//
// The replies, one per line, each ending in \n:
//
//   ACK,<id>,<verb>
//   PLAN,<id>,<axis>,<steps>,<from>,<to>,<speed>,<TRAP|TRI>,<est_us>
//   DONE,<id>,<axis>,<OK|ABORT>,<steps_done>,<from>,<to>,<elapsed_us>
//   POS,<id>,<az>,<el>,<az_steps>,<el_steps>,<ZEROED|UNZEROED>
//   PIN,<id>,<gpio>,<level>
//   LIM,<id>,<az_min>,<az_max>,<el_min>,<el_max>,<spd_min>,<spd_max>
//   CFG,<id>,<az_accel>,<az_min>,<az_max>,<el_accel>,<el_min>,<el_max>,
//       <SAVED|UNSAVED>,<BUILTIN|CUSTOM>
//   PONG,<id>,<target>,<version>,<uptime_ms>
//   MSTAT,<uptime_s>,<az>,<el>,<IDLE|MOVING>
//   ERR,<id>,<code>,<detail>
//
// CFG is one line; it is only split here to fit. SAVED means a restart would
// come back with these same settings, and BUILTIN means they are the numbers
// in motor_config.hpp.
//
// All angles and speeds are whole millidegrees, matching the commands going
// the other way. The laptop turns them back into degrees for the operator.
//
// Every function here writes a complete line into the caller's buffer and
// returns its length. Nothing is printed directly: a line has to leave the
// Pico in a single write, or another message landing in the middle of it
// would corrupt both.
//
// If the buffer is too small these return 0 and write nothing usable. Treat 0
// as "send nothing" - half a line is worse than no line, because the laptop
// would read the fragment as a different record.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "command_parser.hpp"
#include "motion_profile.hpp"

// Comfortably longer than the longest line above, which is CFG at about 75
// characters.
constexpr size_t MAX_REPLY_LEN = 120;

// Why a command was refused.
enum class ErrCode {
    Syntax,     // the line did not make sense
    BadVerb,    // no such command
    BadAxis,    // not AZ or EL
    BadGpio,    // pin number or level out of range, or a pin we must not touch
    BadSpeed,   // outside the allowed speed range
    Limit,      // the move would finish outside the soft limits
    NoZero,     // a move was asked for before the axis was zeroed
    Busy,       // an axis is already moving
    ZeroSteps,  // the angle rounds to no steps at all
    TooLong,    // the command line overran the input buffer
    BadSetting, // SET named a setting that does not exist
    Range,      // a SET value outside what this build allows
    Flash,      // SAVE could not write the settings to flash
};

// The words used on the wire for each of these.
const char *verb_name(Verb v);
const char *axis_name(AxisId a);
const char *err_name(ErrCode e);

size_t format_ack(char *buf, size_t n, uint16_t id, Verb verb);

size_t format_plan(char *buf, size_t n, uint16_t id, AxisId axis,
                   const Move &m);

size_t format_done(char *buf, size_t n, uint16_t id, AxisId axis, bool ok,
                   int32_t steps_done, int32_t from_mdeg, int32_t to_mdeg,
                   uint32_t elapsed_us);

size_t format_pos(char *buf, size_t n, uint16_t id, int32_t az_mdeg,
                  int32_t el_mdeg, int32_t az_steps, int32_t el_steps,
                  bool zeroed);

size_t format_pin(char *buf, size_t n, uint16_t id, uint8_t gpio,
                  uint8_t level);

// The travel limits in force, which settings may have narrowed from the
// built-in ones.
size_t format_lim(char *buf, size_t n, uint16_t id, const AxisSpec &az,
                  const AxisSpec &el);

size_t format_cfg(char *buf, size_t n, uint16_t id, const AxisSpec &az,
                  const AxisSpec &el, bool saved, bool built_in);

size_t format_pong(char *buf, size_t n, uint16_t id, const char *target,
                   const char *version, uint32_t uptime_ms);

size_t format_mstat(char *buf, size_t n, uint32_t uptime_s, int32_t az_mdeg,
                    int32_t el_mdeg, bool moving);

// detail may be null. Commas, newlines and anything unprintable in it become
// '?', and it is cut to 16 characters, so it can never split the line.
size_t format_err(char *buf, size_t n, uint16_t id, ErrCode code,
                  const char *detail);

// A move past the soft limits, with numbers rather than text, so the laptop
// can show exactly how far out it was:
// ERR,<id>,limit,<axis>,<target>,<min>,<max>.
size_t format_err_limit(char *buf, size_t n, uint16_t id, AxisId axis,
                        int32_t target_mdeg, int32_t min_mdeg,
                        int32_t max_mdeg);

// The same shape for a SET value out of range, so the laptop can say what
// would have been allowed: ERR,<id>,range,<axis>,<value>,<lo>,<hi>.
size_t format_err_range(char *buf, size_t n, uint16_t id, AxisId axis,
                        int32_t value, int32_t lo, int32_t hi);
