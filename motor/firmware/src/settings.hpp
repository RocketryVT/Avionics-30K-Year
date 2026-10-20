// The numbers that can be changed at the motor> prompt, without reflashing.
//
// This file holds the rules and the shape of the saved record. Reading and
// writing the flash itself is in settings_flash.cpp, which is kept to that
// one job.
//
// Three numbers per axis can be set:
//
//   ACCEL   acceleration     anywhere in ACCEL_MIN_MDPS2..ACCEL_MAX_MDPS2
//   MIN     lower travel     no lower than motor_config.hpp's, no higher
//                            than the axis's current MAX
//   MAX     upper travel     no higher than motor_config.hpp's, no lower
//                            than the axis's current MIN
//
// So travel can only ever be narrowed at the prompt. There are no limit
// switches on this machine and the limits are all that stands between a
// bad command and a hard stop, so widening them is a change to the code,
// made on purpose and reflashed.
//
// A changed setting is used from the next command on, and is lost at a
// restart until it is saved. Saved settings are written to flash as one
// small record, and read back at every startup.
//
// The record carries a copy of the built-in numbers it was saved against.
// If motor_config.hpp has been changed and reflashed since, the record is
// ignored and the new built-in numbers are used - otherwise editing the
// code and reflashing would appear to do nothing, because the old saved
// settings would quietly win.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "command_parser.hpp"
#include "motor_config.hpp"

// The values a setting may take on one axis, both ends included.
struct SettingRange {
    int32_t lo;
    int32_t hi;
};

// What `setting` may be set to on an axis built with `built_in` that is
// currently running with `current`. Setting::None gives an empty range.
SettingRange allowed_range(Setting setting, const AxisSpec &built_in,
                           const AxisSpec &current);

// Read or change one setting in an axis description.
int32_t setting_value(Setting setting, const AxisSpec &spec);
void put_setting(Setting setting, int32_t value, AxisSpec *spec);

// Do two axis descriptions agree on everything that can be set?
bool same_settings(const AxisSpec &a, const AxisSpec &b);

// ---------------------------------------------------------------------------
// The record kept in flash
// ---------------------------------------------------------------------------

// magic, version, the six built-in numbers, the six settings, a CRC.
constexpr size_t SETTINGS_RECORD_LEN = 4 + 4 + 6 * 4 + 6 * 4 + 4;

// Write the record for these settings into out. Returns the number of bytes
// written, or 0 if out is too small.
size_t encode_settings(const AxisSpec &az_built_in,
                       const AxisSpec &el_built_in, const AxisSpec &az,
                       const AxisSpec &el, uint8_t *out, size_t n);

enum class LoadResult {
    Ok,       // a good record for this build: *az and *el are set from it
    Blank,    // nothing has ever been saved
    Corrupt,  // something is there but it does not read back as a record
    Stale,    // a good record, but saved against different built-in
              // numbers, or with a value this build no longer allows
};

// Read a record back. Only on LoadResult::Ok are *az and *el changed, and
// then only their settable numbers.
LoadResult decode_settings(const uint8_t *in, size_t n,
                           const AxisSpec &az_built_in,
                           const AxisSpec &el_built_in, AxisSpec *az,
                           AxisSpec *el);
