// The numbers that can be changed at the motor> prompt, without reflashing.
//
// This file holds the rules and the shape of the saved record. Reading and
// writing the flash itself is in settings_flash.cpp, which is kept to that
// one job.
//
// Four numbers per axis can be set:
//
//   ACCEL     acceleration   anywhere in the axis's accel_min..accel_max
//   MIN       lower travel   no lower than motor_config.hpp's, no higher
//                            than the axis's current MAX
//   MAX       upper travel   no higher than motor_config.hpp's, no lower
//                            than the axis's current MIN
//   DEADBAND  how close to a target counts as being there, 0 up to the
//             axis's deadband_max. Only used when chasing a target, and
//             meant to be tuned at the bench: raise it if the mechanism
//             chatters, lower it for tighter pointing.
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
// THE GEAR MODE IS PART OF THE RECORD. It decides which built-in numbers an
// axis has, so it has to be known before the rest of the record can be
// judged - a record saved in geared mode is checked against the geared
// built-ins. It also means a restart comes back in the mode the machine is
// actually in, rather than quietly reverting and putting every move out by
// the gear pair's ratio.
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
//
// The acceleration bounds come out of `built_in` rather than from a global,
// because they are scaled with the gearing like every other speed number:
// 360 deg/sec/sec of antenna in geared mode would be far more than 360 at
// the gearbox shaft.
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

// How many of them there are, per axis. Both the record layout and the
// loops over it come from this.
constexpr size_t SETTING_COUNT = 4;

// magic, version, gear mode, the built-in numbers, the settings, a CRC.
constexpr size_t SETTINGS_RECORD_LEN =
    4 + 4 + 4 + 2 * SETTING_COUNT * 4 + 2 * SETTING_COUNT * 4 + 4;

// Write the record for this gear mode and these settings into out. Returns
// the number of bytes written, or 0 if out is too small.
//
// The built-in numbers to snapshot are looked up from `gear`, so there is
// one place that decides which built-ins a mode has: motor_config.hpp.
size_t encode_settings(GearMode gear, const AxisSpec &az, const AxisSpec &el,
                       uint8_t *out, size_t n);

enum class LoadResult {
    Ok,       // a good record for this build: *gear, *az and *el are set
    Blank,    // nothing has ever been saved
    Corrupt,  // something is there but it does not read back as a record
    Stale,    // a good record, but saved against different built-in
              // numbers, or with a value this build no longer allows
};

// Read a record back. Only on LoadResult::Ok are *gear, *az and *el changed,
// and then the axes only in their settable numbers.
//
// On anything else the caller keeps what it passed in, which is how a blank,
// corrupt or stale record comes up in direct mode: that is the safe way to
// be wrong, since it undershoots rather than driving past a hard stop.
LoadResult decode_settings(const uint8_t *in, size_t n, GearMode *gear,
                           AxisSpec *az, AxisSpec *el);
