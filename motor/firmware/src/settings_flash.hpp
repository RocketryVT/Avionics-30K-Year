// Keeping the settings in the Pico's flash, so they survive a restart.
//
// Kept thin on purpose: what the record holds, how it is checked and when it
// is ignored are all in settings.cpp. What is left here is one sector of
// flash, read and written.
#pragma once

#include "motor_config.hpp"
#include "settings.hpp"

// Is the program itself clear of the sector the settings live in? Checked
// once at startup: a program grown into that sector would be overwritten by
// the first SAVE.
bool settings_sector_is_free();

// Read what was saved. Only on LoadResult::Ok are *gear, *az and *el
// changed, so seed them with the mode to fall back to first.
LoadResult load_settings(GearMode *gear, AxisSpec *az, AxisSpec *el);

// Write these settings to flash. Returns once it is done, about 50 ms later.
//
// Interrupts are off while flash is being written, because the program runs
// from that same flash. Never call this while an axis is moving: the step
// loop would stall part way through a move. command_rules.cpp already
// refuses SAVE then.
//
// Nothing is returned: whether it worked is found by reading it back with
// load_settings(), which is the only answer worth trusting.
void save_settings(GearMode gear, const AxisSpec &az, const AxisSpec &el);
