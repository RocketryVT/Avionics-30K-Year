// Deciding what to do about a command, and what to say back.
//
// The split is: this file decides, main.cpp does. decide() is handed a flat
// description of the machine and gives back the lines to send and one thing
// to do. It touches no hardware and keeps no state of its own, so every rule
// about which commands are allowed is here, in one place, and nowhere else.
//
// ORDER MATTERS, AND IT IS THE CALLER'S JOB. Send the lines first, then carry
// out the action. A move's ACK and PLAN have to be on the wire before the
// first pulse, because nothing may be printed between the first pulse and the
// last.
//
// MOVE and MOVETO are the same command but for what the angle means - how
// far to turn, or where to end up. They share every check and every reply;
// only the arithmetic at the front differs, and the ACK echoes whichever
// verb was sent.
//
// A MOVE may name both axes, each with its own speed and angle. Each half is
// checked exactly as a one-axis MOVE would be, azimuth first whichever order
// they were written in, and a refusal of either refuses the whole command:
// nothing moves. Once both are accepted there is one ACK, then a PLAN for
// azimuth and a PLAN for elevation, and both axes start together.
//
// Moves are judged against the settings in force (MachineState's az_spec and
// el_spec), not straight against motor_config.hpp, because acceleration and
// travel can be changed at the prompt, and because which built-in numbers an
// axis has depends on the gear mode. SAVE is the one command whose work -
// writing flash - the caller must do BEFORE sending the reply, since the ACK
// means it worked. See handle() in main.cpp.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "command_parser.hpp"
#include "motion_profile.hpp"
#include "motor_config.hpp"
#include "replies.hpp"

// What the firmware knows about itself at the moment a command arrives.
struct MachineState {
    // True while either axis is moving, or while tracking is armed (or
    // still slowing down after it was switched off). One busy axis blocks
    // commands for both - except STOP, ESTOP, TARGET and TRACK, which exist
    // to be sent while it is.
    bool busy = false;

    // True once TRACK ON has been accepted. A TARGET does nothing until it
    // is, so a stray message cannot move anything.
    bool tracking = false;

    // Where tracking is pointing, in steps, for the TRK report.
    int32_t az_target_steps = 0;
    int32_t el_target_steps = 0;

    // What each axis is turning at, millidegrees/sec, signed.
    int32_t az_mdps = 0;
    int32_t el_mdps = 0;

    bool az_zeroed = false;
    bool el_zeroed = false;
    int32_t az_pos_steps = 0;
    int32_t el_pos_steps = 0;

    // Which gearing the maths is counting. This decides what a degree
    // means on both axes, so it is part of the state every command is
    // judged against, not a setting on the side.
    GearMode gear = GearMode::Direct;

    // The settings in force: the built-in ones for `gear` until something is
    // set or loaded from flash.
    AxisSpec az_spec = AZ_SPEC;
    AxisSpec el_spec = EL_SPEC;

    // True when a restart would come back with different settings than
    // these, because they were changed and not saved.
    bool settings_unsaved = false;

    uint32_t uptime_ms = 0;
    const char *target = "";
    const char *version = "";
};

// The one thing the caller should do, once the lines are sent.
enum class Act {
    Nothing,
    StartMove,
    RequestStop,  // both axes, a ramped stop
    Estop,        // both axes, at once
    SetZero,
    SetPin,
    SetSettings,  // run with az_spec and el_spec from now on
    SaveSettings, // write the settings in force to flash
    SetGear,      // change gear mode: gear, az_spec and el_spec together
    SetTrack,     // arm or disarm tracking on both axes
    SetTarget,    // point here: az_steps and el_steps
};

// The most lines one command is answered with: a two-axis move's ACK and its
// two PLANs.
constexpr size_t MAX_LINES = 3;

struct Decision {
    Act act = Act::Nothing;

    // StartMove: which axes to start and the plan for each. A two-axis MOVE
    // sets both, under the one id.
    bool move_az = false;
    bool move_el = false;
    Move az_move{};
    Move el_move{};
    uint16_t id = 0;

    // SetZero, and SetTarget: which axes, and where. A target always names
    // both, so zero_az/zero_el are not consulted for it.
    bool zero_az = false;
    bool zero_el = false;
    int32_t az_steps = 0;
    int32_t el_steps = 0;

    // SetTrack: armed or not.
    bool track_on = false;

    // SetPin. Driving a step line makes the driver take a step nobody
    // counted, so that axis no longer knows where it is.
    uint8_t gpio = 0;
    uint8_t level = 0;
    bool clear_az_zero = false;
    bool clear_el_zero = false;

    // SetSettings and SetGear: the whole of both axes' settings, as they
    // are to be. SetGear also carries the mode they belong to, and sets the
    // clear_*_zero flags above, because changing mode means the drive train
    // has physically changed and the step count no longer describes a known
    // angle.
    GearMode gear    = GearMode::Direct;
    AxisSpec az_spec = AZ_SPEC;
    AxisSpec el_spec = EL_SPEC;

    // The complete lines to send, in order. There can be none: a stop during
    // a move is answered by the move's own DONE.
    char lines[MAX_LINES][MAX_REPLY_LEN] = {};
    size_t line_count = 0;
};

// Work out what to do about one command. Never touches hardware.
Decision decide(const Command &c, const MachineState &s);
