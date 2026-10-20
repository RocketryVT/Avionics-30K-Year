// Driving one stepper axis: two pins, a direction, and a count of where we
// think the axis is.
//
// This is the only motor code that touches the pins. The arithmetic - how
// many steps, how long between them - is in motion_profile, so what is left
// here is sending the pulses on time.
//
// About the position count: it goes up or down by one for every pulse this
// code sends out. It is what the firmware BELIEVES, not what the gimbal has
// done. The driver's alarm line is not wired, so if a driver faults or the
// mechanism jams, the pulses keep going out and the count keeps changing
// while the antenna stands still. The firmware cannot notice; the operator
// watching the red LED on the driver is the only check.
#pragma once

#include "pico/stdlib.h"

#include "motion_profile.hpp"
#include "motor_config.hpp"

class Stepper {
public:
    Stepper(const AxisSpec &spec, uint pin_pul, uint pin_dir)
        : spec_(spec), pin_pul_(pin_pul), pin_dir_(pin_dir)
    {
    }

    // Claim the two pins as outputs and park them at their idle levels.
    //
    // Push-pull outputs, always - never open-drain, never inputs. See the
    // active-low note in pins.hpp.
    void init();

    // Read the two pins back and check they are at the levels init() set.
    // Catches a pin that was never configured, or one held down by something
    // external. It cannot tell whether a driver is plugged in.
    bool pins_ok() const;

    // --- running a move ----------------------------------------------------

    // Starting a move that plan_move() has approved takes two calls. aim()
    // sets the direction pin and waits for the driver to notice it. begin()
    // then makes the first step due straight away. Neither is called while
    // the axis is moving.
    //
    // They are separate so a two-axis move can aim both axes before beginning
    // either. Otherwise the first axis's opening step would sit waiting while
    // the second axis's direction settled.
    void aim(const Move &m);
    void begin(const Move &m, uint16_t id);

    // Called as often as possible. Sends a pulse if one is due, otherwise
    // returns straight away. Returns true while the move is still running.
    bool tick(absolute_time_t now);

    // Slow down and stop. The axis keeps stepping for as long as it takes to
    // come down from its current speed, so nothing is jerked to a halt.
    void request_stop();

    // Stop on the next loop, with no slowing down. This also clears the zero,
    // because after an abrupt halt the real position is no longer something
    // we can claim to know.
    void estop();

    bool moving() const { return moving_; }
    bool aborted() const { return aborted_; }

    // Non-zero from the start of a move until its DONE has been sent, so the
    // main loop knows there is still a report owing for this axis.
    uint16_t move_id() const { return move_id_; }
    void clear_move_id() { move_id_ = 0; }
    const Move &plan() const { return plan_; }

    // Signed, so it reads the same way as the commanded step count.
    int32_t steps_done() const;

    // First pulse to last pulse: how long the axis was actually turning.
    uint32_t elapsed_us() const;

    // --- where we think we are --------------------------------------------
    int32_t pos_steps() const { return pos_steps_; }
    int32_t pos_mdeg() const { return steps_to_mdeg(spec_, pos_steps_); }

    // Declare the current position. This is what ZERO does.
    void set_pos_steps(int32_t steps)
    {
        pos_steps_ = steps;
        zeroed_    = true;
    }

    // Until an axis has been zeroed, the soft limits are measured from an
    // unknown origin, which is no protection at all, so moves are refused.
    bool zeroed() const { return zeroed_; }
    void clear_zero() { zeroed_ = false; }

private:
    // Point the axis the right way and wait for the driver to notice.
    void set_direction(bool positive);

    // Send one pulse and count it.
    void step();

    const AxisSpec &spec_;
    uint pin_pul_;
    uint pin_dir_;

    int32_t pos_steps_ = 0;
    bool zeroed_       = false;

    // Which way the DIR pin is pointing, so a reversal can be spotted.
    // Starts matching what init() writes.
    bool dir_positive_ = true;

    // --- the move in progress ---------------------------------------------
    Move plan_        = {};
    uint32_t index_   = 0;  // how many pulses sent so far
    bool moving_      = false;
    bool aborted_     = false;
    uint16_t move_id_ = 0;

    absolute_time_t next_step_time_ = {};
    absolute_time_t first_pulse_    = {};
    absolute_time_t last_pulse_     = {};
};
