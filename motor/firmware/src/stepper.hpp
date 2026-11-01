// Driving one stepper axis: two pins, one for each way it turns, and a count
// of where we think the axis is.
//
// This is the only motor code that touches the pins. The arithmetic - how
// many steps, how long between them - is in motion_profile, so what is left
// here is sending the pulses on time.
//
// An axis runs in one of two ways. A PLANNED move is finite: aim(), begin(),
// and it walks a trapezium to its last step and reports. CHASING has no end -
// it holds a target and keeps closing on it, re-deciding its speed every
// tick, so a target that changes needs no replanning. The pulse-sending is
// identical; only what decides the next interval differs.
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
    Stepper(const AxisSpec &spec, uint pin_cw, uint pin_ccw)
        : spec_(spec), pin_cw_(pin_cw), pin_ccw_(pin_ccw)
    {
    }

    // Claim the two pins as outputs and park them HIGH, at rest.
    //
    // Push-pull outputs, always - never open-drain, never inputs. See the
    // active-low note in pins.hpp.
    void init();

    // Read the two pins back and check they are both at rest. Catches a pin
    // that was never configured, or one held down by something external. It
    // cannot tell whether a driver is plugged in.
    bool pins_ok() const;

    // --- running a move ----------------------------------------------------

    // Starting a move that plan_move() has approved takes two calls. aim()
    // picks the line the pulses go out on, and puts back at rest any line
    // the PIN command left LOW. begin() then makes the first step due
    // straight away. Neither is called while the axis is moving.
    //
    // They are separate so a two-axis move can aim both axes before beginning
    // either. Putting a line back means a pause, and otherwise the first
    // axis's opening step would sit waiting through the second axis's.
    void aim(const Move &m);
    void begin(const Move &m, uint16_t id);

    // Called as often as possible. Sends a pulse if one is due, otherwise
    // returns straight away.
    //
    // Returns true while pulses are being sent. An armed axis sitting inside
    // its deadband returns false - it is still chasing, but nothing is
    // turning, which is what the caller needs to know.
    bool tick(absolute_time_t now);

    // --- chasing a target --------------------------------------------------

    // Start chasing from idle, with both lines at rest. The target begins at
    // wherever the axis is, so arming alone emits no steps; it waits for a
    // target.
    void arm_chase();

    // Stop chasing and come to rest, ramping down rather than cutting the
    // pulses. The axis keeps its zero, so it can be armed again and carry on.
    void disarm_chase();

    // Where to point, in steps. Ignored unless armed.
    void set_target_steps(int32_t steps);

    bool chasing() const { return mode_ == Mode::Chasing; }
    int32_t target_steps() const { return target_steps_; }

    // What it is turning at, in millidegrees per second, signed. For
    // telemetry; zero when at rest.
    int32_t velocity_mdps() const;

    // Slow down and stop. The axis keeps stepping for as long as it takes to
    // come down from its current speed, so nothing is jerked to a halt.
    void request_stop();

    // Stop on the next loop, with no slowing down. This also clears the zero,
    // because after an abrupt halt the real position is no longer something
    // we can claim to know.
    void estop();

    // True during a planned move. Not while chasing: a tracking axis has
    // no DONE to report.
    bool moving() const { return mode_ == Mode::Planned; }

    // True whenever the axis may be turning: a planned move, tracking
    // armed, or still slowing down after tracking was switched off. While
    // it is true, every command but the stops, TRACK and TARGET is refused.
    bool busy() const { return mode_ != Mode::Idle; }

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
    enum class Mode {
        Idle,
        Planned,   // following a Move from plan_move, to its last step
        Chasing,   // closing on target_steps_, with no end
        Braking,   // was chasing, now ramping to a stop
    };

    // Put both lines back at rest before pulses start, in case the PIN
    // command left one LOW. Only then does it wait, for that line's driver
    // input to let go.
    void release_lines();

    // Send one pulse on the line for dir_positive_, and count it.
    void step();

    // One tick of the chase or brake loop. Shared, because braking is just
    // chasing an error of zero.
    bool chase_tick(absolute_time_t now, bool braking);

    // Seconds since chase_tick last ran, for the velocity ramp.
    float since_last_chase(absolute_time_t now);

    const AxisSpec &spec_;
    uint pin_cw_;
    uint pin_ccw_;

    int32_t pos_steps_ = 0;
    bool zeroed_       = false;

    // Which way the next pulse turns the axis: which line it goes out on, and
    // which way it is counted. Set by aim() for a planned move and by
    // chase_tick() while chasing.
    bool dir_positive_ = true;

    // --- the move in progress ---------------------------------------------
    Mode mode_        = Mode::Idle;
    Move plan_        = {};
    uint32_t index_   = 0;  // how many pulses sent so far
    bool aborted_     = false;
    uint16_t move_id_ = 0;

    // --- chasing ----------------------------------------------------------
    // Velocity is STATE here, unlike a planned move where it is worked out
    // from how far through the plan the axis is. That is the whole difference
    // between the two: a target that changes has something to update.
    int32_t target_steps_ = 0;
    float v_sps_          = 0.0f;
    absolute_time_t last_chase_ = {};

    absolute_time_t next_step_time_ = {};
    absolute_time_t first_pulse_    = {};
    absolute_time_t last_pulse_     = {};
};
