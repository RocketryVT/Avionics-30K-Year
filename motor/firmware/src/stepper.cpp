#include "stepper.hpp"

#include "hardware/gpio.h"

#include "pins.hpp"

void Stepper::init()
{
    // Set the level before switching the pin to an output. Doing it the other
    // way round makes the pin briefly drive low, and a low on PUL is a step -
    // so the motor would twitch every time the Pico restarted.
    gpio_init(pin_pul_);
    gpio_put(pin_pul_, PUL_IDLE);
    gpio_set_dir(pin_pul_, GPIO_OUT);

    dir_positive_ = true;
    gpio_init(pin_dir_);
    gpio_put(pin_dir_, dir_level(dir_positive_, spec_.invert_dir));
    gpio_set_dir(pin_dir_, GPIO_OUT);
}

bool Stepper::pins_ok() const
{
    return gpio_get(pin_pul_) == PUL_IDLE &&
           gpio_get(pin_dir_) == dir_level(dir_positive_, spec_.invert_dir);
}

void Stepper::set_direction(bool positive)
{
    const bool reversing = (positive != dir_positive_);

    dir_positive_ = positive;
    gpio_put(pin_dir_, dir_level(positive, spec_.invert_dir));

    // The driver wants the direction settled well before the next pulse, and
    // a reversal is where that is tightest, so it gets an extra pause.
    if (reversing) {
        sleep_ms(REVERSE_PAUSE_MS);
    }
    busy_wait_us(DIR_SETUP_US);
}

void Stepper::step()
{
    gpio_put(pin_pul_, PUL_PULSE);
    busy_wait_us(PULSE_LOW_US);
    gpio_put(pin_pul_, PUL_IDLE);

    pos_steps_ += dir_positive_ ? 1 : -1;
}

void Stepper::aim(const Move &m)
{
    set_direction(m.steps >= 0);
}

void Stepper::begin(const Move &m, uint16_t id)
{
    plan_    = m;
    index_   = 0;
    aborted_ = false;
    move_id_ = id;
    moving_  = (m.n_steps > 0);

    // The first step is due immediately. aim() has already waited out the
    // setup time, so the driver is ready.
    next_step_time_ = get_absolute_time();
    first_pulse_    = next_step_time_;
    last_pulse_     = next_step_time_;
}

bool Stepper::tick(absolute_time_t now)
{
    if (!moving_) {
        return false;
    }

    // Not due yet. This is the usual answer - the loop runs far more often
    // than steps are needed.
    if (absolute_time_diff_us(now, next_step_time_) > 0) {
        return true;
    }

    if (index_ == 0) {
        first_pulse_ = now;
    }
    step();
    last_pulse_ = now;
    index_++;

    if (index_ >= plan_.n_steps) {
        moving_ = false;
        return false;
    }

    const uint32_t gap = interval_us(plan_, index_);

    // The next deadline is measured from the last deadline, not from now, so
    // small delays do not add up over thousands of steps.
    absolute_time_t next = delayed_by_us(next_step_time_, gap);

    // But if something held the loop up for longer than a whole step, that
    // would fire a burst of pulses back to back trying to catch up - which is
    // exactly how a stepper loses its position. Better to give up the lost
    // time and carry on at the right rate.
    if (absolute_time_diff_us(next, now) > static_cast<int64_t>(gap)) {
        next = delayed_by_us(now, gap);
    }
    next_step_time_ = next;
    return true;
}

void Stepper::request_stop()
{
    if (!moving_) {
        return;
    }

    // Stop by shortening the move rather than with a separate braking mode.
    // interval_us() already slows down over the last n_ramp steps, so ending
    // the move that many steps from here gives a proper ramp down from
    // whatever speed it is doing now.
    //
    // While still speeding up, the distance needed to slow down is the
    // distance already covered, so it is index_ rather than n_ramp.
    const uint32_t braking   = (index_ < plan_.n_ramp) ? index_ : plan_.n_ramp;
    const uint32_t shortened = index_ + braking + 1;

    if (shortened < plan_.n_steps) {
        plan_.n_steps = shortened;
    }
    aborted_ = true;
}

void Stepper::estop()
{
    if (!moving_) {
        return;
    }
    moving_  = false;
    aborted_ = true;

    // Pulses stopped mid-move with no ramp down. A closed-loop driver will
    // settle somewhere near, but not exactly where our count says, and there
    // is no feedback to find out. Refusing further moves until the operator
    // re-zeros is the honest response.
    zeroed_ = false;
}

int32_t Stepper::steps_done() const
{
    const int32_t n = static_cast<int32_t>(index_);
    return (plan_.steps < 0) ? -n : n;
}

uint32_t Stepper::elapsed_us() const
{
    const int64_t us = absolute_time_diff_us(first_pulse_, last_pulse_);
    return (us < 0) ? 0 : static_cast<uint32_t>(us);
}
