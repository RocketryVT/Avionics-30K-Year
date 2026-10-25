// The numbers that describe the two motor axes: how many steps make a degree,
// how far each axis may travel, how fast it may go and how quickly it may
// change speed. This is the file to edit when the machine changes.
//
// Two integer units are used throughout:
//
//   mdeg  millidegrees          45 degrees  -> 45000
//   mdps  millidegrees/second    5 deg/sec  ->  5000
//
// Integers rather than floats because these values cross the USB link as
// text, and whole numbers survive that exactly. The laptop converts to and
// from degrees for the operator.
//
// EVERY AXIS IS DESCRIBED TWICE, ONCE PER GEAR MODE. The machine is driven
// in one of two mechanical states, and an angle means a different thing in
// each:
//
//   DIRECT  each motor on its own, with nothing after its 10:1 planetary
//           gearbox. Angles are at the GEARBOX OUTPUT SHAFT. This is how
//           each axis is tested by itself.
//   GEARED  the final gear pair is in the drive train, both motors driving
//           the centre piece through the belt. Angles are at the ANTENNA.
//
// The mode is chosen at the motor> prompt with `gear on` / `gear off`, and
// saved with `save`, so no reflash is needed to go either way. See
// command_rules.cpp for what switching does (it costs both axes their zero,
// because the drive train has physically changed).
//
// Acceleration and travel can also be changed at the prompt, with `set` and
// `save` (see settings.hpp). The numbers here are then the BUILT-IN ones:
// what the Pico starts with until settings are saved, and the outer bounds
// any setting must stay inside. Change one here and reflash, and saved
// settings are ignored - the code wins when the code changes.
#pragma once

#include <stdint.h>

// ============================================================================
// Per-axis description
// ============================================================================
// One of these describes one axis in one gear mode. Everything the rest of
// the firmware needs to know about an axis is in here: there are no "the
// speed ceiling is really this" constants elsewhere that would quietly mean
// direct mode only.
struct AxisSpec {
    // Steps per output degree, kept as an exact fraction steps_num/steps_den
    // rather than a decimal. 44.444... never ends, and rounding it here would
    // make the error grow over a long move.
    int32_t steps_num;
    int32_t steps_den;

    // How far this axis may travel, in millidegrees. There are no limit
    // switches on this machine, so these two numbers are the only thing
    // stopping a move from driving the gimbal into a hard stop.
    int32_t min_mdeg;
    int32_t max_mdeg;

    // Set true if a positive command turns the axis the wrong way. That
    // depends on the driver's SW5 switch and on the gear mesh, so it cannot
    // be known until it is tried - and the two gear modes can disagree,
    // because an external gear pair reverses the direction and a belt does
    // not.
    bool invert_dir;

    // The speed a move begins and ends at. See V_START_MDPS.
    int32_t v_start_mdps;

    // How quickly the axis may change speed, in millideg/sec/sec.
    int32_t accel_mdps2;

    // How far accel_mdps2 may be moved at the prompt. See the note on
    // ACCEL_MIN_MDPS2 below.
    int32_t accel_min_mdps2;
    int32_t accel_max_mdps2;

    // The fastest this axis may be commanded, in millideg/sec. See
    // SPEED_MAX_MDPS below.
    int32_t speed_max_mdps;

    // How close to a target counts as being there, in millidegrees, and how
    // far that may be set. See DEADBAND_MDEG.
    int32_t deadband_mdeg;
    int32_t deadband_max_mdeg;
};

// ============================================================================
// The direct-mode numbers
// ============================================================================
// These are the values for an axis driven straight off its gearbox. They are
// the seed for both modes: geared() below scales them for the gear pair.

// Moves start and end at 5 deg/sec rather than at a dead stop.
//
// A stepper does not need to ease away from rest at these speeds: 5 deg/sec
// is about 8 rpm at the motor, behind the 10:1 gearbox. Starting here buys
// two things:
//
//   1. The bench procedure runs at 5 deg/sec, so those moves have no ramp at
//      all. They run at one steady speed and take exactly angle/speed
//      seconds, which keeps the results easy to check by hand.
//   2. Starting from a true zero makes the first step interval enormous
//      (tens of milliseconds), which looks like the program has hung.
constexpr int32_t V_START_MDPS = 5000;  // 5 deg/sec

// How far acceleration may be set at the prompt.
//
// Travel set at the prompt can only narrow the limits below, never widen
// them. Acceleration is different: the point is to try higher values without
// reflashing, so it gets bounds of its own.
//
// These guard against typing mistakes; they are not a promise the motors can
// do it. 360 is twice the 180 aimed for, so a slipped digit (1800) is refused
// while every value worth trying is allowed. 10 keeps a move from being so
// gentle it seems to hang.
constexpr int32_t ACCEL_MIN_MDPS2 = 10000;   //  10 deg/sec/sec
constexpr int32_t ACCEL_MAX_MDPS2 = 360000;  // 360 deg/sec/sec

// The ceiling comes from the motor's torque curve (23HS40-5004D-E1000 on a
// CL57T at 36 V): 181 deg/sec is about 300 rpm at the motor, still on the
// flat part of the curve with roughly 2.0 N.m of pull-out torque, against
// 2.3 N.m at low speed. Past about 600 rpm (360 deg/sec) the torque halves.
// Below 36 V the curve falls off sooner, so measure the supply before
// raising this.
constexpr int32_t SPEED_MAX_MDPS = 181000;  // 181 deg/sec

// The floor exists so a typo like "0" cannot start a move that never ends.
// It is the same in both gear modes: it is not a torque number, just a guard
// against a move that would outlast the test.
constexpr int32_t SPEED_MIN_MDPS = 100;  // 0.1 deg/sec

// ----------------------------------------------------------------------------
// Deadband - how close to a target counts as being there
// ----------------------------------------------------------------------------
// Only used when chasing a target (see chase_velocity in motion_profile.hpp).
// Inside it the axis is commanded to stop, which is what keeps it from
// chattering back and forth on the noise in a position fix.
//
// NOT scaled by the gear pair, unlike every speed number here. This is a
// tolerance on the thing being pointed: half a degree of antenna in geared
// mode, half a degree of gearbox shaft in direct mode. mdeg_to_steps turns it
// into the right number of steps for whichever mode is in force - 22 steps
// direct, 51 on geared azimuth, 32 on geared elevation.
//
// Half a degree is where last year's ground station ran (0.35-0.5 deg on its
// PID loop), and the antenna is a 7-element DL6WU Yagi at about 12 dBi -
// roughly 50 deg of total beamwidth, so +/-25 deg to the half-power point.
// Half a degree is therefore about 2% of the half-beam: validated by use, and
// with enough margin that it can be relaxed to 1-2 deg if the mechanism turns
// out to chatter, at no measurable cost in signal.
//
// Zero is allowed, and means "chase every last step". It is not useful on a
// real fix, which is never that quiet.
constexpr int32_t DEADBAND_MDEG     = 500;   // 0.5 deg
constexpr int32_t DEADBAND_MAX_MDEG = 5000;  // 5   deg

// ----------------------------------------------------------------------------
// How long tracking will go without hearing anything
// ----------------------------------------------------------------------------
// Targets arriving are the only sign the other end is still there. If they
// stop - a radio dropout, a closed laptop, a pulled cable - the axes ramp to
// rest and stay there until something arms tracking again.
//
// This is the guard that pays for clamping a target instead of refusing it.
// A planned move is finite by construction: whatever happens to the laptop,
// it ends. A target is not, so without a watchdog a dropout would leave the
// antenna driving at an angle that was right minutes ago.
//
// It has to be comfortably longer than the gap between fixes, or an ordinary
// quiet moment would look like a failure. 1.5 s suits anything from about
// 1 Hz upwards. (Last year ignored IMU data older than 250 ms, which is the
// same idea at a much higher data rate.)
constexpr uint32_t TRACK_TIMEOUT_MS = 1500;

// ----------------------------------------------------------------------------
// Steps per degree
// ----------------------------------------------------------------------------
// Where 400/9 comes from, which is DIRECT mode:
//
//   driver pulses per motor turn          1600        (CL57TE DIP switches)
//   gearbox                              10 : 1       (planetary)
//
//   steps per gearbox output turn   = 1600 * 10 = 16000
//   steps per gearbox output degree = 16000/360 = 400/9 = 44.444...
//
// Both axes have the same motor, driver setting and gearbox, so both get the
// same fraction. Every multiple of 9 degrees is a whole number of steps:
// 45 degrees is exactly 2000.
//
// THE 1600 IS A SWITCH SETTING, AND NOTHING IN SOFTWARE CAN CHECK IT. It is
// DIP switches 1-4 on each driver, which the CL57T manual reads as:
//
//   1600 pulses/turn (microstep 8)   SW1 on   SW2 off  SW3 on   SW4 on
//    800 pulses/turn (microstep 4)   SW1 off  SW2 on   SW3 on   SW4 on
//
// The drivers on this machine were checked at 1600 on 17 Oct 2026. If one is
// ever swapped or reset, check this first: get it wrong and every angle is
// out by a clean factor of two, while the Pico reports the angle it meant to
// turn. (Last year's ground station ran 800, and its own header comment said
// 1600 - the two disagreed. This machine was measured.)

// ----------------------------------------------------------------------------
// The final gear pair - GEARED mode
// ----------------------------------------------------------------------------
// After each gearbox there is one more stage, between it and the antenna.
// Counted as teeth, so changing one is changing one number:
//
//   azimuth    32 tooth ring driven by a 14 tooth pinion
//   elevation  20 tooth wheel driven by a 14 tooth pulley, on the belt
//
// These come from last year's ground station firmware (axis_task.cpp,
// AZ_GEAR_RATIO and EL_GEAR_RATIO) and are what this year's spec was written
// against. COUNT THE TEETH BEFORE TRUSTING THEM - azimuth's 32:14 has never
// been checked on this machine.
//
// Elevation's 20:14 has. It was found the hard way on 18 Oct 2026, with the
// pair folded into the maths while it was NOT in the drive train: 180
// commanded turned about 270, and 45 turned 50-55. Together that is 322
// against 225, a factor of 1.43 - which is 20/14 exactly. Running the wrong
// mode for the machine is off by the pair's ratio, every move, in whichever
// direction the mistake was made.
struct GearPair {
    int32_t out_teeth;  // the wheel on the antenna side
    int32_t in_teeth;   // the wheel on the gearbox shaft
};

constexpr GearPair AZ_PAIR = {32, 14};  // -> 10 * 32/14 = 160/7 : 1 (22.857)
constexpr GearPair EL_PAIR = {20, 14};  // -> 10 * 20/14 = 100/7 : 1 (14.286)

// Which mechanical state the machine is in. Direct is the safe default: it
// is what a Pico with nothing saved comes up in, and getting it wrong
// undershoots rather than overshooting into a hard stop.
enum class GearMode {
    Direct,  // gearbox only, each axis on its own
    Geared,  // gearbox and the final pair, the whole antenna
};

// Fold a final gear pair into an axis description.
//
// THE RULE: the pair changes what the ANTENNA does, not what the MOTOR is
// asked to do. So steps per degree go up by the pair's ratio, and every
// speed number goes down by it. A command of "5 deg/sec" in geared mode then
// turns the motor at exactly the rate 5 deg/sec turned it in direct mode -
// the rate the bench verified - and the antenna moves more slowly because
// there is more reduction in the way.
//
// Without that, 181 deg/sec in geared mode would ask azimuth's motor for
// about 690 rpm, past where its torque halves. A stepper that stalls there
// keeps being sent pulses and loses position silently, which is the one
// failure this firmware cannot detect: there is no encoder and the driver's
// alarm line is not wired.
//
// Travel and direction are not derived. Travel is an antenna angle rather
// than a gearbox angle, so it is a different measurement, and whether the
// pair reverses the axis depends on the mesh.
//
// The deadband is not derived either, and for the opposite reason to the
// speeds: it is a tolerance on the output, so half a degree means half a
// degree in both modes.
constexpr AxisSpec geared(const AxisSpec &base, const GearPair &pair,
                          int32_t min_mdeg, int32_t max_mdeg, bool invert_dir)
{
    return {
        base.steps_num * pair.out_teeth,
        base.steps_den * pair.in_teeth,
        min_mdeg,
        max_mdeg,
        invert_dir,
        base.v_start_mdps * pair.in_teeth / pair.out_teeth,
        base.accel_mdps2 * pair.in_teeth / pair.out_teeth,
        base.accel_min_mdps2 * pair.in_teeth / pair.out_teeth,
        base.accel_max_mdps2 * pair.in_teeth / pair.out_teeth,
        base.speed_max_mdps * pair.in_teeth / pair.out_teeth,
        base.deadband_mdeg,
        base.deadband_max_mdeg,
    };
}

// ----------------------------------------------------------------------------
// Azimuth (the board calls it Servo 1)
// ----------------------------------------------------------------------------
// Travel is -180 to +180 degrees from the zero, set at the bench on 25 Oct
// 2026 - one full turn, centred on the zero, in both gear modes.
//
// There is no wrap-around anywhere: an angle is a plain number, not a compass
// bearing, so `goto az 5 179` from -179 turns +358 degrees through 0 rather
// than taking the 2-degree short cut across the seam at +/-180. That is the
// point of the limits - whatever is cabled to the axis is never wound more
// than one turn. It does mean the seam is where something dramatic happens:
// a target crossing from +179.9 to -179.9 is a full turn the other way. Zero
// the axis so the seam points somewhere the rocket will not be (behind the
// launch direction, say).
//
// Acceleration is 135 deg/sec/sec, verified at the bench on 18 Oct 2026. It
// reaches the 181 deg/sec ceiling in 1.3 seconds, over about 121 degrees, so
// a move needs about 242 degrees to get up to full speed and back down;
// shorter moves peak lower. Raise it one step at a time - `set az accel 150`
// at the prompt, no reflash - watching the driver's alarm light and the
// return-to-zero reading.
constexpr AxisSpec AZ_SPEC = {
    400,     // steps_num
    9,       // steps_den     -> 44.444 steps per degree
    -180000, // min_mdeg      -180 degrees
    180000,  // max_mdeg      +180 degrees
    true,    // invert_dir    flipped at the bench, 25 Oct 2026: `move az 45
             //               10` went the wrong way. At the time the driver
             //               turned one way for every command, + or -, so
             //               confirm with `move az 5 10` / `move az 5 -10`
             //               once it reverses.
    V_START_MDPS,
    135000,  // accel_mdps2   135 deg/sec/sec (aiming for 180)
    ACCEL_MIN_MDPS2,
    ACCEL_MAX_MDPS2,
    SPEED_MAX_MDPS,
    DEADBAND_MDEG,
    DEADBAND_MAX_MDEG,
};

// Geared: 400/9 * 32/14 = 12800/126 = 6400/63 = 101.587 steps per antenna
// degree. One full turn of antenna, -180 to +180, is 2.3 turns of the gearbox
// shaft. The scaled numbers come out at 2.187 deg/s start, 59.062 deg/s/s
// acceleration and a 79.187 deg/s ceiling.
constexpr AxisSpec AZ_SPEC_GEARED =
    geared(AZ_SPEC, AZ_PAIR,
           -180000, // min_mdeg    -180 degrees of antenna
           180000,  // max_mdeg    +180 degrees of antenna
           false);  // invert_dir  unverified - a ring and pinion may flip it

// ----------------------------------------------------------------------------
// Elevation (the board calls it Servo 2)
// ----------------------------------------------------------------------------
// Direct travel is -25 to 360 degrees of the gearbox shaft. 0 to 360 was
// verified safe at the bench on 18 Oct 2026 with the axis tested on its own;
// the minimum went to -25 on 25 Oct 2026 so moves below the zero could be
// tried. Narrow it at the prompt (`set el min 0`) if that is not wanted - a
// limit can be narrowed without reflashing, only widened by editing here.
//
// Acceleration is the same 135 deg/sec/sec as azimuth. Recheck it once the
// axis is lifting the antenna's weight.
constexpr AxisSpec EL_SPEC = {
    400,     // steps_num
    9,       // steps_den     -> 44.444 steps per degree
    -25000,  // min_mdeg      -25 degrees. Asked for at the bench, 25 Oct 2026
    360000,  // max_mdeg      360 degrees of the gearbox shaft
    false,   // invert_dir    verified at the bench 25 Oct 2026 with
             //               10-degree moves each way. A first try with a
             //               180-degree move suggested true, but a half
             //               turn ends in the same place whichever way it
             //               goes - judge direction with small moves.
    V_START_MDPS,
    135000,  // accel_mdps2   135 deg/sec/sec
    ACCEL_MIN_MDPS2,
    ACCEL_MAX_MDPS2,
    SPEED_MAX_MDPS,
    DEADBAND_MDEG,
    DEADBAND_MAX_MDEG,
};

// Geared: 400/9 * 20/14 = 8000/126 = 4000/63 = 63.492 steps per antenna
// degree. The scaled numbers are 3.500 deg/s start, 94.500 deg/s/s and a
// 126.700 deg/s ceiling.
//
// THE 0..180 TRAVEL HAS NOT BEEN MEASURED ON THE ASSEMBLED ANTENNA. It is
// what the mechanism was described as swinging; it is also the only thing
// between a mistyped command and a hard stop, since there are no limit
// switches. Check it before the first geared move, and narrow it at the
// prompt (`set el max 170`, `save`) rather than here if it turns out tighter.
constexpr AxisSpec EL_SPEC_GEARED =
    geared(EL_SPEC, EL_PAIR,
           -25000,  // min_mdeg    25 degrees below the zero. If the zero is
                    //             horizontal, that is pointing below the
                    //             horizon - last year's system stopped at
                    //             -10, so check the mount clears -25 first
           180000,  // max_mdeg    180 degrees of antenna
           false);  // invert_dir  same as direct, because a belt does not
           //             reverse rotation. Not yet seen with the
                    //             belt on - confirm on the first geared move

// ============================================================================
// Looking an axis up by mode
// ============================================================================
// Taken by value rather than by reference: an AxisSpec is forty bytes and
// this is never called in the step loop.
//
// Two functions rather than one taking an axis, because this header
// deliberately includes nothing but <stdint.h> - no AxisId, no Pico headers -
// so the step maths can be compiled and checked with the laptop's g++.
constexpr AxisSpec az_built_in(GearMode mode)
{
    return (mode == GearMode::Geared) ? AZ_SPEC_GEARED : AZ_SPEC;
}

constexpr AxisSpec el_built_in(GearMode mode)
{
    return (mode == GearMode::Geared) ? EL_SPEC_GEARED : EL_SPEC;
}

// ============================================================================
// Sanity rails on the gap between two steps
// ============================================================================
// Nothing should ever reach these: the fastest any axis may be commanded in
// either mode is one step every 124 us, and the slowest one every 225 ms.
// They hold in geared mode because the speed ceiling was scaled with the
// gearing, so the motor is never asked to go faster than it was in direct
// mode. If a computed interval hits a rail the maths is wrong, and failing
// toward "too slow" is the safe direction.
constexpr uint32_t MIN_INTERVAL_US = 50;      // 20 kHz, far above anything used
constexpr uint32_t MAX_INTERVAL_US = 500000;  // half a second between steps

// The longest time step the chase loop will believe.
//
// It works from the gap since it last ran, and the main loop runs thousands
// of times a second - but a flash write or a blocked USB send can hold it up
// far longer. Taking that gap at face value would allow one enormous jump in
// commanded speed. Clamping it means a stalled loop resumes gently instead,
// which is the same instinct as the catch-up guard in stepper.cpp: give up
// the lost time rather than try to make it back.
constexpr float CHASE_MAX_DT_S = 0.05f;  // 50 ms

// ============================================================================
// Pulse timing
// ============================================================================
// The CL57TE manual's minimums are a 1 us pulse and 5 us of DIR setup. These
// are what last year's ground station ran, which is far more generous. There
// is no reason to shave them: even at full speed one step is still 124 us
// from the next.
constexpr uint32_t PULSE_LOW_US     = 5;    // how long PUL is held LOW
constexpr uint32_t DIR_SETUP_US     = 100;  // settle time after changing DIR
constexpr uint32_t REVERSE_PAUSE_MS = 1;    // extra pause before reversing
