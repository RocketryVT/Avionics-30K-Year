// gs_motor: the program on the motor Pico.
//
// Reads commands from the laptop over USB, moves the two axes, and reports
// what happened. The whole program is one loop:
//
//     forever:
//         let each axis send a pulse if one is due
//         read any bytes that arrived, and act on a complete line
//         report any axis that has just finished
//         send a heartbeat if it is time and nothing is moving
//
// No interrupts, no second core. The loop runs far more often than steps are
// needed - the fastest step rate used here is one every 124 us - so an axis
// is never left waiting on it, and the whole program reads top to bottom.
//
// How a command is handled:
//
//     line_reader     bytes from USB  ->  one line of text
//     command_parser  line of text    ->  Command
//     command_rules   Command         ->  Decision (lines to send, one action)
//     main.cpp        sends the lines, then carries out the action
//
// ONE RULE MATTERS MORE THAN THE REST: nothing is printed between the first
// and last pulse of a move. Writing to USB blocks when the laptop stops
// reading, and a block in the middle of a move would stall the step loop with
// the motor part way round. So the heartbeat is held back while moving, DONE
// is sent after the last pulse, and a command that arrives mid-move is
// answered with a single short line.
//
// Deliberately absent:
//   no ENA, no ALM     not wired on this board; the Pico cannot disable a
//                      driver and cannot tell when one faults
//   no limit switches  none fitted; the soft limits are all there is
//   no encoder         position here is what we believe, not what happened
#include <stdio.h>

#include "pico/stdio_usb.h"
#include "pico/stdlib.h"

#include "command_parser.hpp"
#include "command_rules.hpp"
#include "line_reader.hpp"
#include "motor_config.hpp"
#include "pins.hpp"
#include "replies.hpp"
#include "settings.hpp"
#include "settings_flash.hpp"
#include "stepper.hpp"

constexpr uint32_t HEARTBEAT_PERIOD_MS = 5000;

// Which gearing the maths counts: the gearbox alone, or the gearbox and the
// final pair. Direct until a saved record says otherwise, because a wrong
// guess that way undershoots instead of driving into a hard stop.
static GearMode g_gear = GearMode::Direct;

// The settings in force - acceleration and travel, which can be changed at
// the prompt. They start as the built-in ones for g_gear and at startup are
// replaced by any good ones saved in flash. The steppers read their fixed
// numbers (steps per degree, direction) from here too, so each axis has one
// description, and switching gear mode rewrites it in place.
static AxisSpec g_az_spec = AZ_SPEC;
static AxisSpec g_el_spec = EL_SPEC;

// What a restart would come back with, so the laptop can be told whether the
// settings in force have been saved. The gear mode is saved with them.
static GearMode g_boot_gear = GearMode::Direct;
static AxisSpec g_boot_az   = AZ_SPEC;
static AxisSpec g_boot_el   = EL_SPEC;

static Stepper g_az(g_az_spec, PIN_AZ_PUL, PIN_AZ_DIR);
static Stepper g_el(g_el_spec, PIN_EL_PUL, PIN_EL_DIR);
static LineReader g_reader;

// One buffer, reused. Every reply is built into it whole and then sent with a
// single fputs, so two messages can never interleave.
static char g_out[MAX_REPLY_LEN];

static void send(size_t n)
{
    if (n > 0) {
        fputs(g_out, stdout);
    }
}

// ----------------------------------------------------------------------------
// Startup
// ----------------------------------------------------------------------------

// Wait for the laptop to open the port, so the banner is not printed into a
// port nobody is reading.
static void wait_for_usb_forever()
{
    while (!stdio_usb_connected()) {
        sleep_ms(50);
    }
    sleep_ms(200);  // let the laptop finish opening before the first line
}

// Millidegrees as degrees with three decimals, for the banner.
static void print_mdeg(int32_t mdeg)
{
    const long whole = mdeg / 1000;
    long frac        = mdeg % 1000;
    if (frac < 0) {
        frac = -frac;
    }
    if (mdeg < 0 && whole == 0) {
        printf("-0.%03ld", frac);
    } else {
        printf("%ld.%03ld", whole, frac);
    }
}

static void print_steps_per_deg(const AxisSpec &axis)
{
    const int32_t milli = mdeg_to_steps(axis, 1000000);
    printf("%ld.%03ld", static_cast<long>(milli / 1000),
           static_cast<long>(milli % 1000));
}

// Never die quietly, and never carry on regardless.
static void fatal_forever(const char *reason)
{
    for (;;) {
        printf("[fatal] %s\n", reason);
        sleep_ms(2000);
    }
}

// What the next startup would load: the saved settings if they are good for
// this build, otherwise the built-in ones.
static LoadResult read_boot_settings()
{
    // Seeded with direct mode and its numbers, so a record that is blank,
    // corrupt or stale leaves the machine counting the gearbox alone.
    g_boot_gear = GearMode::Direct;
    g_boot_az   = AZ_SPEC;
    g_boot_el   = EL_SPEC;
    return load_settings(&g_boot_gear, &g_boot_az, &g_boot_el);
}

static const char *settings_note(LoadResult r)
{
    switch (r) {
    case LoadResult::Ok:      return "saved ones in use";
    case LoadResult::Blank:   return "built in";
    case LoadResult::Stale:
        return "built in - saved ones ignored, motor_config.hpp has changed";
    case LoadResult::Corrupt:
        return "built in - saved ones could not be read";
    }
    return "built in";
}

// Spelled out rather than just "on" or "off": getting this wrong puts every
// move out by the gear pair's ratio, and this line is the operator's one
// chance to notice before they type a move.
static const char *gear_note(GearMode g)
{
    return (g == GearMode::Geared)
               ? "GEARED - final pair counted, angles at the ANTENNA"
               : "direct - gearbox only, angles at the GEARBOX OUTPUT SHAFT";
}

static void print_banner(LoadResult loaded)
{
    printf("[motor] pins AZ PUL=GP%u DIR=GP%u  EL PUL=GP%u DIR=GP%u  idle HIGH OK\n",
           PIN_AZ_PUL, PIN_AZ_DIR, PIN_EL_PUL, PIN_EL_DIR);
    printf("[motor] active low: PUL rests HIGH and is pulled LOW to step\n");

    printf("[motor] gear %s\n", gear_note(g_gear));

    printf("[motor] ready az=");
    print_steps_per_deg(g_az_spec);
    printf(" st/deg el=");
    print_steps_per_deg(g_el_spec);
    printf(" st/deg\n");

    printf("[motor] limits az ");
    print_mdeg(g_az_spec.min_mdeg);
    printf("..");
    print_mdeg(g_az_spec.max_mdeg);
    printf("  el ");
    print_mdeg(g_el_spec.min_mdeg);
    printf("..");
    print_mdeg(g_el_spec.max_mdeg);
    printf("\n");

    printf("[motor] accel az ");
    print_mdeg(g_az_spec.accel_mdps2);
    printf("  el ");
    print_mdeg(g_el_spec.accel_mdps2);
    printf(" deg/s/s\n");

    printf("[motor] settings: %s\n", settings_note(loaded));

    printf("[motor] NOT ZEROED - send ZERO before any MOVE\n");
}

// ----------------------------------------------------------------------------
// Commands
// ----------------------------------------------------------------------------

// The machine as it is right now, for decide() to judge.
//
// Taken fresh for every command, so that two commands arriving in one USB
// packet are each judged against the machine as it is when that command is
// handled - a MOVE that the first one started makes the second one busy.
static MachineState snapshot()
{
    MachineState s;
    s.busy         = g_az.moving() || g_el.moving();
    s.az_zeroed    = g_az.zeroed();
    s.el_zeroed    = g_el.zeroed();
    s.az_pos_steps = g_az.pos_steps();
    s.el_pos_steps = g_el.pos_steps();
    s.gear         = g_gear;
    s.az_spec      = g_az_spec;
    s.el_spec      = g_el_spec;
    s.settings_unsaved = g_gear != g_boot_gear ||
                         !same_settings(g_az_spec, g_boot_az) ||
                         !same_settings(g_el_spec, g_boot_el);
    s.uptime_ms    = to_ms_since_boot(get_absolute_time());
    s.target       = GS_TARGET_NAME;
    s.version      = GS_VERSION;
    return s;
}

static void set_test_pin(const Decision &d)
{
    // A spare pin has never been set up, so it needs claiming first.
    //
    // A motor pin has: Stepper::init() already made it an output at its idle
    // level, and gpio_init() would make it an input again for a moment on the
    // way past - letting it float, which pins.hpp warns against. So the level
    // is written straight to it instead.
    if (pin_is_spare(d.gpio)) {
        gpio_init(d.gpio);
        gpio_set_dir(d.gpio, GPIO_OUT);
    }
    gpio_put(d.gpio, d.level != 0);
}

// Carry out what decide() asked for. Nothing here chooses anything.
static void apply(const Decision &d)
{
    switch (d.act) {
    case Act::Nothing:
        break;
    case Act::StartMove:
        // Every axis about to move is aimed before any of them begins. Each
        // aim waits for its driver to see the direction pin, so doing them
        // all first means both axes of a two-axis move set off together.
        if (d.move_az) {
            g_az.aim(d.az_move);
        }
        if (d.move_el) {
            g_el.aim(d.el_move);
        }
        if (d.move_az) {
            g_az.begin(d.az_move, d.id);
        }
        if (d.move_el) {
            g_el.begin(d.el_move, d.id);
        }
        break;
    case Act::RequestStop:
        g_az.request_stop();
        g_el.request_stop();
        break;
    case Act::Estop:
        g_az.estop();
        g_el.estop();
        break;
    case Act::SetZero:
        if (d.zero_az) {
            g_az.set_pos_steps(d.az_steps);
        }
        if (d.zero_el) {
            g_el.set_pos_steps(d.el_steps);
        }
        break;
    case Act::SetPin:
        set_test_pin(d);
        break;
    case Act::SetSettings:
        g_az_spec = d.az_spec;
        g_el_spec = d.el_spec;
        break;
    case Act::SaveSettings:
        break;  // already done in handle(), before the reply
    case Act::SetGear:
        // Both specs are replaced whole, not just their settable numbers:
        // the mode decides steps per degree and every speed number too.
        g_gear    = d.gear;
        g_az_spec = d.az_spec;
        g_el_spec = d.el_spec;
        break;
    }

    // An axis stops knowing where it is when a step goes out uncounted (a
    // step pin driven by hand) or when the drive train changes under it (a
    // gear mode switch). Either way it has to be re-zeroed before it may
    // move again.
    if (d.clear_az_zero) {
        g_az.clear_zero();
    }
    if (d.clear_el_zero) {
        g_el.clear_zero();
    }
}

static void handle(const Command &c)
{
    const Decision d = decide(c, snapshot());

    // A save is the one action whose answer depends on how it went, so it is
    // done before the reply rather than after. It is checked by reading flash
    // back, the same way the next startup will, so the ACK means a restart
    // will come back with these settings.
    if (d.act == Act::SaveSettings) {
        save_settings(g_gear, g_az_spec, g_el_spec);
        read_boot_settings();
        if (g_boot_gear != g_gear || !same_settings(g_boot_az, g_az_spec) ||
            !same_settings(g_boot_el, g_el_spec)) {
            send(format_err(g_out, sizeof(g_out), c.id, ErrCode::Flash, ""));
            return;
        }
    }

    // Lines first, then the action. A move's ACK and PLANs have to be on the
    // wire before its first pulse.
    for (size_t i = 0; i < d.line_count; i++) {
        fputs(d.lines[i], stdout);
    }
    apply(d);
}

// Read whatever has arrived without waiting for more.
static void poll_input()
{
    for (;;) {
        const int ch = getchar_timeout_us(0);
        if (ch < 0) {
            return;
        }

        const LineReader::Result r = g_reader.feed(static_cast<char>(ch));
        if (r == LineReader::Result::Line) {
            handle(parse_command(g_reader.line()));
        } else if (r == LineReader::Result::TooLong) {
            send(format_err(g_out, sizeof(g_out), 0, ErrCode::TooLong, ""));
        }
    }
}

// Called once each time an axis finishes, to report what it did.
static void report_done(Stepper &s, AxisId axis)
{
    send(format_done(g_out, sizeof(g_out), s.move_id(), axis, !s.aborted(),
                     s.steps_done(), s.plan().from_mdeg, s.pos_mdeg(),
                     s.elapsed_us()));
}

int main()
{
    stdio_init_all();
    wait_for_usb_forever();

    printf("=== gs_pico_link %s %s ===\n", GS_TARGET_NAME, GS_VERSION);

    g_az.init();
    g_el.init();

    if (!g_az.pins_ok()) {
        fatal_forever("azimuth pins did not read back idle HIGH");
    }
    if (!g_el.pins_ok()) {
        fatal_forever("elevation pins did not read back idle HIGH");
    }

    // Load the settings before the banner, so the banner shows what is in
    // force.
    if (!settings_sector_is_free()) {
        fatal_forever("the program has grown into the settings sector");
    }
    const LoadResult loaded = read_boot_settings();
    g_gear    = g_boot_gear;
    g_az_spec = g_boot_az;
    g_el_spec = g_boot_el;

    print_banner(loaded);

    absolute_time_t next_heartbeat =
        delayed_by_ms(get_absolute_time(), HEARTBEAT_PERIOD_MS);
    bool was_moving = false;

    for (;;) {
        const absolute_time_t now = get_absolute_time();

        const bool az_moving = g_az.tick(now);
        const bool el_moving = g_el.tick(now);
        const bool moving    = az_moving || el_moving;

        poll_input();

        // An axis that has just finished reports once. Checked after tick()
        // so the last pulse is already out.
        if (was_moving && !moving) {
            if (!g_az.moving() && g_az.move_id() != 0) {
                report_done(g_az, AxisId::Az);
                g_az.clear_move_id();
            }
            if (!g_el.moving() && g_el.move_id() != 0) {
                report_done(g_el, AxisId::El);
                g_el.clear_move_id();
            }
        }
        was_moving = moving;

        // The heartbeat stays quiet while moving - see the rule at the top of
        // this file - and while someone is part way through typing a line,
        // where it would scramble the screen.
        //
        // Either way the deadline is pushed along rather than left in the
        // past, so the heartbeat resumes a full period after things go quiet
        // instead of firing the instant a move ends or a line is finished.
        if (moving || g_reader.partial()) {
            next_heartbeat = delayed_by_ms(now, HEARTBEAT_PERIOD_MS);
        } else if (absolute_time_diff_us(now, next_heartbeat) <= 0) {
            next_heartbeat = delayed_by_ms(now, HEARTBEAT_PERIOD_MS);
            send(format_mstat(g_out, sizeof(g_out),
                              to_ms_since_boot(now) / 1000, g_az.pos_mdeg(),
                              g_el.pos_mdeg(), false));
        }
    }
}
