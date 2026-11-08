#include <cstdint>
#include <cstdio>
#include <cstring>

#include "hardware/uart.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"

#include "gps_parser.hpp"
#include "position_packet.hpp"
#include "radio_link.hpp"
#include "sentence_collector.hpp"
#include "radio_tracking_config.hpp"
#include "line_reader.hpp"
#include "replay_config.hpp"
#include "replay_position.hpp"

// ---------------------------------------------------------------------------
// Application configuration and timing
// ---------------------------------------------------------------------------

namespace
{
    constexpr unsigned TRANSMIT_PERIOD_MS = 200;

    constexpr unsigned GPS_BAUD = 115200;
    constexpr unsigned GPS_RX_PIN = 1; // Physical header pin 2.
}

// ---------------------------------------------------------------------------
// Startup and peripheral initialization
// ---------------------------------------------------------------------------

void initializeConsole()
{
    stdio_init_all();

    while (!stdio_usb_connected())
        sleep_ms(10);

    std::printf("=== RFM69HCW initialization test ===\n");
}

void initializeGps()
{
    // Receive GPS TX on GP1. No GPS RX connection is needed.
    uart_init(uart0, GPS_BAUD);
    uart_set_format(uart0, 8, 1, UART_PARITY_NONE);
    uart_set_hw_flow(uart0, false, false);
    gpio_set_function(GPS_RX_PIN, GPIO_FUNC_UART);
}

// ---------------------------------------------------------------------------
// GPS byte collection and position parsing returns 0 if successful 1 if not.
// ---------------------------------------------------------------------------

int updatePositionFromGps(SentenceCollector &collector, GPSPosition &position)
{
    int status = 1;

    while (uart_is_readable(uart0))
    {
        const char byte = uart_getc(uart0);
        const auto sentence = collector.feed(byte);

        if (!sentence)
            continue;

        // Consume the view before feeding another byte into its buffer.
        const auto parsedPosition = parseGGA(*sentence);

        if (parsedPosition)
        {
            position = *parsedPosition;
            status = 0;
        }
    }
    return status;
}

// ---------------------------------------------------------------------------
// Packet transmission diagnostics and scheduling
// ---------------------------------------------------------------------------

void reportTransmit(const RadioTransmitResult &result)
{
    switch (result.status)
    {
        case RadioTransmitStatus::Idle:
        case RadioTransmitStatus::Started:
            break;
        case RadioTransmitStatus::EncodeRejected:
            std::printf("TX encoding failed\n");
            break;
        case RadioTransmitStatus::StartFailed:
            std::printf("TX seq=%lu start=%d cleanup=%d\n",
                        static_cast<unsigned long>(result.sequence),
                        result.radioState,
                        result.cleanupState);
            break;
        case RadioTransmitStatus::Sent:
        case RadioTransmitStatus::TimedOut:
            std::printf("TX seq=%lu start=0 sent=%d G0=%d IRQ2=0x%02X\n",
                        static_cast<unsigned long>(result.sequence),
                        result.status == RadioTransmitStatus::Sent ? 1 : 0,
                        result.dio0 ? 1 : 0,
                        static_cast<unsigned>(result.irq2));
            if (result.cleanupState != 0)
                std::printf("TX cleanup failed, code %d; transmission stopped\n",
                            result.cleanupState);
            break;
    }
}

void runTransmitter(SentenceCollector collector)
{
    // Hold the initial pose until the operator starts the simulation clock.
    enum class ReplayPhase { Hold, Running, Finished };

    ReplayPhase replay_phase = ReplayPhase::Hold;
    absolute_time_t replay_start{};
    LineReader replay_input;
    bool replay_primed = false;

    if constexpr (RADIO_POSITION_SOURCE == RadioPositionSource::Replay)
    {
        if (!REPLAY_GEOID_SEPARATION_M)
        {
            std::printf("Replay needs a geoid separation\n");
            return;
        }
        std::printf("Replay holding at t=0. START runs; STOP ends.\n");
    }

    std::uint32_t sequence = 0;
    PositionPacket latestPacket{};
    bool newFixAvailable = false;
    auto nextTransmit = get_absolute_time();
    RadioTransmitState tx;

    while (true)
    {
        if constexpr (RADIO_POSITION_SOURCE == RadioPositionSource::Replay)
        {
            // Limit console work so every iteration still services the radio.
            for (unsigned i = 0; i < 32; ++i)
            {
                const int ch = getchar_timeout_us(0);
                if (ch < 0)
                    break;

                const auto input = replay_input.feed(static_cast<char>(ch));
                if (input == LineReader::Result::Line)
                {
                    if (std::strcmp(replay_input.line(), "START") == 0 &&
                        replay_phase == ReplayPhase::Hold)
                    {
                        if (!replay_primed)
                        {
                            std::printf("Waiting for first held transmission\n");
                        }
                        else
                        {
                            replay_start = get_absolute_time();
                            replay_phase = ReplayPhase::Running;
                            std::printf("Replay started\n");
                        }
                    }
                    else if (std::strcmp(replay_input.line(), "STOP") == 0)
                    {
                        replay_phase = ReplayPhase::Finished;
                        std::printf("Replay stopped; no new packets\n");
                    }
                    else
                    {
                        std::printf("Use START or STOP; reboot to repeat\n");
                    }
                }
                else if (input == LineReader::Result::TooLong)
                {
                    std::printf("Replay command too long\n");
                }
            }
        }
        else if (updatePositionFromGps(collector, latestPacket.position) == 0)
        {
            latestPacket.flags = POSITION_FLAG_VALID;
            latestPacket.timestamp =
                to_ms_since_boot(get_absolute_time());
            newFixAvailable = true;
        }

        // Service the in-flight send while GPS/console work continues each loop.
        const auto tx_result = checkTransmit(tx);
        reportTransmit(tx_result);
        if (replay_phase == ReplayPhase::Hold &&
            tx_result.status == RadioTransmitStatus::Sent)
            replay_primed = true;

        if (tx.faulted)
            return;

        // Keep sends separate and leave at least 200 ms between their starts.
        if (!tx.busy && time_reached(nextTransmit))
        {
            if constexpr (RADIO_POSITION_SOURCE == RadioPositionSource::Replay)
            {
                if (replay_phase == ReplayPhase::Finished)
                    continue;

                // Sample when ready to send; hold mode always samples t=0.
                const auto sampled_at = get_absolute_time();
                const double elapsed_seconds =
                    replay_phase == ReplayPhase::Hold ? 0.0 :
                    absolute_time_diff_us(replay_start, sampled_at) / 1'000'000.0;

                const auto position = replayPositionAt(
                    elapsed_seconds, *REPLAY_GEOID_SEPARATION_M);

                if (!position)
                {
                    replay_phase = ReplayPhase::Finished;
                    std::printf("Replay finished; no new packets\n");
                    continue;
                }

                latestPacket.position = *position;
                latestPacket.flags = POSITION_FLAG_VALID;
                // Packet time is uptime; simulation time stays separate.
                latestPacket.timestamp = to_ms_since_boot(sampled_at);
                newFixAvailable = true;
            }

            if (newFixAvailable)
            {
                latestPacket.sequence = sequence++;
                newFixAvailable = false;
                nextTransmit = delayed_by_ms(
                    get_absolute_time(), TRANSMIT_PERIOD_MS);
                reportTransmit(startPositionTransmit(latestPacket, tx));
            }
        }

        tight_loop_contents();
    }
}

// ---------------------------------------------------------------------------
// Packet reception diagnostics
// ---------------------------------------------------------------------------

void runReceiver()
{
    // initializeRadio(Receiver) has already configured and started reception.
    std::printf("RX listening\n");

    while (true)
    {
        const auto result = pollReceiver();
        if (result.status == RadioReceiveStatus::Idle)
        {
            tight_loop_contents();
            continue;
        }

        std::printf("RX ready: G0=%d IRQ2=0x%02X\n",
                    result.dio0 ? 1 : 0,
                    static_cast<unsigned>(result.irq2));

        switch (result.status)
        {
            case RadioReceiveStatus::Idle:
                break;
            case RadioReceiveStatus::Packet:
                std::printf("RX seq=%lu lat=%.7f lon=%.7f alt=%.2f m\n",
                            static_cast<unsigned long>(result.packet->sequence),
                            result.packet->position.latitude,
                            result.packet->position.longitude,
                            result.packet->position.altitude);
                break;
            case RadioReceiveStatus::ReadRejected:
                std::printf("RX rejected: status=%d length=%lu\n",
                            result.radioState,
                            static_cast<unsigned long>(result.length));
                break;
            case RadioReceiveStatus::DecodeRejected:
                std::printf("RX invalid position packet\n");
                break;
            case RadioReceiveStatus::Fault:
                std::printf("RX restart failed, code %d\n", result.radioState);
                return;
        }
    }
}

// ---------------------------------------------------------------------------
// Application entry point
// ---------------------------------------------------------------------------

int main()
{
    initializeConsole();

    // Initialize the radio once; the application loops service it afterward.
#ifdef RADIO_LINK_TRANSMITTER
    const int radioState = initializeRadio(RadioMode::Transmitter);
#else
    const int radioState = initializeRadio(RadioMode::Receiver);
#endif

    if (radioState != 0)
    {
        std::printf("[RF69] initialization failed, code %d\n", radioState);
    }
    else
    {
        std::printf("[RF69] ready\n");
#ifdef RADIO_LINK_TRANSMITTER
        // Replay uses the compiled table; only live GPS needs the UART.
        if constexpr (RADIO_POSITION_SOURCE == RadioPositionSource::Gps)
            initializeGps();
        SentenceCollector collector;
        runTransmitter(collector);
#else
        runReceiver();
#endif
    }

    // Keep USB available to read the result. This test remains idle.
    while (true)
    {
        sleep_ms(1000);
    }
}
