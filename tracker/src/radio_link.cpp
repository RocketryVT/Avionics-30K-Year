#include "radio_link.hpp"
#include "radio_config.hpp"

#include <RadioLib.h>

#include "hal/RPiPico/PicoHal.h"

namespace
{
    spi_inst_t *const radioSpi = spi_get_instance(RADIO_SPI_BUS);

    class RadioPicoHal : public PicoHal
    {
    public:
        using PicoHal::PicoHal;

        void digitalWrite(std::uint32_t pin,
                          std::uint32_t value) override
        {
            if (pin == RADIO_CS_PIN && value == GpioLevelHigh)
            {
                // Finish clocking before releasing chip select.
                while (spi_is_busy(radioSpi))
                    tight_loop_contents();

                sleep_us(2);
            }

            PicoHal::digitalWrite(pin, value);

            // Allow chip-select setup and recovery time.
            if (pin == RADIO_CS_PIN)
                sleep_us(2);
        }
    };

    // Constructor order: SPI bus, MISO, MOSI, SCK.
    RadioPicoHal hal(radioSpi, RADIO_MISO_PIN, RADIO_MOSI_PIN, RADIO_SCK_PIN);

    // Constructor order: hardware layer, CS, DIO0, RESET.
    Module module(&hal, RADIO_CS_PIN, RADIO_DIO0_PIN, RADIO_RESET_PIN);

    RF69 radio(&module);
}

int initializeRadio(RadioMode mode)
{
    if constexpr (RADIO_ENABLE_PIN.has_value())
    {
        // Enable radio power before attempting SPI initialization.
        const unsigned enablePin = *RADIO_ENABLE_PIN;
        gpio_init(enablePin);
        gpio_put(enablePin, 1);
        gpio_set_dir(enablePin, GPIO_OUT);
        sleep_ms(RADIO_ENABLE_SETTLE_MS);
    }

    ConfigFSK_t config;
    config.frequency = RADIO_FREQUENCY_MHZ;

    int state = radio.begin(config);
    if (state != RADIOLIB_ERR_NONE)
        return state;

    // Select the HCW module's high-power output path.
    state = radio.setOutputPower(RADIO_POWER_DBM, true);
    if (state != RADIOLIB_ERR_NONE)
        return state;

    if (mode == RadioMode::Receiver)
        return radio.startReceive();

    return RADIOLIB_ERR_NONE;
}

RadioTransmitResult startPositionTransmit(
    const PositionPacket &packet,
    RadioTransmitState &tx)
{
    RadioTransmitResult result;
    if (tx.busy || tx.faulted)
        return result;

    result.sequence = packet.sequence;
    const auto bytes = encodePositionPacket(packet);
    if (!bytes)
    {
        result.status = RadioTransmitStatus::EncodeRejected;
        return result;
    }

    result.radioState = radio.startTransmit(bytes->data(), bytes->size());
    if (result.radioState != RADIOLIB_ERR_NONE)
    {
        // A failed start may have partially configured the radio.
        result.cleanupState = radio.finishTransmit();
        tx.faulted = result.cleanupState != RADIOLIB_ERR_NONE;
        result.status = RadioTransmitStatus::StartFailed;
        return result;
    }

    // RF69 has copied the encoded bytes into its FIFO before returning.
    tx.sequence = packet.sequence;
    tx.deadline = make_timeout_time_ms(TRANSMIT_TIMEOUT_MS);
    tx.busy = true;
    result.status = RadioTransmitStatus::Started;
    return result;
}

RadioTransmitResult checkTransmit(RadioTransmitState &tx)
{
    RadioTransmitResult result;
    if (!tx.busy)
        return result;

    result.irq2 = module.SPIreadRegister(RADIOLIB_RF69_REG_IRQ_FLAGS_2);
    const bool sent = (result.irq2 & RADIOLIB_RF69_IRQ_PACKET_SENT) != 0;

    // Check once per loop, leaving time to collect GPS data.
    if (!sent && !time_reached(tx.deadline))
        return result;

    // Capture these before finishTransmit clears the flags.
    result.sequence = tx.sequence;
    result.dio0 = gpio_get(RADIO_DIO0_PIN);
    result.cleanupState = radio.finishTransmit();
    tx.busy = false;
    tx.faulted = result.cleanupState != RADIOLIB_ERR_NONE;
    result.status = sent ? RadioTransmitStatus::Sent
                         : RadioTransmitStatus::TimedOut;
    return result;
}

RadioReceiveResult pollReceiver()
{
    RadioReceiveResult result;

    // Preserve the working radio's internal packet-ready check.
    result.irq2 = module.SPIreadRegister(RADIOLIB_RF69_REG_IRQ_FLAGS_2);
    if ((result.irq2 & RADIOLIB_RF69_IRQ_PAYLOAD_READY) == 0)
        return result;

    // Capture diagnostics before reading/restarting clears the radio flags.
    result.dio0 = gpio_get(RADIO_DIO0_PIN);

    PositionPacketBytes bytes{};
    result.length = radio.getPacketLength();
    const int readState = radio.readData(bytes.data(), bytes.size());

    // Reading leaves the radio in standby; resume listening on every path.
    // A restart failure takes precedence: do not return a usable packet.
    result.radioState = radio.startReceive();
    if (result.radioState != RADIOLIB_ERR_NONE)
    {
        result.status = RadioReceiveStatus::Fault;
        return result;
    }

    result.radioState = readState;
    if (readState != RADIOLIB_ERR_NONE || result.length != bytes.size())
    {
        result.status = RadioReceiveStatus::ReadRejected;
        return result;
    }

    result.packet = decodePositionPacket(bytes.data(), bytes.size());
    result.status = result.packet ? RadioReceiveStatus::Packet
                                  : RadioReceiveStatus::DecodeRejected;
    return result;
}
