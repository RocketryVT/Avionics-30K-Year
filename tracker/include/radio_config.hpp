#pragma once

#include <optional>
#include "gps_position.hpp"

/**
 * @brief Defines each board's radio wiring using Pico GPIO numbers.
 * @note The transmitter build defines RADIO_LINK_TRANSMITTER; the motor build
 *       uses the receiver wiring. RADIO_SPI_BUS is 0 for SPI0 or 1 for SPI1,
 *       and controls both the HAL and its chip-select timing guard.
 */
#ifdef RADIO_LINK_TRANSMITTER
inline constexpr unsigned RADIO_SPI_BUS = 0;
inline constexpr unsigned RADIO_MISO_PIN = 16;  ///< Physical header pin 21.
inline constexpr unsigned RADIO_MOSI_PIN = 19;  ///< Physical header pin 25.
inline constexpr unsigned RADIO_SCK_PIN = 18;   ///< Physical header pin 24.
inline constexpr unsigned RADIO_CS_PIN = 17;    ///< Physical header pin 22.
inline constexpr unsigned RADIO_DIO0_PIN = 20;  ///< Physical header pin 26.
inline constexpr unsigned RADIO_RESET_PIN = 21; ///< Physical header pin 27.
inline constexpr std::optional<unsigned> RADIO_ENABLE_PIN =
    std::nullopt; ///< No enable connection on the transmitter.
#else
inline constexpr unsigned RADIO_SPI_BUS = 1;
inline constexpr unsigned RADIO_MISO_PIN = 8;   ///< Physical header pin 11.
inline constexpr unsigned RADIO_MOSI_PIN = 11;  ///< Physical header pin 15.
inline constexpr unsigned RADIO_SCK_PIN = 10;   ///< Physical header pin 14.
inline constexpr unsigned RADIO_CS_PIN = 9;     ///< Physical header pin 12.
inline constexpr unsigned RADIO_DIO0_PIN = 27;  ///< Physical header pin 32.
inline constexpr unsigned RADIO_RESET_PIN = 26; ///< Physical header pin 31.
inline constexpr std::optional<unsigned> RADIO_ENABLE_PIN =
    28; ///< Physical header pin 34; HIGH enables radio power.
#endif

static_assert(RADIO_SPI_BUS <= 1, "RADIO_SPI_BUS must be 0 or 1");

/** @brief Allows radio power to settle after asserting a configured enable pin. */
inline constexpr unsigned RADIO_ENABLE_SETTLE_MS = 10;

/** @brief Sets the RF69 carrier in MHz, as required by RadioLib 7.8.1. */
inline constexpr float RADIO_FREQUENCY_MHZ = 915.0f;
/** @brief Sets transmit output power in dBm on the configured HCW power path. */
inline constexpr int RADIO_POWER_DBM = 10;

/** @brief Minimum interval between transmission starts, in milliseconds. */
inline constexpr unsigned TRANSMIT_PERIOD_MS = 200;
/** @brief Deadline for local transmit completion, in milliseconds. */
inline constexpr unsigned TRANSMIT_TIMEOUT_MS = 250;

/** @brief Selects live GPS or the existing simulated trajectory as the radio source. */
enum class RadioPositionSource
{
    Gps,
    Replay
};

/**
 * @brief Selects the transmitter source and matching ground-station reference.
 * @note This is a compile-time choice. Rebuild and flash both boards together.
 */
inline constexpr RadioPositionSource RADIO_POSITION_SOURCE =
    RadioPositionSource::Gps;

/**
 * @brief Defines the ground-station reference used only in live GPS mode.
 *
 * @note Latitude/longitude are decimal degrees; altitude is WGS84 ellipsoidal
 *       meters. std::nullopt refuses radio arming in GPS mode. Replay uses
 *       REPLAY_STATION from replay_config.hpp instead.
 */
inline constexpr std::optional<GPSPosition> RADIO_LIVE_STATION =
    GPSPosition{
        37.1950303431,
        -80.3944877669,
        648.0 - 32.815 // MSL height plus estimated EGM96 geoid separation.
};
