#pragma once

#include <optional>
#include "gps_position.hpp"

/**
 * @brief Defines the common radio wiring using Pico GPIO numbers.
 * @note The HAL and its chip-select timing guard use SPI0 in radio_link.cpp.
 */
inline constexpr unsigned RADIO_MISO_PIN = 16;  ///< Physical header pin 21.
inline constexpr unsigned RADIO_MOSI_PIN = 19;  ///< Physical header pin 25.
inline constexpr unsigned RADIO_SCK_PIN = 18;   ///< Physical header pin 24.
inline constexpr unsigned RADIO_CS_PIN = 17;    ///< Physical header pin 22.
inline constexpr unsigned RADIO_DIO0_PIN = 20;  ///< Physical header pin 26.
inline constexpr unsigned RADIO_RESET_PIN = 21; ///< Physical header pin 27.

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
    RadioPositionSource::Replay;

/**
 * @brief Defines the ground-station reference used only in live GPS mode.
 *
 * @note Latitude/longitude are decimal degrees; altitude is WGS84 ellipsoidal
 *       meters. std::nullopt refuses radio arming in GPS mode. Replay uses
 *       REPLAY_STATION from replay_config.hpp instead.
 */
inline constexpr std::optional<GPSPosition> RADIO_LIVE_STATION =
    std::nullopt;
