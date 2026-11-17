#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

#include "pico/stdlib.h"
#include "position_packet.hpp"

/** @brief Selects whether initialization prepares transmission or starts reception. */
enum class RadioMode
{
    Transmitter,
    Receiver
};

/**
 * @brief Configures the RF69 and guarded Pico SPI hardware for the selected role.
 *
 * @param mode Receiver starts listening; Transmitter leaves the radio ready to send.
 * @return Zero on success, otherwise a RadioLib error code.
 * @note Call at startup before servicing reception or starting a transmission.
 */
int initializeRadio(RadioMode mode);

/**
 * @brief Retains an in-flight transmission's state between application iterations.
 *
 * @note Reuse the same instance for the shared radio. Cleanup failure latches
 *       faulted and prevents subsequent starts through this state.
 */
struct RadioTransmitState
{
    /// True while a send is awaiting completion or timeout; does not block the CPU.
    bool busy = false;
    /// True when cleanup could not restore standby.
    bool faulted = false;
    /// Absolute transmit-completion deadline on this Pico's clock.
    absolute_time_t deadline{};
    /// Sequence of the packet in flight, independent of newer GPS fixes.
    std::uint32_t sequence = 0;
};

/**
 * @brief Describes the outcome of one transmit start or completion check.
 * @note Idle means no new event; a transmission may still be busy.
 */
enum class RadioTransmitStatus
{
    Idle,
    Started,
    EncodeRejected,
    StartFailed,
    Sent,
    TimedOut
};

/**
 * @brief Reports a transmit event and diagnostics for the application to print.
 *
 * @note Sent confirms local completion, not a receiver acknowledgment.
 *       Check cleanupState or tx.faulted even after a Sent result.
 */
struct RadioTransmitResult
{
    RadioTransmitStatus status = RadioTransmitStatus::Idle;
    /// Packet sequence associated with the event; not meaningful for Idle.
    std::uint32_t sequence = 0;
    /// Driver status from startTransmit(), when a start was attempted.
    int radioState = 0;
    /// Driver status from finishTransmit(), when cleanup was attempted.
    int cleanupState = 0;
    /// IRQ2 snapshot from a completion check, before cleanup clears flags.
    std::uint8_t irq2 = 0;
    /// DIO0 level sampled before completion cleanup.
    bool dio0 = false;
};

/**
 * @brief Encodes and queues one position without waiting for its radio airtime.
 *
 * @param packet Position and metadata to encode and copy into the RF69 FIFO.
 * @param tx Persistent state used to prevent overlapping sends.
 * @return The start outcome; Idle if tx is already busy or faulted.
 * @note SPI work is synchronous. The radio owns the copied bytes after return;
 *       call checkTransmit() on later iterations to finish or time out the send.
 */
RadioTransmitResult startPositionTransmit(
    const PositionPacket &packet,
    RadioTransmitState &tx);

/**
 * @brief Checks packet-sent status and the transmit deadline once, then returns.
 *
 * @param tx State of the transmission being serviced.
 * @return Idle while waiting, or Sent/TimedOut after attempting cleanup.
 * @note Completion or the configured timeout clears busy. Failed cleanup sets faulted.
 */
RadioTransmitResult checkTransmit(RadioTransmitState &tx);

/**
 * @brief Describes one receive-service result.
 * @note Packet passes transport/codec checks; ordering and motor eligibility
 *       remain the application's responsibility.
 */
enum class RadioReceiveStatus
{
    Idle,
    Packet,
    ReadRejected,
    DecodeRejected,
    Fault
};

/**
 * @brief Stores a decoded position or receive error with hardware diagnostics.
 *
 * @note packet has a value only for Packet status. Fault means receive restart
 *       failed; stop polling until reception has been reinitialized.
 */
struct RadioReceiveResult
{
    RadioReceiveStatus status = RadioReceiveStatus::Idle;
    std::optional<PositionPacket> packet;
    /// Driver read/restart status; length or decoding rejection may still be zero.
    int radioState = 0;
    /// Received payload length reported by the radio, before application decoding.
    std::size_t length = 0;
    /// IRQ2 snapshot used for the packet-ready check.
    std::uint8_t irq2 = 0;
    /// DIO0 level sampled when a ready packet was observed.
    bool dio0 = false;
};

/**
 * @brief Services at most one already-ready packet without printing diagnostics.
 *
 * @return Idle if no packet is ready, otherwise a packet, rejection, or fault.
 * @note Initialize in Receiver mode first. FIFO reads and receive restart are
 *       synchronous; this function does not wait for a future packet to arrive.
 */
RadioReceiveResult pollReceiver();
