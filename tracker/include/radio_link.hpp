#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

#include "pico/stdlib.h"
#include "position_packet.hpp"

enum class RadioMode
{
    Transmitter,
    Receiver
};

// Configure the RF69, including the guarded Pico SPI hardware layer.
// Receiver mode also starts listening. Returns 0 on success or a RadioLib error.
// Call once at startup, before polling reception or starting transmissions.
int initializeRadio(RadioMode mode);

struct RadioTransmitState
{
    bool busy = false;
    bool faulted = false; // Stop sending if cleanup cannot restore standby.
    absolute_time_t deadline{};
    std::uint32_t sequence = 0;
};

enum class RadioTransmitStatus
{
    Idle,
    Started,
    EncodeRejected,
    StartFailed,
    Sent,
    TimedOut
};

// Diagnostics for the application to report; the radio module never prints.
struct RadioTransmitResult
{
    RadioTransmitStatus status = RadioTransmitStatus::Idle;
    std::uint32_t sequence = 0;
    int radioState = 0;
    int cleanupState = 0;
    std::uint8_t irq2 = 0;
    bool dio0 = false;
};

// Encode and start one transmission without waiting for completion.
// Returns Idle without touching the radio if tx is already busy or faulted.
RadioTransmitResult startPositionTransmit(
    const PositionPacket &packet,
    RadioTransmitState &tx);

// Check completion or timeout once. Sent means local completion, not an ACK.
// Cleanup failure sets tx.faulted and prevents further transmission starts.
RadioTransmitResult checkTransmit(RadioTransmitState &tx);

enum class RadioReceiveStatus
{
    Idle,
    Packet,
    ReadRejected,
    DecodeRejected,
    Fault
};

struct RadioReceiveResult
{
    RadioReceiveStatus status = RadioReceiveStatus::Idle;
    std::optional<PositionPacket> packet;
    int radioState = 0;
    std::size_t length = 0;
    std::uint8_t irq2 = 0;
    bool dio0 = false;
};

// Process at most one packet without waiting for one or printing diagnostics.
// Packet extraction and receive restart still use synchronous SPI calls.
// A Fault means the caller must stop polling until reception is reinitialized.
RadioReceiveResult pollReceiver();
