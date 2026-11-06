#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <string_view>

/**
 * @brief Collects incoming GPS bytes into newline-terminated NMEA sentences.
 *
 * @note Keep the same instance alive between reads so partial sentences persist.
 *       Storage is fixed at 256 characters, including '$' and any CR, excluding
 *       the terminating LF. No dynamic allocation or checksum validation occurs.
 */
class SentenceCollector
{
public:
    /**
     * @brief Processes one byte and reports a completed sentence when LF arrives.
     *
     * @param byte Next received character, including any framing or noise bytes.
     * @return A view of the completed sentence without LF, or std::nullopt while
     *         waiting for '$', collecting characters, or discarding overflow.
     *
     * @note A new '$' replaces any incomplete sentence. After overflow, bytes
     *       are ignored until another '$'. A CR before LF remains in the view.
     * @warning The view refers to this object's reusable buffer. Consume it
     *          before the next feed() call and before destroying the collector.
     *          The returned text is not guaranteed to be null-terminated.
     */
    std::optional<std::string_view> feed(char byte);

private:
    /// Characters collected for the current sentence.
    std::array<char, 256> buffer{};
    /// Number of occupied buffer positions.
    std::size_t length = 0;
    /// Whether a '$' has started a sentence that is still being collected.
    bool collecting = false;
};
