#include "sentence_collector.hpp"

std::optional<std::string_view> SentenceCollector::feed(char byte)
{
    // Start a new sentence, replacing any incomplete one.
    if (byte == '$')
    {
        length = 0;
        collecting = true;
    }

    // Ignore noise before a sentence starts.
    if (!collecting)
        return std::nullopt;

    // Return the completed sentence without the newline.
    if (byte == '\n')
    {
        collecting = false;

        const std::string_view sentence(buffer.data(), length);
        length = 0;

        return sentence;
    }

    // Discard a sentence that exceeds the buffer capacity.
    if (length >= buffer.size())
    {
        length = 0;
        collecting = false;
        return std::nullopt;
    }

    // Store this byte and wait for more.
    buffer[length] = byte;
    ++length;

    return std::nullopt;
}