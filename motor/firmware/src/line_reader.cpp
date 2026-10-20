#include "line_reader.hpp"

// Start the next line from empty.
void LineReader::reset()
{
    len_      = 0;
    overflow_ = false;
}

LineReader::Result LineReader::feed(char c)
{
    // Either newline character ends a line. Handling both means Windows
    // (\r\n), Unix (\n) and old Mac (\r) senders all work without the caller
    // caring. With \r\n the \r finishes the line and the \n then arrives on
    // an empty buffer, which is ignored just below.
    if (c == '\n' || c == '\r') {
        if (overflow_) {
            reset();
            return Result::TooLong;
        }
        if (len_ == 0) {
            return Result::None;  // blank line, or the \n of a \r\n pair
        }
        buf_[len_] = '\0';
        len_       = 0;  // ready for the next line; buf_ still holds this one
        return Result::Line;
    }

    // Already past the limit: swallow the rest of this line in silence. The
    // report happens once, when the newline finally arrives.
    if (overflow_) {
        return Result::None;
    }

    if (len_ >= MAX_LINE_LEN) {
        overflow_ = true;
        return Result::None;
    }

    buf_[len_++] = c;
    return Result::None;
}
