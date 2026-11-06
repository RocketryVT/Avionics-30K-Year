// Unit tests for GPS sentence framing and its connection to the GGA parser.
// No third-party C++ test framework is required.

#include "test_helpers.hpp"
#include "sentence_collector.hpp"
#include "gps_parser.hpp"

using test::expectEquals;

#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

namespace
{
    void expectValidSentence(
        const std::string &name,
        SentenceCollector &collector,
        const char byte,
        std::string_view expected)
    {
        const auto result = collector.feed(byte);
        test::expectTrue(name + " returns a sentence", result.has_value());

        if (result)
        {
            expectEquals(name, *result, expected);
        }
    }

    void expectInvalidSentence(
        const std::string &name,
        SentenceCollector &collector,
        const char byte)
    {
        const auto result = collector.feed(byte);
        test::expectTrue(name + " returns nullopt", !result.has_value());
    }

    constexpr std::string_view CAPTURED_SENTENCE =
        "$GNGGA,152653.50,3713.41001,N,08026.53136,W,2,12,0.62,648.3,M,-33.4,M,,*7B";
    constexpr std::string_view STANDARD_SENTENCE =
        "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";

    // Feed a whole chunk while checking that none of its bytes completes a line.
    void expectIncompleteSentence(
        const std::string &name,
        SentenceCollector &collector,
        std::string_view bytes)
    {
        bool returnedSentence = false;

        for (char byte : bytes)
        {
            if (collector.feed(byte))
                returnedSentence = true;
        }

        test::expectTrue(name + " produces no completed sentence", !returnedSentence);
    }

    // Copy each result before feeding another byte: returned views share the buffer.
    std::vector<std::string> collectSentences(
        SentenceCollector &collector,
        std::string_view bytes)
    {
        std::vector<std::string> sentences;

        for (char byte : bytes)
        {
            const auto result = collector.feed(byte);

            if (result)
                sentences.emplace_back(*result);
        }

        return sentences;
    }

    void testSentenceBoundaries()
    {
        SentenceCollector collector;

        expectInvalidSentence("Newline before a start marker", collector, '\n');
        expectInvalidSentence("Carriage return before a start marker", collector, '\r');
        expectIncompleteSentence("Complete text without a newline", collector, CAPTURED_SENTENCE);
        expectValidSentence("LF completes the sentence", collector, '\n', CAPTURED_SENTENCE);
        expectInvalidSentence("A second newline does not repeat the sentence", collector, '\n');
        expectIncompleteSentence("Text after completion needs a new start marker", collector, "tail\r\n");

        // The collector removes LF. The existing parser accepts the remaining CR.
        const std::string withCarriageReturn = std::string(CAPTURED_SENTENCE) + '\r';
        expectIncompleteSentence("CR does not finish the sentence", collector, withCarriageReturn);
        expectValidSentence("CRLF preserves CR in the returned text", collector, '\n', withCarriageReturn);
    }

    void testChunkedInput()
    {
        SentenceCollector collector;

        // These calls represent separate passes through the UART polling loop.
        expectIncompleteSentence("First UART chunk", collector, CAPTURED_SENTENCE.substr(0, 12));
        expectIncompleteSentence("Second UART chunk", collector, CAPTURED_SENTENCE.substr(12, 25));
        expectIncompleteSentence("Final UART chunk", collector, CAPTURED_SENTENCE.substr(37));
        expectValidSentence("Chunks assemble into one sentence", collector, '\n', CAPTURED_SENTENCE);
    }

    void testNoiseAndRestart()
    {
        SentenceCollector collector;

        // Explicit length includes NUL and non-ASCII bytes from a mixed stream.
        const char noise[] = {
            '\0', static_cast<char>(0xB5), 'b', static_cast<char>(0xFF), '\r', '\n', 'x'};
        expectIncompleteSentence("Binary noise before a sentence", collector,
                                 std::string_view(noise, sizeof(noise)));
        expectIncompleteSentence("Sentence after binary noise", collector, CAPTURED_SENTENCE);
        expectValidSentence("Noise is excluded from the sentence", collector, '\n', CAPTURED_SENTENCE);

        expectIncompleteSentence("Truncated old sentence", collector, "$GNGGA,incomplete");
        expectIncompleteSentence("New start marker replaces old text", collector, STANDARD_SENTENCE);
        expectValidSentence("Restarted sentence contains only new text", collector, '\n', STANDARD_SENTENCE);

        expectIncompleteSentence("Repeated start markers", collector, "$$$$");
        expectIncompleteSentence("Text following the final start marker", collector, CAPTURED_SENTENCE.substr(1));
        expectValidSentence("Only the final start marker is retained", collector, '\n', CAPTURED_SENTENCE);
    }

    void testConsecutiveSentences()
    {
        SentenceCollector collector;
        const std::string stream = std::string(CAPTURED_SENTENCE) + '\n' +
                                   std::string(STANDARD_SENTENCE) + '\n';
        const auto sentences = collectSentences(collector, stream);

        expectEquals("Back-to-back sentence count", sentences.size(), std::size_t{2});

        if (sentences.size() == 2)
        {
            expectEquals("First sentence stays in order", sentences[0], CAPTURED_SENTENCE);
            expectEquals("Second sentence stays in order", sentences[1], STANDARD_SENTENCE);
        }
    }

    void testIndependentCollectors()
    {
        SentenceCollector first;
        SentenceCollector second;

        expectIncompleteSentence("First collector starts a partial sentence", first, CAPTURED_SENTENCE.substr(0, 20));
        expectIncompleteSentence("Second collector receives another sentence", second, STANDARD_SENTENCE);
        expectIncompleteSentence("First collector resumes its own sentence", first, CAPTURED_SENTENCE.substr(20));
        expectValidSentence("First collector keeps its state", first, '\n', CAPTURED_SENTENCE);
        expectValidSentence("Second collector keeps its state", second, '\n', STANDARD_SENTENCE);
    }

    void testBufferCapacityAndRecovery()
    {
        // Current public behavior: up to 256 stored characters, including '$'.
        // These synthetic payloads exercise framing, not GGA field validation.
        constexpr std::size_t BUFFER_CAPACITY = 256;
        const std::string atCapacity = "$" + std::string(BUFFER_CAPACITY - 1, 'x');
        SentenceCollector collector;

        expectIncompleteSentence("A sentence exactly filling the buffer", collector, atCapacity);
        expectValidSentence("Newline can finish a full buffer", collector, '\n', atCapacity);
        expectIncompleteSentence("A new sentence after a full buffer", collector, CAPTURED_SENTENCE);
        expectValidSentence("Full-buffer completion resets the length", collector, '\n', CAPTURED_SENTENCE);

        for (std::size_t extra : {std::size_t{1}, std::size_t{100}})
        {
            const std::string name = "Overflow by " + std::to_string(extra) + " bytes";
            const std::string oversized = "$" + std::string(BUFFER_CAPACITY - 1 + extra, 'x');

            expectIncompleteSentence(name, collector, oversized);
            expectInvalidSentence(name + " discards the line", collector, '\n');
            expectIncompleteSentence(name + " ignores trailing noise", collector, "tail\n");
            expectIncompleteSentence(name + " accepts the next start marker", collector, CAPTURED_SENTENCE);
            expectValidSentence(name + " recovers for the next sentence", collector, '\n', CAPTURED_SENTENCE);
        }

        expectIncompleteSentence("Full buffer awaiting a newline", collector, atCapacity);
        expectIncompleteSentence("Start marker can restart a full buffer", collector, CAPTURED_SENTENCE);
        expectValidSentence("Restarting a full buffer keeps the new sentence", collector, '\n', CAPTURED_SENTENCE);

        expectIncompleteSentence("Overflow without a terminating newline", collector, atCapacity + "x");
        expectIncompleteSentence("Restart after overflow without newline", collector, STANDARD_SENTENCE);
        expectValidSentence("A start marker alone restores synchronization", collector, '\n', STANDARD_SENTENCE);
    }

    void testCollectedGGA()
    {
        SentenceCollector collector;
        const std::string stream = "noise\r\n" + std::string(CAPTURED_SENTENCE) + "\r\n";
        std::size_t sentenceCount = 0;
        std::optional<GPSPosition> position;

        for (char byte : stream)
        {
            const auto sentence = collector.feed(byte);

            if (sentence)
            {
                ++sentenceCount;
                position = parseGGA(*sentence);
            }
        }

        expectEquals("Captured UART stream produces one sentence", sentenceCount, std::size_t{1});
        test::expectTrue("Collected captured sentence parses into a GPSPosition", position.has_value());

        if (position)
        {
            test::expectNear("Collected latitude", position->latitude, 37.22350016666667, 1e-8);
            test::expectNear("Collected longitude", position->longitude, -80.44218933333333, 1e-8);
            test::expectNear("Collected ellipsoidal altitude", position->altitude, 614.9, 1e-8);
        }

        // Framing still returns a bad-checksum line; validation belongs to parseGGA.
        std::string badChecksum(CAPTURED_SENTENCE);
        badChecksum.back() = '0';
        expectIncompleteSentence("Corrupted checksum text is still collected", collector, badChecksum);

        const auto corruptedSentence = collector.feed('\n');
        test::expectTrue("Corrupted checksum line is framed", corruptedSentence.has_value());

        if (corruptedSentence)
            test::expectTrue("Parser rejects the framed bad checksum", !parseGGA(*corruptedSentence));

        // A later complete fix must still be usable after a parser rejection.
        expectIncompleteSentence("Valid fix after a bad checksum", collector, CAPTURED_SENTENCE);
        const auto recoveredSentence = collector.feed('\n');
        test::expectTrue("Recovery sentence is framed", recoveredSentence.has_value());

        if (recoveredSentence)
            test::expectTrue("Recovery sentence parses successfully", parseGGA(*recoveredSentence).has_value());
    }
}

int main()
{
    testSentenceBoundaries();
    testChunkedInput();
    testNoiseAndRestart();
    testConsecutiveSentences();
    testIndependentCollectors();
    testBufferCapacityAndRecovery();
    testCollectedGGA();

    return test::summary();
}
