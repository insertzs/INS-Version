/*
 * Synthetic tests for target-specific bidirectional DShot helpers.
 */

#include <stdint.h>

#include <vector>

extern "C" {
    #include "drivers/dshot_bidir_utils.h"
}

#include "gtest/gtest.h"

namespace {

constexpr uint8_t kGcrEncode[16] = {
    25, 27, 18, 19, 29, 21, 22, 23,
    26, 9, 10, 11, 30, 13, 14, 15
};

uint16_t encodeChecksumNibble(uint16_t rawValue)
{
    const uint8_t nibble0 = rawValue & 0x0f;
    const uint8_t nibble1 = (rawValue >> 4) & 0x0f;
    const uint8_t nibble2 = (rawValue >> 8) & 0x0f;

    return 0x0f ^ nibble0 ^ nibble1 ^ nibble2;
}

uint16_t makeTelemetryWord(uint16_t rawValue)
{
    return (rawValue << 4) | encodeChecksumNibble(rawValue);
}

uint32_t makeTelemetryRunLengthBits(uint16_t rawValue)
{
    const uint16_t word = makeTelemetryWord(rawValue);
    uint32_t gcr20 = 0;

    for (int nibbleShift = 12; nibbleShift >= 0; nibbleShift -= 4) {
        const uint8_t nibble = (word >> nibbleShift) & 0x0f;
        gcr20 = (gcr20 << 5) | kGcrEncode[nibble];
    }

    return (1U << 20) | gcr20;
}

std::vector<uint32_t> makeCaptureBufferFromRunLengthBits(uint32_t runLengthBits)
{
    std::vector<uint32_t> runs;

    for (int bitIndex = 20; bitIndex >= 0; ) {
        int runLength = 1;
        bitIndex--;

        while (bitIndex >= 0 && !(runLengthBits & (1U << bitIndex))) {
            runLength++;
            bitIndex--;
        }

        runs.push_back(runLength);
    }

    std::vector<uint32_t> edges;
    edges.reserve(runs.size());
    edges.push_back(0);

    uint32_t timestamp = 0;
    for (size_t i = 0; i + 1 < runs.size(); i++) {
        timestamp += runs[i] * 16U;
        edges.push_back(timestamp);
    }

    return edges;
}

std::vector<uint32_t> makeCaptureBuffer(uint16_t rawValue)
{
    return makeCaptureBufferFromRunLengthBits(makeTelemetryRunLengthBits(rawValue));
}

} // namespace

TEST(DshotBidirUnittest, DecodesRoundTripTelemetryWords)
{
    const uint16_t rawValues[] = { 0x0fff, 0x0123, 0x02a5, 0x07ff, 0x0001 };

    for (const uint16_t rawValue : rawValues) {
        const std::vector<uint32_t> capture = makeCaptureBuffer(rawValue);
        ASSERT_GT(capture.size(), 7U);
        EXPECT_EQ(dshotBidirDecodeTelemetryPacket(capture.data(), capture.size()), rawValue);
    }
}

TEST(DshotBidirUnittest, RejectsInvalidChecksum)
{
    std::vector<uint32_t> capture = makeCaptureBuffer(0x02a5);
    capture[1] += 16U;

    EXPECT_EQ(dshotBidirDecodeTelemetryPacket(capture.data(), capture.size()), DSHOT_BIDIR_INVALID);
}

TEST(DshotBidirUnittest, RejectsMissingOrImpossibleCaptureData)
{
    EXPECT_EQ(dshotBidirDecodeTelemetryPacket(nullptr, 0), DSHOT_BIDIR_INVALID);

    const uint32_t impossible[] = { 0, 0, 16, 32, 48, 64, 80, 96 };
    EXPECT_EQ(dshotBidirDecodeTelemetryPacket(impossible, sizeof(impossible) / sizeof(impossible[0])), DSHOT_BIDIR_INVALID);
}

TEST(DshotBidirUnittest, RejectsInvalidGcrSymbols)
{
    uint32_t runLengthBits = makeTelemetryRunLengthBits(0x0001);
    runLengthBits &= ~(0x1FU << 15);
    runLengthBits |= 0x01U << 15;

    const std::vector<uint32_t> capture = makeCaptureBufferFromRunLengthBits(runLengthBits);
    EXPECT_EQ(dshotBidirDecodeTelemetryPacket(capture.data(), capture.size()), DSHOT_BIDIR_INVALID);
}

TEST(DshotBidirUnittest, DecodesErpmAndComputesMechanicalRpm)
{
    EXPECT_EQ(dshotBidirDecodeErpmTelemetryValue(0x0fff), 0U);

    const uint16_t telemetryValue = 0x123;
    const uint32_t erpm = dshotBidirDecodeErpmTelemetryValue(telemetryValue);
    ASSERT_NE(erpm, DSHOT_BIDIR_INVALID);
    EXPECT_EQ(erpm, 2062U);
    EXPECT_EQ(dshotBidirComputeRpm(erpm, 14), 29457U);
    EXPECT_EQ(dshotBidirDecodeErpmTelemetryValue(0x423), 4286U);
    EXPECT_EQ(dshotBidirComputeRpm(erpm, 0), 0U);
}

TEST(DshotBidirUnittest, ComputesCaptureWindowAndRuntimeActivation)
{
    EXPECT_EQ(dshotBidirComputeCaptureWindowUs(6000000U), 88U);
    EXPECT_EQ(dshotBidirComputeCaptureWindowUs(0U), DSHOT_BIDIR_CAPTURE_DEADTIME_US);

    EXPECT_TRUE(dshotBidirIsRuntimeActive(true, 4, 4, true));
    EXPECT_FALSE(dshotBidirIsRuntimeActive(true, 3, 4, true));
    EXPECT_FALSE(dshotBidirIsRuntimeActive(true, 4, 4, false));
    EXPECT_FALSE(dshotBidirIsRuntimeActive(false, 4, 4, true));
}
