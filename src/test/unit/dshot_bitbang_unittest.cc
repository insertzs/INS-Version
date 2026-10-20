#include <gtest/gtest.h>
#include <vector>

extern "C" {
#include "drivers/dshot_bitbang.h"
#include "drivers/dshot_bitbang_decode.h"
}

namespace {

constexpr uint16_t kPortMask = 1U << 3;
constexpr uint8_t kPinIndex = 3U;

} // namespace

TEST(DshotBitbangUnittest, InitializesNonInvertedBsrrFrame)
{
    uint32_t buffer[DSHOT_BITBANG_OUTPUT_BUFFER_LENGTH] = {};

    dshotBitbangOutputDataInit(buffer, kPortMask, false);

    const uint32_t setMask = kPortMask;
    const uint32_t resetMask = static_cast<uint32_t>(kPortMask) << 16;

    for (size_t bitIndex = 0; bitIndex < DSHOT_BITBANG_FRAME_BITS; bitIndex++) {
        const size_t base = bitIndex * DSHOT_BITBANG_OUTPUT_STATES_PER_BIT;
        EXPECT_EQ(buffer[base + 0], setMask);
        EXPECT_EQ(buffer[base + 1], 0U);
        EXPECT_EQ(buffer[base + 2], resetMask);
    }

    const size_t holdIndex = DSHOT_BITBANG_FRAME_BITS * DSHOT_BITBANG_OUTPUT_STATES_PER_BIT;
    EXPECT_EQ(buffer[holdIndex + 0], resetMask);
    EXPECT_EQ(buffer[holdIndex + 1], 0U);
    EXPECT_EQ(buffer[holdIndex + 2], 0U);
}

TEST(DshotBitbangUnittest, InitializesInvertedBsrrFrame)
{
    uint32_t buffer[DSHOT_BITBANG_OUTPUT_BUFFER_LENGTH] = {};

    dshotBitbangOutputDataInit(buffer, kPortMask, true);

    const uint32_t setMask = static_cast<uint32_t>(kPortMask) << 16;
    const uint32_t resetMask = kPortMask;

    for (size_t bitIndex = 0; bitIndex < DSHOT_BITBANG_FRAME_BITS; bitIndex++) {
        const size_t base = bitIndex * DSHOT_BITBANG_OUTPUT_STATES_PER_BIT;
        EXPECT_EQ(buffer[base + 0], setMask);
        EXPECT_EQ(buffer[base + 1], 0U);
        EXPECT_EQ(buffer[base + 2], resetMask);
    }

    const size_t holdIndex = DSHOT_BITBANG_FRAME_BITS * DSHOT_BITBANG_OUTPUT_STATES_PER_BIT;
    EXPECT_EQ(buffer[holdIndex + 0], resetMask);
    EXPECT_EQ(buffer[holdIndex + 1], 0U);
    EXPECT_EQ(buffer[holdIndex + 2], 0U);
}

TEST(DshotBitbangUnittest, SetsMiddleStateForZeroBitsOnly)
{
    uint32_t buffer[DSHOT_BITBANG_OUTPUT_BUFFER_LENGTH] = {};

    dshotBitbangOutputDataInit(buffer, kPortMask, false);
    dshotBitbangOutputDataSet(buffer, kPinIndex, 0x8000U, false);

    const uint32_t middleBit = 1U << (kPinIndex + 16U);
    EXPECT_EQ(buffer[0 * DSHOT_BITBANG_OUTPUT_STATES_PER_BIT + 1U], 0U);
    EXPECT_EQ(buffer[1 * DSHOT_BITBANG_OUTPUT_STATES_PER_BIT + 1U], middleBit);
    EXPECT_EQ(buffer[15 * DSHOT_BITBANG_OUTPUT_STATES_PER_BIT + 1U], middleBit);
}

TEST(DshotBitbangUnittest, ClearsOnlyMiddleState)
{
    uint32_t buffer[DSHOT_BITBANG_OUTPUT_BUFFER_LENGTH] = {};

    dshotBitbangOutputDataInit(buffer, kPortMask, true);
    dshotBitbangOutputDataSet(buffer, kPinIndex, 0x0000U, true);
    dshotBitbangOutputDataClear(buffer);

    const uint32_t setMask = static_cast<uint32_t>(kPortMask) << 16;
    const uint32_t resetMask = kPortMask;

    for (size_t bitIndex = 0; bitIndex < DSHOT_BITBANG_FRAME_BITS; bitIndex++) {
        const size_t base = bitIndex * DSHOT_BITBANG_OUTPUT_STATES_PER_BIT;
        EXPECT_EQ(buffer[base + 0], setMask);
        EXPECT_EQ(buffer[base + 1], 0U);
        EXPECT_EQ(buffer[base + 2], resetMask);
    }
}

TEST(DshotBitbangUnittest, ComputesExpectedTimingHelpers)
{
    EXPECT_EQ(dshotBitbangComputeOutputStateHz(300000U), 900000U);
    EXPECT_EQ(dshotBitbangComputeOutputStateHz(0U), 0U);

    EXPECT_EQ(dshotBitbangComputeFrameTimeUs(300000U), 56U);
    EXPECT_EQ(dshotBitbangComputeFrameTimeUs(0U), 0U);

    EXPECT_EQ(dshotBitbangComputeInputSampleHz(300000U), 1125000U);
    EXPECT_EQ(dshotBitbangComputeInputSampleHz(0U), 0U);
}

namespace {
std::vector<uint16_t> makeSamples(uint16_t raw, unsigned preamble, uint8_t pin)
{
    const uint8_t gcr[] = {25, 27, 18, 19, 29, 21, 22, 23, 26, 9, 10, 11, 30, 13, 14, 15};
    const uint16_t checksum = 0xf ^ (raw & 0xf) ^ ((raw >> 4) & 0xf) ^ ((raw >> 8) & 0xf);
    const uint16_t word = (raw << 4) | checksum;
    uint32_t encoded = 1;
    for (int shift = 12; shift >= 0; shift -= 4) {
        encoded = (encoded << 5) | gcr[(word >> shift) & 0xf];
    }
    const uint16_t mask = 1U << pin;
    std::vector<uint16_t> samples(preamble, mask);
    uint16_t level = mask;
    for (int bit = 20; bit >= 0; bit--) {
        if (encoded & (1U << bit)) {
            level ^= mask;
        }
        samples.insert(samples.end(), 3, level);
    }
    samples.resize(140, level);
    return samples;
}
}

TEST(DshotBitbangUnittest, RejectsShortDmaCapturesAndInvalidPin)
{
    EXPECT_EQ(dshotBitbangDecodeTelemetry(nullptr, 0, 6), DSHOT_BITBANG_DECODE_NOEDGE);
    for (unsigned count = 0; count < 57; count++) {
        std::vector<uint16_t> samples(count, 0xffff);
        EXPECT_EQ(dshotBitbangDecodeTelemetry(samples.data(), count, 6), DSHOT_BITBANG_DECODE_NOEDGE);
    }
    std::vector<uint16_t> samples(140, 0xffff);
    EXPECT_EQ(dshotBitbangDecodeTelemetry(samples.data(), samples.size(), 16), DSHOT_BITBANG_DECODE_NOEDGE);
    EXPECT_EQ(dshotBitbangDecodeTelemetry(samples.data(), samples.size(), 6), DSHOT_BITBANG_DECODE_NOEDGE);
}

TEST(DshotBitbangUnittest, DecodesAllPayloadsOnEachMotorPin)
{
    for (uint8_t pin = 6; pin <= 9; pin++) {
        for (uint16_t raw = 0; raw < 4096; raw++) {
            auto samples = makeSamples(raw, 30, pin);
            ASSERT_EQ(dshotBitbangDecodeTelemetry(samples.data(), samples.size(), pin), raw)
                << "pin=" << unsigned(pin) << " raw=" << raw;
        }
    }
}

TEST(DshotBitbangUnittest, DifferentMotorResponseDelaysDoNotSharePreambleState)
{
    for (unsigned preamble : {60U, 5U, 40U, 1U, 30U}) {
        auto samples = makeSamples(0x123, preamble, 7);
        EXPECT_EQ(dshotBitbangDecodeTelemetry(samples.data(), samples.size(), 7), 0x123U);
    }
}
