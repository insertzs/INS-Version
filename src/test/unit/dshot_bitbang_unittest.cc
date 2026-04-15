#include <gtest/gtest.h>

extern "C" {
#include "drivers/dshot_bitbang.h"
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
