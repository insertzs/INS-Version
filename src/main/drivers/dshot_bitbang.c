#include "drivers/dshot_bitbang.h"

#include <stddef.h>

void dshotBitbangOutputDataInit(uint32_t *buffer, uint16_t portMask, bool inverted)
{
    const uint32_t resetMask = inverted ? portMask : ((uint32_t)portMask << 16);
    const uint32_t setMask = inverted ? ((uint32_t)portMask << 16) : portMask;

    for (size_t symbolIndex = 0; symbolIndex < DSHOT_BITBANG_FRAME_BITS; symbolIndex++) {
        const size_t base = symbolIndex * DSHOT_BITBANG_OUTPUT_STATES_PER_BIT;
        buffer[base + 0] |= setMask;
        buffer[base + 1] = 0;
        buffer[base + 2] |= resetMask;
    }

    // Hold the line at idle for one extra bit so the ESC can sample the last symbol cleanly.
    const size_t holdIndex = DSHOT_BITBANG_FRAME_BITS * DSHOT_BITBANG_OUTPUT_STATES_PER_BIT;
    buffer[holdIndex + 0] |= resetMask;
    buffer[holdIndex + 1] = 0;
    buffer[holdIndex + 2] = 0;
}

void dshotBitbangOutputDataSet(uint32_t *buffer, uint8_t pinIndex, uint16_t packet, bool inverted)
{
    const uint32_t middleBit = inverted ? (1U << pinIndex) : (1U << (pinIndex + 16U));

    for (size_t bitIndex = 0; bitIndex < DSHOT_BITBANG_FRAME_BITS; bitIndex++) {
        if ((packet & 0x8000U) == 0U) {
            buffer[bitIndex * DSHOT_BITBANG_OUTPUT_STATES_PER_BIT + 1U] |= middleBit;
        }
        packet <<= 1;
    }
}

void dshotBitbangOutputDataClear(uint32_t *buffer)
{
    for (size_t bitIndex = 0; bitIndex < DSHOT_BITBANG_FRAME_BITS; bitIndex++) {
        buffer[bitIndex * DSHOT_BITBANG_OUTPUT_STATES_PER_BIT + 1U] = 0;
    }
}

uint32_t dshotBitbangComputeFrameTimeUs(uint32_t dshotHz)
{
    if (dshotHz == 0U) {
        return 0;
    }

    return (1000000U * 17U * DSHOT_BITBANG_OUTPUT_STATES_PER_BIT) / dshotHz;
}

uint32_t dshotBitbangComputeInputSampleHz(uint32_t dshotHz)
{
    if (dshotHz == 0U) {
        return 0;
    }

    return (dshotHz * 5U * 2U * DSHOT_BITBANG_TELEMETRY_OVERSAMPLE) / 24U;
}
