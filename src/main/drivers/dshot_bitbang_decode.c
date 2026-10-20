#include <stddef.h>

#include "drivers/dshot_bitbang_decode.h"

#define DSHOT_BITBANG_MIN_VALID_SAMPLES 57U
#define DSHOT_BITBANG_MAX_VALID_SAMPLES 69U

static uint32_t dshotBitbangDecodeValue(uint32_t value)
{
#define INVALID_NIBBLE 0xFFU
    value &= 0xFFFFFU;

    static const uint8_t decodeTable[32] = {
        INVALID_NIBBLE, INVALID_NIBBLE, INVALID_NIBBLE, INVALID_NIBBLE,
        INVALID_NIBBLE, INVALID_NIBBLE, INVALID_NIBBLE, INVALID_NIBBLE,
        INVALID_NIBBLE, 9, 10, 11, INVALID_NIBBLE, 13, 14, 15,
        INVALID_NIBBLE, INVALID_NIBBLE, 2, 3, INVALID_NIBBLE, 5, 6, 7,
        INVALID_NIBBLE, 0, 8, 1, INVALID_NIBBLE, 4, 12, INVALID_NIBBLE
    };

    const uint8_t nibble0 = decodeTable[value & 0x1FU];
    const uint8_t nibble1 = decodeTable[(value >> 5) & 0x1FU];
    const uint8_t nibble2 = decodeTable[(value >> 10) & 0x1FU];
    const uint8_t nibble3 = decodeTable[(value >> 15) & 0x1FU];

    if (nibble0 == INVALID_NIBBLE || nibble1 == INVALID_NIBBLE ||
        nibble2 == INVALID_NIBBLE || nibble3 == INVALID_NIBBLE) {
        return DSHOT_BITBANG_DECODE_INVALID;
    }

    uint32_t decodedValue = nibble0;
    decodedValue |= (uint32_t)nibble1 << 4;
    decodedValue |= (uint32_t)nibble2 << 8;
    decodedValue |= (uint32_t)nibble3 << 12;

    uint32_t csum = decodedValue;
    csum ^= csum >> 8;
    csum ^= csum >> 4;

    if ((csum & 0xFU) != 0xFU || decodedValue > 0xFFFFU) {
        return DSHOT_BITBANG_DECODE_INVALID;
    }

    return decodedValue >> 4;
#undef INVALID_NIBBLE
}

uint32_t dshotBitbangDecodeTelemetry(uint16_t buffer[], uint32_t count, uint32_t bit)
{
    // A timed-out DMA may have produced fewer samples than one packet. Check
    // before subtracting from count or constructing any pointers.
    if (!buffer || count < DSHOT_BITBANG_MIN_VALID_SAMPLES || bit >= 16U) {
        return DSHOT_BITBANG_DECODE_NOEDGE;
    }
    const uint32_t mask = 1U << bit;
    uint16_t lastValue = 0;
    uint32_t value = 0;

    uint16_t *p = buffer;
    uint16_t *endP = p + count - DSHOT_BITBANG_MIN_VALID_SAMPLES;

    while (p < endP) {
        if (!(*p++ & mask)) {
            break;
        }
    }

    const uint32_t startMargin = (uint32_t)(p - buffer);

    if (p >= endP) {
        return DSHOT_BITBANG_DECODE_NOEDGE;
    }

    const uint32_t available = count - startMargin;
    const uint32_t remaining = available < DSHOT_BITBANG_MAX_VALID_SAMPLES ? available : DSHOT_BITBANG_MAX_VALID_SAMPLES;
    uint16_t *oldP = p;
    uint32_t bits = 0;
    endP = p + remaining;

    while (endP > p) {
        if ((*p++ & mask) != lastValue) {
            if (endP > p) {
                const int roundedLen = (int)((p - oldP + 1) / 3);
                const int len = roundedLen > 0 ? roundedLen : 1;
                if (bits + (uint32_t)len > 21U) {
                    return DSHOT_BITBANG_DECODE_INVALID;
                }
                bits += (uint32_t)len;
                value <<= len;
                value |= 1U << (len - 1);
                oldP = p;
                lastValue = *(p - 1) & mask;
            }
        }
    }

    if (bits < 18U) {
        return DSHOT_BITBANG_DECODE_NOEDGE;
    }

    const int trailingLen = 21 - (int)bits;
    if (trailingLen < 0) {
        return DSHOT_BITBANG_DECODE_NOEDGE;
    }

    if (trailingLen > 0) {
        value <<= trailingLen;
        value |= 1U << (trailingLen - 1);
    }

    return dshotBitbangDecodeValue(value);
}
