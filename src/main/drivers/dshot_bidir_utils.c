#include <stddef.h>

#include "drivers/dshot_bidir_utils.h"

uint32_t dshotBidirDecodeTelemetryPacket(const uint32_t buffer[], uint32_t count)
{
    return dshotBidirDecodeTelemetryPacketEx(buffer, count, NULL);
}

uint32_t dshotBidirDecodeTelemetryPacketEx(const uint32_t buffer[], uint32_t count, dshotBidirDecodeTrace_t *trace)
{
    if (trace) {
        trace->gcrValue = 0;
        trace->decodedValue = 0;
        trace->bitCount = 0;
        trace->status = DSHOT_BIDIR_DECODE_STATUS_EMPTY;
    }

    if (!buffer || count == 0) {
        return DSHOT_BIDIR_INVALID;
    }

    uint32_t value = 0;
    uint32_t oldValue = buffer[0];
    int bits = 0;

    for (uint32_t i = 1; i <= count; i++) {
        int len;

        if (i < count) {
            const int diff = (int)(buffer[i] - oldValue);
            if (bits >= 21) {
                break;
            }
            len = (diff + 8) / 16;
        } else {
            len = 21 - bits;
        }

        if (len <= 0 || (bits + len) > 21) {
            if (trace) {
                trace->gcrValue = value;
                trace->bitCount = bits;
                trace->status = DSHOT_BIDIR_DECODE_STATUS_LENGTH;
            }
            return DSHOT_BIDIR_INVALID;
        }

        value <<= len;
        value |= 1U << (len - 1);
        oldValue = buffer[i];
        bits += len;
    }

    if (bits != 21) {
        if (trace) {
            trace->gcrValue = value;
            trace->bitCount = bits;
            trace->status = DSHOT_BIDIR_DECODE_STATUS_LENGTH;
        }
        return DSHOT_BIDIR_INVALID;
    }

    if (trace) {
        trace->gcrValue = value;
        trace->bitCount = bits;
    }

    static const uint8_t decodeTable[32] = {
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 9, 10, 11, 0xFF, 13, 14, 15,
        0xFF, 0xFF, 2, 3, 0xFF, 5, 6, 7,
        0xFF, 0, 8, 1, 0xFF, 4, 12, 0xFF
    };

    const uint8_t nibble0 = decodeTable[value & 0x1f];
    const uint8_t nibble1 = decodeTable[(value >> 5) & 0x1f];
    const uint8_t nibble2 = decodeTable[(value >> 10) & 0x1f];
    const uint8_t nibble3 = decodeTable[(value >> 15) & 0x1f];

    if (nibble0 == 0xFF || nibble1 == 0xFF || nibble2 == 0xFF || nibble3 == 0xFF) {
        if (trace) {
            trace->status = DSHOT_BIDIR_DECODE_STATUS_GCR;
        }
        return DSHOT_BIDIR_INVALID;
    }

    uint32_t decodedValue = nibble0;
    decodedValue |= (uint32_t)nibble1 << 4;
    decodedValue |= (uint32_t)nibble2 << 8;
    decodedValue |= (uint32_t)nibble3 << 12;

    if (trace) {
        trace->decodedValue = (uint16_t)decodedValue;
    }

    uint32_t csum = decodedValue;
    csum ^= csum >> 8;
    csum ^= csum >> 4;

    if ((csum & 0xf) != 0xf) {
        if (trace) {
            trace->status = DSHOT_BIDIR_DECODE_STATUS_CHECKSUM;
        }
        return DSHOT_BIDIR_INVALID;
    }

    if (trace) {
        trace->status = DSHOT_BIDIR_DECODE_STATUS_OK;
    }

    return decodedValue >> 4;
}

uint32_t dshotBidirDecodeErpmTelemetryValue(uint16_t value)
{
    if (value == 0x0fff) {
        return 0;
    }

    value = (value & 0x01ff) << ((value & 0xfe00) >> 9);
    if (!value) {
        return DSHOT_BIDIR_INVALID;
    }

    return (1000000U * 60U / 100U + value / 2U) / value;
}

uint32_t dshotBidirComputeRpm(uint32_t erpm, uint8_t motorPoleCount)
{
    const uint8_t polePairs = motorPoleCount / 2;

    if (polePairs == 0) {
        return 0;
    }

    return (erpm * 100U + polePairs / 2U) / polePairs;
}

uint32_t dshotBidirComputeCaptureWindowUs(uint32_t dshotHz)
{
    if (dshotHz == 0) {
        return DSHOT_BIDIR_CAPTURE_DEADTIME_US;
    }

    return DSHOT_BIDIR_CAPTURE_DEADTIME_US +
        (1000000U * (DSHOT_BIDIR_TIMER_BIT_COUNT * DSHOT_BIDIR_TIMER_BIT_LENGTH)) / dshotHz;
}

bool dshotBidirIsRuntimeActive(bool requestedMode, uint8_t configuredMotorCount, uint8_t motorCount, bool driverReady)
{
    return requestedMode && driverReady && motorCount > 0 && configuredMotorCount == motorCount;
}
