#pragma once

#include <stdbool.h>
#include <stdint.h>

#define DSHOT_BIDIR_INVALID                0xFFFFU
#define DSHOT_BIDIR_TIMER_BIT_COUNT        16U
#define DSHOT_BIDIR_TIMER_BIT_LENGTH       20U
#define DSHOT_BIDIR_CAPTURE_DEADTIME_US    35U

typedef enum {
    DSHOT_BIDIR_DECODE_STATUS_OK = 0,
    DSHOT_BIDIR_DECODE_STATUS_EMPTY,
    DSHOT_BIDIR_DECODE_STATUS_LENGTH,
    DSHOT_BIDIR_DECODE_STATUS_GCR,
    DSHOT_BIDIR_DECODE_STATUS_CHECKSUM,
} dshotBidirDecodeStatus_e;

typedef struct {
    uint32_t gcrValue;
    uint16_t decodedValue;
    uint8_t bitCount;
    uint8_t status;
} dshotBidirDecodeTrace_t;

uint32_t dshotBidirDecodeTelemetryPacket(const uint32_t buffer[], uint32_t count);
uint32_t dshotBidirDecodeTelemetryPacketEx(const uint32_t buffer[], uint32_t count, dshotBidirDecodeTrace_t *trace);
uint32_t dshotBidirDecodeErpmTelemetryValue(uint16_t value);
uint32_t dshotBidirComputeRpm(uint32_t erpm, uint8_t motorPoleCount);
uint32_t dshotBidirComputeCaptureWindowUs(uint32_t dshotHz);
bool dshotBidirIsRuntimeActive(bool requestedMode, uint8_t configuredMotorCount, uint8_t motorCount, bool driverReady);
