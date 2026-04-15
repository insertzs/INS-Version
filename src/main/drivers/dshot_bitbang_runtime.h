#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "drivers/timer.h"

typedef enum {
    DSHOT_BITBANG_RUNTIME_DECODE_NONE = 0,
    DSHOT_BITBANG_RUNTIME_DECODE_OK,
    DSHOT_BITBANG_RUNTIME_DECODE_NOEDGE,
    DSHOT_BITBANG_RUNTIME_DECODE_INVALID,
    DSHOT_BITBANG_RUNTIME_DECODE_ERPM,
} dshotBitbangRuntimeDecodeStatus_e;

bool dshotBitbangRuntimeInit(uint32_t dshotSymbolHz);
bool dshotBitbangRuntimeConfigureMotor(const timerHardware_t *timerHardware, uint8_t motorIndex, bool enableOutput);
bool dshotBitbangRuntimeIsActive(void);
bool dshotBitbangRuntimeIsReady(void);
bool dshotBitbangRuntimeUpdate(void);
void dshotBitbangRuntimeBeginFrame(void);
void dshotBitbangRuntimeSetMotorValue(uint8_t motorIndex, uint16_t value, bool requestTelemetry);
void dshotBitbangRuntimeStartFrame(void);
ioTag_t dshotBitbangRuntimeGetMotorPinTag(uint8_t motorIndex);
uint8_t dshotBitbangRuntimeGetState(void);
uint16_t dshotBitbangRuntimeGetRequestSource(void);
uint32_t dshotBitbangRuntimeGetReadCount(void);
uint32_t dshotBitbangRuntimeGetInvalidCount(void);
uint32_t dshotBitbangRuntimeGetNoEdgeCount(void);
uint32_t dshotBitbangRuntimeGetTimeoutCount(void);
uint32_t dshotBitbangRuntimeGetOutputCallbackCount(void);
uint32_t dshotBitbangRuntimeGetInputCallbackCount(void);
uint32_t dshotBitbangRuntimeGetLastLatencyUs(void);
uint32_t dshotBitbangRuntimeGetLastSampleCount(void);
uint32_t dshotBitbangRuntimeGetLastRawValue(uint8_t motorIndex);
uint32_t dshotBitbangRuntimeGetLastErpmValue(uint8_t motorIndex);
uint32_t dshotBitbangRuntimeGetLastRpmValue(uint8_t motorIndex);
uint32_t dshotBitbangRuntimeGetMotorReadCount(uint8_t motorIndex);
uint32_t dshotBitbangRuntimeGetMotorInvalidCount(uint8_t motorIndex);
uint32_t dshotBitbangRuntimeGetMotorNoEdgeCount(uint8_t motorIndex);
uint8_t dshotBitbangRuntimeGetLastDecodeMotor(void);
uint8_t dshotBitbangRuntimeGetLastDecodeStatus(void);
