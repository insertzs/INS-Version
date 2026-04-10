#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "drivers/timer.h"

void dshotBidirInit(uint32_t dshotHz);
bool dshotBidirAttachMotor(uint8_t motorIndex, TCH_t *tch, void *dmaBurstBuffer, void *dmaOutputBuffer);
bool dshotBidirIsReady(void);
bool dshotBidirUsesBurstOutput(void);
bool dshotBidirUpdate(void);
void dshotBidirOnFrameStarted(void);
uint32_t dshotBidirGetReadCount(void);
uint32_t dshotBidirGetInvalidPacketCount(void);
uint32_t dshotBidirGetNoEdgeCount(void);
uint32_t dshotBidirGetTimeoutCount(void);
uint32_t dshotBidirGetBurstCompleteCount(void);
uint32_t dshotBidirGetCaptureStartCount(void);
uint8_t dshotBidirGetState(void);
uint16_t dshotBidirGetBurstRequestSource(void);
uint32_t dshotBidirGetLastBurstLatencyUs(void);
uint8_t dshotBidirGetLastEvent(void);
uint32_t dshotBidirGetLastEventElapsedUs(void);
uint32_t dshotBidirGetOverflowCallbackCount(void);
uint32_t dshotBidirGetOutputDmaCallbackCount(void);
uint8_t dshotBidirGetLastOutputUpdateCount(void);
uint8_t dshotBidirGetLastOutputExpectedMask(void);
uint8_t dshotBidirGetLastOutputCompleteMask(void);
uint32_t dshotBidirGetLastTimerArr(void);
uint32_t dshotBidirGetLastTimerCnt(void);
uint8_t dshotBidirGetOutputUpdateCount(void);
uint8_t dshotBidirGetOutputExpectedMask(void);
uint8_t dshotBidirGetOutputCompleteMask(void);
uint32_t dshotBidirGetOutputWindowUs(void);
uint32_t dshotBidirGetCaptureWindowUs(void);
uint8_t dshotBidirGetLastEdgeCount(uint8_t motorIndex);
uint16_t dshotBidirGetLastRawValue(uint8_t motorIndex);
uint32_t dshotBidirGetLastErpmValue(uint8_t motorIndex);
uint32_t dshotBidirGetLastRpmValue(uint8_t motorIndex);
uint32_t dshotBidirGetMotorReadCount(uint8_t motorIndex);
uint32_t dshotBidirGetMotorInvalidCount(uint8_t motorIndex);
uint32_t dshotBidirGetMotorNoEdgeCount(uint8_t motorIndex);
uint32_t dshotBidirGetMotorIrqEdgeCallbackCount(uint8_t motorIndex);
uint8_t dshotBidirGetMotorLastInterestingDiag(uint8_t motorIndex);
uint8_t dshotBidirGetMotorLastInterestingEdgeCount(uint8_t motorIndex);
uint16_t dshotBidirGetMotorLastInterestingRawValue(uint8_t motorIndex);
uint32_t dshotBidirGetMotorLastInterestingErpmValue(uint8_t motorIndex);
uint32_t dshotBidirGetMotorLastInterestingRpmValue(uint8_t motorIndex);
uint8_t dshotBidirGetLastDecodeMotor(void);
uint8_t dshotBidirGetLastDecodeStatus(void);
uint8_t dshotBidirGetLastDecodeEdgeCount(void);
uint32_t dshotBidirGetLastDecodeGcrValue(void);
uint16_t dshotBidirGetLastDecodeWord(void);
uint16_t dshotBidirGetLastDecodeDelta(uint8_t deltaIndex);
