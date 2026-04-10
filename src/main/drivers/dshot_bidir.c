/*
 * This file is part of INAV Project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */

#include <stddef.h>
#include <string.h>

#include "platform.h"

#ifdef USE_DSHOT_BIDIR

#include "build/debug.h"

#include "common/time.h"

#include "drivers/dma.h"
#include "drivers/dshot_bidir_utils.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/pwm_output.h"
#include "drivers/timer.h"

#include "flight/mixer.h"
#include "sensors/rpm_source.h"

#define DSHOT_BIDIR_MOTOR_LIMIT            4
#define DSHOT_BIDIR_CAPTURE_BUFFER_LEN     32
#define DSHOT_BIDIR_MIN_GCR_EDGES          7
#define DSHOT_BIDIR_OUTPUT_PERIOD          (DSHOT_BIDIR_TIMER_BIT_LENGTH - 1)
#define DSHOT_BIDIR_CAPTURE_FILTER         0
#define DSHOT_BIDIR_BURST_LENGTH           (18 * 4)
#define DSHOT_BIDIR_OUTPUT_BUFFER_LEN      (DSHOT_BIDIR_BURST_LENGTH / 4)
#define DSHOT_BIDIR_OUTPUT_COMPLETE_UPDATES (DSHOT_BIDIR_OUTPUT_BUFFER_LEN + 1U)
#define DSHOT_BIDIR_STARTUP_COMPARE_VALUE  0xFFFF
#define DSHOT_BIDIR_RUNTIME_IDLE_COMPARE   DSHOT_BIDIR_STARTUP_COMPARE_VALUE
#define DSHOT_BIDIR_CAPTURE_START_DELAY_TICKS 0U
#define DSHOT_BIDIR_PENDING_TIMEOUT_US     600U

typedef enum {
    DSHOT_BIDIR_STATE_READY = 0,
    DSHOT_BIDIR_STATE_OUTPUT_PENDING,
    DSHOT_BIDIR_STATE_CAPTURING,
} dshotBidirState_e;

typedef enum {
    DSHOT_BIDIR_EVENT_NONE = 0,
    DSHOT_BIDIR_EVENT_CAPTURE_START,
    DSHOT_BIDIR_EVENT_TIMEOUT,
    DSHOT_BIDIR_EVENT_DECODE_COMPLETE,
} dshotBidirEvent_e;

typedef enum {
    DSHOT_BIDIR_DIAG_NONE = 0,
    DSHOT_BIDIR_DIAG_OK,
    DSHOT_BIDIR_DIAG_NOEDGE,
    DSHOT_BIDIR_DIAG_EMPTY,
    DSHOT_BIDIR_DIAG_LENGTH,
    DSHOT_BIDIR_DIAG_GCR,
    DSHOT_BIDIR_DIAG_CHECKSUM,
    DSHOT_BIDIR_DIAG_ERPM,
} dshotBidirDiag_e;

typedef struct {
    TCH_t *tch;
    DMA_t dma;
    uint16_t dmaSource;
    bool useInterruptCapture;
    bool configured;
    volatile uint8_t irqEdgeCount;
    timerCallbacks_t irqCallbacks;
    uint32_t captureBuffer[DSHOT_BIDIR_CAPTURE_BUFFER_LEN];
    TIM_OCInitTypeDef outputInit;
    TIM_ICInitTypeDef inputCaptureInit;
    DMA_InitTypeDef outputDmaInit;
    DMA_InitTypeDef inputDmaInit;
} dshotBidirMotor_t;

typedef struct {
    TIM_TypeDef *timer;
    TCH_t *burstTch;
    DMA_t burstDma;
    timerDMASafeType_t *dmaBurstBuffer;
    DMA_InitTypeDef burstOutputDmaInit;
    uint16_t burstRequestSource;
    uint16_t statusRequestSource;
    uint32_t outputWindowUs;
    uint32_t captureWindowUs;
    timeUs_t frameStartedUs;
    volatile timeUs_t captureStartedUs;
    volatile dshotBidirState_e state;
    volatile uint8_t outputExpectedMask;
    volatile uint8_t outputCompleteMask;
    volatile uint8_t outputUpdateCount;
    timerCallbacks_t burstTimerCallbacks;
    bool burstHandlerInstalled;
    bool useBurstOutput;
} dshotBidirContext_t;

static dshotBidirContext_t dshotBidirCtx;
static dshotBidirMotor_t dshotBidirMotors[DSHOT_BIDIR_MOTOR_LIMIT];
static const timerCallbacks_t dshotBidirCallbacksDisabled = { 0 };
static uint32_t dshotBidirReadCount = 0;
static uint32_t dshotBidirInvalidPacketCount = 0;
static uint32_t dshotBidirNoEdgeCount = 0;
static uint32_t dshotBidirTimeoutCount = 0;
static uint32_t dshotBidirBurstCompleteCount = 0;
static uint32_t dshotBidirCaptureStartCount = 0;
static uint32_t dshotBidirLastBurstLatencyUs = 0;
static uint32_t dshotBidirOverflowCallbackCount = 0;
static uint32_t dshotBidirOutputDmaCallbackCount = 0;
static uint32_t dshotBidirLastEventElapsedUs = 0;
static uint8_t dshotBidirLastEvent = DSHOT_BIDIR_EVENT_NONE;
static uint8_t dshotBidirLastOutputUpdateCount = 0;
static uint8_t dshotBidirLastOutputExpectedMask = 0;
static uint8_t dshotBidirLastOutputCompleteMask = 0;
static uint32_t dshotBidirLastTimerArr = 0;
static uint32_t dshotBidirLastTimerCnt = 0;
static uint8_t dshotBidirLastEdgeCount[DSHOT_BIDIR_MOTOR_LIMIT];
static uint16_t dshotBidirLastRawValue[DSHOT_BIDIR_MOTOR_LIMIT];
static uint32_t dshotBidirLastErpmValue[DSHOT_BIDIR_MOTOR_LIMIT];
static uint32_t dshotBidirLastRpmValue[DSHOT_BIDIR_MOTOR_LIMIT];
static uint8_t dshotBidirLastDecodeMotor = DSHOT_BIDIR_MOTOR_LIMIT;
static uint8_t dshotBidirLastDecodeStatus = DSHOT_BIDIR_DIAG_NONE;
static uint8_t dshotBidirLastDecodeEdgeCount = 0;
static uint32_t dshotBidirLastDecodeGcrValue = 0;
static uint16_t dshotBidirLastDecodeWord = 0;
static uint16_t dshotBidirLastDecodeDeltas[4];
static uint32_t dshotBidirMotorReadCount[DSHOT_BIDIR_MOTOR_LIMIT];
static uint32_t dshotBidirMotorInvalidCount[DSHOT_BIDIR_MOTOR_LIMIT];
static uint32_t dshotBidirMotorNoEdgeCount[DSHOT_BIDIR_MOTOR_LIMIT];
static uint32_t dshotBidirMotorIrqEdgeCallbackCount[DSHOT_BIDIR_MOTOR_LIMIT];
static uint8_t dshotBidirMotorLastInterestingDiag[DSHOT_BIDIR_MOTOR_LIMIT];
static uint8_t dshotBidirMotorLastInterestingEdgeCount[DSHOT_BIDIR_MOTOR_LIMIT];
static uint16_t dshotBidirMotorLastInterestingRawValue[DSHOT_BIDIR_MOTOR_LIMIT];
static uint32_t dshotBidirMotorLastInterestingErpmValue[DSHOT_BIDIR_MOTOR_LIMIT];
static uint32_t dshotBidirMotorLastInterestingRpmValue[DSHOT_BIDIR_MOTOR_LIMIT];

static void dshotBidirLatchTrace(dshotBidirEvent_e event, uint32_t elapsedUs)
{
    dshotBidirLastEvent = (uint8_t)event;
    dshotBidirLastEventElapsedUs = elapsedUs;
    dshotBidirLastOutputUpdateCount = dshotBidirCtx.outputUpdateCount;
    dshotBidirLastOutputExpectedMask = dshotBidirCtx.outputExpectedMask;
    dshotBidirLastOutputCompleteMask = dshotBidirCtx.outputCompleteMask;
    dshotBidirLastTimerArr = dshotBidirCtx.timer ? dshotBidirCtx.timer->ARR : 0;
    dshotBidirLastTimerCnt = dshotBidirCtx.timer ? dshotBidirCtx.timer->CNT : 0;
}

static void dshotBidirApplyTimerPeriod(uint16_t period)
{
    TIM_TypeDef *timer = dshotBidirCtx.timer;

    if (!timer) {
        return;
    }

    TIM_ARRPreloadConfig(timer, ENABLE);
    timer->ARR = period;
    TIM_SetCounter(timer, 0);

    // Force the preload value into the active ARR immediately.
    TIM_GenerateEvent(timer, TIM_EventSource_Update);
    TIM_ClearFlag(timer, TIM_FLAG_Update);
}

static void dshotBidirLatchDecodeTrace(uint8_t motorIndex, dshotBidirDiag_e status, const dshotBidirDecodeTrace_t *trace, const uint32_t *buffer, uint32_t edgeCount)
{
    dshotBidirLastDecodeMotor = motorIndex;
    dshotBidirLastDecodeStatus = (uint8_t)status;
    dshotBidirLastDecodeEdgeCount = edgeCount > UINT8_MAX ? UINT8_MAX : (uint8_t)edgeCount;
    dshotBidirLastDecodeGcrValue = trace ? trace->gcrValue : 0;
    dshotBidirLastDecodeWord = trace ? trace->decodedValue : 0;
    memset(dshotBidirLastDecodeDeltas, 0, sizeof(dshotBidirLastDecodeDeltas));

    if (!buffer || edgeCount < 2U) {
        return;
    }

    const uint32_t maxDeltaCount = (uint32_t)ARRAYLEN(dshotBidirLastDecodeDeltas);
    const uint32_t deltaCount = (edgeCount - 1U < maxDeltaCount) ? (edgeCount - 1U) : maxDeltaCount;
    for (uint32_t i = 0; i < deltaCount; i++) {
        dshotBidirLastDecodeDeltas[i] = (uint16_t)(buffer[i + 1U] - buffer[i]);
    }
}

static void dshotBidirLatchMotorInteresting(uint8_t motorIndex, dshotBidirDiag_e status, uint32_t edgeCount, uint16_t rawValue, uint32_t erpm, uint32_t rpm)
{
    if (motorIndex >= DSHOT_BIDIR_MOTOR_LIMIT) {
        return;
    }

    dshotBidirMotorLastInterestingDiag[motorIndex] = (uint8_t)status;
    dshotBidirMotorLastInterestingEdgeCount[motorIndex] = edgeCount > UINT8_MAX ? UINT8_MAX : (uint8_t)edgeCount;
    dshotBidirMotorLastInterestingRawValue[motorIndex] = rawValue;
    dshotBidirMotorLastInterestingErpmValue[motorIndex] = erpm;
    dshotBidirMotorLastInterestingRpmValue[motorIndex] = rpm;
}

static void dshotBidirWaitCaptureStartDelay(void)
{
    TIM_TypeDef *timer = dshotBidirCtx.timer;

    if (!timer || DSHOT_BIDIR_CAPTURE_START_DELAY_TICKS == 0U) {
        return;
    }

    const uint16_t startCount = (uint16_t)timer->CNT;
    const uint16_t period = (uint16_t)(timer->ARR + 1U);

    while (true) {
        const uint16_t currentCount = (uint16_t)timer->CNT;
        const uint16_t delta = (currentCount >= startCount) ?
            (currentCount - startCount) :
            (uint16_t)((period - startCount) + currentCount);

        if (delta >= DSHOT_BIDIR_CAPTURE_START_DELAY_TICKS) {
            return;
        }
    }
}

static void dshotBidirUpdateDebug(uint32_t readDelta, uint32_t invalidDelta, uint32_t noEdgeDelta, uint32_t timeoutDelta)
{
    DEBUG_SET(DEBUG_DSHOT_BIDIR, 0, dshotBidirLastRpmValue[0]);
    DEBUG_SET(DEBUG_DSHOT_BIDIR, 1, dshotBidirLastRpmValue[1]);
    DEBUG_SET(DEBUG_DSHOT_BIDIR, 2, dshotBidirLastRpmValue[2]);
    DEBUG_SET(DEBUG_DSHOT_BIDIR, 3, dshotBidirLastRpmValue[3]);
    DEBUG_SET(DEBUG_DSHOT_BIDIR, 4, readDelta);
    DEBUG_SET(DEBUG_DSHOT_BIDIR, 5, invalidDelta);
    DEBUG_SET(DEBUG_DSHOT_BIDIR, 6, noEdgeDelta);
    DEBUG_SET(DEBUG_DSHOT_BIDIR, 7, timeoutDelta);
}

static uint16_t dshotBidirTimerDmaSource(uint8_t channelIndex)
{
    switch (channelIndex) {
        case 0:
            return TIM_DMA_CC1;
        case 1:
            return TIM_DMA_CC2;
        case 2:
            return TIM_DMA_CC3;
        case 3:
            return TIM_DMA_CC4;
        default:
            return 0;
    }
}

static uint16_t dshotBidirTimerChannel(uint8_t channelIndex)
{
    switch (channelIndex) {
        case 0:
            return TIM_Channel_1;
        case 1:
            return TIM_Channel_2;
        case 2:
            return TIM_Channel_3;
        case 3:
            return TIM_Channel_4;
        default:
            return TIM_Channel_1;
    }
}

static uint32_t dshotBidirBurstDmaChannel(const TCH_t *tch)
{
#ifdef STM32F4
    if (tch && tch->timHw->tim == TIM4 && tch->timCtx->DMASource == TIM_DMA_Update) {
        return DMA_Channel_2;
    }
#endif

    return dmaGetChannelByTag(tch->timHw->dmaTag);
}

static uint32_t dshotBidirComputeOutputWindowUs(uint32_t dshotHz)
{
    if (dshotHz == 0) {
        return 0;
    }

    return (1000000U * ((DSHOT_BIDIR_BURST_LENGTH / 4U) * DSHOT_BIDIR_TIMER_BIT_LENGTH)) / dshotHz;
}

static uint8_t dshotBidirGetMotorBit(const dshotBidirMotor_t *motor)
{
    const ptrdiff_t motorIndex = motor - dshotBidirMotors;

    if (motorIndex < 0 || motorIndex >= DSHOT_BIDIR_MOTOR_LIMIT) {
        return 0;
    }

    return (uint8_t)(1U << motorIndex);
}

static dshotBidirMotor_t *dshotBidirFindMotorByTch(TCH_t *tch)
{
    for (uint8_t i = 0; i < DSHOT_BIDIR_MOTOR_LIMIT; i++) {
        if (dshotBidirMotors[i].configured && dshotBidirMotors[i].tch == tch) {
            return &dshotBidirMotors[i];
        }
    }

    return NULL;
}

static bool dshotBidirOriginalOutputInverted(const dshotBidirMotor_t *motor)
{
    return (motor->tch->timHw->output & TIMER_OUTPUT_INVERTED) != 0;
}

static bool dshotBidirWireOutputInverted(const dshotBidirMotor_t *motor)
{
    // Bidirectional DShot on STM32 timers still uses the normal DShot line
    // polarity; only the telemetry request/checksum semantics are inverted.
    return dshotBidirOriginalOutputInverted(motor);
}

static ioConfig_t dshotBidirGetIoConfig(const dshotBidirMotor_t *motor)
{
    return dshotBidirOriginalOutputInverted(motor) ? IOCFG_AF_PP_PD : IOCFG_AF_PP_UP;
}

static void dshotBidirConfigureIo(const dshotBidirMotor_t *motor)
{
    const IO_t io = IOGetByTag(motor->tch->timHw->tag);
    IOConfigGPIOAF(io, dshotBidirGetIoConfig(motor), motor->tch->timHw->alternateFunction);
}

static void dshotBidirConfigureOutputChannel(const dshotBidirMotor_t *motor, uint16_t value)
{
    TIM_OCInitTypeDef outputInit;

    TIM_OCStructInit(&outputInit);
    outputInit.TIM_OCMode = TIM_OCMode_PWM1;
    outputInit.TIM_Pulse = value;

    if (motor->tch->timHw->output & TIMER_OUTPUT_N_CHANNEL) {
        outputInit.TIM_OutputState = TIM_OutputState_Disable;
        outputInit.TIM_OutputNState = TIM_OutputNState_Enable;
        outputInit.TIM_OCNPolarity = dshotBidirWireOutputInverted(motor) ? TIM_OCPolarity_Low : TIM_OCPolarity_High;
        outputInit.TIM_OCNIdleState = TIM_OCIdleState_Reset;
    } else {
        outputInit.TIM_OutputState = TIM_OutputState_Enable;
        outputInit.TIM_OutputNState = TIM_OutputNState_Disable;
        outputInit.TIM_OCPolarity = dshotBidirWireOutputInverted(motor) ? TIM_OCPolarity_Low : TIM_OCPolarity_High;
        outputInit.TIM_OCIdleState = TIM_OCIdleState_Set;
    }

    switch (motor->tch->timHw->channelIndex) {
        case 0:
            TIM_OC1Init(motor->tch->timHw->tim, &outputInit);
            TIM_OC1PreloadConfig(motor->tch->timHw->tim, TIM_OCPreload_Enable);
            break;
        case 1:
            TIM_OC2Init(motor->tch->timHw->tim, &outputInit);
            TIM_OC2PreloadConfig(motor->tch->timHw->tim, TIM_OCPreload_Enable);
            break;
        case 2:
            TIM_OC3Init(motor->tch->timHw->tim, &outputInit);
            TIM_OC3PreloadConfig(motor->tch->timHw->tim, TIM_OCPreload_Enable);
            break;
        case 3:
            TIM_OC4Init(motor->tch->timHw->tim, &outputInit);
            TIM_OC4PreloadConfig(motor->tch->timHw->tim, TIM_OCPreload_Enable);
            break;
    }
}

static void dshotBidirInvalidateAll(void)
{
    for (uint8_t i = 0; i < DSHOT_BIDIR_MOTOR_LIMIT; i++) {
        rpmSourceInvalidateDshotBidir(i);
    }
}

static void dshotBidirIrqCaptureCallback(TCH_t *tch, uint32_t value)
{
    if (dshotBidirCtx.state != DSHOT_BIDIR_STATE_CAPTURING) {
        return;
    }

    for (uint8_t i = 0; i < DSHOT_BIDIR_MOTOR_LIMIT; i++) {
        dshotBidirMotor_t *motor = &dshotBidirMotors[i];
        if (!motor->configured || !motor->useInterruptCapture || motor->tch != tch) {
            continue;
        }

        const uint8_t edgeCount = motor->irqEdgeCount;
        if (edgeCount < DSHOT_BIDIR_CAPTURE_BUFFER_LEN) {
            motor->captureBuffer[edgeCount] = value;
            motor->irqEdgeCount = edgeCount + 1;
        }
        return;
    }
}

static void dshotBidirConfigureInputCapture(const dshotBidirMotor_t *motor)
{
    TIM_ICInitTypeDef icInit;

    TIM_ICStructInit(&icInit);
    icInit.TIM_Channel = dshotBidirTimerChannel(motor->tch->timHw->channelIndex);
    icInit.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
    icInit.TIM_ICSelection = TIM_ICSelection_DirectTI;
    icInit.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    icInit.TIM_ICFilter = DSHOT_BIDIR_CAPTURE_FILTER;

    TIM_ICInit(motor->tch->timHw->tim, &icInit);
}

static void dshotBidirDisableBurstTimerCallback(void)
{
    if (dshotBidirCtx.burstTch) {
        timerChConfigCallbacks(dshotBidirCtx.burstTch, (timerCallbacks_t *)&dshotBidirCallbacksDisabled);
    }
}

static void dshotBidirRestoreOutputDma(dshotBidirMotor_t *motor)
{
    if (motor->useInterruptCapture || !motor->dma) {
        return;
    }

    DMA_Cmd(motor->dma->ref, DISABLE);
    TIM_DMACmd(motor->tch->timHw->tim, motor->dmaSource, DISABLE);
    DMA_DeInit(motor->dma->ref);
    DMA_Init(motor->dma->ref, &motor->outputDmaInit);
    DMA_ITConfig(motor->dma->ref, DMA_IT_TC, ENABLE);
}

static void dshotBidirRestoreOutputChannel(dshotBidirMotor_t *motor)
{
    if (!motor->configured) {
        return;
    }

    timerChCaptureDisable(motor->tch);
    if (motor->useInterruptCapture) {
        timerChConfigCallbacks(motor->tch, (timerCallbacks_t *)&dshotBidirCallbacksDisabled);
    }

    if (!dshotBidirCtx.useBurstOutput) {
        dshotBidirRestoreOutputDma(motor);
    }

    dshotBidirConfigureIo(motor);
    dshotBidirConfigureOutputChannel(motor, DSHOT_BIDIR_RUNTIME_IDLE_COMPARE);
    timerPWMStart(motor->tch);
}

static void dshotBidirRestoreBurstDma(void)
{
    if (!dshotBidirCtx.burstDma || !dshotBidirCtx.dmaBurstBuffer) {
        return;
    }

    DMA_Cmd(dshotBidirCtx.burstDma->ref, DISABLE);
    TIM_DMACmd(dshotBidirCtx.timer, dshotBidirCtx.burstRequestSource, DISABLE);
    DMA_DeInit(dshotBidirCtx.burstDma->ref);
    DMA_Init(dshotBidirCtx.burstDma->ref, &dshotBidirCtx.burstOutputDmaInit);
    DMA_ITConfig(dshotBidirCtx.burstDma->ref, DMA_IT_TC, ENABLE);
}

static void dshotBidirStopOutputBurst(void)
{
    if (!dshotBidirCtx.burstDma || !dshotBidirCtx.timer || !dshotBidirCtx.burstTch) {
        return;
    }

    DMA_Cmd(dshotBidirCtx.burstDma->ref, DISABLE);
    TIM_DMACmd(dshotBidirCtx.timer, dshotBidirCtx.burstRequestSource, DISABLE);
    DMA_CLEAR_FLAG(dshotBidirCtx.burstDma, DMA_IT_TCIF);
}

static void dshotBidirStartInputCapture(void)
{
    TIM_TypeDef *timer = dshotBidirCtx.timer;

    if (!timer) {
        dshotBidirCtx.state = DSHOT_BIDIR_STATE_READY;
        return;
    }

    dshotBidirCtx.state = DSHOT_BIDIR_STATE_CAPTURING;
    dshotBidirStopOutputBurst();
    dshotBidirDisableBurstTimerCallback();
    dshotBidirCaptureStartCount++;
    dshotBidirLatchTrace(DSHOT_BIDIR_EVENT_CAPTURE_START, cmpTimeUs(micros(), dshotBidirCtx.frameStartedUs));

    dshotBidirApplyTimerPeriod(0xFFFF);

    for (uint8_t i = 0; i < DSHOT_BIDIR_MOTOR_LIMIT; i++) {
        dshotBidirMotor_t *motor = &dshotBidirMotors[i];
        if (!motor->configured) {
            continue;
        }

        memset(motor->captureBuffer, 0, sizeof(motor->captureBuffer));
        motor->irqEdgeCount = 0;
        dshotBidirConfigureIo(motor);
        dshotBidirConfigureInputCapture(motor);
        timerChClearCaptureAndUpdateFlags(motor->tch);

        if (motor->useInterruptCapture) {
            timerChConfigCallbacks(motor->tch, &motor->irqCallbacks);
        } else {
            DMA_Cmd(motor->dma->ref, DISABLE);
            TIM_DMACmd(timer, motor->dmaSource, DISABLE);
            DMA_CLEAR_FLAG(motor->dma, DMA_IT_TCIF);
            DMA_DeInit(motor->dma->ref);
            DMA_Init(motor->dma->ref, &motor->inputDmaInit);
            DMA_ITConfig(motor->dma->ref, DMA_IT_TC, DISABLE);
            DMA_Cmd(motor->dma->ref, ENABLE);
            TIM_DMACmd(timer, motor->dmaSource, ENABLE);
        }

        timerChCaptureEnable(motor->tch);
    }

    dshotBidirCtx.captureStartedUs = micros();
}

static void dshotBidirRestoreReadyState(void)
{
    if (dshotBidirCtx.timer) {
        dshotBidirApplyTimerPeriod(DSHOT_BIDIR_OUTPUT_PERIOD);
    }

    dshotBidirDisableBurstTimerCallback();
    dshotBidirRestoreBurstDma();

    for (uint8_t i = 0; i < DSHOT_BIDIR_MOTOR_LIMIT; i++) {
        dshotBidirRestoreOutputChannel(&dshotBidirMotors[i]);
    }

    dshotBidirCtx.outputCompleteMask = 0;
    dshotBidirCtx.outputExpectedMask = 0;
    dshotBidirCtx.outputUpdateCount = 0;
    dshotBidirCtx.state = DSHOT_BIDIR_STATE_READY;
}

static uint32_t dshotBidirGetCapturedEdgeCount(const dshotBidirMotor_t *motor)
{
    if (motor->useInterruptCapture) {
        return motor->irqEdgeCount;
    }

    return DSHOT_BIDIR_CAPTURE_BUFFER_LEN - DMA_GetCurrDataCounter(motor->dma->ref);
}

static void dshotBidirStopInputCapture(void)
{
    for (uint8_t i = 0; i < DSHOT_BIDIR_MOTOR_LIMIT; i++) {
        dshotBidirMotor_t *motor = &dshotBidirMotors[i];
        if (!motor->configured) {
            continue;
        }

        timerChCaptureDisable(motor->tch);

        if (motor->useInterruptCapture) {
            timerChConfigCallbacks(motor->tch, (timerCallbacks_t *)&dshotBidirCallbacksDisabled);
            continue;
        }

        DMA_Cmd(motor->dma->ref, DISABLE);
        TIM_DMACmd(dshotBidirCtx.timer, motor->dmaSource, DISABLE);
    }
}

static void dshotBidirDecodeCapture(void)
{
    uint32_t readDelta = 0;
    uint32_t invalidDelta = 0;
    uint32_t noEdgeDelta = 0;

    for (uint8_t i = 0; i < DSHOT_BIDIR_MOTOR_LIMIT; i++) {
        dshotBidirMotor_t *motor = &dshotBidirMotors[i];
        if (!motor->configured) {
            rpmSourceInvalidateDshotBidir(i);
            dshotBidirLastEdgeCount[i] = 0;
            dshotBidirLastRawValue[i] = 0;
            dshotBidirLastErpmValue[i] = 0;
            dshotBidirLastRpmValue[i] = 0;
            continue;
        }

        const uint32_t edgeCount = dshotBidirGetCapturedEdgeCount(motor);
        dshotBidirLastEdgeCount[i] = edgeCount;
        if (edgeCount <= DSHOT_BIDIR_MIN_GCR_EDGES) {
            dshotBidirNoEdgeCount++;
            noEdgeDelta++;
            rpmSourceInvalidateDshotBidir(i);
            dshotBidirLastRawValue[i] = 0;
            dshotBidirLastErpmValue[i] = 0;
            dshotBidirLastRpmValue[i] = 0;
            dshotBidirLatchDecodeTrace(i, DSHOT_BIDIR_DIAG_NOEDGE, NULL, motor->captureBuffer, edgeCount);
            continue;
        }

        dshotBidirReadCount++;
        readDelta++;
        dshotBidirDecodeTrace_t decodeTrace;
        const uint32_t rawValue = dshotBidirDecodeTelemetryPacketEx(motor->captureBuffer, edgeCount, &decodeTrace);
        if (rawValue == DSHOT_BIDIR_INVALID) {
            dshotBidirInvalidPacketCount++;
            invalidDelta++;
            rpmSourceInvalidateDshotBidir(i);
            dshotBidirLastRawValue[i] = 0;
            dshotBidirLastErpmValue[i] = 0;
            dshotBidirLastRpmValue[i] = 0;
            dshotBidirDiag_e diag = DSHOT_BIDIR_DIAG_EMPTY;
            switch ((dshotBidirDecodeStatus_e)decodeTrace.status) {
                case DSHOT_BIDIR_DECODE_STATUS_LENGTH:
                    diag = DSHOT_BIDIR_DIAG_LENGTH;
                    break;
                case DSHOT_BIDIR_DECODE_STATUS_GCR:
                    diag = DSHOT_BIDIR_DIAG_GCR;
                    break;
                case DSHOT_BIDIR_DECODE_STATUS_CHECKSUM:
                    diag = DSHOT_BIDIR_DIAG_CHECKSUM;
                    break;
                case DSHOT_BIDIR_DECODE_STATUS_EMPTY:
                default:
                    diag = DSHOT_BIDIR_DIAG_EMPTY;
                    break;
            }
            dshotBidirLatchDecodeTrace(i, diag, &decodeTrace, motor->captureBuffer, edgeCount);
            continue;
        }

        const uint32_t erpm = dshotBidirDecodeErpmTelemetryValue((uint16_t)rawValue);
        if (erpm == DSHOT_BIDIR_INVALID) {
            dshotBidirInvalidPacketCount++;
            invalidDelta++;
            rpmSourceInvalidateDshotBidir(i);
            dshotBidirLastRawValue[i] = (uint16_t)rawValue;
            dshotBidirLastErpmValue[i] = 0;
            dshotBidirLastRpmValue[i] = 0;
            dshotBidirLatchDecodeTrace(i, DSHOT_BIDIR_DIAG_ERPM, &decodeTrace, motor->captureBuffer, edgeCount);
            continue;
        }

        const uint32_t rpm = dshotBidirComputeRpm(erpm, motorConfig()->motorPoleCount);
        dshotBidirLastRawValue[i] = (uint16_t)rawValue;
        dshotBidirLastErpmValue[i] = erpm;
        dshotBidirLastRpmValue[i] = rpm;
        rpmSourceSetDshotBidirRpm(i, rpm);
        dshotBidirLatchDecodeTrace(i, DSHOT_BIDIR_DIAG_OK, &decodeTrace, motor->captureBuffer, edgeCount);
    }

    dshotBidirUpdateDebug(readDelta, invalidDelta, noEdgeDelta, 0);
    dshotBidirLatchTrace(DSHOT_BIDIR_EVENT_DECODE_COMPLETE, cmpTimeUs(micros(), dshotBidirCtx.captureStartedUs));
}

static void dshotBidirOutputDmaCallback(TCH_t *tch)
{
    if (dshotBidirCtx.useBurstOutput || dshotBidirCtx.state != DSHOT_BIDIR_STATE_OUTPUT_PENDING) {
        return;
    }

    dshotBidirOutputDmaCallbackCount++;

    dshotBidirMotor_t *motor = dshotBidirFindMotorByTch(tch);
    if (!motor) {
        return;
    }

    dshotBidirCtx.outputCompleteMask |= dshotBidirGetMotorBit(motor);
    if (dshotBidirCtx.outputCompleteMask != dshotBidirCtx.outputExpectedMask) {
        return;
    }

    dshotBidirBurstCompleteCount++;
    dshotBidirLastBurstLatencyUs = cmpTimeUs(micros(), dshotBidirCtx.frameStartedUs);
    dshotBidirStartInputCapture();
}

static void dshotBidirBurstTimerOverflowCallback(TCH_t *tch, uint32_t capture)
{
    UNUSED(tch);
    UNUSED(capture);

    if (!dshotBidirCtx.useBurstOutput || dshotBidirCtx.state != DSHOT_BIDIR_STATE_OUTPUT_PENDING) {
        return;
    }

    dshotBidirOverflowCallbackCount++;
    if (++dshotBidirCtx.outputUpdateCount < DSHOT_BIDIR_OUTPUT_COMPLETE_UPDATES) {
        return;
    }

    dshotBidirBurstCompleteCount++;
    dshotBidirLastBurstLatencyUs = cmpTimeUs(micros(), dshotBidirCtx.frameStartedUs);
    dshotBidirStartInputCapture();
}

static void dshotBidirBurstDmaHandler(DMA_t descriptor)
{
    if (!DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF)) {
        return;
    }

    dshotBidirOutputDmaCallbackCount++;
    DMA_Cmd(descriptor->ref, DISABLE);
    TIM_DMACmd(dshotBidirCtx.timer, dshotBidirCtx.burstRequestSource, DISABLE);
    DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);

    if (!dshotBidirCtx.useBurstOutput || dshotBidirCtx.state != DSHOT_BIDIR_STATE_OUTPUT_PENDING) {
        return;
    }

    dshotBidirWaitCaptureStartDelay();
    dshotBidirBurstCompleteCount++;
    dshotBidirLastBurstLatencyUs = cmpTimeUs(micros(), dshotBidirCtx.frameStartedUs);
    dshotBidirStartInputCapture();
}

void dshotBidirInit(uint32_t dshotHz)
{
    memset(&dshotBidirCtx, 0, sizeof(dshotBidirCtx));
    memset(dshotBidirMotors, 0, sizeof(dshotBidirMotors));
    memset(dshotBidirLastEdgeCount, 0, sizeof(dshotBidirLastEdgeCount));
    memset(dshotBidirLastRawValue, 0, sizeof(dshotBidirLastRawValue));
    memset(dshotBidirLastErpmValue, 0, sizeof(dshotBidirLastErpmValue));
    memset(dshotBidirLastRpmValue, 0, sizeof(dshotBidirLastRpmValue));
    dshotBidirReadCount = 0;
    dshotBidirInvalidPacketCount = 0;
    dshotBidirNoEdgeCount = 0;
    dshotBidirTimeoutCount = 0;
    dshotBidirBurstCompleteCount = 0;
    dshotBidirCaptureStartCount = 0;
    dshotBidirLastBurstLatencyUs = 0;
    dshotBidirOverflowCallbackCount = 0;
    dshotBidirOutputDmaCallbackCount = 0;
    dshotBidirLastEventElapsedUs = 0;
    dshotBidirLastEvent = DSHOT_BIDIR_EVENT_NONE;
    dshotBidirLastOutputUpdateCount = 0;
    dshotBidirLastOutputExpectedMask = 0;
    dshotBidirLastOutputCompleteMask = 0;
    dshotBidirLastTimerArr = 0;
    dshotBidirLastTimerCnt = 0;
    dshotBidirLastDecodeMotor = DSHOT_BIDIR_MOTOR_LIMIT;
    dshotBidirLastDecodeStatus = DSHOT_BIDIR_DIAG_NONE;
    dshotBidirLastDecodeEdgeCount = 0;
    dshotBidirLastDecodeGcrValue = 0;
    dshotBidirLastDecodeWord = 0;
    memset(dshotBidirLastDecodeDeltas, 0, sizeof(dshotBidirLastDecodeDeltas));
    memset(dshotBidirMotorReadCount, 0, sizeof(dshotBidirMotorReadCount));
    memset(dshotBidirMotorInvalidCount, 0, sizeof(dshotBidirMotorInvalidCount));
    memset(dshotBidirMotorNoEdgeCount, 0, sizeof(dshotBidirMotorNoEdgeCount));
    memset(dshotBidirMotorIrqEdgeCallbackCount, 0, sizeof(dshotBidirMotorIrqEdgeCallbackCount));
    memset(dshotBidirMotorLastInterestingDiag, 0, sizeof(dshotBidirMotorLastInterestingDiag));
    memset(dshotBidirMotorLastInterestingEdgeCount, 0, sizeof(dshotBidirMotorLastInterestingEdgeCount));
    memset(dshotBidirMotorLastInterestingRawValue, 0, sizeof(dshotBidirMotorLastInterestingRawValue));
    memset(dshotBidirMotorLastInterestingErpmValue, 0, sizeof(dshotBidirMotorLastInterestingErpmValue));
    memset(dshotBidirMotorLastInterestingRpmValue, 0, sizeof(dshotBidirMotorLastInterestingRpmValue));

    dshotBidirCtx.outputWindowUs = dshotBidirComputeOutputWindowUs(dshotHz);
    dshotBidirCtx.captureWindowUs = dshotBidirComputeCaptureWindowUs(dshotHz);
    dshotBidirCtx.state = DSHOT_BIDIR_STATE_READY;
    dshotBidirUpdateDebug(0, 0, 0, 0);
}

bool dshotBidirAttachMotor(uint8_t motorIndex, TCH_t *tch, void *dmaBurstBuffer, void *dmaOutputBuffer)
{
    if (motorIndex >= DSHOT_BIDIR_MOTOR_LIMIT || !tch || tch->timHw->tim != TIM4) {
        return false;
    }

    dshotBidirMotor_t *motor = &dshotBidirMotors[motorIndex];

    motor->tch = tch;
    motor->dmaSource = dshotBidirTimerDmaSource(tch->timHw->channelIndex);
    motor->dma = (tch->timHw->dmaTag != DMA_NONE) ? dmaGetByTag(tch->timHw->dmaTag) : NULL;
    motor->useInterruptCapture = (motor->dma == NULL);
    motor->irqCallbacks.callbackEdge = dshotBidirIrqCaptureCallback;

    if (!motor->useInterruptCapture) {
        if (!dmaOutputBuffer) {
            return false;
        }

        if (dmaBurstBuffer && motor->dma != tch->timCtx->dmaBurstRef) {
            if (dmaGetOwner(motor->dma) != OWNER_FREE) {
                return false;
            }

            dmaInit(motor->dma, OWNER_TIMER, motorIndex);
        }

        DMA_StructInit(&motor->outputDmaInit);
        motor->outputDmaInit.DMA_Channel = dmaGetChannelByTag(tch->timHw->dmaTag);
        motor->outputDmaInit.DMA_PeripheralBaseAddr = (uint32_t)timerCCR(tch);
        motor->outputDmaInit.DMA_Memory0BaseAddr = (uint32_t)dmaOutputBuffer;
        motor->outputDmaInit.DMA_DIR = DMA_DIR_MemoryToPeripheral;
        motor->outputDmaInit.DMA_BufferSize = DSHOT_BIDIR_OUTPUT_BUFFER_LEN;
        motor->outputDmaInit.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        motor->outputDmaInit.DMA_MemoryInc = DMA_MemoryInc_Enable;
        motor->outputDmaInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
        motor->outputDmaInit.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
        motor->outputDmaInit.DMA_Mode = DMA_Mode_Normal;
        motor->outputDmaInit.DMA_Priority = DMA_Priority_High;

        DMA_StructInit(&motor->inputDmaInit);
        motor->inputDmaInit.DMA_Channel = dmaGetChannelByTag(tch->timHw->dmaTag);
        motor->inputDmaInit.DMA_PeripheralBaseAddr = (uint32_t)timerCCR(tch);
        motor->inputDmaInit.DMA_Memory0BaseAddr = (uint32_t)motor->captureBuffer;
        motor->inputDmaInit.DMA_DIR = DMA_DIR_PeripheralToMemory;
        motor->inputDmaInit.DMA_BufferSize = DSHOT_BIDIR_CAPTURE_BUFFER_LEN;
        motor->inputDmaInit.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        motor->inputDmaInit.DMA_MemoryInc = DMA_MemoryInc_Enable;
        motor->inputDmaInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
        motor->inputDmaInit.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
        motor->inputDmaInit.DMA_Mode = DMA_Mode_Normal;
        motor->inputDmaInit.DMA_Priority = DMA_Priority_High;
    }

    if (!dshotBidirCtx.timer) {
        dshotBidirCtx.timer = tch->timHw->tim;
    }

    if (!dshotBidirCtx.burstDma && tch->timCtx->dmaBurstRef && dmaBurstBuffer) {
        dshotBidirCtx.burstDma = tch->timCtx->dmaBurstRef;
        dshotBidirCtx.burstTch = tch;
        dshotBidirCtx.dmaBurstBuffer = (timerDMASafeType_t *)dmaBurstBuffer;
        dshotBidirCtx.burstRequestSource = tch->timCtx->DMASource;
        dshotBidirCtx.statusRequestSource = tch->timCtx->DMASource;
        dshotBidirCtx.useBurstOutput = true;
        timerChInitCallbacks(&dshotBidirCtx.burstTimerCallbacks, NULL, NULL, dshotBidirBurstTimerOverflowCallback);

        DMA_StructInit(&dshotBidirCtx.burstOutputDmaInit);
        dshotBidirCtx.burstOutputDmaInit.DMA_Channel = dshotBidirBurstDmaChannel(tch);
        dshotBidirCtx.burstOutputDmaInit.DMA_PeripheralBaseAddr = (uint32_t)&tch->timHw->tim->DMAR;
        dshotBidirCtx.burstOutputDmaInit.DMA_Memory0BaseAddr = (uint32_t)dmaBurstBuffer;
        dshotBidirCtx.burstOutputDmaInit.DMA_DIR = DMA_DIR_MemoryToPeripheral;
        dshotBidirCtx.burstOutputDmaInit.DMA_BufferSize = DSHOT_BIDIR_BURST_LENGTH;
        dshotBidirCtx.burstOutputDmaInit.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        dshotBidirCtx.burstOutputDmaInit.DMA_MemoryInc = DMA_MemoryInc_Enable;
        dshotBidirCtx.burstOutputDmaInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
        dshotBidirCtx.burstOutputDmaInit.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
        dshotBidirCtx.burstOutputDmaInit.DMA_Mode = DMA_Mode_Normal;
        dshotBidirCtx.burstOutputDmaInit.DMA_Priority = DMA_Priority_High;

        dmaSetHandler(dshotBidirCtx.burstDma, dshotBidirBurstDmaHandler, NVIC_PRIO_TIMER_DMA, 0);
        dshotBidirCtx.burstHandlerInstalled = true;
    } else if (!motor->useInterruptCapture && getMotorCount() <= 3) {
        timerChConfigDmaCallback(motor->tch, dshotBidirOutputDmaCallback);
        dshotBidirCtx.useBurstOutput = false;
    }

    motor->configured = true;
    dshotBidirConfigureIo(motor);

    TIM_OCStructInit(&motor->outputInit);
    motor->outputInit.TIM_OCMode = TIM_OCMode_PWM1;
    if (motor->tch->timHw->output & TIMER_OUTPUT_N_CHANNEL) {
        motor->outputInit.TIM_OutputState = TIM_OutputState_Disable;
        motor->outputInit.TIM_OutputNState = TIM_OutputNState_Enable;
        motor->outputInit.TIM_OCNPolarity = dshotBidirWireOutputInverted(motor) ? TIM_OCPolarity_Low : TIM_OCPolarity_High;
        motor->outputInit.TIM_OCNIdleState = TIM_OCIdleState_Reset;
    } else {
        motor->outputInit.TIM_OutputState = TIM_OutputState_Enable;
        motor->outputInit.TIM_OutputNState = TIM_OutputNState_Disable;
        motor->outputInit.TIM_OCPolarity = dshotBidirWireOutputInverted(motor) ? TIM_OCPolarity_Low : TIM_OCPolarity_High;
        motor->outputInit.TIM_OCIdleState = TIM_OCIdleState_Set;
    }

    TIM_ICStructInit(&motor->inputCaptureInit);
    motor->inputCaptureInit.TIM_Channel = dshotBidirTimerChannel(tch->timHw->channelIndex);
    motor->inputCaptureInit.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
    motor->inputCaptureInit.TIM_ICSelection = TIM_ICSelection_DirectTI;
    motor->inputCaptureInit.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    motor->inputCaptureInit.TIM_ICFilter = DSHOT_BIDIR_CAPTURE_FILTER;

    dshotBidirConfigureOutputChannel(motor, DSHOT_BIDIR_STARTUP_COMPARE_VALUE);
    timerPWMStart(motor->tch);
    return true;
}

bool dshotBidirIsReady(void)
{
    return dshotBidirCtx.useBurstOutput ? dshotBidirCtx.burstHandlerInstalled : (dshotBidirCtx.timer != NULL);
}

bool dshotBidirUsesBurstOutput(void)
{
    return dshotBidirCtx.useBurstOutput;
}

bool dshotBidirUpdate(void)
{
    const timeUs_t now = micros();

    if (dshotBidirCtx.state == DSHOT_BIDIR_STATE_OUTPUT_PENDING) {
        const timeDelta_t elapsed = cmpTimeUs(now, dshotBidirCtx.frameStartedUs);
        const uint32_t pendingTimeoutUs = (dshotBidirCtx.outputWindowUs * 4U > DSHOT_BIDIR_PENDING_TIMEOUT_US) ?
            (dshotBidirCtx.outputWindowUs * 4U) :
            DSHOT_BIDIR_PENDING_TIMEOUT_US;

        if (elapsed < (timeDelta_t)pendingTimeoutUs) {
            return false;
        }

        dshotBidirTimeoutCount++;
        dshotBidirLatchTrace(DSHOT_BIDIR_EVENT_TIMEOUT, (uint32_t)elapsed);
        dshotBidirUpdateDebug(0, 0, 0, 1);
        dshotBidirRestoreReadyState();
        return true;
    }

    if (dshotBidirCtx.state != DSHOT_BIDIR_STATE_CAPTURING) {
        return true;
    }

    if (cmpTimeUs(now, dshotBidirCtx.captureStartedUs) < (timeDelta_t)dshotBidirCtx.captureWindowUs) {
        return false;
    }

    dshotBidirStopInputCapture();
    dshotBidirDecodeCapture();
    dshotBidirRestoreReadyState();
    return true;
}

void dshotBidirOnFrameStarted(void)
{
    dshotBidirCtx.frameStartedUs = micros();
    dshotBidirCtx.outputCompleteMask = 0;
    dshotBidirCtx.outputExpectedMask = 0;
    dshotBidirCtx.outputUpdateCount = 0;
    if (dshotBidirCtx.useBurstOutput) {
        if (dshotBidirCtx.burstTch) {
            timerChConfigCallbacks(dshotBidirCtx.burstTch, &dshotBidirCtx.burstTimerCallbacks);
        }
    } else {
        dshotBidirCtx.statusRequestSource = 0;
        for (uint8_t i = 0; i < DSHOT_BIDIR_MOTOR_LIMIT; i++) {
            const dshotBidirMotor_t *motor = &dshotBidirMotors[i];
            if (!motor->configured) {
                continue;
            }

            dshotBidirCtx.outputExpectedMask |= dshotBidirGetMotorBit(motor);
            dshotBidirCtx.statusRequestSource |= motor->dmaSource;
        }
    }
    dshotBidirCtx.state = DSHOT_BIDIR_STATE_OUTPUT_PENDING;
}

uint32_t dshotBidirGetReadCount(void)
{
    return dshotBidirReadCount;
}

uint32_t dshotBidirGetInvalidPacketCount(void)
{
    return dshotBidirInvalidPacketCount;
}

uint32_t dshotBidirGetNoEdgeCount(void)
{
    return dshotBidirNoEdgeCount;
}

uint32_t dshotBidirGetTimeoutCount(void)
{
    return dshotBidirTimeoutCount;
}

uint32_t dshotBidirGetBurstCompleteCount(void)
{
    return dshotBidirBurstCompleteCount;
}

uint32_t dshotBidirGetCaptureStartCount(void)
{
    return dshotBidirCaptureStartCount;
}

uint8_t dshotBidirGetState(void)
{
    return (uint8_t)dshotBidirCtx.state;
}

uint16_t dshotBidirGetBurstRequestSource(void)
{
    return dshotBidirCtx.burstRequestSource;
}

uint32_t dshotBidirGetLastBurstLatencyUs(void)
{
    return dshotBidirLastBurstLatencyUs;
}

uint8_t dshotBidirGetLastEvent(void)
{
    return dshotBidirLastEvent;
}

uint32_t dshotBidirGetLastEventElapsedUs(void)
{
    return dshotBidirLastEventElapsedUs;
}

uint32_t dshotBidirGetOverflowCallbackCount(void)
{
    return dshotBidirOverflowCallbackCount;
}

uint32_t dshotBidirGetOutputDmaCallbackCount(void)
{
    return dshotBidirOutputDmaCallbackCount;
}

uint8_t dshotBidirGetLastOutputUpdateCount(void)
{
    return dshotBidirLastOutputUpdateCount;
}

uint8_t dshotBidirGetLastOutputExpectedMask(void)
{
    return dshotBidirLastOutputExpectedMask;
}

uint8_t dshotBidirGetLastOutputCompleteMask(void)
{
    return dshotBidirLastOutputCompleteMask;
}

uint32_t dshotBidirGetLastTimerArr(void)
{
    return dshotBidirLastTimerArr;
}

uint32_t dshotBidirGetLastTimerCnt(void)
{
    return dshotBidirLastTimerCnt;
}

uint8_t dshotBidirGetOutputUpdateCount(void)
{
    return dshotBidirCtx.outputUpdateCount;
}

uint8_t dshotBidirGetOutputExpectedMask(void)
{
    return dshotBidirCtx.outputExpectedMask;
}

uint8_t dshotBidirGetOutputCompleteMask(void)
{
    return dshotBidirCtx.outputCompleteMask;
}

uint32_t dshotBidirGetOutputWindowUs(void)
{
    return dshotBidirCtx.outputWindowUs;
}

uint32_t dshotBidirGetCaptureWindowUs(void)
{
    return dshotBidirCtx.captureWindowUs;
}

uint8_t dshotBidirGetLastEdgeCount(uint8_t motorIndex)
{
    if (motorIndex >= DSHOT_BIDIR_MOTOR_LIMIT) {
        return 0;
    }

    return dshotBidirLastEdgeCount[motorIndex];
}

uint16_t dshotBidirGetLastRawValue(uint8_t motorIndex)
{
    if (motorIndex >= DSHOT_BIDIR_MOTOR_LIMIT) {
        return 0;
    }

    return dshotBidirLastRawValue[motorIndex];
}

uint32_t dshotBidirGetLastErpmValue(uint8_t motorIndex)
{
    if (motorIndex >= DSHOT_BIDIR_MOTOR_LIMIT) {
        return 0;
    }

    return dshotBidirLastErpmValue[motorIndex];
}

uint32_t dshotBidirGetLastRpmValue(uint8_t motorIndex)
{
    if (motorIndex >= DSHOT_BIDIR_MOTOR_LIMIT) {
        return 0;
    }

    return dshotBidirLastRpmValue[motorIndex];
}

uint32_t dshotBidirGetMotorReadCount(uint8_t motorIndex)
{
    if (motorIndex >= DSHOT_BIDIR_MOTOR_LIMIT) {
        return 0;
    }

    return dshotBidirMotorReadCount[motorIndex];
}

uint32_t dshotBidirGetMotorInvalidCount(uint8_t motorIndex)
{
    if (motorIndex >= DSHOT_BIDIR_MOTOR_LIMIT) {
        return 0;
    }

    return dshotBidirMotorInvalidCount[motorIndex];
}

uint32_t dshotBidirGetMotorNoEdgeCount(uint8_t motorIndex)
{
    if (motorIndex >= DSHOT_BIDIR_MOTOR_LIMIT) {
        return 0;
    }

    return dshotBidirMotorNoEdgeCount[motorIndex];
}

uint32_t dshotBidirGetMotorIrqEdgeCallbackCount(uint8_t motorIndex)
{
    if (motorIndex >= DSHOT_BIDIR_MOTOR_LIMIT) {
        return 0;
    }

    return dshotBidirMotorIrqEdgeCallbackCount[motorIndex];
}

uint8_t dshotBidirGetMotorLastInterestingDiag(uint8_t motorIndex)
{
    if (motorIndex >= DSHOT_BIDIR_MOTOR_LIMIT) {
        return 0;
    }

    return dshotBidirMotorLastInterestingDiag[motorIndex];
}

uint8_t dshotBidirGetMotorLastInterestingEdgeCount(uint8_t motorIndex)
{
    if (motorIndex >= DSHOT_BIDIR_MOTOR_LIMIT) {
        return 0;
    }

    return dshotBidirMotorLastInterestingEdgeCount[motorIndex];
}

uint16_t dshotBidirGetMotorLastInterestingRawValue(uint8_t motorIndex)
{
    if (motorIndex >= DSHOT_BIDIR_MOTOR_LIMIT) {
        return 0;
    }

    return dshotBidirMotorLastInterestingRawValue[motorIndex];
}

uint32_t dshotBidirGetMotorLastInterestingErpmValue(uint8_t motorIndex)
{
    if (motorIndex >= DSHOT_BIDIR_MOTOR_LIMIT) {
        return 0;
    }

    return dshotBidirMotorLastInterestingErpmValue[motorIndex];
}

uint32_t dshotBidirGetMotorLastInterestingRpmValue(uint8_t motorIndex)
{
    if (motorIndex >= DSHOT_BIDIR_MOTOR_LIMIT) {
        return 0;
    }

    return dshotBidirMotorLastInterestingRpmValue[motorIndex];
}

uint8_t dshotBidirGetLastDecodeMotor(void)
{
    return dshotBidirLastDecodeMotor;
}

uint8_t dshotBidirGetLastDecodeStatus(void)
{
    return dshotBidirLastDecodeStatus;
}

uint8_t dshotBidirGetLastDecodeEdgeCount(void)
{
    return dshotBidirLastDecodeEdgeCount;
}

uint32_t dshotBidirGetLastDecodeGcrValue(void)
{
    return dshotBidirLastDecodeGcrValue;
}

uint16_t dshotBidirGetLastDecodeWord(void)
{
    return dshotBidirLastDecodeWord;
}

uint16_t dshotBidirGetLastDecodeDelta(uint8_t deltaIndex)
{
    if (deltaIndex >= ARRAYLEN(dshotBidirLastDecodeDeltas)) {
        return 0;
    }

    return dshotBidirLastDecodeDeltas[deltaIndex];
}

#else

void dshotBidirInit(uint32_t dshotHz)
{
    UNUSED(dshotHz);
}

bool dshotBidirAttachMotor(uint8_t motorIndex, TCH_t *tch, void *dmaBurstBuffer, void *dmaOutputBuffer)
{
    UNUSED(motorIndex);
    UNUSED(tch);
    UNUSED(dmaBurstBuffer);
    UNUSED(dmaOutputBuffer);
    return false;
}

bool dshotBidirIsReady(void)
{
    return false;
}

bool dshotBidirUsesBurstOutput(void)
{
    return false;
}

bool dshotBidirUpdate(void)
{
    return true;
}

void dshotBidirOnFrameStarted(void)
{
}

uint32_t dshotBidirGetReadCount(void)
{
    return 0;
}

uint32_t dshotBidirGetInvalidPacketCount(void)
{
    return 0;
}

uint32_t dshotBidirGetNoEdgeCount(void)
{
    return 0;
}

uint32_t dshotBidirGetTimeoutCount(void)
{
    return 0;
}

uint32_t dshotBidirGetBurstCompleteCount(void)
{
    return 0;
}

uint32_t dshotBidirGetCaptureStartCount(void)
{
    return 0;
}

uint8_t dshotBidirGetState(void)
{
    return 0;
}

uint16_t dshotBidirGetBurstRequestSource(void)
{
    return 0;
}

uint32_t dshotBidirGetLastBurstLatencyUs(void)
{
    return 0;
}

uint8_t dshotBidirGetLastEvent(void)
{
    return 0;
}

uint32_t dshotBidirGetLastEventElapsedUs(void)
{
    return 0;
}

uint32_t dshotBidirGetOverflowCallbackCount(void)
{
    return 0;
}

uint32_t dshotBidirGetOutputDmaCallbackCount(void)
{
    return 0;
}

uint8_t dshotBidirGetLastOutputUpdateCount(void)
{
    return 0;
}

uint8_t dshotBidirGetLastOutputExpectedMask(void)
{
    return 0;
}

uint8_t dshotBidirGetLastOutputCompleteMask(void)
{
    return 0;
}

uint32_t dshotBidirGetLastTimerArr(void)
{
    return 0;
}

uint32_t dshotBidirGetLastTimerCnt(void)
{
    return 0;
}

uint8_t dshotBidirGetOutputUpdateCount(void)
{
    return 0;
}

uint8_t dshotBidirGetOutputExpectedMask(void)
{
    return 0;
}

uint8_t dshotBidirGetOutputCompleteMask(void)
{
    return 0;
}

uint32_t dshotBidirGetOutputWindowUs(void)
{
    return 0;
}

uint32_t dshotBidirGetCaptureWindowUs(void)
{
    return 0;
}

uint8_t dshotBidirGetLastEdgeCount(uint8_t motorIndex)
{
    UNUSED(motorIndex);
    return 0;
}

uint16_t dshotBidirGetLastRawValue(uint8_t motorIndex)
{
    UNUSED(motorIndex);
    return 0;
}

uint32_t dshotBidirGetLastErpmValue(uint8_t motorIndex)
{
    UNUSED(motorIndex);
    return 0;
}

uint32_t dshotBidirGetLastRpmValue(uint8_t motorIndex)
{
    UNUSED(motorIndex);
    return 0;
}

uint32_t dshotBidirGetMotorReadCount(uint8_t motorIndex)
{
    UNUSED(motorIndex);
    return 0;
}

uint32_t dshotBidirGetMotorInvalidCount(uint8_t motorIndex)
{
    UNUSED(motorIndex);
    return 0;
}

uint32_t dshotBidirGetMotorNoEdgeCount(uint8_t motorIndex)
{
    UNUSED(motorIndex);
    return 0;
}

uint32_t dshotBidirGetMotorIrqEdgeCallbackCount(uint8_t motorIndex)
{
    UNUSED(motorIndex);
    return 0;
}

uint8_t dshotBidirGetMotorLastInterestingDiag(uint8_t motorIndex)
{
    UNUSED(motorIndex);
    return 0;
}

uint8_t dshotBidirGetMotorLastInterestingEdgeCount(uint8_t motorIndex)
{
    UNUSED(motorIndex);
    return 0;
}

uint16_t dshotBidirGetMotorLastInterestingRawValue(uint8_t motorIndex)
{
    UNUSED(motorIndex);
    return 0;
}

uint32_t dshotBidirGetMotorLastInterestingErpmValue(uint8_t motorIndex)
{
    UNUSED(motorIndex);
    return 0;
}

uint32_t dshotBidirGetMotorLastInterestingRpmValue(uint8_t motorIndex)
{
    UNUSED(motorIndex);
    return 0;
}

#endif
