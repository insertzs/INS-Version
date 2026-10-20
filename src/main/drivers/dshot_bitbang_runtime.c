#include "platform.h"

#if !defined(UNIT_TEST) && defined(STM32F4)

#include <string.h>

#include "build/atomic.h"
#include "build/debug.h"

#include "common/maths.h"

#include "drivers/dma.h"
#include "drivers/dshot_bidir_utils.h"
#include "drivers/dshot_bitbang.h"
#include "drivers/dshot_bitbang_decode.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/nvic.h"
#include "drivers/resource.h"
#include "drivers/time.h"

#include "fc/config.h"

#include "flight/mixer.h"

#include "sensors/rpm_source.h"

#include "drivers/dshot_bitbang_runtime.h"

#define DSHOT_BITBANG_MAX_MOTORS 4U
#define DSHOT_BITBANG_INPUT_BUFFER_LENGTH 140U
#define DSHOT_BITBANG_TELEMETRY_TIMEOUT_US 2000U
#define DSHOT_BITBANG_MIN_TIMER_ARR 16U

typedef enum {
    DSHOT_BITBANG_STATE_DISABLED = 0,
    DSHOT_BITBANG_STATE_OUTPUT,
    DSHOT_BITBANG_STATE_INPUT,
} dshotBitbangState_e;

typedef struct {
    bool configured;
    IO_t io;
    ioTag_t tag;
    uint8_t pinIndex;
} dshotBitbangMotor_t;

typedef struct {
    bool initialized;
    bool configured;
    bool suspended;
    uint8_t configuredMotorCount;
    const timerHardware_t *pacerTimHw;
    TCH_t *pacerTch;
    DMA_t dma;
    GPIO_TypeDef *gpio;
    uint16_t dmaSource;
    uint16_t portMask;
    uint32_t gpioModeMask;
    uint32_t gpioModeInput;
    uint32_t gpioModeOutput;
    uint32_t gpioIdleBSRR;
    uint16_t outputARR;
    uint16_t inputARR;
    volatile dshotBitbangState_e state;
    volatile bool telemetryPending;
    volatile bool outputPending;
    timeUs_t frameStartedUs;
    timeUs_t captureStartedUs;

    DMA_InitTypeDef outputDmaInit;
    DMA_InitTypeDef inputDmaInit;
    uint32_t outputBuffer[DSHOT_BITBANG_OUTPUT_BUFFER_LENGTH];
    uint16_t inputBuffer[DSHOT_BITBANG_INPUT_BUFFER_LENGTH];

    uint32_t readCount;
    uint32_t invalidPacketCount;
    uint32_t noEdgeCount;
    uint32_t timeoutCount;
    uint16_t lastRawValue[DSHOT_BITBANG_MAX_MOTORS];
    uint32_t lastErpmValue[DSHOT_BITBANG_MAX_MOTORS];
    uint32_t lastRpmValue[DSHOT_BITBANG_MAX_MOTORS];
    uint32_t motorReadCount[DSHOT_BITBANG_MAX_MOTORS];
    uint32_t motorInvalidCount[DSHOT_BITBANG_MAX_MOTORS];
    uint32_t motorNoEdgeCount[DSHOT_BITBANG_MAX_MOTORS];
    uint32_t outputCallbackCount;
    uint32_t inputCallbackCount;
    uint32_t outputOnlyFrameCount;
    uint32_t telemetryFrameCount;
    uint32_t lastLatencyUs;
    uint32_t lastSampleCount;
    uint8_t lastDecodeMotor;
    uint8_t lastDecodeStatus;
} dshotBitbangRuntimeCtx_t;

static dshotBitbangRuntimeCtx_t dshotBitbangCtx;
static dshotBitbangMotor_t dshotBitbangMotors[DSHOT_BITBANG_MAX_MOTORS];

static bool dshotBitbangComputeTimerArr(uint32_t timerHz, uint32_t targetHz, uint16_t *arr)
{
    if (targetHz == 0U) {
        return false;
    }

    const uint32_t period = timerHz / targetHz;
    if (period == 0U) {
        return false;
    }

    const uint32_t candidateArr = period - 1U;
    if (candidateArr < DSHOT_BITBANG_MIN_TIMER_ARR || candidateArr > 0xffffU) {
        return false;
    }

    *arr = (uint16_t)candidateArr;
    return true;
}

static uint16_t dshotBitbangPreparePacket(uint16_t value, bool requestTelemetry)
{
    uint16_t packet = (value << 1) | (requestTelemetry ? 1U : 0U);
    int csum = 0;
    int csumData = packet;

    for (int i = 0; i < 3; i++) {
        csum ^= csumData;
        csumData >>= 4;
    }

    csum = (~csum) & 0xF;
    packet = (packet << 4) | (uint16_t)csum;
    return packet;
}

static uint16_t dshotBitbangPacerDmaSource(uint8_t channelIndex)
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

static const timerHardware_t *dshotBitbangFindPacerHardware(void)
{
    for (int i = 0; i < timerHardwareCount; i++) {
        if (timerHardware[i].tim == TIM1 && timerHardware[i].channelIndex == 0) {
            return &timerHardware[i];
        }
    }

    return NULL;
}

static void dshotBitbangUpdateDebug(uint32_t readDelta, uint32_t invalidDelta, uint32_t noEdgeDelta, uint32_t timeoutDelta)
{
    DEBUG_SET(DEBUG_DSHOT_BIDIR, 0, dshotBitbangCtx.lastRpmValue[0]);
    DEBUG_SET(DEBUG_DSHOT_BIDIR, 1, dshotBitbangCtx.lastRpmValue[1]);
    DEBUG_SET(DEBUG_DSHOT_BIDIR, 2, dshotBitbangCtx.lastRpmValue[2]);
    DEBUG_SET(DEBUG_DSHOT_BIDIR, 3, dshotBitbangCtx.lastRpmValue[3]);
    DEBUG_SET(DEBUG_DSHOT_BIDIR, 4, readDelta);
    DEBUG_SET(DEBUG_DSHOT_BIDIR, 5, invalidDelta);
    DEBUG_SET(DEBUG_DSHOT_BIDIR, 6, noEdgeDelta);
    DEBUG_SET(DEBUG_DSHOT_BIDIR, 7, timeoutDelta);
}

static void dshotBitbangApplyDmaConfig(const DMA_InitTypeDef *init)
{
    DMA_Cmd(dshotBitbangCtx.dma->ref, DISABLE);
    DMA_DeInit(dshotBitbangCtx.dma->ref);
    DMA_Init(dshotBitbangCtx.dma->ref, (DMA_InitTypeDef *)init);
    DMA_ITConfig(dshotBitbangCtx.dma->ref, DMA_IT_TC | DMA_IT_TE | DMA_IT_DME | DMA_IT_FE, ENABLE);
    DMA_CLEAR_FLAG(dshotBitbangCtx.dma, DMA_IT_TCIF | DMA_IT_TEIF | DMA_IT_DMEIF | DMA_IT_FEIF);
}

static void dshotBitbangSwitchToOutput(void)
{
#if defined(STM32F4)
    dshotBitbangCtx.gpio->BSRRL = (uint16_t)dshotBitbangCtx.gpioIdleBSRR;
    dshotBitbangCtx.gpio->BSRRH = (uint16_t)(dshotBitbangCtx.gpioIdleBSRR >> 16);
#else
    WRITE_REG(dshotBitbangCtx.gpio->BSRR, dshotBitbangCtx.gpioIdleBSRR);
#endif

    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        MODIFY_REG(dshotBitbangCtx.gpio->MODER, dshotBitbangCtx.gpioModeMask, dshotBitbangCtx.gpioModeOutput);
    }

    dshotBitbangApplyDmaConfig(&dshotBitbangCtx.outputDmaInit);
    dshotBitbangCtx.pacerTimHw->tim->CNT = 0;
    dshotBitbangCtx.pacerTimHw->tim->ARR = dshotBitbangCtx.outputARR;
    dshotBitbangCtx.state = DSHOT_BITBANG_STATE_OUTPUT;
}

static void dshotBitbangSwitchToInput(void)
{
    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        MODIFY_REG(dshotBitbangCtx.gpio->MODER, dshotBitbangCtx.gpioModeMask, dshotBitbangCtx.gpioModeInput);
    }

    dshotBitbangApplyDmaConfig(&dshotBitbangCtx.inputDmaInit);
    dshotBitbangCtx.pacerTimHw->tim->CNT = 0;
    dshotBitbangCtx.pacerTimHw->tim->ARR = dshotBitbangCtx.inputARR;
    DMA_Cmd(dshotBitbangCtx.dma->ref, ENABLE);
    dshotBitbangCtx.captureStartedUs = micros();
    dshotBitbangCtx.state = DSHOT_BITBANG_STATE_INPUT;
}

static void dshotBitbangDmaHandler(DMA_t descriptor)
{
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TEIF | DMA_IT_DMEIF | DMA_IT_FEIF)) {
        // Never leave an asserted error interrupt spinning above USB/MSP priority.
        TIM_DMACmd(dshotBitbangCtx.pacerTimHw->tim, dshotBitbangCtx.dmaSource, DISABLE);
        DMA_Cmd(descriptor->ref, DISABLE);
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF | DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_DMEIF | DMA_IT_FEIF);
        dshotBitbangCtx.timeoutCount++;
        dshotBitbangCtx.telemetryPending = false;
        dshotBitbangCtx.outputPending = false;
        dshotBitbangCtx.state = DSHOT_BITBANG_STATE_DISABLED;
        return;
    }
    if (!DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF)) {
        return;
    }

    DMA_Cmd(descriptor->ref, DISABLE);
    TIM_DMACmd(dshotBitbangCtx.pacerTimHw->tim, dshotBitbangCtx.dmaSource, DISABLE);
    DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF | DMA_IT_TEIF | DMA_IT_DMEIF | DMA_IT_FEIF);

    if (dshotBitbangCtx.state == DSHOT_BITBANG_STATE_OUTPUT) {
        dshotBitbangCtx.outputCallbackCount++;
        dshotBitbangCtx.lastLatencyUs = micros() - dshotBitbangCtx.frameStartedUs;

        // Every inverted-checksum frame permits an eRPM reply, including zero
        // throttle and packets whose separate telemetry-request bit is clear.
        dshotBitbangCtx.telemetryFrameCount++;
        dshotBitbangSwitchToInput();
        dshotBitbangCtx.telemetryPending = true;
        dshotBitbangCtx.outputPending = false;
        TIM_DMACmd(dshotBitbangCtx.pacerTimHw->tim, dshotBitbangCtx.dmaSource, ENABLE);
    } else if (dshotBitbangCtx.state == DSHOT_BITBANG_STATE_INPUT) {
        dshotBitbangCtx.inputCallbackCount++;
        dshotBitbangCtx.telemetryPending = false;
    }
}

static void dshotBitbangInitPacerChannel(void)
{
    TIM_OCInitTypeDef outputInit;
    TIM_OCStructInit(&outputInit);
    outputInit.TIM_OCMode = TIM_OCMode_PWM1;
    outputInit.TIM_OCIdleState = TIM_OCIdleState_Set;
    outputInit.TIM_OutputState = TIM_OutputState_Enable;
    outputInit.TIM_OCPolarity = TIM_OCPolarity_Low;
    outputInit.TIM_Pulse = 10;

    TIM_TimeBaseInitTypeDef baseInit;
    TIM_TimeBaseStructInit(&baseInit);
    baseInit.TIM_Prescaler = 0;
    baseInit.TIM_ClockDivision = TIM_CKD_DIV1;
    baseInit.TIM_CounterMode = TIM_CounterMode_Up;
    baseInit.TIM_Period = dshotBitbangCtx.outputARR;

    TIM_Cmd(dshotBitbangCtx.pacerTimHw->tim, DISABLE);
    TIM_TimeBaseInit(dshotBitbangCtx.pacerTimHw->tim, &baseInit);
    TIM_ARRPreloadConfig(dshotBitbangCtx.pacerTimHw->tim, ENABLE);

    switch (dshotBitbangCtx.pacerTimHw->channelIndex) {
        case 0:
            TIM_OC1Init(dshotBitbangCtx.pacerTimHw->tim, &outputInit);
            TIM_OC1PreloadConfig(dshotBitbangCtx.pacerTimHw->tim, TIM_OCPreload_Enable);
            break;
        case 1:
            TIM_OC2Init(dshotBitbangCtx.pacerTimHw->tim, &outputInit);
            TIM_OC2PreloadConfig(dshotBitbangCtx.pacerTimHw->tim, TIM_OCPreload_Enable);
            break;
        case 2:
            TIM_OC3Init(dshotBitbangCtx.pacerTimHw->tim, &outputInit);
            TIM_OC3PreloadConfig(dshotBitbangCtx.pacerTimHw->tim, TIM_OCPreload_Enable);
            break;
        case 3:
            TIM_OC4Init(dshotBitbangCtx.pacerTimHw->tim, &outputInit);
            TIM_OC4PreloadConfig(dshotBitbangCtx.pacerTimHw->tim, TIM_OCPreload_Enable);
            break;
    }

    TIM_CtrlPWMOutputs(dshotBitbangCtx.pacerTimHw->tim, ENABLE);
    TIM_Cmd(dshotBitbangCtx.pacerTimHw->tim, ENABLE);
}

static void dshotBitbangDecodeSamples(void)
{
    uint32_t readDelta = 0;
    uint32_t invalidDelta = 0;
    uint32_t noEdgeDelta = 0;

    const uint32_t sampleCount = DSHOT_BITBANG_INPUT_BUFFER_LENGTH - DMA_GetCurrDataCounter(dshotBitbangCtx.dma->ref);
    dshotBitbangCtx.lastSampleCount = sampleCount;
    dshotBitbangCtx.lastDecodeMotor = 0;
    dshotBitbangCtx.lastDecodeStatus = DSHOT_BITBANG_RUNTIME_DECODE_NONE;

    for (uint8_t i = 0; i < DSHOT_BITBANG_MAX_MOTORS; i++) {
        if (!dshotBitbangMotors[i].configured) {
            rpmSourceInvalidateDshotBidir(i);
            dshotBitbangCtx.lastRawValue[i] = 0;
            dshotBitbangCtx.lastErpmValue[i] = 0;
            dshotBitbangCtx.lastRpmValue[i] = 0;
            continue;
        }

        dshotBitbangCtx.lastDecodeMotor = i;

        const uint32_t rawValue = dshotBitbangDecodeTelemetry(dshotBitbangCtx.inputBuffer, sampleCount, dshotBitbangMotors[i].pinIndex);
        if (rawValue == DSHOT_BITBANG_DECODE_NOEDGE) {
            dshotBitbangCtx.noEdgeCount++;
            dshotBitbangCtx.motorNoEdgeCount[i]++;
            dshotBitbangCtx.lastDecodeStatus = DSHOT_BITBANG_RUNTIME_DECODE_NOEDGE;
            noEdgeDelta++;
            rpmSourceInvalidateDshotBidir(i);
            dshotBitbangCtx.lastRawValue[i] = 0;
            dshotBitbangCtx.lastErpmValue[i] = 0;
            dshotBitbangCtx.lastRpmValue[i] = 0;
            continue;
        }

        if (rawValue == DSHOT_BITBANG_DECODE_INVALID) {
            dshotBitbangCtx.invalidPacketCount++;
            dshotBitbangCtx.motorInvalidCount[i]++;
            dshotBitbangCtx.lastDecodeStatus = DSHOT_BITBANG_RUNTIME_DECODE_INVALID;
            invalidDelta++;
            rpmSourceInvalidateDshotBidir(i);
            dshotBitbangCtx.lastRawValue[i] = 0;
            dshotBitbangCtx.lastErpmValue[i] = 0;
            dshotBitbangCtx.lastRpmValue[i] = 0;
            continue;
        }

        dshotBitbangCtx.readCount++;
        dshotBitbangCtx.motorReadCount[i]++;
        readDelta++;

        const uint32_t erpm = dshotBidirDecodeErpmTelemetryValue((uint16_t)rawValue);
        if (erpm == DSHOT_BIDIR_INVALID) {
            dshotBitbangCtx.invalidPacketCount++;
            dshotBitbangCtx.motorInvalidCount[i]++;
            dshotBitbangCtx.lastDecodeStatus = DSHOT_BITBANG_RUNTIME_DECODE_ERPM;
            invalidDelta++;
            rpmSourceInvalidateDshotBidir(i);
            dshotBitbangCtx.lastRawValue[i] = (uint16_t)rawValue;
            dshotBitbangCtx.lastErpmValue[i] = 0;
            dshotBitbangCtx.lastRpmValue[i] = 0;
            continue;
        }

        const uint32_t rpm = dshotBidirComputeRpm(erpm, motorConfig()->motorPoleCount);
        dshotBitbangCtx.lastRawValue[i] = (uint16_t)rawValue;
        dshotBitbangCtx.lastErpmValue[i] = erpm;
        dshotBitbangCtx.lastRpmValue[i] = rpm;
        dshotBitbangCtx.lastDecodeStatus = DSHOT_BITBANG_RUNTIME_DECODE_OK;
        rpmSourceSetDshotBidirRpm(i, rpm);
    }

    dshotBitbangUpdateDebug(readDelta, invalidDelta, noEdgeDelta, 0);
}

bool dshotBitbangRuntimeInit(uint32_t dshotSymbolHz)
{
    if (dshotSymbolHz == 0U) {
        return false;
    }

    memset(&dshotBitbangCtx, 0, sizeof(dshotBitbangCtx));
    memset(dshotBitbangMotors, 0, sizeof(dshotBitbangMotors));

    dshotBitbangCtx.pacerTimHw = dshotBitbangFindPacerHardware();
    if (!dshotBitbangCtx.pacerTimHw) {
        return false;
    }

    dshotBitbangCtx.pacerTch = timerGetTCH(dshotBitbangCtx.pacerTimHw);
    if (!dshotBitbangCtx.pacerTch) {
        return false;
    }

    dshotBitbangCtx.dma = dmaGetByTag(dshotBitbangCtx.pacerTimHw->dmaTag);
    if (!dshotBitbangCtx.dma || dmaGetOwner(dshotBitbangCtx.dma) != OWNER_FREE) {
        return false;
    }

    dshotBitbangCtx.dmaSource = dshotBitbangPacerDmaSource(dshotBitbangCtx.pacerTimHw->channelIndex);
    const uint32_t pacerTimerHz = timerClock(dshotBitbangCtx.pacerTimHw->tim);
    if (!dshotBitbangComputeTimerArr(pacerTimerHz, dshotBitbangComputeOutputStateHz(dshotSymbolHz), &dshotBitbangCtx.outputARR)) {
        return false;
    }

    if (!dshotBitbangComputeTimerArr(pacerTimerHz, dshotBitbangComputeInputSampleHz(dshotSymbolHz), &dshotBitbangCtx.inputARR)) {
        return false;
    }

    dmaInit(dshotBitbangCtx.dma, OWNER_DSHOT_BITBANG, 0);
    dmaSetHandler(dshotBitbangCtx.dma, dshotBitbangDmaHandler, NVIC_PRIO_TIMER_DMA, (uint32_t)dshotBitbangCtx.pacerTch);

    dshotBitbangInitPacerChannel();
    rpmSourceResetDshotBidir();
    dshotBitbangUpdateDebug(0, 0, 0, 0);
    dshotBitbangCtx.initialized = true;
    dshotBitbangCtx.state = DSHOT_BITBANG_STATE_OUTPUT;
    return true;
}

bool dshotBitbangRuntimeConfigureMotor(const timerHardware_t *timerHardware, uint8_t motorIndex, bool enableOutput)
{
    if (!dshotBitbangCtx.initialized || !timerHardware || motorIndex >= DSHOT_BITBANG_MAX_MOTORS) {
        return false;
    }

    if (timerHardware->tim != TIM4) {
        return false;
    }

    const IO_t io = IOGetByTag(timerHardware->tag);
    if (!io || IO_GPIOPortIdx(io) != IO_GPIOPortIdx(IOGetByTag(IO_TAG(PB6)))) {
        return false;
    }

    dshotBitbangMotor_t *motor = &dshotBitbangMotors[motorIndex];
    motor->configured = true;
    motor->io = io;
    motor->tag = timerHardware->tag;
    motor->pinIndex = (uint8_t)IO_GPIOPinIdx(io);
    dshotBitbangCtx.gpio = IO_GPIO(io);
    dshotBitbangCtx.portMask |= IO_Pin(io);
    dshotBitbangCtx.gpioModeMask |= (GPIO_MODER_MODER0 << (motor->pinIndex * 2));
    dshotBitbangCtx.gpioModeInput |= (GPIO_Mode_IN << (motor->pinIndex * 2));
    dshotBitbangCtx.gpioModeOutput |= (GPIO_Mode_OUT << (motor->pinIndex * 2));
    dshotBitbangCtx.gpioIdleBSRR |= IO_Pin(io);

    IOInit(io, OWNER_MOTOR, RESOURCE_OUTPUT, RESOURCE_INDEX(motorIndex));
    IOConfigGPIO(io, IO_CONFIG(GPIO_Mode_OUT, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_UP));
    IOWrite(io, true);

    ZERO_FARRAY(dshotBitbangCtx.outputBuffer);
    dshotBitbangOutputDataInit(dshotBitbangCtx.outputBuffer, dshotBitbangCtx.portMask, true);

    DMA_StructInit(&dshotBitbangCtx.outputDmaInit);
    dshotBitbangCtx.outputDmaInit.DMA_Mode = DMA_Mode_Normal;
    dshotBitbangCtx.outputDmaInit.DMA_Channel = dmaGetChannelByTag(dshotBitbangCtx.pacerTimHw->dmaTag);
    dshotBitbangCtx.outputDmaInit.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dshotBitbangCtx.outputDmaInit.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dshotBitbangCtx.outputDmaInit.DMA_FIFOMode = DMA_FIFOMode_Enable;
    dshotBitbangCtx.outputDmaInit.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    dshotBitbangCtx.outputDmaInit.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    dshotBitbangCtx.outputDmaInit.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    dshotBitbangCtx.outputDmaInit.DMA_Priority = DMA_Priority_High;
    dshotBitbangCtx.outputDmaInit.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    dshotBitbangCtx.outputDmaInit.DMA_BufferSize = DSHOT_BITBANG_OUTPUT_BUFFER_LENGTH;
    dshotBitbangCtx.outputDmaInit.DMA_PeripheralBaseAddr = (uint32_t)&dshotBitbangCtx.gpio->BSRRL;
    dshotBitbangCtx.outputDmaInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    dshotBitbangCtx.outputDmaInit.DMA_Memory0BaseAddr = (uint32_t)dshotBitbangCtx.outputBuffer;
    dshotBitbangCtx.outputDmaInit.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;

    DMA_StructInit(&dshotBitbangCtx.inputDmaInit);
    dshotBitbangCtx.inputDmaInit.DMA_Mode = DMA_Mode_Normal;
    dshotBitbangCtx.inputDmaInit.DMA_Channel = dmaGetChannelByTag(dshotBitbangCtx.pacerTimHw->dmaTag);
    dshotBitbangCtx.inputDmaInit.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dshotBitbangCtx.inputDmaInit.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dshotBitbangCtx.inputDmaInit.DMA_FIFOMode = DMA_FIFOMode_Enable;
    dshotBitbangCtx.inputDmaInit.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    dshotBitbangCtx.inputDmaInit.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    dshotBitbangCtx.inputDmaInit.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    dshotBitbangCtx.inputDmaInit.DMA_Priority = DMA_Priority_VeryHigh;
    dshotBitbangCtx.inputDmaInit.DMA_DIR = DMA_DIR_PeripheralToMemory;
    dshotBitbangCtx.inputDmaInit.DMA_BufferSize = DSHOT_BITBANG_INPUT_BUFFER_LENGTH;
    dshotBitbangCtx.inputDmaInit.DMA_PeripheralBaseAddr = (uint32_t)&dshotBitbangCtx.gpio->IDR;
    dshotBitbangCtx.inputDmaInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    dshotBitbangCtx.inputDmaInit.DMA_Memory0BaseAddr = (uint32_t)dshotBitbangCtx.inputBuffer;
    dshotBitbangCtx.inputDmaInit.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;

    dshotBitbangCtx.configuredMotorCount = 0;
    for (uint8_t i = 0; i < DSHOT_BITBANG_MAX_MOTORS; i++) {
        dshotBitbangCtx.configuredMotorCount += dshotBitbangMotors[i].configured ? 1U : 0U;
    }

    dshotBitbangCtx.configured = enableOutput;
    dshotBitbangSwitchToOutput();
    return true;
}

bool dshotBitbangRuntimeIsActive(void)
{
    return dshotBitbangCtx.initialized;
}

bool dshotBitbangRuntimeIsReady(void)
{
    return dshotBitbangCtx.initialized && dshotBitbangCtx.configured && !dshotBitbangCtx.suspended &&
        dshotBitbangCtx.configuredMotorCount == getMotorCount();
}

void dshotBitbangRuntimeStop(void)
{
    if (!dshotBitbangCtx.initialized) {
        return;
    }
    ATOMIC_BLOCK(NVIC_PRIO_TIMER_DMA) {
        dshotBitbangCtx.suspended = true;
        TIM_DMACmd(dshotBitbangCtx.pacerTimHw->tim, dshotBitbangCtx.dmaSource, DISABLE);
        DMA_Cmd(dshotBitbangCtx.dma->ref, DISABLE);
        DMA_CLEAR_FLAG(dshotBitbangCtx.dma, DMA_IT_TCIF | DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_DMEIF | DMA_IT_FEIF);
        dshotBitbangCtx.outputPending = false;
        dshotBitbangCtx.telemetryPending = false;
        dshotBitbangCtx.state = DSHOT_BITBANG_STATE_DISABLED;
        if (dshotBitbangCtx.gpio) {
            MODIFY_REG(dshotBitbangCtx.gpio->MODER, dshotBitbangCtx.gpioModeMask, dshotBitbangCtx.gpioModeInput);
        }
    }
    rpmSourceResetDshotBidir();
}

void dshotBitbangRuntimeResume(void)
{
    if (dshotBitbangCtx.initialized) {
        dshotBitbangCtx.suspended = false;
    }
}

bool dshotBitbangRuntimeUpdate(void)
{
    if (!dshotBitbangCtx.initialized || dshotBitbangCtx.suspended) {
        return true;
    }

    // A TX transfer also needs ownership and a deadline. Otherwise the main
    // loop can rewrite its buffer/NDTR while DMA is still reading it.
    if (dshotBitbangCtx.outputPending) {
        if ((uint32_t)cmpTimeUs(micros(), dshotBitbangCtx.frameStartedUs) < DSHOT_BITBANG_TELEMETRY_TIMEOUT_US) {
            return false;
        }
        ATOMIC_BLOCK(NVIC_PRIO_TIMER_DMA) {
            TIM_DMACmd(dshotBitbangCtx.pacerTimHw->tim, dshotBitbangCtx.dmaSource, DISABLE);
            DMA_Cmd(dshotBitbangCtx.dma->ref, DISABLE);
            DMA_CLEAR_FLAG(dshotBitbangCtx.dma, DMA_IT_TCIF | DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_DMEIF | DMA_IT_FEIF);
            dshotBitbangCtx.outputPending = false;
            dshotBitbangCtx.telemetryPending = false;
            dshotBitbangCtx.state = DSHOT_BITBANG_STATE_DISABLED;
            dshotBitbangCtx.timeoutCount++;
        }
    }

    if (dshotBitbangCtx.telemetryPending) {
        if ((uint32_t)cmpTimeUs(micros(), dshotBitbangCtx.captureStartedUs) < DSHOT_BITBANG_TELEMETRY_TIMEOUT_US) {
            return false;
        }

        ATOMIC_BLOCK(NVIC_PRIO_TIMER_DMA) {
            TIM_DMACmd(dshotBitbangCtx.pacerTimHw->tim, dshotBitbangCtx.dmaSource, DISABLE);
            DMA_Cmd(dshotBitbangCtx.dma->ref, DISABLE);
            DMA_CLEAR_FLAG(dshotBitbangCtx.dma, DMA_IT_TCIF | DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_DMEIF | DMA_IT_FEIF);
            dshotBitbangCtx.telemetryPending = false;
            dshotBitbangCtx.state = DSHOT_BITBANG_STATE_DISABLED;
            dshotBitbangCtx.timeoutCount++;
        }
        dshotBitbangUpdateDebug(0, 0, 0, 1);
    }

    if (dshotBitbangCtx.state == DSHOT_BITBANG_STATE_INPUT) {
        dshotBitbangDecodeSamples();
        dshotBitbangSwitchToOutput();
    } else if (dshotBitbangCtx.state == DSHOT_BITBANG_STATE_DISABLED) {
        for (uint8_t i = 0; i < DSHOT_BITBANG_MAX_MOTORS; i++) {
            rpmSourceInvalidateDshotBidir(i);
        }
        dshotBitbangSwitchToOutput();
    }

    return true;
}

void dshotBitbangRuntimeBeginFrame(void)
{
    if (!dshotBitbangCtx.initialized) {
        return;
    }

    dshotBitbangOutputDataClear(dshotBitbangCtx.outputBuffer);
}

void dshotBitbangRuntimeSetMotorValue(uint8_t motorIndex, uint16_t value, bool requestTelemetry)
{
    if (!dshotBitbangCtx.initialized || motorIndex >= DSHOT_BITBANG_MAX_MOTORS || !dshotBitbangMotors[motorIndex].configured) {
        return;
    }

    const uint16_t packet = dshotBitbangPreparePacket(value, requestTelemetry);
    dshotBitbangOutputDataSet(dshotBitbangCtx.outputBuffer, dshotBitbangMotors[motorIndex].pinIndex, packet, true);
}

void dshotBitbangRuntimeStartFrame(void)
{
    if (!dshotBitbangRuntimeIsReady() || dshotBitbangCtx.outputPending || dshotBitbangCtx.telemetryPending) {
        return;
    }

    if (dshotBitbangCtx.state == DSHOT_BITBANG_STATE_INPUT) {
        dshotBitbangSwitchToOutput();
    }

    dshotBitbangCtx.frameStartedUs = micros();
    dshotBitbangCtx.telemetryPending = false;
    dshotBitbangCtx.outputPending = true;
    DMA_CLEAR_FLAG(dshotBitbangCtx.dma, DMA_IT_TCIF | DMA_IT_TEIF | DMA_IT_DMEIF | DMA_IT_FEIF);
    DMA_SetCurrDataCounter(dshotBitbangCtx.dma->ref, DSHOT_BITBANG_OUTPUT_BUFFER_LENGTH);
    DMA_Cmd(dshotBitbangCtx.dma->ref, ENABLE);
    TIM_DMACmd(dshotBitbangCtx.pacerTimHw->tim, dshotBitbangCtx.dmaSource, ENABLE);
}

ioTag_t dshotBitbangRuntimeGetMotorPinTag(uint8_t motorIndex)
{
    if (motorIndex >= DSHOT_BITBANG_MAX_MOTORS || !dshotBitbangMotors[motorIndex].configured) {
        return IOTAG_NONE;
    }

    return dshotBitbangMotors[motorIndex].tag;
}

uint8_t dshotBitbangRuntimeGetState(void)
{
    return dshotBitbangCtx.state;
}

uint16_t dshotBitbangRuntimeGetRequestSource(void)
{
    return dshotBitbangCtx.dmaSource;
}

uint32_t dshotBitbangRuntimeGetReadCount(void)
{
    return dshotBitbangCtx.readCount;
}

uint32_t dshotBitbangRuntimeGetInvalidCount(void)
{
    return dshotBitbangCtx.invalidPacketCount;
}

uint32_t dshotBitbangRuntimeGetNoEdgeCount(void)
{
    return dshotBitbangCtx.noEdgeCount;
}

uint32_t dshotBitbangRuntimeGetTimeoutCount(void)
{
    return dshotBitbangCtx.timeoutCount;
}

uint32_t dshotBitbangRuntimeGetOutputCallbackCount(void)
{
    return dshotBitbangCtx.outputCallbackCount;
}

uint32_t dshotBitbangRuntimeGetInputCallbackCount(void)
{
    return dshotBitbangCtx.inputCallbackCount;
}

uint32_t dshotBitbangRuntimeGetOutputOnlyFrameCount(void)
{
    return dshotBitbangCtx.outputOnlyFrameCount;
}

uint32_t dshotBitbangRuntimeGetTelemetryFrameCount(void)
{
    return dshotBitbangCtx.telemetryFrameCount;
}

uint32_t dshotBitbangRuntimeGetLastLatencyUs(void)
{
    return dshotBitbangCtx.lastLatencyUs;
}

uint32_t dshotBitbangRuntimeGetLastSampleCount(void)
{
    return dshotBitbangCtx.lastSampleCount;
}

uint32_t dshotBitbangRuntimeGetLastRawValue(uint8_t motorIndex)
{
    return (motorIndex < DSHOT_BITBANG_MAX_MOTORS) ? dshotBitbangCtx.lastRawValue[motorIndex] : 0;
}

uint32_t dshotBitbangRuntimeGetLastErpmValue(uint8_t motorIndex)
{
    return (motorIndex < DSHOT_BITBANG_MAX_MOTORS) ? dshotBitbangCtx.lastErpmValue[motorIndex] : 0;
}

uint32_t dshotBitbangRuntimeGetLastRpmValue(uint8_t motorIndex)
{
    return (motorIndex < DSHOT_BITBANG_MAX_MOTORS) ? dshotBitbangCtx.lastRpmValue[motorIndex] : 0;
}

uint32_t dshotBitbangRuntimeGetMotorReadCount(uint8_t motorIndex)
{
    return (motorIndex < DSHOT_BITBANG_MAX_MOTORS) ? dshotBitbangCtx.motorReadCount[motorIndex] : 0;
}

uint32_t dshotBitbangRuntimeGetMotorInvalidCount(uint8_t motorIndex)
{
    return (motorIndex < DSHOT_BITBANG_MAX_MOTORS) ? dshotBitbangCtx.motorInvalidCount[motorIndex] : 0;
}

uint32_t dshotBitbangRuntimeGetMotorNoEdgeCount(uint8_t motorIndex)
{
    return (motorIndex < DSHOT_BITBANG_MAX_MOTORS) ? dshotBitbangCtx.motorNoEdgeCount[motorIndex] : 0;
}

uint8_t dshotBitbangRuntimeGetLastDecodeMotor(void)
{
    return dshotBitbangCtx.lastDecodeMotor;
}

uint8_t dshotBitbangRuntimeGetLastDecodeStatus(void)
{
    return dshotBitbangCtx.lastDecodeStatus;
}

#else

#include "drivers/dshot_bitbang_runtime.h"

bool dshotBitbangRuntimeInit(uint32_t dshotHz)
{
    (void)dshotHz;
    return false;
}

bool dshotBitbangRuntimeConfigureMotor(const timerHardware_t *timerHardware, uint8_t motorIndex, bool enableOutput)
{
    (void)timerHardware;
    (void)motorIndex;
    (void)enableOutput;
    return false;
}

bool dshotBitbangRuntimeIsActive(void)
{
    return false;
}

bool dshotBitbangRuntimeIsReady(void)
{
    return false;
}

void dshotBitbangRuntimeStop(void)
{
}

void dshotBitbangRuntimeResume(void)
{
}

bool dshotBitbangRuntimeUpdate(void)
{
    return true;
}

void dshotBitbangRuntimeBeginFrame(void)
{
}

void dshotBitbangRuntimeSetMotorValue(uint8_t motorIndex, uint16_t value, bool requestTelemetry)
{
    (void)motorIndex;
    (void)value;
    (void)requestTelemetry;
}

void dshotBitbangRuntimeStartFrame(void)
{
}

ioTag_t dshotBitbangRuntimeGetMotorPinTag(uint8_t motorIndex)
{
    (void)motorIndex;
    return IOTAG_NONE;
}

uint8_t dshotBitbangRuntimeGetState(void)
{
    return 0;
}

uint16_t dshotBitbangRuntimeGetRequestSource(void)
{
    return 0;
}

uint32_t dshotBitbangRuntimeGetReadCount(void)
{
    return 0;
}

uint32_t dshotBitbangRuntimeGetInvalidCount(void)
{
    return 0;
}

uint32_t dshotBitbangRuntimeGetNoEdgeCount(void)
{
    return 0;
}

uint32_t dshotBitbangRuntimeGetTimeoutCount(void)
{
    return 0;
}

uint32_t dshotBitbangRuntimeGetOutputCallbackCount(void)
{
    return 0;
}

uint32_t dshotBitbangRuntimeGetInputCallbackCount(void)
{
    return 0;
}

uint32_t dshotBitbangRuntimeGetOutputOnlyFrameCount(void)
{
    return 0;
}

uint32_t dshotBitbangRuntimeGetTelemetryFrameCount(void)
{
    return 0;
}

uint32_t dshotBitbangRuntimeGetLastLatencyUs(void)
{
    return 0;
}

uint32_t dshotBitbangRuntimeGetLastSampleCount(void)
{
    return 0;
}

uint32_t dshotBitbangRuntimeGetLastRawValue(uint8_t motorIndex)
{
    (void)motorIndex;
    return 0;
}

uint32_t dshotBitbangRuntimeGetLastErpmValue(uint8_t motorIndex)
{
    (void)motorIndex;
    return 0;
}

uint32_t dshotBitbangRuntimeGetLastRpmValue(uint8_t motorIndex)
{
    (void)motorIndex;
    return 0;
}

uint32_t dshotBitbangRuntimeGetMotorReadCount(uint8_t motorIndex)
{
    (void)motorIndex;
    return 0;
}

uint32_t dshotBitbangRuntimeGetMotorInvalidCount(uint8_t motorIndex)
{
    (void)motorIndex;
    return 0;
}

uint32_t dshotBitbangRuntimeGetMotorNoEdgeCount(uint8_t motorIndex)
{
    (void)motorIndex;
    return 0;
}

uint8_t dshotBitbangRuntimeGetLastDecodeMotor(void)
{
    return 0;
}

uint8_t dshotBitbangRuntimeGetLastDecodeStatus(void)
{
    return DSHOT_BITBANG_RUNTIME_DECODE_NONE;
}

#endif
