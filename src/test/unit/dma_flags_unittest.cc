#include <cstdint>
#include <gtest/gtest.h>

// Exercise the production F4 register macros with host register storage.
// dma.h's UNIT_TEST fallback makes DMA_TypeDef a scalar, so provide the F4
// register shape here instead.
#ifdef UNIT_TEST
#undef UNIT_TEST
#define RESTORE_DMA_UNIT_TEST
#endif
#define STM32F4
struct DMA_TypeDef {
    uint32_t LISR, HISR, LIFCR, HIFCR;
};
struct DMA_Stream_TypeDef {};
using IRQn_Type = int;
#include "drivers/dma.h"
#ifdef RESTORE_DMA_UNIT_TEST
#define UNIT_TEST
#endif

TEST(DmaFlagsUnittest, ClearsCompositeMaskOnEveryStream)
{
    for (uint32_t shift : {0U, 6U, 16U, 22U, 32U, 38U, 48U, 54U}) {
        DMA_TypeDef registers = {};
        dmaChannelDescriptor_t descriptor = {};
        descriptor.dma = &registers;
        descriptor.flagsShift = shift;
        DMA_t dma = &descriptor;
        DMA_CLEAR_FLAG(dma, DMA_IT_TCIF | DMA_IT_TEIF | DMA_IT_DMEIF | DMA_IT_FEIF);
        const uint32_t expected = 0x2dU << (shift % 32);
        EXPECT_EQ(shift >= 32 ? registers.HIFCR : registers.LIFCR, expected) << shift;
        EXPECT_EQ(shift >= 32 ? registers.LIFCR : registers.HIFCR, 0U) << shift;
    }
}

TEST(DmaFlagsUnittest, ReadsCompositeMaskWithoutReadingAnotherStream)
{
    for (uint32_t shift : {6U, 16U, 22U, 38U, 48U, 54U}) {
        DMA_TypeDef registers = {};
        dmaChannelDescriptor_t descriptor = {};
        descriptor.dma = &registers;
        descriptor.flagsShift = shift;
        DMA_t dma = &descriptor;
        uint32_t &status = shift >= 32 ? registers.HISR : registers.LISR;
        status = DMA_IT_TEIF << (shift % 32);
        EXPECT_NE(DMA_GET_FLAG_STATUS(dma, DMA_IT_TEIF | DMA_IT_DMEIF | DMA_IT_FEIF), 0U) << shift;
        status = DMA_IT_TEIF; // unrelated stream 0/4
        EXPECT_EQ(DMA_GET_FLAG_STATUS(dma, DMA_IT_TEIF | DMA_IT_DMEIF | DMA_IT_FEIF), 0U) << shift;
    }
}
