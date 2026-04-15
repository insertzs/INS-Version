#pragma once

#include <stdbool.h>
#include <stdint.h>

#define DSHOT_BITBANG_OUTPUT_STATES_PER_BIT 3U
#define DSHOT_BITBANG_FRAME_BITS 16U
#define DSHOT_BITBANG_HOLD_STATES 3U
#define DSHOT_BITBANG_OUTPUT_BUFFER_LENGTH ((DSHOT_BITBANG_FRAME_BITS * DSHOT_BITBANG_OUTPUT_STATES_PER_BIT) + DSHOT_BITBANG_HOLD_STATES)
#define DSHOT_BITBANG_TELEMETRY_OVERSAMPLE 3U

void dshotBitbangOutputDataInit(uint32_t *buffer, uint16_t portMask, bool inverted);
void dshotBitbangOutputDataSet(uint32_t *buffer, uint8_t pinIndex, uint16_t packet, bool inverted);
void dshotBitbangOutputDataClear(uint32_t *buffer);
uint32_t dshotBitbangComputeOutputStateHz(uint32_t dshotSymbolHz);
uint32_t dshotBitbangComputeFrameTimeUs(uint32_t dshotSymbolHz);
uint32_t dshotBitbangComputeInputSampleHz(uint32_t dshotSymbolHz);
