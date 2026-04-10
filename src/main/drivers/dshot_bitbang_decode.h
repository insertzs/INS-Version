#pragma once

#include <stdint.h>

#define DSHOT_BITBANG_DECODE_INVALID 0xFFFFFFFFU
#define DSHOT_BITBANG_DECODE_NOEDGE  0xFFFFFFFEU

uint32_t dshotBitbangDecodeTelemetry(uint16_t buffer[], uint32_t count, uint32_t bit);
