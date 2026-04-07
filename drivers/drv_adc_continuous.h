#pragma once

#include "drv_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  fw_status_t (*start)(void* ctx, uint8_t* channel, uint8_t count,
                        uint16_t* buffer, uint16_t length);
  fw_status_t (*stop)(void* ctx);
  void* ctx;
} adc_continuous_t;

#ifdef __cplusplus
}
#endif