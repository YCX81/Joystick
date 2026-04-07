#pragma once

#include "drv_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  drv_status_t (*init)(void* ctx);
  drv_status_t (*read)(void* ctx, adc_channel_id_t channel, uint16_t* raw);
  drv_status_t (*deinit)(void* ctx);
  void* ctx;
} adc_reader_t;

#ifdef __cplusplus
}
#endif