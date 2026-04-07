#pragma once

#include "drv_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  fw_status_t (*read)(void* ctx, adc_channel_id_t channel, uint32_t* mv);
  void* ctx;
} adc_voltage_reader_t;

#ifdef __cplusplus
}
#endif