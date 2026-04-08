#pragma once

#include "drv_common.h"
#include "error.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  fw_status_t (*init)(void* ctx);
  fw_status_t (*read)(void* ctx, adc_channel_id_t channel, uint16_t* raw);
  fw_status_t (*deinit)(void* ctx);
  void* ctx;
} adc_reader_t;

typedef struct {
  fw_status_t (*read)(void* ctx, adc_channel_id_t channel, uint32_t* mv);
  void* ctx;
} adc_voltage_reader_t;

typedef struct {
  fw_status_t (*start)(void* ctx, uint8_t* channel, uint8_t count,
                       uint16_t* buffer, uint16_t length);
  fw_status_t (*stop)(void* ctx);
  void* ctx;
} adc_continuous_t;

#ifdef __cplusplus
}
#endif