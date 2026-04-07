#pragma once

#include "drv_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  drv_status_t (*read)(void* ctx, uint8_t pin_id, uint8_t* state);
  void* ctx;
} gpio_reader_t;

typedef struct {
  drv_status_t (*write)(void* ctx, uint8_t pin_id, uint8_t state);
  drv_status_t (*toggle)(void* ctx, uint8_t pin_id);
  void* ctx;
} gpio_writer_t;

#ifdef __cplusplus
}
#endif