#pragma once

#include "drv_common.h"
#include "error.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  fw_status_t (*read)(void* ctx, gpio_pin_id_t pin_id, uint8_t* state);
  void* ctx;
} gpio_reader_t;

typedef struct {
  fw_status_t (*write)(void* ctx, gpio_pin_id_t pin_id, uint8_t state);
  fw_status_t (*toggle)(void* ctx, gpio_pin_id_t pin_id);
  void* ctx;
} gpio_writer_t;

#ifdef __cplusplus
}
#endif