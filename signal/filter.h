#pragma once

#include <stdint.h>

#include "error.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FILTER_MAX_SAMPLES 16

typedef struct {
  uint32_t samples[FILTER_MAX_SAMPLES];
  uint8_t size;
  uint8_t index;
} filter_t;

fw_status_t filter_init(filter_t* f, uint8_t size);

fw_status_t filter_update(filter_t* f, uint32_t raw);

fw_status_t filter_get_value(filter_t* f, uint32_t* filter_value);

#ifdef __cplusplus
}
#endif