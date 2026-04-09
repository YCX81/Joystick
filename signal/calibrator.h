#pragma once

#include <stdint.h>

#include "error.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint32_t output_range;
  uint32_t raw_min;
  uint32_t raw_max;
} calibrator_t;

fw_status_t calibrator_init(calibrator_t* cal, uint32_t output_range,
                            uint32_t raw_min, uint32_t raw_max);

fw_status_t calibrator_calibrate(calibrator_t* cal, uint32_t raw,
                                 uint32_t* output_value);

#ifdef __cplusplus
}
#endif