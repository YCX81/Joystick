#include "calibrator.h"

fw_status_t calibrator_init(calibrator_t* cal, uint32_t output_range,
                            uint32_t raw_min, uint32_t raw_max) {
  if (cal == NULL) return FW_INVALID_ARG;
  if (output_range == 0) return FW_INVALID_ARG;
  if (raw_min >= raw_max) return FW_INVALID_ARG;
  cal->output_range = output_range;
  cal->raw_min = raw_min;
  cal->raw_max = raw_max;
  return FW_OK;
}

fw_status_t calibrator_calibrate(calibrator_t* cal, uint32_t raw,
                                 uint32_t* output_value) {
  if (cal == NULL) return FW_INVALID_ARG;
  if (output_value == NULL) return FW_INVALID_ARG;
  if (raw <= cal->raw_min) {
    *output_value = 0;
    return FW_OK;
  } else if (raw >= cal->raw_max) {
    *output_value = cal->output_range;
    return FW_OK;
  } else {
    *output_value = (raw - cal->raw_min) * (cal->output_range) /
                    (cal->raw_max - cal->raw_min);
    return FW_OK;
  }
}