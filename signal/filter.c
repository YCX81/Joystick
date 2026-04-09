#include "filter.h"

fw_status_t filter_init(filter_t* f, uint8_t size) {
  if (f == NULL) return FW_INVALID_ARG;
  if (size == 0 || size > FILTER_MAX_SAMPLES) return FW_INVALID_ARG;
  f->size = size;
  return FW_OK;
}

fw_status_t filter_update(filter_t* f, uint32_t raw) {
  if (f == NULL) return FW_INVALID_ARG;
  f->samples[f->index] = raw;
  f->index++;
  if (f->index >= f->size) {
    f->index = 0;
  }
  return FW_OK;
}

fw_status_t filter_get_value(filter_t* f, uint32_t* filter_value) {
  if (f == NULL) return FW_INVALID_ARG;
  if (filter_value == NULL) return FW_INVALID_ARG;
  uint32_t value = 0;
  for (uint8_t i = 0; i < f->size; i++) {
    value += f->samples[i];
  }
  *filter_value = value / f->size;
  return FW_OK;
}
