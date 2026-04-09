#include "fault_detector.h"

#include "error.h"

fw_status_t fault_detector_init(fault_detector_t* fd, uint16_t low_threshold,
                                uint16_t high_threshold) {
  if (fd == NULL) return FW_INVALID_ARG;
  if (high_threshold <= low_threshold) return FW_INVALID_ARG;
  fd->high_threshold = high_threshold;
  fd->low_threshold = low_threshold;
  return FW_OK;
}

fw_status_t fault_detector_check(fault_detector_t* fd, uint16_t raw,
                                 fault_status_t* result) {
  if (fd == NULL) return FW_INVALID_ARG;
  if (result == NULL) return FW_INVALID_ARG;
  if (raw >= fd->low_threshold && raw <= fd->high_threshold) {
    *result = FAULT_NONE;
  } else if (raw > fd->high_threshold) {
    *result = FAULT_SHORT;
  } else if (raw < fd->low_threshold) {
    *result = FAULT_OPEN;
  }
  return FW_OK;
}
