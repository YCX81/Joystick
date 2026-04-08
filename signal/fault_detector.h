#pragma once

#include "error.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { FAULT_NONE, FAULT_OPEN, FAULT_SHORT } fault_status_t;

typedef struct {
  uint16_t low_threshold;
  uint16_t high_threshold;
} fault_detector_t;

fw_status_t fault_detector_init(fault_detector_t* fd, uint16_t low_threshold,
                                uint16_t high_threshold);

fw_status_t fault_detector_check(fault_detector_t* fd, uint16_t raw,
                                 fault_status_t* result);

#ifdef __cplusplus
}
#endif