#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  FW_OK,
  FW_ERROR,
  FW_BUSY,
  FW_TIMEOUT,
  FW_INVALID_ARG,
  FW_STATUS_MAX
} fw_status_t;

#ifdef __cplusplus
}
#endif
