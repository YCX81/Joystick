#pragma once

#include "drv_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  drv_status_t (*init)(void* ctx, uint32_t clock_hz);
  drv_status_t (*deinit)(void* ctx);
  drv_status_t (*transfer)(void* ctx, const uint8_t cs_pin,
                           const uint8_t* tx_buf, uint8_t* rx_buf,
                           uint16_t length, uint32_t timeout_ms);
  void* ctx;
} spi_port_t;

#ifdef __cplusplus
}
#endif