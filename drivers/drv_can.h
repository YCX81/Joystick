#pragma once

#include "drv_common.h"
#include "error.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint8_t is_extended;
  uint8_t is_fd;
  uint32_t id;
  uint8_t data[64];
  uint8_t dlc;
} can_frame_t;

typedef struct {
  fw_status_t (*init)(void* ctx, uint32_t bitrate);
  fw_status_t (*deinit)(void* ctx);
  fw_status_t (*set_filter)(void* ctx, uint32_t id, uint32_t mask);
  fw_status_t (*get_bus_state)(void* ctx, uint8_t* tx_err, uint8_t* rx_err);
  fw_status_t (*send)(void* ctx, const can_frame_t* frame);
  fw_status_t (*try_recv)(void* ctx, can_frame_t* frame);
  fw_status_t (*wait_recv)(void* ctx, can_frame_t* frame, uint32_t timeout_ms);
  void* ctx;
} can_port_t;

#ifdef __cplusplus
}
#endif