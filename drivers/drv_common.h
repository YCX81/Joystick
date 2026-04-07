#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  DRV_OK,
  DRV_ERROR,
  DRV_BUSY,
  DRV_TIMEOUT,
  DRV_INVALID_ARG,
  DRV_STATUS_MAX
} drv_status_t;

typedef uint8_t adc_channel_id_t;
#define ADC_CH_MAX 64

typedef uint8_t gpio_pin_id_t;
#define GPIO_PIN_MAX 64

#ifdef __cplusplus
}
#endif