#pragma once

#include <stdint.h>

#include "error.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef uint8_t adc_channel_id_t;
#define ADC_CH_MAX 64

typedef uint8_t gpio_pin_id_t;
#define GPIO_PIN_MAX 64

#ifdef __cplusplus
}
#endif