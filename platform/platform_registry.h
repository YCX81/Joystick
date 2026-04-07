#pragma once

#include "drv_adc_continuous.h"
#include "drv_adc_reader.h"
#include "drv_adc_voltage_reader.h"
#include "drv_can.h"
#include "drv_gpio.h"
#include "drv_spi.h"

#ifdef __cplusplus
extern "C" {
#endif

void platform_register_adc(const adc_reader_t* driver);

const adc_reader_t* platform_get_adc(void);

void platform_register_adc_voltage(const adc_voltage_reader_t* driver);

const adc_voltage_reader_t* platform_get_adc_voltage(void);

void platform_register_adc_continuous(const adc_continuous_t* driver);

const adc_continuous_t* platform_get_adc_continuous(void);

void platform_register_gpio_reader(const gpio_reader_t* driver);

const gpio_reader_t* platform_get_gpio_reader(void);

void platform_register_gpio_writer(const gpio_writer_t* driver);

const gpio_writer_t* platform_get_gpio_writer(void);

void platform_register_spi_port(const spi_port_t* driver);

const spi_port_t* platform_get_spi_port(void);

void platform_register_can_port(const can_port_t* driver);

const can_port_t* platform_get_can_port(void);

#ifdef __cplusplus
}
#endif