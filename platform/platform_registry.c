#include "platform_registry.h"

static const adc_reader_t* adc_reader_driver;
static const adc_voltage_reader_t* adc_voltage_reader_driver;
static const adc_continuous_t* adc_continuous_driver;
static const gpio_reader_t* gpio_reader_driver;
static const gpio_writer_t* gpio_writer_driver;
static const spi_port_t* spi_port_driver;
static const can_port_t* can_port_driver;

void platform_register_adc(const adc_reader_t* driver) {
  adc_reader_driver = driver;
}

const adc_reader_t* platform_get_adc(void) { return adc_reader_driver; }

void platform_register_adc_voltage(const adc_voltage_reader_t* driver) {
  adc_voltage_reader_driver = driver;
}

const adc_voltage_reader_t* platform_get_adc_voltage(void) {
  return adc_voltage_reader_driver;
}

void platform_register_adc_continuous(const adc_continuous_t* driver) {
  adc_continuous_driver = driver;
}

const adc_continuous_t* platform_get_adc_continuous(void) {
  return adc_continuous_driver;
}

void platform_register_gpio_reader(const gpio_reader_t* driver) {
  gpio_reader_driver = driver;
}

const gpio_reader_t* platform_get_gpio_reader(void) {
  return gpio_reader_driver;
}

void platform_register_gpio_writer(const gpio_writer_t* driver) {
  gpio_writer_driver = driver;
}

const gpio_writer_t* platform_get_gpio_writer(void) {
  return gpio_writer_driver;
}

void platform_register_spi_port(const spi_port_t* driver) {
  spi_port_driver = driver;
}

const spi_port_t* platform_get_spi_port(void) { return spi_port_driver; }

void platform_register_can_port(const can_port_t* driver) {
  can_port_driver = driver;
}

const can_port_t* platform_get_can_port(void) { return can_port_driver; }
