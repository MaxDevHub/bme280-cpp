# BME280 Component - C++

C++20 driver for Bosch BME280 sensor for ESP-IDF. Supports both I2C and SPI (3-wire and 4-wire) interfaces.

## Add component to project

```bash
idf.py add-dependency "maxdevhub/bme280-cpp=*"
```

## Example of BME280 usage

```cpp

#include "BME280.hpp"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

constexpr gpio_num_t I2C_SDA = GPIO_NUM_15;
constexpr gpio_num_t I2C_SCL = GPIO_NUM_14;

constexpr gpio_num_t SPI_MOSI = GPIO_NUM_22;
constexpr gpio_num_t SPI_MISO = GPIO_NUM_21;
constexpr gpio_num_t SPI_SCLK = GPIO_NUM_20;

spi_bus_config_t spiBusConfig;

i2c_master_bus_config_t i2cMasterBusConfig;
i2c_master_bus_handle_t bus_handle;

static void i2cInit() {
  i2cMasterBusConfig = {.i2c_port = I2C_NUM_0,
                        .sda_io_num = I2C_SDA,
                        .scl_io_num = I2C_SCL,
                        .clk_source = I2C_CLK_SRC_DEFAULT,
                        .glitch_ignore_cnt = 7,
                        .flags = {
                            .enable_internal_pullup = true,
                            .allow_pd = false,
                        }};

  i2c_new_master_bus(&i2cMasterBusConfig, &bus_handle);
}

static void spiInit() {
  spiBusConfig = {
      .mosi_io_num = SPI_MOSI,
      .miso_io_num = SPI_MISO,
      .sclk_io_num = SPI_SCLK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 16,
  };

  spi_bus_initialize(SPI2_HOST, &spiBusConfig, SPI_DMA_CH_AUTO);
}

extern "C" void app_main(void) {
  spiInit();
  i2cInit();

  BME280::I2CBus i2c(bus_handle, 100'000);
  BME280::BME280 sensorI2C(i2c);

  sensorI2C.setFilter(BME280::Filter::FILTER_16);
  sensorI2C.setOversmplingTemp(BME280::Oversampling::OVERSAMPLING_16X);
  sensorI2C.setOversmplingPress(BME280::Oversampling::OVERSAMPLING_16X);
  sensorI2C.setOversmplingHum(BME280::Oversampling::OVERSAMPLING_16X);

  BME280::SPIBus spi(SPI2_HOST, GPIO_NUM_19, 1'000'000);
  BME280::BME280 sensorSpi(spi);

  sensorSpi.setFilter(BME280::Filter::FILTER_16);
  sensorSpi.setOversmplingTemp(BME280::Oversampling::OVERSAMPLING_16X);
  sensorSpi.setOversmplingPress(BME280::Oversampling::OVERSAMPLING_16X);
  sensorSpi.setOversmplingHum(BME280::Oversampling::OVERSAMPLING_16X);

  for (;;) {
    sensorSpi.setMode(BME280::Mode::FORCED_MODE);
    sensorI2C.setMode(BME280::Mode::FORCED_MODE);

    while (sensorSpi.isMesuring() && sensorI2C.isMesuring()) {
      vTaskDelay(pdMS_TO_TICKS(10));
    }

    double tempI2C = sensorI2C.compensationTempDouble();
    double pressI2C = sensorI2C.compensationPressDouble() / 100.0;
    double humI2C = sensorI2C.compensationHumDouble();

    double tempSpi = sensorSpi.compensationTempDouble();
    double pressSpi = sensorSpi.compensationPressDouble() / 100.0;
    double humSpi = sensorSpi.compensationHumDouble();
    
    ...

    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}


```
