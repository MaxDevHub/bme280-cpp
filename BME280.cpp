#include "BME280.hpp"

#include "driver/gpio.h"

namespace BME280 {

I2CBus::I2CBus(i2c_master_bus_handle_t &busHandle, uint32_t freqHz,
               Addr devAddr) noexcept
    : _devAddr(devAddr) {
  _devConfig = {.dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address = I2C_DEVICE_ADDRESS_NOT_USED,
                .scl_speed_hz = freqHz,
                .scl_wait_us = 0,
                .flags = {
                    .disable_ack_check = 0,
                }};

  ESP_ERROR_CHECK(
      i2c_master_bus_add_device(busHandle, &_devConfig, &_devHandle));
}

void I2CBus::writeReg(uint8_t reg, uint8_t data) const noexcept {
  uint8_t writeAddr = (static_cast<uint8_t>(_devAddr) << 1U);

  i2c_operation_job_t i2c_ops[] = {
      {.command = I2C_MASTER_CMD_START},
      {.command = I2C_MASTER_CMD_WRITE,
       .write = {.ack_check = true, .data = &writeAddr, .total_bytes = 1}},
      {.command = I2C_MASTER_CMD_WRITE,
       .write = {.ack_check = true, .data = &reg, .total_bytes = 1}},
      {.command = I2C_MASTER_CMD_WRITE,
       .write = {.ack_check = true, .data = &data, .total_bytes = 1}},
      {.command = I2C_MASTER_CMD_STOP}};

  ESP_ERROR_CHECK(i2c_master_execute_defined_operations(
      _devHandle, i2c_ops, sizeof(i2c_ops) / sizeof(i2c_operation_job_t), -1));
};

uint8_t I2CBus::readReg(uint8_t reg) const noexcept {
  uint8_t data;

  uint8_t readAddr = (static_cast<uint8_t>(_devAddr) << 1U) | 0x01U;
  uint8_t writeAddr = (static_cast<uint8_t>(_devAddr) << 1U);

  i2c_operation_job_t i2c_ops[] = {
      {.command = I2C_MASTER_CMD_START},
      {.command = I2C_MASTER_CMD_WRITE,
       .write = {.ack_check = true, .data = &writeAddr, .total_bytes = 1}},
      {.command = I2C_MASTER_CMD_WRITE,
       .write = {.ack_check = true, .data = &reg, .total_bytes = 1}},
      {.command = I2C_MASTER_CMD_START},
      {.command = I2C_MASTER_CMD_WRITE,
       .write = {.ack_check = true, .data = &readAddr, .total_bytes = 1}},
      {.command = I2C_MASTER_CMD_READ,
       .read = {.ack_value = I2C_NACK_VAL, .data = &data, .total_bytes = 1}},
      {.command = I2C_MASTER_CMD_STOP}};

  ESP_ERROR_CHECK(i2c_master_execute_defined_operations(
      _devHandle, i2c_ops, sizeof(i2c_ops) / sizeof(i2c_operation_job_t), -1));

  return data;
};

I2CBus::~I2CBus() { ESP_ERROR_CHECK(i2c_master_bus_rm_device(_devHandle)); }

SPIBus::SPIBus(spi_host_device_t spiHost, gpio_num_t csPin, int freqHz,
               bool is3WireMode)
    : _csPin(csPin), _is3WireMode(is3WireMode) {
  ESP_ERROR_CHECK(gpio_set_direction(_csPin, GPIO_MODE_OUTPUT));

  _devConfig = {
      .mode = 0,
      .clock_source = SPI_CLK_SRC_DEFAULT,
      .clock_speed_hz = freqHz,
      .spics_io_num = _csPin,
      .queue_size = 7,
  };

  if (_is3WireMode)
    _devConfig.flags = SPI_DEVICE_3WIRE;

  ESP_ERROR_CHECK(spi_bus_add_device(spiHost, &_devConfig, &_devHandle));
};

void SPIBus::writeReg(uint8_t reg, uint8_t data) const noexcept {
  uint8_t writeReg = reg & static_cast<uint8_t>(0x7F);

  spi_transaction_t spiTransaction = {
      .flags = SPI_TRANS_USE_TXDATA,
      .length = 16,
      .tx_data = {writeReg, data},
  };

  ESP_ERROR_CHECK(gpio_set_level(_csPin, 0));

  ESP_ERROR_CHECK(spi_device_polling_transmit(_devHandle, &spiTransaction));

  vTaskDelay(pdMS_TO_TICKS(1));

  ESP_ERROR_CHECK(gpio_set_level(_csPin, 1));
};

uint8_t SPIBus::readReg(uint8_t reg) const noexcept {
  uint8_t rReg = reg | static_cast<uint8_t>(0x80);

  uint8_t instrData[] = {rReg};
  spi_transaction_t spiTransaction = {
      .flags = SPI_TRANS_USE_RXDATA,
      .length = 16,
      .rxlength = 16,
      .tx_buffer = instrData,
  };

  gpio_set_level(_csPin, 0);

  ESP_ERROR_CHECK(spi_device_polling_transmit(_devHandle, &spiTransaction));

  vTaskDelay(pdMS_TO_TICKS(1));

  gpio_set_level(_csPin, 1);

  return spiTransaction.rx_data[1];
};

SPIBus::~SPIBus() {
  ESP_ERROR_CHECK(gpio_set_direction(_csPin, GPIO_MODE_DISABLE));
  ESP_ERROR_CHECK(gpio_set_pull_mode(_csPin, GPIO_FLOATING));
  ESP_ERROR_CHECK(gpio_reset_pin(_csPin));
  ESP_ERROR_CHECK(spi_bus_remove_device(_devHandle));
}

bool BME280::isInitialized() const noexcept { return _isInitialized; };

uint8_t BME280::getChipId() const noexcept {
  return _sensorBus.readReg(ID_ADDR);
};

void BME280::setMode(const Mode mode) const noexcept {
  uint8_t tempReg = _sensorBus.readReg(CTRL_MEAS_ADDR);
  _sensorBus.writeReg(CTRL_MEAS_ADDR,
                      (tempReg & ~MODE_MASK) | static_cast<uint8_t>(mode));
}

Mode BME280::getMode() const noexcept {
  return static_cast<Mode>(_sensorBus.readReg(CTRL_MEAS_ADDR) & MODE_MASK);
}

void BME280::setOversmplingTemp(
    const Oversampling oversampling) const noexcept {
  uint8_t tempReg = _sensorBus.readReg(CTRL_MEAS_ADDR);
  _sensorBus.writeReg(CTRL_MEAS_ADDR, (tempReg & ~OSRS_TEMP_MASK) |
                                          (static_cast<uint8_t>(oversampling)
                                           << OSRS_TEMP_POS));
}

Oversampling BME280::getOversamplingTemp() const noexcept {
  return static_cast<Oversampling>(
      (_sensorBus.readReg(CTRL_MEAS_ADDR) & OSRS_TEMP_MASK) >> OSRS_TEMP_POS);
}

void BME280::setOversmplingPress(Oversampling oversampling) const noexcept {
  uint8_t tempReg = _sensorBus.readReg(CTRL_MEAS_ADDR);
  _sensorBus.writeReg(CTRL_MEAS_ADDR, (tempReg & ~OSRS_PRESS_MASK) |
                                          (static_cast<uint8_t>(oversampling)
                                           << OSRS_PRESS_POS));
}

Oversampling BME280::getOversamplingPress() const noexcept {
  return static_cast<Oversampling>(
      (_sensorBus.readReg(CTRL_MEAS_ADDR) & OSRS_PRESS_MASK) >> OSRS_PRESS_POS);
}

void BME280::setOversmplingHum(const Oversampling oversampling) const noexcept {
  uint8_t tempReg = _sensorBus.readReg(CTRL_HUM_ADDR);
  _sensorBus.writeReg(CTRL_HUM_ADDR, (tempReg & ~OSRS_HUM_MASK) |
                                         static_cast<uint8_t>(oversampling));
}

Oversampling BME280::getOversamplingHum() const noexcept {
  return static_cast<Oversampling>(_sensorBus.readReg(CTRL_HUM_ADDR) &
                                   OSRS_HUM_MASK);
}

void BME280::setStandByTime(const StandByTime standByTime) const noexcept {
  uint8_t tempReg = _sensorBus.readReg(CONFIG_ADDR);
  _sensorBus.writeReg(CONFIG_ADDR,
                      (tempReg & ~STANDBY_MASK) |
                          (static_cast<uint8_t>(standByTime) << STANDBY_POS));
}

StandByTime BME280::getStandByTime() const noexcept {
  return static_cast<StandByTime>(
      (_sensorBus.readReg(CONFIG_ADDR) & STANDBY_MASK) >> STANDBY_POS);
}

void BME280::setFilter(const Filter filter) const noexcept {
  uint8_t tempReg = _sensorBus.readReg(CONFIG_ADDR);
  _sensorBus.writeReg(CONFIG_ADDR,
                      (tempReg & ~FILTER_MASK) |
                          (static_cast<uint8_t>(filter) << FILTER_POS));
}

Filter BME280::getFilter() const noexcept {
  return static_cast<Filter>((_sensorBus.readReg(CONFIG_ADDR) & FILTER_MASK) >>
                             FILTER_POS);
}

void BME280::setSpi3W(const Spi3W spi3W) const noexcept {
  uint8_t tempReg = _sensorBus.readReg(CONFIG_ADDR);
  _sensorBus.writeReg(CONFIG_ADDR,
                      (tempReg & ~SPI3W_MASK) | (static_cast<uint8_t>(spi3W)));
}

Spi3W BME280::getSpi3W() const noexcept {
  return static_cast<Spi3W>(_sensorBus.readReg(CONFIG_ADDR) & SPI3W_MASK);
}

bool BME280::isMesuring() const noexcept {
  uint8_t status = _sensorBus.readReg(STATUS_ADDR);
  return static_cast<bool>(status & STATUS_MEASURING_MASK);
}

int32_t BME280::compensationTemp() noexcept {
  int32_t adcT = getRawTemp();

  int32_t var1, var2, t;
  var1 = ((adcT >> 3) - (static_cast<int32_t>(_calibParams.digT1) << 1)) *
             static_cast<int32_t>(_calibParams.digT2) >>
         11;
  var2 = (((((adcT >> 4) - static_cast<int32_t>(_calibParams.digT1)) *
            ((adcT >> 4) - static_cast<int32_t>(_calibParams.digT1))) >>
           12) *
          static_cast<int32_t>(_calibParams.digT3)) >>
         14;
  _calibParams.tFine = var1 + var2;
  t = (_calibParams.tFine * 5 + 128) >> 8;
  return t;
}

double BME280::compensationTempDouble() noexcept {
  int32_t adcT = getRawTemp();

  double var1, var2;
  var1 = (static_cast<double>(adcT) / 16384.0 -
          (static_cast<double>(_calibParams.digT1) / 1024.0)) *
         static_cast<double>(_calibParams.digT2);
  var2 = ((static_cast<double>(adcT) / 131072.0 -
           (static_cast<double>(_calibParams.digT1) / 8192.0)) *
              static_cast<double>(adcT) / 131072.0 -
          (static_cast<double>(_calibParams.digT1) / 8192.0)) *
         static_cast<double>(_calibParams.digT3);
  _calibParams.tFine = static_cast<int32_t>(var1 + var2);
  return (var1 + var2) / 5120.0;
}

uint32_t BME280::compensationPress() const noexcept {
  int32_t adcP = getRawPress();

  int32_t var1, var2;
  uint32_t p;
  var1 = (static_cast<int32_t>(_calibParams.tFine) >> 1) -
         static_cast<int32_t>(64000);
  var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) *
         static_cast<int32_t>(_calibParams.digP6);
  var2 = var2 + ((var1 * static_cast<int32_t>(_calibParams.digP5)) << 1);
  var2 = (var2 >> 2) + (static_cast<int32_t>(_calibParams.digP4) << 16);
  var1 = (((_calibParams.digP3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) +
          ((static_cast<int32_t>(_calibParams.digP2) * var1) >> 1)) >>
         18;
  var1 = ((32768 + var1) * static_cast<int32_t>(_calibParams.digP1)) >> 15;
  if (var1 == 0) {
    return 0;
  }
  p = static_cast<uint32_t>((static_cast<int32_t>(1048576) - adcP) -
                            (var2 >> 12)) *
      3125;
  if (p < 0x8000000) {
    p = (p << 1) / static_cast<uint32_t>(var1);
  } else {
    p = (p / static_cast<uint32_t>(var1)) * 2;
  }
  var1 = (static_cast<int32_t>(_calibParams.digP9) *
          static_cast<int32_t>(((p >> 3) * (p >> 3)) >> 13)) >>
         12;
  var2 = (static_cast<int32_t>(p >> 2) *
          static_cast<int32_t>(_calibParams.digP8)) >>
         13;
  p = static_cast<uint32_t>(static_cast<int32_t>(p) +
                            ((var1 + var2 + _calibParams.digP7) >> 4));

  return p;
}

double BME280::compensationPressDouble() const noexcept {
  int32_t adcP = getRawPress();

  double var1, var2, p;
  var1 = (static_cast<double>(_calibParams.tFine) / 2.0) - 64000.0;
  var2 = var1 * var1 * static_cast<double>(_calibParams.digP6) / 32768.0;
  var2 = var2 + var1 * static_cast<double>(_calibParams.digP5) * 2.0;
  var2 = (var2 / 4.0) + (static_cast<double>(_calibParams.digP4) * 65536.0);
  var1 = (static_cast<double>(_calibParams.digP3) * var1 * var1 / 524288.0 +
          static_cast<double>(_calibParams.digP2) * var1) /
         524288.0;
  var1 = (1.0 + var1 / 32768.0) * static_cast<double>(_calibParams.digP1);
  if (var1 == 0.0) {
    return 0.0;
  }

  p = 1048576.0 - static_cast<double>(adcP);
  p = (p - (var2 / 4096.0)) * 6250.0 / var1;
  var1 = static_cast<double>(_calibParams.digP9) * p * p / 2147483648.0;
  var2 = p * static_cast<double>(_calibParams.digP8) / 32768.0;
  p = p + (var1 + var2 + static_cast<double>(_calibParams.digP7)) / 16.0;
  return p;
}

uint32_t BME280::compensationHum() const noexcept {
  int32_t adcH = getRawHum();

  int32_t h;
  h = _calibParams.tFine - static_cast<int32_t>(76800);
  h = ((((static_cast<int32_t>(adcH) << 14) -
         (static_cast<int32_t>(_calibParams.digH4) << 20) -
         (static_cast<int32_t>(_calibParams.digH5) * h)) +
        static_cast<int32_t>(16384)) >>
       15) *
      (((((((h * static_cast<int32_t>(_calibParams.digH6)) >> 10) *
           (((h * static_cast<int32_t>(_calibParams.digH3)) >> 11) +
            static_cast<int32_t>(32768))) >>
          10) +
         static_cast<int32_t>(2097152)) *
            static_cast<int32_t>(_calibParams.digH2) +
        8192) >>
       14);
  h = h - (((((h >> 15) * (h >> 15)) >> 7) *
            static_cast<int32_t>(_calibParams.digH1)) >>
           4);
  h = h < 0 ? 0 : h;
  h = h > 419430400 ? 419430400 : h;
  return static_cast<uint32_t>(h >> 12);
}

double BME280::compensationHumDouble() const noexcept {
  int32_t adcH = getRawHum();

  double h;

  h = static_cast<double>(_calibParams.tFine) - 76800.0;
  h = (adcH - (static_cast<double>(_calibParams.digH4) * 64.0 +
               static_cast<double>(_calibParams.digH5) / 16384.0 * h)) *
      (static_cast<double>(_calibParams.digH2) / 65536.0 *
       (1.0 +
        static_cast<double>(_calibParams.digH6) / 67108864.0 * h *
            (1.0 + static_cast<double>(_calibParams.digH3) / 67108864.0 * h)));
  h = h * (1.0 - static_cast<double>(_calibParams.digH1) * h / 524288.0);
  if (h > 100.0) {
    h = 100.0;
  } else if (h < 0.0) {
    h = 0.0;
  }
  return h;
}

void BME280::reset() const noexcept {
  _sensorBus.writeReg(RESET_ADDR, RESET_VALUE);
}

bool BME280::isCalibrationUpdateDone() const noexcept {
  uint8_t status = _sensorBus.readReg(STATUS_ADDR);
  return static_cast<bool>(status & STATUS_UPDATE_MASK);
}

int32_t BME280::getRawTemp() const noexcept {
  int32_t xlsb = static_cast<int32_t>(_sensorBus.readReg(TEMP_XLSB_ADDR)) >> 4;
  int32_t lsb = static_cast<int32_t>(_sensorBus.readReg(TEMP_LSB_ADDR)) << 4;
  int32_t msb = static_cast<int32_t>(_sensorBus.readReg(TEMP_MSB_ADDR)) << 12;

  return (msb | lsb | xlsb);
}

int32_t BME280::getRawPress() const noexcept {
  int32_t xlsb = static_cast<int32_t>(_sensorBus.readReg(PRESS_XLSB_ADDR)) >> 4;
  int32_t lsb = static_cast<int32_t>(_sensorBus.readReg(PRESS_LSB_ADDR)) << 4;
  int32_t msb = static_cast<int32_t>(_sensorBus.readReg(PRESS_MSB_ADDR)) << 12;

  return (msb | lsb | xlsb);
}

int32_t BME280::getRawHum() const noexcept {
  int32_t lsb = static_cast<int32_t>(_sensorBus.readReg(HUM_LSB_ADDR));
  int32_t msb = static_cast<int32_t>(_sensorBus.readReg(HUM_MSB_ADDR)) << 8;

  return (msb | lsb);
}

void BME280::readCalibData() noexcept {
  _calibParams.digT1 =
      (static_cast<uint16_t>(_sensorBus.readReg(DIG_T1_MSB_ADDR)) << 8) |
      static_cast<uint16_t>(_sensorBus.readReg(DIG_T1_LSB_ADDR));
  _calibParams.digT2 =
      (static_cast<int16_t>(_sensorBus.readReg(DIG_T2_MSB_ADDR)) << 8) |
      static_cast<int16_t>(_sensorBus.readReg(DIG_T2_LSB_ADDR));
  _calibParams.digT3 =
      (static_cast<int16_t>(_sensorBus.readReg(DIG_T3_MSB_ADDR)) << 8) |
      static_cast<int16_t>(_sensorBus.readReg(DIG_T3_LSB_ADDR));

  _calibParams.digP1 =
      (static_cast<uint16_t>(_sensorBus.readReg(DIG_P1_MSB_ADDR)) << 8) |
      static_cast<uint16_t>(_sensorBus.readReg(DIG_P1_LSB_ADDR));
  _calibParams.digP2 =
      (static_cast<int16_t>(_sensorBus.readReg(DIG_P2_MSB_ADDR)) << 8) |
      static_cast<int16_t>(_sensorBus.readReg(DIG_P2_LSB_ADDR));
  _calibParams.digP3 =
      (static_cast<int16_t>(_sensorBus.readReg(DIG_P3_MSB_ADDR)) << 8) |
      static_cast<int16_t>(_sensorBus.readReg(DIG_P3_LSB_ADDR));
  _calibParams.digP4 =
      (static_cast<int16_t>(_sensorBus.readReg(DIG_P4_MSB_ADDR)) << 8) |
      static_cast<int16_t>(_sensorBus.readReg(DIG_P4_LSB_ADDR));
  _calibParams.digP5 =
      (static_cast<int16_t>(_sensorBus.readReg(DIG_P5_MSB_ADDR)) << 8) |
      static_cast<int16_t>(_sensorBus.readReg(DIG_P5_LSB_ADDR));
  _calibParams.digP6 =
      (static_cast<int16_t>(_sensorBus.readReg(DIG_P6_MSB_ADDR)) << 8) |
      static_cast<int16_t>(_sensorBus.readReg(DIG_P6_LSB_ADDR));
  _calibParams.digP7 =
      (static_cast<int16_t>(_sensorBus.readReg(DIG_P7_MSB_ADDR)) << 8) |
      static_cast<int16_t>(_sensorBus.readReg(DIG_P7_LSB_ADDR));
  _calibParams.digP8 =
      (static_cast<int16_t>(_sensorBus.readReg(DIG_P8_MSB_ADDR)) << 8) |
      static_cast<int16_t>(_sensorBus.readReg(DIG_P8_LSB_ADDR));
  _calibParams.digP9 =
      (static_cast<int16_t>(_sensorBus.readReg(DIG_P9_MSB_ADDR)) << 8) |
      static_cast<int16_t>(_sensorBus.readReg(DIG_P9_LSB_ADDR));

  _calibParams.digH1 = _sensorBus.readReg(DIG_H1_ADDR);
  _calibParams.digH2 =
      (static_cast<int16_t>(_sensorBus.readReg(DIG_H2_MSB_ADDR)) << 8) |
      static_cast<int16_t>(_sensorBus.readReg(DIG_H2_LSB_ADDR));
  _calibParams.digH3 = _sensorBus.readReg(DIG_H3_ADDR);
  _calibParams.digH4 =
      (static_cast<int16_t>(_sensorBus.readReg(DIG_H4_MSB_ADDR)) << 4) |
      (static_cast<int16_t>(_sensorBus.readReg(DIG_H4_LSB_ADDR) & 0x0F));
  _calibParams.digH5 =
      (static_cast<int16_t>(_sensorBus.readReg(DIG_H5_MSB_ADDR)) << 4) |
      (static_cast<int16_t>(_sensorBus.readReg(DIG_H5_LSB_ADDR) >> 4));
  _calibParams.digH6 = static_cast<int8_t>(_sensorBus.readReg(DIG_H6_ADDR));
}

} // namespace BME280
