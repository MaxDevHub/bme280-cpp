#pragma once

#include <type_traits>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace BME280 {

constexpr uint8_t CHIP_ID = 0x60;
constexpr uint8_t RESET_VALUE = 0xB6;

constexpr uint8_t HUM_LSB_ADDR = 0xFE;
constexpr uint8_t HUM_MSB_ADDR = 0xFD;
constexpr uint8_t TEMP_XLSB_ADDR = 0xFC;
constexpr uint8_t TEMP_LSB_ADDR = 0xFB;
constexpr uint8_t TEMP_MSB_ADDR = 0xFA;
constexpr uint8_t PRESS_XLSB_ADDR = 0xF9;
constexpr uint8_t PRESS_LSB_ADDR = 0xF8;
constexpr uint8_t PRESS_MSB_ADDR = 0xF7;
constexpr uint8_t CONFIG_ADDR = 0xF5;
constexpr uint8_t CTRL_MEAS_ADDR = 0xF4;
constexpr uint8_t STATUS_ADDR = 0xF3;
constexpr uint8_t CTRL_HUM_ADDR = 0xF2;
constexpr uint8_t RESET_ADDR = 0xE0;
constexpr uint8_t ID_ADDR = 0xD0;

constexpr int TEMP_XLSB_POS = 4;
constexpr int PRESS_XLSB_POS = 4;
constexpr int FILTER_POS = 2;
constexpr int STANDBY_POS = 5;
constexpr int OSRS_PRESS_POS = 2;
constexpr int OSRS_TEMP_POS = 5;
constexpr int STATUS_MEASURING_POS = 3;

constexpr uint8_t SPI3W_MASK = 0x01;
constexpr uint8_t FILTER_MASK = 0x1C;
constexpr uint8_t STANDBY_MASK = 0xE0;
constexpr uint8_t MODE_MASK = 0x03;
constexpr uint8_t OSRS_PRESS_MASK = 0x1C;
constexpr uint8_t OSRS_TEMP_MASK = 0xE0;
constexpr uint8_t STATUS_UPDATE_MASK = 0x01;
constexpr uint8_t STATUS_MEASURING_MASK = 0x08;
constexpr uint8_t OSRS_HUM_MASK = 0x07;

constexpr uint8_t DIG_T1_LSB_ADDR = 0x88;
constexpr uint8_t DIG_T1_MSB_ADDR = 0x89;
constexpr uint8_t DIG_T2_LSB_ADDR = 0x8A;
constexpr uint8_t DIG_T2_MSB_ADDR = 0x8B;
constexpr uint8_t DIG_T3_LSB_ADDR = 0x8C;
constexpr uint8_t DIG_T3_MSB_ADDR = 0x8D;
constexpr uint8_t DIG_P1_LSB_ADDR = 0x8E;
constexpr uint8_t DIG_P1_MSB_ADDR = 0x8F;
constexpr uint8_t DIG_P2_LSB_ADDR = 0x90;
constexpr uint8_t DIG_P2_MSB_ADDR = 0x91;
constexpr uint8_t DIG_P3_LSB_ADDR = 0x92;
constexpr uint8_t DIG_P3_MSB_ADDR = 0x93;
constexpr uint8_t DIG_P4_LSB_ADDR = 0x94;
constexpr uint8_t DIG_P4_MSB_ADDR = 0x95;
constexpr uint8_t DIG_P5_LSB_ADDR = 0x96;
constexpr uint8_t DIG_P5_MSB_ADDR = 0x97;
constexpr uint8_t DIG_P6_LSB_ADDR = 0x98;
constexpr uint8_t DIG_P6_MSB_ADDR = 0x99;
constexpr uint8_t DIG_P7_LSB_ADDR = 0x9A;
constexpr uint8_t DIG_P7_MSB_ADDR = 0x9B;
constexpr uint8_t DIG_P8_LSB_ADDR = 0x9C;
constexpr uint8_t DIG_P8_MSB_ADDR = 0x9D;
constexpr uint8_t DIG_P9_LSB_ADDR = 0x9E;
constexpr uint8_t DIG_P9_MSB_ADDR = 0x9F;
constexpr uint8_t DIG_H1_ADDR = 0xA1;
constexpr uint8_t DIG_H2_LSB_ADDR = 0xE1;
constexpr uint8_t DIG_H2_MSB_ADDR = 0xE2;
constexpr uint8_t DIG_H3_ADDR = 0xE3;
constexpr uint8_t DIG_H4_LSB_ADDR = 0xE5;
constexpr uint8_t DIG_H4_MSB_ADDR = 0xE4;
constexpr uint8_t DIG_H5_LSB_ADDR = 0xE5;
constexpr uint8_t DIG_H5_MSB_ADDR = 0xE6;
constexpr uint8_t DIG_H6_ADDR = 0xE7;

enum class Addr : uint8_t { BME280_ADDR_PRIM = 0x76, BME280_ADDR_SEC = 0x77 };

enum class Mode : uint8_t {
  SLEEP_MODE = 0x00,
  FORCED_MODE = 0x01,
  NORMAL_MODE = 0x03
};

enum class Oversampling : uint8_t {
  SKIPPED = 0x00,
  OVERSAMPLING_1X = 0x01,
  OVERSAMPLING_2X = 0x02,
  OVERSAMPLING_4X = 0x03,
  OVERSAMPLING_8X = 0x04,
  OVERSAMPLING_16X = 0x05
};

enum class Filter : uint8_t {
  FILTER_OFF = 0x00,
  FILTER_2 = 0x01,
  FILTER_4 = 0x02,
  FILTER_8 = 0x03,
  FILTER_16 = 0x04
};

enum class StandByTime : uint8_t {
  STANDBY_TIME_0_5_MS = 0x00,
  STANDBY_TIME_62_5_MS = 0x01,
  STANDBY_TIME_125_MS = 0x02,
  STANDBY_TIME_250_MS = 0x03,
  STANDBY_TIME_500_MS = 0x04,
  STANDBY_TIME_1000_MS = 0x05,
  STANDBY_TIME_10_MS = 0x06,
  STANDBY_TIME_20_MS = 0x07
};

enum class Spi3W : uint8_t { OFF = 0x00, ON = 0x01 };

class ISensorBus {
public:
  virtual void writeReg(uint8_t reg, uint8_t data) const noexcept = 0;
  [[nodiscard]]
  virtual uint8_t readReg(uint8_t reg) const noexcept = 0;
  virtual ~ISensorBus() = default;
};

class I2CBus final : public ISensorBus {
public:
  I2CBus(i2c_master_bus_handle_t &busHandle, uint32_t freqHz,
         Addr devAddr = Addr::BME280_ADDR_PRIM) noexcept;

  void writeReg(uint8_t reg, uint8_t data) const noexcept override;
  [[nodiscard]]
  uint8_t readReg(uint8_t reg) const noexcept override;

private:
  i2c_master_dev_handle_t _devHandle;
  i2c_device_config_t _devConfig;

  Addr _devAddr;
};

class SPIBus final : public ISensorBus {
public:
  SPIBus(spi_host_device_t spiHost, gpio_num_t csPin, int freqHz,
         bool is3WireMode = false);

  void writeReg(uint8_t reg, uint8_t data) const noexcept override;
  [[nodiscard]]
  uint8_t readReg(uint8_t reg) const noexcept override;

  inline bool getIs3WireMode() const noexcept { return _is3WireMode; }

private:
  spi_device_interface_config_t _devConfig;
  spi_device_handle_t _devHandle;

  gpio_num_t _csPin;
  bool _is3WireMode;
};

template <typename T>
concept SensorBus = std::is_base_of_v<ISensorBus, T>;

class BME280 {
public:
  BME280() = delete;
  BME280(const BME280 &) = delete;
  BME280(BME280 &&) = delete;

  template <SensorBus T>
  explicit BME280(const T &sensorBus) noexcept : _sensorBus(sensorBus) {
    reset();

    if constexpr (std::is_same_v<SPIBus, T>) {
      if (sensorBus.getIs3WireMode())
        setSpi3W(Spi3W::ON);
    }

    while (isCalibrationUpdateDone()) {
      vTaskDelay(pdMS_TO_TICKS(1));
    };

    if (getChipId() != CHIP_ID) {
      _isInitialized = false;
      return;
    }

    readCalibData();
  };

  void setInitConfig() const noexcept;
  [[nodiscard]]
  bool isInitialized() const noexcept;

  [[nodiscard]]
  uint8_t getChipId() const noexcept;

  void setMode(Mode mode) const noexcept;
  [[nodiscard]]
  Mode getMode() const noexcept;

  void setOversmplingTemp(const Oversampling oversampling) const noexcept;
  [[nodiscard]]
  Oversampling getOversamplingTemp() const noexcept;

  void setOversmplingPress(const Oversampling oversampling) const noexcept;
  [[nodiscard]]
  Oversampling getOversamplingPress() const noexcept;

  void setOversmplingHum(const Oversampling oversampling) const noexcept;
  [[nodiscard]]
  Oversampling getOversamplingHum() const noexcept;

  void setStandByTime(const StandByTime standByTime) const noexcept;
  [[nodiscard]]
  StandByTime getStandByTime() const noexcept;

  void setFilter(const Filter filter) const noexcept;
  [[nodiscard]]
  Filter getFilter() const noexcept;

  [[nodiscard]]
  bool isMesuring() const noexcept;

  [[nodiscard]]
  int32_t compensationTemp() noexcept;
  [[nodiscard]]
  double compensationTempDouble() noexcept;

  [[nodiscard]]
  uint32_t compensationPress() const noexcept;
  [[nodiscard]]
  double compensationPressDouble() const noexcept;

  [[nodiscard]]
  uint32_t compensationHum() const noexcept;
  [[nodiscard]]
  double compensationHumDouble() const noexcept;

private:
  const ISensorBus &_sensorBus;

  bool _isInitialized{true};

  void reset() const noexcept;

  [[nodiscard]]
  bool isCalibrationUpdateDone() const noexcept;

  void setSpi3W(const Spi3W spi3W) const noexcept;
  [[nodiscard]]
  Spi3W getSpi3W() const noexcept;

  [[nodiscard]]
  int32_t getRawTemp() const noexcept;
  [[nodiscard]]
  int32_t getRawPress() const noexcept;
  [[nodiscard]]
  int32_t getRawHum() const noexcept;

  void readCalibData() noexcept;

  struct {
    uint16_t digT1;
    int16_t digT2;
    int16_t digT3;

    uint16_t digP1;
    int16_t digP2;
    int16_t digP3;
    int16_t digP4;
    int16_t digP5;
    int16_t digP6;
    int16_t digP7;
    int16_t digP8;
    int16_t digP9;

    uint8_t digH1;
    int16_t digH2;
    uint8_t digH3;
    int16_t digH4;
    int16_t digH5;
    int8_t digH6;

    int32_t tFine;
  } _calibParams{};
};

} // namespace BME280
