#pragma once

#include <type_traits>

#include "driver/i2c_master.h"
#include "driver/spi_master.h"
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

/**
 * @brief Interface for sensor communication buses
 */
class ISensorBus {
public:
  /**
   * @brief Virtual destructor for proper polymorphic cleanup
   */
  virtual ~ISensorBus() = default;

  /**
   * @brief Read a register from the sensor
   * @param reg Register address to read
   * @return The value read from the register
   */
  [[nodiscard]] virtual uint8_t readReg(uint8_t reg) const noexcept = 0;

  /**
   * @brief Write to a sensor register
   * @param reg Register address to write
   * @param data Data byte to write
   */
  virtual void writeReg(uint8_t reg, uint8_t data) const noexcept = 0;
};

/**
 * @brief I2C bus implementation for sensor communication
 * @implements ISensorBus
 * @final
 */
class I2CBus final : public ISensorBus {
public:
  /**
   * @brief Constructor a new I2CBus instance
   * @param busHandle Reference to initialized I2C master bus handle
   * @param freqHz I2C clock frequency in Hz
   * @param devaddr Device address (from Addr enum)
   */
  I2CBus(i2c_master_bus_handle_t &busHandle, uint32_t freqHz,
         Addr devAddr = Addr::BME280_ADDR_PRIM) noexcept;

  /**
   * @brief Destroy the I2CBus instance
   */
  ~I2CBus() override;

  /**
   * @brief Write to a sensor register
   * @param reg Register address to write
   * @param data Data byte to write
   */
  void writeReg(uint8_t reg, uint8_t data) const noexcept override;

  /**
   * @brief Read from a sensor register
   * @param reg Register address to read
   * @return The read register value
   */
  [[nodiscard]] uint8_t readReg(uint8_t reg) const noexcept override;

private:
  i2c_master_dev_handle_t _devHandle; ///< I2C device handle
  i2c_device_config_t _devConfig;     ///< I2C device configuration

  Addr _devAddr; ///< Device I2C address
};

/**
 * @brief SPI bus implementation for sensor communication
 * @implements ISensorBus
 * @final
 */
class SPIBus final : public ISensorBus {
public:
  /**
   * @brief Constructor a new SPIBus instance
   * @param spiHost SPI host device
   * @param csPin Chip select GPIO pin
   * @param freqHz SPI clock frequency in Hz
   * @param is3WireMode Enable 3-wire (half-duplex) mode
   */
  SPIBus(spi_host_device_t spiHost, gpio_num_t csPin, int freqHz,
         bool is3WireMode = false);

  /**
   * @brief Destroy the SPIBus instance
   */
  ~SPIBus() override;

  /**
   * @brief Write to a sensor register
   * @param reg Register address to write
   * @param data Data byte to write
   */
  void writeReg(uint8_t reg, uint8_t data) const noexcept override;

  /**
   * @brief Read from a sensor register
   * @param reg Register address to read
   * @return The read register value
   */
  [[nodiscard]] uint8_t readReg(uint8_t reg) const noexcept override;

  /**
   * @brief Check if 3-wire mode is enable
   * @return true if in 3-wire mode, false for 4-wire
   */
  inline bool getIs3WireMode() const noexcept { return _is3WireMode; }

private:
  spi_device_interface_config_t _devConfig; ///< SPI device configuration
  spi_device_handle_t _devHandle;           ///< SPI device handle

  gpio_num_t _csPin; ///< Chip select GPIO pin
  bool _is3WireMode; ///< 3-wire mode flag
};

template <typename T>
concept SensorBus = std::is_base_of_v<ISensorBus, T>;

class BME280 {
public:
  BME280() = delete;
  BME280(const BME280 &) = delete;
  BME280(BME280 &&) = delete;

  /**
   * @brief Constructor a new BME280 sensor instance
   * @tparam T Type of sensor bus must base of ISensorBus
   * @param sensorBus Reference to the communication bus (SPI/I2C)
   * @warning If chip ID verification fails, sensor will be marked as not
   * initialized
   */
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

  /**
   * @brief Checks if the sensor was successfull initialized
   * @return true if sensor is properly initialized, false if initialization
   * failed
   */
  [[nodiscard]]
  bool isInitialized() const noexcept;

  /**
   * @brief Reads the sensor's chip ID
   * @warning A return value other than 0x60 indicates either:
   *          - A different sensor model is connected
   *          - There may be a communication problem with the device
   */
  [[nodiscard]]
  uint8_t getChipId() const noexcept;

  /**
   * @brief Sets the sensor's operating mode
   * @param mode The operating mode to set (from Mode enum)
   */
  void setMode(Mode mode) const noexcept;

  /**
   * @brief Gets the sensor's operating mode
   * @return The operating mode (from Mode enum)
   */
  [[nodiscard]]
  Mode getMode() const noexcept;

  /**
   * @brief Sets the sensor's temperature oversampling
   * @param oversampling The temperature oversampling to set (from Oversampling
   * enum)
   */
  void setOversmplingTemp(const Oversampling oversampling) const noexcept;

  /**
   * @brief Gets the sensor's temperature oversampling
   * @return The current temperature oversampling (from Oversampling enum)
   */
  [[nodiscard]]
  Oversampling getOversamplingTemp() const noexcept;

  /**
   * @brief Sets the sensor's pressure oversampling
   * @param oversampling The pressure oversampling to set (from Oversampling
   * enum)
   */
  void setOversmplingPress(const Oversampling oversampling) const noexcept;

  /**
   * @brief Gets the sensor's pressure oversampling
   * @return The current pressure oversampling (from Oversampling enum)
   */
  [[nodiscard]]
  Oversampling getOversamplingPress() const noexcept;

  /**
   * @brief Sets the sensor's humidity oversampling
   * @param oversampling The humidity oversampling to set (from Oversamplig
   * enum)
   */
  void setOversmplingHum(const Oversampling oversampling) const noexcept;

  /**
   * @brief Gets the sensor's humidity oversampling
   * @return The current humidity oversampling (from Oversamplig enum)
   */
  [[nodiscard]]
  Oversampling getOversamplingHum() const noexcept;

  /**
   * @brief Sets the standby time between measurments in NORMAL mode
   * @param standByTime The standby duration (from StandByTime enum)
   */
  void setStandByTime(const StandByTime standByTime) const noexcept;

  /**
   * @brief Gets the standby time between measurments in NORMAL mode
   * @return The standby duration (from StandByTime enum)
   */
  [[nodiscard]]
  StandByTime getStandByTime() const noexcept;

  /**
   * @brief Sets the config IIR filter
   * @param filter The filter coefficient (from Filter enum)
   */
  void setFilter(const Filter filter) const noexcept;

  /**
   * @brief Gets the config IIR filter
   * @return The filter coefficient (from Filter enum)
   */
  [[nodiscard]]
  Filter getFilter() const noexcept;

  /**
   * @brief Checks if the sensor is currently performing a measurment
   * @return true if a measurment is in progress, false othetwise
   */
  [[nodiscard]]
  bool isMesuring() const noexcept;

  /**
   * @brief Performs temperature compensation on raw sensor data
   * @return Return temperature in DegC, resolution is 0.01 DegC. Output value
   * of "5123" equals 51.23 DegC
   */
  [[nodiscard]]
  int32_t compensationTemp() noexcept;

  /**
   * @brief Performs temperature compensation on raw sensor data
   * @return Returns temperature in DegC, double precision. Outpu value of
   * "51.23" equals 51.23 DegC.
   */
  [[nodiscard]]
  double compensationTempDouble() noexcept;

  /**
   * @brief Performs pressure compensation on raw sensor data
   * @return Returns pressure in Pa as unsigned 32 bit integer. Output value of
   * "96386" equals 96386 Pa = 963.86 hPa
   */
  [[nodiscard]]
  uint32_t compensationPress() const noexcept;

  /**
   * @brief Performs pressure compensation on raw sensor data
   * @return Returns pressure in Pa as double. Output value of "96386.2" equals
   * 96386.2 Pa = 963.862 hPa
   */
  [[nodiscard]]
  double compensationPressDouble() const noexcept;

  /**
   * @brief Performs humidity compensation on raw sensor data
   * @return Returns humidity in %RH as usigned 32 bit integer in Q22.10 format
   * (22 integer and 10 fractional bits). Output value of "47445" represents
   * 47445/1024 = 46.333 %RH
   */
  [[nodiscard]]
  uint32_t compensationHum() const noexcept;

  /**
   * @brief Performs humidity compensation on raw sensor data
   * @return Returns humidity in %rH as double. Output value of "46.332"
   * represents 46.332 %rH
   */
  [[nodiscard]]
  double compensationHumDouble() const noexcept;

private:
  /**
   * @brief Reference to the sensor communication bus interface
   */
  const ISensorBus &_sensorBus;

  /**
   * @brief Tracks whether the sensor was successfully initialized
   */
  bool _isInitialized{true};

  /**
   * @brief Performs a soft reset of the BME280 sensor
   */
  void reset() const noexcept;

  /**
   * @brief Checks if calibration data copying is complete
   * @return true if calibration data is ready, false if still being copied
   */
  [[nodiscard]]
  bool isCalibrationUpdateDone() const noexcept;

  /**
   * @brief Sets the SPI interface mode (3-wire or standart 4-wire)
   * @param spi3W The SPI mode to set (from Spi3W enum)
   */
  void setSpi3W(const Spi3W spi3W) const noexcept;

  /**
   * @brief Gets the SPI interface mode (3-wire or standart 4-wire)
   * @return The SPI mode (from Spi3W enum)
   */
  [[nodiscard]]
  Spi3W getSpi3W() const noexcept;

  /**
   * @brief Reads the uncompesated temperature value from sensor
   * @return Raw temperature ADC value
   */
  [[nodiscard]]
  int32_t getRawTemp() const noexcept;

  /**
   * @brief Reads the uncompensated pressure value from sensor
   * @return Raw pressure ADC value
   */
  [[nodiscard]]
  int32_t getRawPress() const noexcept;

  /**
   * @brief Reads the uncompensated humidity value from sensor
   * @return Raw humidity ADC value
   */
  [[nodiscard]]
  int32_t getRawHum() const noexcept;

  /**
   * @brief Reads and stores all calibration coefficients from the sensor's NVM
   */
  void readCalibData() noexcept;

  /**
   * @brief Storage for all BME280 sensor calibration coefficients
   */
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
