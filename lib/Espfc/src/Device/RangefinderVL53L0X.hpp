#pragma once

#include "Device/BusDevice.hpp"
#include "Device/RangefinderDevice.hpp"

namespace Espfc::Device {

class RangefinderVL53L0X : public RangefinderDevice
{
public:
  int begin(BusDevice* bus) final;
  int begin(BusDevice* bus, uint8_t addr) final;

  RangefinderDeviceType getType() const final;
  int getRate() const final { return 10; } // 10 Hz

  uint16_t readDistance() final; // mm
  bool testConnection() final;

private:
  static constexpr uint8_t DEVICE_ADDRESS = 0x29;
  static constexpr uint8_t WHO_AM_I_REG = 0xC0;
  static constexpr uint8_t WHO_AM_I_VALUE = 0xEE;
  
  // VL53L0X registers
  static constexpr uint8_t SYSRANGE_START = 0x00;
  static constexpr uint8_t RESULT_RANGE_STATUS = 0x14;
  static constexpr uint8_t RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN = 0xBC;
  static constexpr uint8_t RESULT_CORE_RANGING_TOTAL_EVENTS_RTN = 0xC0;
  static constexpr uint8_t RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF = 0xD0;
  static constexpr uint8_t RESULT_CORE_RANGING_TOTAL_EVENTS_REF = 0xD4;
  static constexpr uint8_t RESULT_PEAK_SIGNAL_RATE_REF = 0xB6;
  static constexpr uint8_t ALGO_PART_TO_PART_RANGE_OFFSET = 0x28;
  static constexpr uint8_t I2C_SLAVE_DEVICE_ADDRESS = 0x8A;
  static constexpr uint8_t MSRC_CONFIG_CONTROL = 0x60;
  static constexpr uint8_t PRE_RANGE_CONFIG_MIN_SNR = 0x27;
  static constexpr uint8_t PRE_RANGE_CONFIG_VALID_PHASE_HIGH = 0x56;
  static constexpr uint8_t PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT = 0x64;
  static constexpr uint8_t FINAL_RANGE_CONFIG_MIN_SNR = 0x67;
  static constexpr uint8_t FINAL_RANGE_CONFIG_VALID_PHASE_HIGH = 0x64;
  static constexpr uint8_t FINAL_RANGE_MIN_COUNT_RATE_RTN_LIMIT = 0x44;
  static constexpr uint8_t PRE_RANGE_CONFIG_SIGMA_THRESH_HI = 0x61;
  static constexpr uint8_t PRE_RANGE_CONFIG_SIGMA_THRESH_LO = 0x62;
  static constexpr uint8_t PRE_RANGE_CONFIG_VCSEL_PERIOD = 0x50;
  static constexpr uint8_t PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x51;
  static constexpr uint8_t PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x52;
  static constexpr uint8_t SYSTEM_HISTOGRAM_BIN = 0x81;
  static constexpr uint8_t HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT = 0x33;
  static constexpr uint8_t HISTOGRAM_CONFIG_READOUT_CTRL = 0x55;
  static constexpr uint8_t FINAL_RANGE_CONFIG_VCSEL_PERIOD = 0x70;
  static constexpr uint8_t FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x71;
  static constexpr uint8_t FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x72;
  static constexpr uint8_t CROSSTALK_COMPENSATION_PEAK_RATE_MCPS = 0x20;
  static constexpr uint8_t SYSTEM_SEQUENCE_CONFIG = 0x01;
  static constexpr uint8_t SYSTEM_RANGE_CONFIG = 0x09;
  static constexpr uint8_t SYSTEM_INTERMEASUREMENT_PERIOD = 0x04;
  static constexpr uint8_t SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A;
  static constexpr uint8_t GPIO_HV_MUX_ACTIVE_HIGH = 0x84;
  static constexpr uint8_t SYSTEM_INTERRUPT_CLEAR = 0x0B;
  static constexpr uint8_t RESULT_INTERRUPT_STATUS = 0x13;
  static constexpr uint8_t POWER_MANAGEMENT_GO1_POWER_STATUS = 0x14;

  int8_t writeReg(uint8_t reg, uint8_t val);
  int8_t writeReg16(uint8_t reg, uint16_t val);
  int8_t readReg(uint8_t reg, uint8_t& val);
  int8_t readReg16(uint8_t reg, uint16_t& val);
  int8_t readReg32(uint8_t reg, uint32_t& val);
  
  void init();
};

} // namespace Espfc::Device
