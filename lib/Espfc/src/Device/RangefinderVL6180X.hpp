#pragma once

#include "Device/BusDevice.hpp"
#include "Device/RangefinderDevice.hpp"

namespace Espfc::Device {

class RangefinderVL6180X : public RangefinderDevice
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
  static constexpr uint8_t WHO_AM_I_REG = 0x000;
  static constexpr uint8_t WHO_AM_I_VALUE = 0xB4;
  
  // VL6180X registers
  static constexpr uint8_t SYSTEM_FRESH_OUT_OF_RESET = 0x016;
  static constexpr uint8_t SYSRANGE_START = 0x018;
  static constexpr uint8_t SYSRANGE_RESULT_RANGE = 0x062;
  static constexpr uint8_t SYSTEM_INTERRUPT_CLEAR = 0x015;
  static constexpr uint8_t RESULT_INTERRUPT_STATUS_GPIO = 0x04F;

  int8_t writeReg(uint16_t reg, uint8_t val);
  int8_t readReg(uint16_t reg, uint8_t& val);
  int8_t readReg16(uint16_t reg, uint16_t& val);
  
  void init();
};

} // namespace Espfc::Device
