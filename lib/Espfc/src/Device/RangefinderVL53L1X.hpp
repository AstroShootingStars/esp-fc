#pragma once

#include "Device/BusDevice.hpp"
#include "Device/RangefinderDevice.hpp"

namespace Espfc::Device {

class RangefinderVL53L1X : public RangefinderDevice
{
public:
  int begin(BusDevice* bus) final;
  int begin(BusDevice* bus, uint8_t addr) final;

  RangefinderDeviceType getType() const final;
  int getRate() const final { return 20; } // 20 Hz

  uint16_t readDistance() final; // mm
  bool testConnection() final;

private:
  static constexpr uint8_t DEVICE_ADDRESS = 0x29;
  static constexpr uint16_t WHO_AM_I_REG = 0x010F;
  static constexpr uint8_t WHO_AM_I_VALUE = 0xEA;

  static constexpr uint16_t SYSTEM_START = 0x0087;
  static constexpr uint16_t GPIO_TIO_HV_STATUS = 0x0031;
  static constexpr uint16_t RANGE_RESULT = 0x0096;
  static constexpr uint16_t SYSTEM_INTERRUPT_CLEAR = 0x0086;

  int8_t writeReg(uint16_t reg, uint8_t val);
  int8_t readReg(uint16_t reg, uint8_t& val);
  int8_t readReg16(uint16_t reg, uint16_t& val);

  void init();
};

} // namespace Espfc::Device
