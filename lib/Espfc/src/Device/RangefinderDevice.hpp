#pragma once

#include "BusAwareDevice.hpp"

namespace Espfc::Device {

enum RangefinderDeviceType
{
  RANGEFINDER_DEFAULT = 0,
  RANGEFINDER_NONE = 1,
  RANGEFINDER_VL53L0X = 2,
  RANGEFINDER_VL6180X = 3,
  RANGEFINDER_VL53L1X = 4,
  RANGEFINDER_MAX
};

class RangefinderDevice : public BusAwareDevice
{
public:
  using DeviceType = RangefinderDeviceType;

  virtual ~RangefinderDevice() = default;

  virtual int begin(BusDevice* bus) = 0;
  virtual int begin(BusDevice* bus, uint8_t addr) = 0;

  virtual RangefinderDeviceType getType() const = 0;
  virtual int getRate() const { return 10; } // Hz

  virtual uint16_t readDistance() = 0; // mm
  virtual bool testConnection() = 0;

  static const char** getNames();
  static const char* getName(DeviceType type);
};

} // namespace Espfc::Device
