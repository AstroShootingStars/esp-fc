#pragma once

#include <cstdint>
#include "Device/BusAwareDevice.hpp"

namespace Espfc::Device {

enum OledDeviceType
{
  OLED_DEFAULT = 0,
  OLED_NONE = 1,
  OLED_SSD1306 = 2,
  OLED_MAX
};

class OledDevice : public BusAwareDevice
{
public:
  using DeviceType = OledDeviceType;

  virtual ~OledDevice() = default;

  virtual int begin(BusDevice* bus) = 0;
  virtual int begin(BusDevice* bus, uint8_t addr) = 0;

  virtual DeviceType getType() const = 0;
  virtual bool testConnection() = 0;

  static const char** getNames();
  static const char* getName(DeviceType type);
};

} // namespace Espfc::Device
