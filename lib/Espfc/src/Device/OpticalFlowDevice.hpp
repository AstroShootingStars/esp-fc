#pragma once

#include <cstdint>
#include "Device/BusAwareDevice.hpp"

namespace Espfc::Device {

enum OpticalFlowDeviceType
{
  OPFLOW_DEFAULT = 0,
  OPFLOW_NONE = 1,
  OPFLOW_MSP = 2,
  OPFLOW_MATEK_3901_L0X = 3,
  OPFLOW_MICOAIR_MTF_02P = 4,
  OPFLOW_MAX
};

struct OpticalFlowData
{
  uint8_t quality = 0;
  int32_t motionX = 0;
  int32_t motionY = 0;
  uint32_t timeUs = 0;
};

class OpticalFlowDevice : public BusAwareDevice
{
public:
  using DeviceType = OpticalFlowDeviceType;

  virtual ~OpticalFlowDevice() = default;

  virtual int begin(BusDevice* bus) = 0;
  virtual int begin(BusDevice* bus, uint8_t addr) = 0;

  virtual DeviceType getType() const = 0;
  virtual int getRate() const { return 100; } // Hz

  virtual bool readMotion(OpticalFlowData& data) = 0;
  virtual bool testConnection() = 0;

  static const char** getNames();
  static const char* getName(DeviceType type);
};

} // namespace Espfc::Device
