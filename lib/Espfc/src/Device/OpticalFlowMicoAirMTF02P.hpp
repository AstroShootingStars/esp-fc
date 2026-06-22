#pragma once

#include "Device/OpticalFlowDevice.hpp"

namespace Espfc::Device {

class OpticalFlowMicoAirMTF02P : public OpticalFlowDevice
{
public:
  int begin(BusDevice* bus) final;
  int begin(BusDevice* bus, uint8_t addr) final;

  DeviceType getType() const final { return OPFLOW_MICOAIR_MTF_02P; }
  int getRate() const final { return 100; }

  bool readMotion(OpticalFlowData& data) final;
  bool testConnection() final;

private:
  static constexpr uint8_t DEVICE_ADDRESS = 0x42;
  static constexpr uint8_t LEGACY_ADDRESS = 0x33;

  bool readPayload(uint8_t addr, OpticalFlowData* data);
  uint8_t _detectedAddress = DEVICE_ADDRESS;
};

} // namespace Espfc::Device
