#include "Device/OpticalFlowMicoAirMTF02P.hpp"
#include <Arduino.h>

namespace Espfc::Device {

namespace {
static inline int32_t readLe32(const uint8_t* p)
{
  return (int32_t)(((uint32_t)p[0]) | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}
}

int OpticalFlowMicoAirMTF02P::begin(BusDevice* bus)
{
  if (!bus) return 0;

  if (begin(bus, DEVICE_ADDRESS)) return 1;
  if (begin(bus, LEGACY_ADDRESS)) return 1;

  return 0;
}

int OpticalFlowMicoAirMTF02P::begin(BusDevice* bus, uint8_t addr)
{
  if (!bus) return 0;

  setBus(bus, addr);
  if (!testConnection()) return 0;

  _detectedAddress = addr;
  return 1;
}

bool OpticalFlowMicoAirMTF02P::readMotion(OpticalFlowData& data)
{
  return readPayload(_detectedAddress, &data);
}

bool OpticalFlowMicoAirMTF02P::testConnection()
{
  return readPayload(_addr, nullptr);
}

bool OpticalFlowMicoAirMTF02P::readPayload(uint8_t addr, OpticalFlowData* data)
{
  if (!_bus) return false;

  uint8_t payload[9] = {0};
  // MTF-02P can be configured to emit MSPv2 sensor payloads over I2C.
  const int8_t status = _bus->read(addr, 0x00, sizeof(payload), payload);
  if (status != (int8_t)sizeof(payload)) return false;

  if (data)
  {
    data->quality = payload[0];
    data->motionX = readLe32(&payload[1]);
    data->motionY = readLe32(&payload[5]);
    data->timeUs = micros();
  }

  return true;
}

} // namespace Espfc::Device
