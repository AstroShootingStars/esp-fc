#include "Device/OpticalFlowDevice.hpp"
#include "Hal/Pgm.h"

namespace Espfc::Device {

const char** OpticalFlowDevice::getNames()
{
  static const char* devChoices[] = {
    PSTR("AUTO"),
    PSTR("NONE"),
    PSTR("MSP"),
    PSTR("MATEK_3901_L0X"),
    PSTR("MICOAIR_MTF_02P"),
    NULL
  };
  return devChoices;
}

const char* OpticalFlowDevice::getName(DeviceType type)
{
  if (type >= OPFLOW_MAX) return PSTR("?");
  return getNames()[type];
}

} // namespace Espfc::Device
