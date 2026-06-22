#include "Device/OledDevice.hpp"
#include "Hal/Pgm.h"

namespace Espfc::Device {

const char** OledDevice::getNames()
{
  static const char* devChoices[] = {
    PSTR("AUTO"),
    PSTR("NONE"),
    PSTR("SSD1306"),
    NULL
  };
  return devChoices;
}

const char* OledDevice::getName(DeviceType type)
{
  if (type >= OLED_MAX) return PSTR("?");
  return getNames()[type];
}

} // namespace Espfc::Device
