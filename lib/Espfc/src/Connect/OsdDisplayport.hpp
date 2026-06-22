#pragma once

#include "Model.h"
#include "Connect/Msp.hpp"

namespace Espfc {
namespace Connect {

class OsdDisplayport
{
public:
  explicit OsdDisplayport(Model& model);

  void begin();
  void update(Device::SerialDevice& s);

private:
  bool sendDisplayport(Device::SerialDevice& s, const uint8_t *payload, size_t len) const;
  bool sendSimple(Device::SerialDevice& s, uint8_t command) const;
  bool sendWriteString(Device::SerialDevice& s, uint8_t row, uint8_t col, uint8_t attr, const char *text) const;
  void renderPage(Device::SerialDevice& s) const;

  Model& _model;
  Utils::Timer _heartbeatTimer;
  Utils::Timer _drawTimer;
  Utils::Timer _pageTimer;
  uint8_t _page;
};

}
}
