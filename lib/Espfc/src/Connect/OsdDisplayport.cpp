#include "Connect/OsdDisplayport.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>

extern "C" {
  #include "msp/msp_protocol.h"
}

namespace Espfc {
namespace Connect {

namespace {

constexpr uint8_t MSP_DP_HEARTBEAT = 0;
constexpr uint8_t MSP_DP_CLEAR_SCREEN = 2;
constexpr uint8_t MSP_DP_WRITE_STRING = 3;
constexpr uint8_t MSP_DP_DRAW_SCREEN = 4;

constexpr size_t MSP_OSD_MAX_STRING_LENGTH = 30;
constexpr uint32_t OSD_HEARTBEAT_US = 500000;
constexpr uint32_t OSD_DRAW_US = 200000;
constexpr uint32_t OSD_PAGE_US = 2000000;

}

OsdDisplayport::OsdDisplayport(Model& model): _model(model), _page(0) {}

void OsdDisplayport::begin()
{
  _heartbeatTimer.setInterval(OSD_HEARTBEAT_US);
  _drawTimer.setInterval(OSD_DRAW_US);
  _pageTimer.setInterval(OSD_PAGE_US);
  _page = 0;
}

void OsdDisplayport::update(Device::SerialDevice& s)
{
  if(!_model.config.osd.enabled) return;
  if(!_model.config.osd.mspDisplayport) return;

  if(_heartbeatTimer.check())
  {
    sendSimple(s, MSP_DP_HEARTBEAT);
  }

  if(_pageTimer.check())
  {
    _page = (_page + 1) % 2;
  }

  if(!_drawTimer.check()) return;

  if(!sendSimple(s, MSP_DP_CLEAR_SCREEN)) return;
  renderPage(s);
  sendSimple(s, MSP_DP_DRAW_SCREEN);
}

bool OsdDisplayport::sendDisplayport(Device::SerialDevice& s, const uint8_t *payload, size_t len) const
{
  if(!payload || len == 0 || len > MSP_BUF_OUT_SIZE) return false;

  MspResponse r;
  r.version = MSP_V1;
  r.cmd = MSP_DISPLAYPORT;
  r.result = 1;

  for(size_t i = 0; i < len; i++)
  {
    r.writeU8(payload[i]);
  }

  uint8_t frame[MSP_BUF_OUT_SIZE + 12] = {0};
  const size_t frameLen = r.serialize(frame, sizeof(frame));
  if(frameLen == 0) return false;

  const int freeBytes = s.availableForWrite();
  if(freeBytes > 0 && (size_t)freeBytes < frameLen) return false;

  return s.write(frame, frameLen) == frameLen;
}

bool OsdDisplayport::sendSimple(Device::SerialDevice& s, uint8_t command) const
{
  return sendDisplayport(s, &command, 1);
}

bool OsdDisplayport::sendWriteString(Device::SerialDevice& s, uint8_t row, uint8_t col, uint8_t attr, const char *text) const
{
  if(!text) return false;

  char clipped[MSP_OSD_MAX_STRING_LENGTH + 1] = {0};
  std::strncpy(clipped, text, MSP_OSD_MAX_STRING_LENGTH);

  const size_t textLen = std::strlen(clipped);
  uint8_t payload[MSP_OSD_MAX_STRING_LENGTH + 4] = {0};
  payload[0] = MSP_DP_WRITE_STRING;
  payload[1] = row;
  payload[2] = col;
  payload[3] = attr;
  std::memcpy(&payload[4], clipped, textLen);

  return sendDisplayport(s, payload, textLen + 4);
}

void OsdDisplayport::renderPage(Device::SerialDevice& s) const
{
  char line[MSP_OSD_MAX_STRING_LENGTH + 1] = {0};

  if(_page == 0)
  {
    std::snprintf(line, sizeof(line), "ESP-FC %s", _model.isModeActive(MODE_ARMED) ? "ARMED" : "DISARM");
    sendWriteString(s, 0, 0, 0, line);

    std::snprintf(line, sizeof(line), "BAT %.2fV %3d%%", _model.state.battery.voltage, (int)_model.state.battery.percentage);
    sendWriteString(s, 1, 0, 0, line);

    std::snprintf(line, sizeof(line), "RSSI %4u SAT %2u", _model.getRssi(), _model.state.gps.numSats);
    sendWriteString(s, 2, 0, 0, line);

    std::snprintf(line, sizeof(line), "ALT %.1fm VAR %.1f", _model.state.altitude.height, _model.state.altitude.vario);
    sendWriteString(s, 3, 0, 0, line);
  }
  else
  {
    std::snprintf(line, sizeof(line), "GYRO %4ld LOOP %4ld", (long)_model.state.gyro.timer.rate, (long)_model.state.loopTimer.rate);
    sendWriteString(s, 0, 0, 0, line);

    std::snprintf(line, sizeof(line), "MIX %4ld TEL %4ld", (long)_model.state.mixer.timer.rate, (long)_model.state.telemetryTimer.rate);
    sendWriteString(s, 1, 0, 0, line);

    std::snprintf(line, sizeof(line), "RPM %4.0f %4.0f", _model.state.output.telemetry.rpm[0], _model.state.output.telemetry.rpm[1]);
    sendWriteString(s, 2, 0, 0, line);

    std::snprintf(line, sizeof(line), "RNG %4u FIX %1u", _model.state.rangefinder[RANGEFINDER_BOTTOM].distance, _model.state.gps.fix ? 1 : 0);
    sendWriteString(s, 3, 0, 0, line);
  }
}

}
}
