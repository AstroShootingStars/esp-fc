#include "Device/InputFPort.h"
#include "Utils/MemoryHelper.h"
#include <cstring>
#include <algorithm>

namespace Espfc {
namespace Device {

InputFPort::InputFPort() : _serial(NULL), _state(FPORT_START), _idx(0), _new_data(false), _expected_crc(0) {}

int InputFPort::begin(Device::SerialDevice * serial)
{
  _serial = serial;
  std::fill_n(_channels, CHANNELS, 1500);
  std::fill_n(_data, FPORT_FRAME_SIZE, 0);
  return 1;
}

InputStatus FAST_CODE_ATTR InputFPort::update()
{
  if(!_serial) return INPUT_IDLE;

  size_t len = _serial->available();
  if(len)
  {
    uint8_t buff[64] = {0};
    len = std::min(len, (size_t)sizeof(buff));
    _serial->readMany(buff, len);
    for(size_t i = 0; i < len; i++)
    {
      parse(buff[i]);
    }
  }

  if(_new_data)
  {
    _new_data = false;
    return INPUT_RECEIVED;
  }

  return INPUT_IDLE;
}

void FAST_CODE_ATTR InputFPort::parse(int d)
{
  uint8_t c = (uint8_t)(d & 0xff);
  
  switch(_state)
  {
    case FPORT_START:
      if(c == FPORT_START_BYTE)
      {
        _idx = 0;
        _state = FPORT_DATA;
      }
      break;

    case FPORT_DATA:
      if(c == FPORT_START_BYTE)
      {
        // New frame start, reset
        _idx = 0;
        break;
      }
      
      _data[_idx++] = c;
      if(_idx >= FPORT_FRAME_SIZE)
      {
        _state = FPORT_CRC;
      }
      break;

    case FPORT_CRC:
    {
      _expected_crc = c;
      
      // Verify CRC
      uint8_t calc_crc = crc8(_data, FPORT_FRAME_SIZE);
      if(calc_crc == _expected_crc)
      {
        apply();
      }
      
      _state = FPORT_END;
      break;
    }

    case FPORT_END:
      if(c == FPORT_START_BYTE)
      {
        _state = FPORT_START;
      }
      break;
  }
}

void FAST_CODE_ATTR InputFPort::apply()
{
  // FPort frame: [STATUS] [CH0_H] [CH0_L] [CH1_H] [CH1_L] ... [CH7_H] [CH7_L] [RSSI]
  // 8 channels of 11-bit data packed into 16 bits each
  // Extended frame carries additional 8 channels
  
  // Parse first 8 channels from data
  for(size_t ch = 0; ch < 8; ch++)
  {
    size_t idx = 1 + (ch * 2);
    if(idx + 1 < FPORT_FRAME_SIZE)
    {
      // Extract 11-bit channel value
      uint16_t raw = ((uint16_t)_data[idx] << 8) | _data[idx + 1];
      _channels[ch] = convert(raw & 0x07FF);
    }
  }
  
  _new_data = true;
}

uint8_t FAST_CODE_ATTR InputFPort::crc8(const uint8_t* data, size_t len)
{
  uint8_t crc = 0;
  for(size_t i = 0; i < len; i++)
  {
    crc ^= data[i];
    for(int j = 0; j < 8; j++)
    {
      if(crc & 0x80)
      {
        crc = (crc << 1) ^ 0xD5;
      }
      else
      {
        crc = crc << 1;
      }
    }
  }
  return crc;
}

uint16_t FAST_CODE_ATTR InputFPort::convert(int v)
{
  // Convert S.Bus 11-bit value to PWM microseconds
  // Input range: 0-2047 (11-bit), output range: 988-2012 µs
  return 988 + (v >> 1);
}

uint16_t FAST_CODE_ATTR InputFPort::get(uint8_t i) const
{
  if(i < CHANNELS) return _channels[i];
  return 1500;
}

void FAST_CODE_ATTR InputFPort::get(uint16_t * data, size_t len) const
{
  const uint16_t * src = _channels;
  len = std::min(len, (size_t)CHANNELS);
  while(len--)
  {
    *data++ = *src++;
  }
}

size_t InputFPort::getChannelCount() const { return CHANNELS; }

bool InputFPort::needAverage() const { return false; }

}
}
