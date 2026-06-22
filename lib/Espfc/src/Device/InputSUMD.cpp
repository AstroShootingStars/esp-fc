#include "Device/InputSUMD.h"
#include "Utils/MemoryHelper.h"
#include <cstring>
#include <algorithm>

namespace Espfc {
namespace Device {

InputSUMD::InputSUMD() : _serial(NULL), _state(SUMD_SYNC), _idx(0), _channel_count(0), _new_data(false), _expected_crc(0) {}

int InputSUMD::begin(Device::SerialDevice * serial)
{
  _serial = serial;
  std::fill_n(_channels, MAX_SUMD_CHANNELS, 1500);
  std::fill_n(_data, SUMD_MAX_FRAME, 0);
  return 1;
}

InputStatus FAST_CODE_ATTR InputSUMD::update()
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

void FAST_CODE_ATTR InputSUMD::parse(int d)
{
  uint8_t c = (uint8_t)(d & 0xff);
  
  switch(_state)
  {
    case SUMD_SYNC:
      if(c == SUMD_SYNC_BYTE || c == SUMD_SYNC_BYTE_RJ01)
      {
        _data[0] = c;
        _state = SUMD_STATUS;
        _idx = 1;
      }
      break;

    case SUMD_STATUS:
      _data[1] = c;  // Status: 0x01 = valid, 0x00 = lost frames
      _state = SUMD_CHANNEL_COUNT;
      break;

    case SUMD_CHANNEL_COUNT:
      _channel_count = c;
      if(_channel_count > MAX_SUMD_CHANNELS)
      {
        _channel_count = MAX_SUMD_CHANNELS;
      }
      _data[2] = c;
      _idx = 3;
      _state = SUMD_CHANNELS;
      break;

    case SUMD_CHANNELS:
      _data[_idx++] = c;
      if(_idx >= 3 + (_channel_count * 2))
      {
        _state = SUMD_CRC_HIGH;
      }
      break;

    case SUMD_CRC_HIGH:
      _expected_crc = (uint16_t)c << 8;
      _state = SUMD_CRC_LOW;
      break;

    case SUMD_CRC_LOW:
      _expected_crc |= c;
      _data[_idx++] = c;
      
      // Verify CRC and apply
      uint16_t calc_crc = crc16_ccitt(_data, 3 + (_channel_count * 2));
      if(calc_crc == _expected_crc)
      {
        apply();
      }
      
      _state = SUMD_SYNC;
      _idx = 0;
      break;
  }
}

void FAST_CODE_ATTR InputSUMD::apply()
{
  // Parse channel data from frame
  // Frame: [SYNC] [STATUS] [COUNT] [CH0_H] [CH0_L] [CH1_H] [CH1_L] ... [CRC_H] [CRC_L]
  
  for(uint8_t ch = 0; ch < _channel_count && ch < MAX_SUMD_CHANNELS; ch++)
  {
    size_t idx = 3 + (ch * 2);
    if(idx + 1 < SUMD_MAX_FRAME)
    {
      uint16_t raw = ((uint16_t)_data[idx] << 8) | _data[idx + 1];
      _channels[ch] = convert(raw);
    }
  }
  
  _new_data = true;
}

uint16_t FAST_CODE_ATTR InputSUMD::crc16_ccitt(const uint8_t* data, size_t len)
{
  uint16_t crc = 0;
  for(size_t i = 0; i < len; i++)
  {
    crc ^= (uint16_t)data[i] << 8;
    for(int j = 0; j < 8; j++)
    {
      if(crc & 0x8000)
      {
        crc = (crc << 1) ^ 0x1021;
      }
      else
      {
        crc = crc << 1;
      }
      crc &= 0xffff;
    }
  }
  return crc;
}

uint16_t FAST_CODE_ATTR InputSUMD::convert(uint16_t raw)
{
  // SUMD uses 16-bit values: 0x2000 = 1500µs (center), range 0x1000-0x3000 ≈ 1000-2000µs
  // Convert to standard 988-2012 range
  if(raw < 0x1000) raw = 0x1000;
  if(raw > 0x3000) raw = 0x3000;
  return 988 + ((raw - 0x1000) >> 4);
}

uint16_t FAST_CODE_ATTR InputSUMD::get(uint8_t i) const
{
  if(i < MAX_SUMD_CHANNELS) return _channels[i];
  return 1500;
}

void FAST_CODE_ATTR InputSUMD::get(uint16_t * data, size_t len) const
{
  const uint16_t * src = _channels;
  len = std::min(len, (size_t)MAX_SUMD_CHANNELS);
  while(len--)
  {
    *data++ = *src++;
  }
}

size_t InputSUMD::getChannelCount() const { return MAX_SUMD_CHANNELS; }

bool InputSUMD::needAverage() const { return false; }

}
}
