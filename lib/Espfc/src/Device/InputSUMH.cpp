#include "Device/InputSUMH.h"
#include "Utils/MemoryHelper.h"
#include <cstring>
#include <algorithm>

namespace Espfc {
namespace Device {

InputSUMH::InputSUMH() : _serial(NULL), _state(SUMH_SYNC), _idx(0), _channel_count(0), _new_data(false), _expected_crc(0) {}

int InputSUMH::begin(Device::SerialDevice * serial)
{
  _serial = serial;
  std::fill_n(_channels, MAX_SUMH_CHANNELS, 1500);
  std::fill_n(_data, SUMH_MAX_FRAME, 0);
  return 1;
}

InputStatus FAST_CODE_ATTR InputSUMH::update()
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

void FAST_CODE_ATTR InputSUMH::parse(int d)
{
  uint8_t c = (uint8_t)(d & 0xff);
  
  switch(_state)
  {
    case SUMH_SYNC:
      if(c == SUMH_SYNC_BYTE)
      {
        _data[0] = c;
        _state = SUMH_STATUS;
        _idx = 1;
      }
      break;

    case SUMH_STATUS:
      _data[1] = c;  // Status byte
      _state = SUMH_CHANNEL_COUNT;
      break;

    case SUMH_CHANNEL_COUNT:
      _channel_count = c;
      if(_channel_count > MAX_SUMH_CHANNELS)
      {
        _channel_count = MAX_SUMH_CHANNELS;
      }
      _data[2] = c;
      _idx = 3;
      _state = SUMH_CHANNELS;
      break;

    case SUMH_CHANNELS:
      _data[_idx++] = c;
      if(_idx >= 3 + _channel_count)
      {
        _state = SUMH_CRC;
      }
      break;

    case SUMH_CRC:
      _expected_crc = c;
      
      // Verify CRC (XOR of all bytes)
      uint8_t calc_crc = crc_xor(_data, 3 + _channel_count);
      if(calc_crc == _expected_crc)
      {
        apply();
      }
      
      _state = SUMH_SYNC;
      _idx = 0;
      break;
  }
}

void FAST_CODE_ATTR InputSUMH::apply()
{
  // Parse channel data from frame
  // Frame: [SYNC] [STATUS] [COUNT] [CH0] [CH1] ... [CRC]
  // Each channel is 1 byte: 0-255 mapped to PWM values
  
  for(uint8_t ch = 0; ch < _channel_count && ch < MAX_SUMH_CHANNELS; ch++)
  {
    size_t idx = 3 + ch;
    if(idx < SUMH_MAX_FRAME)
    {
      _channels[ch] = convert(_data[idx]);
    }
  }
  
  _new_data = true;
}

uint8_t FAST_CODE_ATTR InputSUMH::crc_xor(const uint8_t* data, size_t len)
{
  uint8_t crc = 0;
  for(size_t i = 0; i < len; i++)
  {
    crc ^= data[i];
  }
  return crc;
}

uint16_t FAST_CODE_ATTR InputSUMH::convert(uint8_t raw)
{
  // SUMH uses 8-bit values: 0x7F = center (1500µs), range 0-255 ≈ 1000-2000µs
  return 1000 + ((uint16_t)raw * 4);  // 0-255 * 4 = 0-1020, offset 1000 = 1000-2020
}

uint16_t FAST_CODE_ATTR InputSUMH::get(uint8_t i) const
{
  if(i < MAX_SUMH_CHANNELS) return _channels[i];
  return 1500;
}

void FAST_CODE_ATTR InputSUMH::get(uint16_t * data, size_t len) const
{
  const uint16_t * src = _channels;
  len = std::min(len, (size_t)MAX_SUMH_CHANNELS);
  while(len--)
  {
    *data++ = *src++;
  }
}

size_t InputSUMH::getChannelCount() const { return MAX_SUMH_CHANNELS; }

bool InputSUMH::needAverage() const { return false; }

}
}
