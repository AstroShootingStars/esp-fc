#include "Device/InputSpektrum.h"
#include "Utils/MemoryHelper.h"
#include <cstring>
#include <algorithm>

namespace Espfc {
namespace Device {

InputSpektrum::InputSpektrum() : _serial(NULL), _state(SPEKTRUM_FADE), _mode(SPEKTRUM_2048), _idx(0), _new_data(false), _frame_id(0) {}

int InputSpektrum::begin(Device::SerialDevice * serial)
{
  _serial = serial;
  std::fill_n(_channels, CHANNELS, 1500);
  std::fill_n(_data, SPEKTRUM_FRAME_SIZE, 0);
  return 1;
}

InputStatus FAST_CODE_ATTR InputSpektrum::update()
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

void FAST_CODE_ATTR InputSpektrum::parse(int d)
{
  uint8_t c = (uint8_t)(d & 0xff);
  
  switch(_state)
  {
    case SPEKTRUM_FADE:
      // Fade/RSSI byte - detect frame ID for mode
      _data[0] = c;
      _state = SPEKTRUM_FRAME_ID_LOW;
      _idx = 1;
      break;

    case SPEKTRUM_FRAME_ID_LOW:
      _data[1] = c;
      _state = SPEKTRUM_FRAME_ID_HIGH;
      break;

    case SPEKTRUM_FRAME_ID_HIGH:
      // Frame ID byte determines resolution (0x01=2048, 0x02=1024)
      _frame_id = c;
      _mode = (c == 0x01) ? SPEKTRUM_2048 : SPEKTRUM_1024;
      _idx = 2;
      _state = SPEKTRUM_DATA;
      break;

    case SPEKTRUM_DATA:
      _data[_idx++] = c;
      if(_idx >= SPEKTRUM_FRAME_SIZE)
      {
        apply();
        _state = SPEKTRUM_FADE;
        _idx = 0;
      }
      break;
  }
}

void FAST_CODE_ATTR InputSpektrum::apply()
{
  // Parse 7 channel pairs from 14 data bytes
  // Frame format: [FADE] [FRAME_ID_LOW] [FRAME_ID_HIGH] [CH0_HIGH] [CH0_LOW] [CH1_HIGH] [CH1_LOW] ... [CH6_HIGH] [CH6_LOW]
  
  for(size_t ch = 0; ch < 7 && ch < CHANNELS; ch++)
  {
    size_t idx = 2 + (ch * 2);
    if(idx + 1 < SPEKTRUM_FRAME_SIZE)
    {
      uint16_t raw = ((uint16_t)_data[idx] << 8) | _data[idx + 1];
      
      // Extract channel number and value
      uint8_t ch_num = (raw >> 11) & 0x0F;
      uint16_t ch_val = raw & 0x07FF;
      
      if(ch_num < CHANNELS)
      {
        _channels[ch_num] = convert(ch_val, _mode);
      }
    }
  }
  
  _new_data = true;
}

uint16_t FAST_CODE_ATTR InputSpektrum::convert(uint16_t raw, SpektrumMode mode)
{
  if(mode == SPEKTRUM_2048)
  {
    // 2048 mode: 12-bit value, convert to 988-2012 us
    // Raw range: 0-4095 → PWM range: 988-2012
    return 988 + (raw >> 1);  // Simplified: raw/2 gives approximately correct scaling
  }
  else
  {
    // 1024 mode: 11-bit value, already in 988-2012 range
    return raw;
  }
}

uint16_t FAST_CODE_ATTR InputSpektrum::get(uint8_t i) const
{
  if(i < CHANNELS) return _channels[i];
  return 1500;
}

void FAST_CODE_ATTR InputSpektrum::get(uint16_t * data, size_t len) const
{
  const uint16_t * src = _channels;
  len = std::min(len, (size_t)CHANNELS);
  while(len--)
  {
    *data++ = *src++;
  }
}

size_t InputSpektrum::getChannelCount() const { return CHANNELS; }

bool InputSpektrum::needAverage() const { return false; }

}
}
