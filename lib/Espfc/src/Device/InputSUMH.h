#pragma once

#include "Device/SerialDevice.h"
#include "Device/InputDevice.h"
#include <cstdint>
#include <cstddef>
#include <algorithm>

namespace Espfc {
namespace Device {

// Graupner SUMH Protocol (similar to SUMD but different sync byte and CRC)
// Frame format: [SYNC(0xA9)] [STATUS] [CHANNELCOUNT] [CH0_HIGH] [CH0_LOW] ... [CRC]
// Frame rate: ~11ms, up to 8 channels supported

class InputSUMH: public InputDevice
{
  public:
    enum SumhState {
      SUMH_SYNC,
      SUMH_STATUS,
      SUMH_CHANNEL_COUNT,
      SUMH_CHANNELS,
      SUMH_CRC
    };

    InputSUMH();

    int begin(Device::SerialDevice * serial);
    InputStatus update() override;
    uint16_t get(uint8_t i) const override;
    void get(uint16_t * data, size_t len) const override;
    size_t getChannelCount() const override;
    bool needAverage() const override;

  private:
    void parse(int d);
    void apply();
    uint8_t crc_xor(const uint8_t* data, size_t len);
    uint16_t convert(uint8_t raw);

    static constexpr size_t MAX_SUMH_CHANNELS = 8;
    static constexpr size_t SUMH_MAX_FRAME = 3 + (MAX_SUMH_CHANNELS * 1) + 1;  // header + channels + CRC
    static constexpr uint8_t SUMH_SYNC_BYTE = 0xA9;

    Device::SerialDevice * _serial;
    SumhState _state;
    uint8_t _idx;
    uint8_t _channel_count;
    bool _new_data;
    uint8_t _expected_crc;

    uint8_t _data[SUMH_MAX_FRAME];
    uint16_t _channels[MAX_SUMH_CHANNELS];
};

}
}
