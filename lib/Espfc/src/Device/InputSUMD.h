#pragma once

#include "Device/SerialDevice.h"
#include "Device/InputDevice.h"
#include <cstdint>
#include <cstddef>
#include <algorithm>

namespace Espfc {
namespace Device {

// Graupner SUMD Protocol
// Frame format: [SYNC(0xA8)] [STATUS] [CHANNELCOUNT] [CH0_HIGH] [CH0_LOW] ... [CH15_HIGH] [CH15_LOW] [CRC_HIGH] [CRC_LOW]
// SYNC = 0xA8, Status byte, channel count, 2 bytes per channel, 16-bit CRC

class InputSUMD: public InputDevice
{
  public:
    enum SumdState {
      SUMD_SYNC,
      SUMD_STATUS,
      SUMD_CHANNEL_COUNT,
      SUMD_CHANNELS,
      SUMD_CRC_HIGH,
      SUMD_CRC_LOW
    };

    InputSUMD();

    int begin(Device::SerialDevice * serial);
    InputStatus update() override;
    uint16_t get(uint8_t i) const override;
    void get(uint16_t * data, size_t len) const override;
    size_t getChannelCount() const override;
    bool needAverage() const override;

  private:
    void parse(int d);
    void apply();
    uint16_t crc16_ccitt(const uint8_t* data, size_t len);
    uint16_t convert(uint16_t raw);

    static constexpr size_t MAX_SUMD_CHANNELS = 16;
    static constexpr size_t SUMD_MAX_FRAME = 3 + (MAX_SUMD_CHANNELS * 2) + 2;  // header + channels + CRC
    static constexpr uint8_t SUMD_SYNC_BYTE = 0xA8;
    static constexpr uint8_t SUMD_SYNC_BYTE_RJ01 = 0xBA;

    Device::SerialDevice * _serial;
    SumdState _state;
    uint8_t _idx;
    uint8_t _channel_count;
    bool _new_data;
    uint16_t _expected_crc;

    uint8_t _data[SUMD_MAX_FRAME];
    uint16_t _channels[MAX_SUMD_CHANNELS];
};

}
}
