#pragma once

#include "Device/SerialDevice.h"
#include "Device/InputDevice.h"
#include <cstdint>
#include <cstddef>
#include <algorithm>

namespace Espfc {
namespace Device {

// Spektrum DSMX/DSM2 Protocol - 1024 or 2048 resolution
// Format: [FADE/RSSI] [4×CHANNEL_DATA...] [4×CHANNEL_DATA...]
// Each channel = 16-bit word with [1:RES][11:CHANNEL][16:VALUE]
// 1024 mode: 11-bit values (988-2012 us)
// 2048 mode: 12-bit values (312-1694 us)

class InputSpektrum: public InputDevice
{
  public:
    enum SpektrumMode {
      SPEKTRUM_1024 = 0,      // 22ms frame rate
      SPEKTRUM_2048 = 1       // 11ms frame rate
    };

    enum SpektrumState {
      SPEKTRUM_FADE,
      SPEKTRUM_FRAME_ID_LOW,
      SPEKTRUM_FRAME_ID_HIGH,
      SPEKTRUM_DATA
    };

    InputSpektrum();

    int begin(Device::SerialDevice * serial);
    InputStatus update() override;
    uint16_t get(uint8_t i) const override;
    void get(uint16_t * data, size_t len) const override;
    size_t getChannelCount() const override;
    bool needAverage() const override;

  private:
    void parse(int d);
    void apply();
    uint16_t convert(uint16_t raw, SpektrumMode mode);

    static constexpr size_t SPEKTRUM_FRAME_SIZE = 16;  // 2 header + 14 channel bytes
    static constexpr size_t CHANNELS = 12;

    Device::SerialDevice * _serial;
    SpektrumState _state;
    SpektrumMode _mode;
    uint8_t _idx;
    bool _new_data;
    uint8_t _frame_id;

    uint8_t _data[SPEKTRUM_FRAME_SIZE];
    uint16_t _channels[CHANNELS];
};

}
}
