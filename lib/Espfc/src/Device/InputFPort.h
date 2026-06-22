#pragma once

#include "Device/SerialDevice.h"
#include "Device/InputDevice.h"
#include <cstdint>
#include <cstddef>
#include <algorithm>

namespace Espfc {
namespace Device {

// FrSky FPort Protocol (2.4 GHz module)
// Similar to S.Bus but with added bidirectional telemetry capability
// Frame format: [START(0x7E)] [13 data bytes with bit 7 as flag] [CRC8] [END(0x7E)]
// 16 channels available, 200 Hz update rate

class InputFPort: public InputDevice
{
  public:
    enum FPortState {
      FPORT_START,
      FPORT_DATA,
      FPORT_CRC,
      FPORT_END
    };

    InputFPort();

    int begin(Device::SerialDevice * serial);
    InputStatus update() override;
    uint16_t get(uint8_t i) const override;
    void get(uint16_t * data, size_t len) const override;
    size_t getChannelCount() const override;
    bool needAverage() const override;

  private:
    void parse(int d);
    void apply();
    uint8_t crc8(const uint8_t* data, size_t len);
    uint16_t convert(int v);

    static constexpr size_t FPORT_FRAME_SIZE = 13;  // Data bytes (excluding start/crc/end)
    static constexpr size_t CHANNELS = 16;
    static constexpr uint8_t FPORT_START_BYTE = 0x7E;

    Device::SerialDevice * _serial;
    FPortState _state;
    uint8_t _idx;
    bool _new_data;
    uint8_t _expected_crc;

    uint8_t _data[FPORT_FRAME_SIZE];
    uint16_t _channels[CHANNELS];
};

}
}
