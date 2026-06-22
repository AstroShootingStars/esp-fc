#include <algorithm>
#include "InputCRSF.h"
#include "Utils/MemoryHelper.h"

namespace Espfc::Device {

using namespace Espfc::Rc;

InputCRSF::InputCRSF(): _serial(NULL), _telemetry(NULL), _state(CRSF_ADDR), _idx(0), _new_data(false) {}

int InputCRSF::begin(Device::SerialDevice * serial, TelemetryManager * telemetry)
{
  _serial = serial;
  _telemetry = telemetry;
  _telemetry_next = micros() + TELEMETRY_INTERVAL;
  std::fill_n((uint8_t*)&_frame, sizeof(_frame), 0);
  std::fill_n(_channels, CHANNELS, 0);
  return 1;
}

InputStatus FAST_CODE_ATTR InputCRSF::update()
{
  if(!_serial) return INPUT_IDLE;

  size_t len = _serial->available();
  if(len)
  {
    uint8_t buff[64] = {0};
    len = std::min(len, sizeof(buff));
    _serial->readMany(buff, len);
    size_t i = 0;
    while(i < len)
    {
      parse(_frame, buff[i++]);
    }
  }

  if(_telemetry && micros() > _telemetry_next)
  {
    _telemetry_next = micros() + TELEMETRY_INTERVAL;
    _telemetry->process(*_serial, TELEMETRY_PROTOCOL_CRSF);
  }

  if(_new_data)
  {
    _new_data = false;
    return INPUT_RECEIVED;
  }

  return INPUT_IDLE;
}

uint16_t FAST_CODE_ATTR InputCRSF::get(uint8_t i) const
{
  return _channels[i];
}

void FAST_CODE_ATTR InputCRSF::get(uint16_t * data, size_t len) const
{
  const uint16_t * src = _channels;
  while(len--)
  {
    *data++ = *src++;
  }
}

size_t InputCRSF::getChannelCount() const { return CHANNELS; }

bool InputCRSF::needAverage() const { return false; }

void FAST_CODE_ATTR InputCRSF::parse(CrsfMessage& msg, int d)
{
  uint8_t *data = reinterpret_cast<uint8_t*>(&msg);
  uint8_t c = (uint8_t)(d & 0xff);
  switch(_state)
  {
    case CRSF_ADDR:
      if(c == CRSF_SYNC_BYTE)
      {
        data[_idx++] = c;
        _state = CRSF_SIZE;
      }
      break;
    case CRSF_SIZE:
      if(c >= 2 && c <= CRSF_FRAME_SIZE_MAX - 2) // allowed size is in range 2-62
      {
        data[_idx++] = c;
        _state = CRSF_TYPE;
      } else {
        reset();
      }
      break;
    case CRSF_TYPE:
      if(c == CRSF_FRAMETYPE_RC_CHANNELS_PACKED || c == CRSF_FRAMETYPE_LINK_STATISTICS || c == CRSF_FRAMETYPE_MSP_REQ || c == CRSF_FRAMETYPE_MSP_WRITE)
      {
        data[_idx++] = c;
        if (msg.size > 2) {
          _state = CRSF_DATA;
        } else {
          _state = CRSF_CRC; // no payload, next byte is crc
        }
      } else {
        reset();
      }
      break;
    case CRSF_DATA:
      data[_idx++] = c;
      if(_idx > msg.size) // _idx is incremented here and operator > accounts as size - 2
      {
        _state = CRSF_CRC;
      }
      break;
    case CRSF_CRC:
      data[_idx++] = c;
      reset();
      uint8_t crc = msg.crc();
      if(c == crc) {
        apply(msg);
      }
      break;
    }
}

void FAST_CODE_ATTR InputCRSF::reset()
{
  _state = CRSF_ADDR;
  _idx = 0;
}

void FAST_CODE_ATTR InputCRSF::apply(const CrsfMessage& msg)
{
  switch (msg.type)
  {
    case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
      applyChannels(msg);
      break;

    case CRSF_FRAMETYPE_LINK_STATISTICS:
      applyLinkStats(msg);
      break;

    case CRSF_FRAMETYPE_MSP_REQ:
    case CRSF_FRAMETYPE_MSP_WRITE:
      applyMspReq(msg);
      break;

    default:
      break;
  }
}

void FAST_CODE_ATTR InputCRSF::applyLinkStats(const CrsfMessage& msg)
{
  // CRSF Link Statistics (uplink RX <- TX direction)
  // Payload structure (10 bytes):
  //   uint8_t  uplink_RSSI_1         - RSSI antenna 1 (dBm * -1, 0 = -0dBm, 100 = -100dBm)
  //   uint8_t  uplink_RSSI_2         - RSSI antenna 2 (dBm * -1)
  //   uint8_t  uplink_Link_quality   - Link quality (0-100%), packet success rate
  //   int8_t   uplink_SNR            - Signal-to-noise ratio (dB)
  //   uint8_t  active_antenna        - Active antenna (0 or 1)
  //   uint8_t  rf_Mode               - RF mode (0=4Hz, 1=50Hz, 2=150Hz)
  //   uint8_t  uplink_TX_Power       - TX power level (0=0mW, 1=10mW, 2=25mW, 3=100mW, 4=500mW, 5=1000mW, 6=2000mW, 7=250mW)
  //   uint8_t  downlink_RSSI         - RSSI downlink (dBm * -1)
  //   uint8_t  downlink_Link_quality - Downlink link quality (0-100%)
  //   int8_t   downlink_SNR          - Downlink SNR (dB)
  const auto * stats = reinterpret_cast<const CrsfLinkStats*>(msg.payload);
  
  if(stats) {
    // Store link statistics for MSP queries and telemetry
    // Note: Implementation would integrate with model state if receiver link stats tracking is added
    (void)stats->uplink_RSSI_1;
    (void)stats->uplink_RSSI_2;
    (void)stats->uplink_Link_quality;
    (void)stats->uplink_SNR;
    (void)stats->active_antenna;
    (void)stats->rf_Mode;
    (void)stats->uplink_TX_Power;
    (void)stats->downlink_RSSI;
    (void)stats->downlink_Link_quality;
    (void)stats->downlink_SNR;
  }
}

void FAST_CODE_ATTR InputCRSF::applyChannels(const CrsfMessage& msg)
{
  const auto * data = reinterpret_cast<const CrsfData*>(msg.payload);
  Crsf::decodeRcDataShift8(_channels, data);
  //Crsf::decodeRcData(_channels, frame);
  _new_data = true;
}

void FAST_CODE_ATTR InputCRSF::applyMspReq(const CrsfMessage& frame)
{
  if(!_telemetry) return;

  uint8_t origin = 0;

  Crsf::decodeMsp(frame, _msg, origin);

  if(_msg.isCmd() && _msg.isReady())
  {
    _telemetry->processMsp(*_serial, TELEMETRY_PROTOCOL_CRSF, _msg, origin);
  }

  _telemetry_next = micros() + TELEMETRY_INTERVAL;
}

}

