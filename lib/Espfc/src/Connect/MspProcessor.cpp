#include "Connect/MspProcessor.hpp"
#include "Hardware.h"
#include <platform.h>
#include <algorithm>
#include <limits>
#if defined(ESPFC_MULTI_CORE) && defined(ESPFC_FREE_RTOS)
#include <driver/timer.h>
#endif

#define VTXCOMMON_MSP_BANDCHAN_CHKVAL ((uint16_t)((7 << 3) + 7))

extern "C" {
  #include "config/feature.h"
  #include "msp/msp_protocol.h"
  #include "msp/msp_protocol_v2_common.h"
  #include "msp/msp_protocol_v2_betaflight.h"
  #include "io/serial_4way.h"
  #include "blackbox/blackbox_io.h"
  int blackboxCalculatePDenom(int rateNum, int rateDenom);
  uint8_t blackboxCalculateSampleRate(uint16_t pRatio);
  uint8_t blackboxGetRateDenom(void);
  uint16_t blackboxGetPRatio(void);
}

#ifndef MSP2_SENSOR_RANGEFINDER
#define MSP2_SENSOR_RANGEFINDER 0x1F01
#endif

#ifndef MSP2_SENSOR_OPTIC_FLOW
#define MSP2_SENSOR_OPTIC_FLOW  0x1F02
#endif

#ifndef MSP2_GET_TEXT
#define MSP2_GET_TEXT 0x3006
#endif

#ifndef MSP2_SET_TEXT
#define MSP2_SET_TEXT 0x3007
#endif

#ifndef MSP2_SENSOR_CONFIG_ACTIVE
#define MSP2_SENSOR_CONFIG_ACTIVE 0x300A
#endif

#ifndef MSP2_GYRO_SENSOR_ACTIVE
#define MSP2_GYRO_SENSOR_ACTIVE 0x300D
#endif

#ifndef MSP2TEXT_PILOT_NAME
#define MSP2TEXT_PILOT_NAME 1
#endif

#ifndef MSP2TEXT_CRAFT_NAME
#define MSP2TEXT_CRAFT_NAME 2
#endif

// Some BF headers mark these as deprecated and omit defines, but configurators
// may still use them to read/write displayed gyro/loop frequencies.
#ifndef MSP_LOOP_TIME
#define MSP_LOOP_TIME 73
#endif

#ifndef MSP_SET_LOOP_TIME
#define MSP_SET_LOOP_TIME 74
#endif

#ifndef MSP2_ESPFC_LANDING_ASSIST_CONFIG
#define MSP2_ESPFC_LANDING_ASSIST_CONFIG      0x1F03
#endif

#ifndef MSP2_ESPFC_SET_LANDING_ASSIST_CONFIG
#define MSP2_ESPFC_SET_LANDING_ASSIST_CONFIG  0x1F04
#endif

#ifndef MSP2_ESPFC_ALT_FUSION_CONFIG
#define MSP2_ESPFC_ALT_FUSION_CONFIG          0x1F05
#endif

#ifndef MSP2_ESPFC_SET_ALT_FUSION_CONFIG
#define MSP2_ESPFC_SET_ALT_FUSION_CONFIG      0x1F06
#endif

#ifndef MSP2_ESPFC_SENSOR_OFFSET
#define MSP2_ESPFC_SENSOR_OFFSET              0x1F07
#endif

#ifndef MSP2_ESPFC_SET_SENSOR_OFFSET
#define MSP2_ESPFC_SET_SENSOR_OFFSET          0x1F08
#endif

#ifndef MSP2_ESPFC_SENSOR_SCALE
#define MSP2_ESPFC_SENSOR_SCALE               0x1F09
#endif

#ifndef MSP2_ESPFC_SET_SENSOR_SCALE
#define MSP2_ESPFC_SET_SENSOR_SCALE           0x1F0A
#endif

#ifndef MSP2_ESPFC_CALIBRATION_DATA
#define MSP2_ESPFC_CALIBRATION_DATA           0x1F0B
#endif

#ifndef MSP2_ESPFC_SENSOR_STATUS
#define MSP2_ESPFC_SENSOR_STATUS              0x1F0C
#endif

#ifndef MSP2_ESPFC_OBSTACLE_AVOIDANCE_CONFIG
#define MSP2_ESPFC_OBSTACLE_AVOIDANCE_CONFIG  0x1F0D
#endif

#ifndef MSP2_ESPFC_SET_OBSTACLE_AVOIDANCE_CONFIG
#define MSP2_ESPFC_SET_OBSTACLE_AVOIDANCE_CONFIG 0x1F0E
#endif

#ifndef MSP2_ESPFC_SENSOR_RANGEFINDER_FRONT
#define MSP2_ESPFC_SENSOR_RANGEFINDER_FRONT   0x1F0F
#endif

#ifndef MSP2_ESPFC_SENSOR_RANGEFINDER_STATUS
#define MSP2_ESPFC_SENSOR_RANGEFINDER_STATUS  0x1F10
#endif

#ifndef MSP2_ESPFC_RANGEFINDER_CONFIG
#define MSP2_ESPFC_RANGEFINDER_CONFIG         0x1F11
#endif

#ifndef MSP2_ESPFC_SET_RANGEFINDER_CONFIG
#define MSP2_ESPFC_SET_RANGEFINDER_CONFIG     0x1F12
#endif

// Extended receiver protocol support - using standard Betaflight MSP commands only
// Custom extensions disabled to maintain Betaflight configurator compatibility

namespace {

constexpr uint8_t OSD_FLAGS_OSD_FEATURE = (1 << 0);
constexpr uint8_t OSD_FLAGS_OSD_HARDWARE_FRSKYOSD = (1 << 3);
constexpr uint8_t OSD_FLAGS_OSD_HARDWARE_MAX_7456 = (1 << 4);
constexpr uint8_t OSD_FLAGS_OSD_DEVICE_DETECTED = (1 << 5);
constexpr uint8_t OSD_FLAGS_OSD_MSP_DEVICE = (1 << 6);

constexpr uint8_t MSP_DP_HEARTBEAT = 0;
constexpr uint8_t MSP_DP_RELEASE = 1;
constexpr uint8_t MSP_DP_CLEAR_SCREEN = 2;
constexpr uint8_t MSP_DP_WRITE_STRING = 3;
constexpr uint8_t MSP_DP_DRAW_SCREEN = 4;
constexpr uint8_t MSP_DP_OPTIONS = 5;
constexpr uint8_t MSP_DP_SYS = 6;
constexpr uint8_t MSP_DP_FONTCHAR_WRITE = 7;

constexpr uint8_t ESPFC_OSD_WARNING_COUNT = 16;

enum SerialSpeedIndex {
  SERIAL_SPEED_INDEX_AUTO = 0,
  SERIAL_SPEED_INDEX_9600,
  SERIAL_SPEED_INDEX_19200,
  SERIAL_SPEED_INDEX_38400,
  SERIAL_SPEED_INDEX_57600,
  SERIAL_SPEED_INDEX_115200,
  SERIAL_SPEED_INDEX_230400,
  SERIAL_SPEED_INDEX_250000,
  SERIAL_SPEED_INDEX_400000,
  SERIAL_SPEED_INDEX_460800,
  SERIAL_SPEED_INDEX_500000,
  SERIAL_SPEED_INDEX_921600,
  SERIAL_SPEED_INDEX_1000000,
  SERIAL_SPEED_INDEX_1500000,
  SERIAL_SPEED_INDEX_2000000,
  SERIAL_SPEED_INDEX_2470000,
};

static SerialSpeedIndex toBaudIndex(int32_t speed)
{
  using namespace Espfc;
  if(speed >= SERIAL_SPEED_2470000) return SERIAL_SPEED_INDEX_2470000;
  if(speed >= SERIAL_SPEED_2000000) return SERIAL_SPEED_INDEX_2000000;
  if(speed >= SERIAL_SPEED_1500000) return SERIAL_SPEED_INDEX_1500000;
  if(speed >= SERIAL_SPEED_1000000) return SERIAL_SPEED_INDEX_1000000;
  if(speed >= SERIAL_SPEED_921600)  return SERIAL_SPEED_INDEX_921600;
  if(speed >= SERIAL_SPEED_500000)  return SERIAL_SPEED_INDEX_500000;
  if(speed >= SERIAL_SPEED_460800)  return SERIAL_SPEED_INDEX_460800;
  if(speed >= SERIAL_SPEED_400000)  return SERIAL_SPEED_INDEX_400000;
  if(speed >= SERIAL_SPEED_250000)  return SERIAL_SPEED_INDEX_250000;
  if(speed >= SERIAL_SPEED_230400)  return SERIAL_SPEED_INDEX_230400;
  if(speed >= SERIAL_SPEED_115200)  return SERIAL_SPEED_INDEX_115200;
  if(speed >= SERIAL_SPEED_57600)   return SERIAL_SPEED_INDEX_57600;
  if(speed >= SERIAL_SPEED_38400)   return SERIAL_SPEED_INDEX_38400;
  if(speed >= SERIAL_SPEED_19200)   return SERIAL_SPEED_INDEX_19200;
  if(speed >= SERIAL_SPEED_9600)    return SERIAL_SPEED_INDEX_9600;
  return SERIAL_SPEED_INDEX_AUTO;
}


static void writeMsp2Text(Espfc::Connect::MspResponse& r, uint8_t textType, const char* text)
{
  const char* safeText = text ? text : "";
  const size_t textLen = std::min<size_t>(strlen(safeText), Espfc::MODEL_NAME_LEN);
  r.writeU8(textType);
  r.writeU8((uint8_t)textLen);
  r.writeData(safeText, (int)textLen);
}

static void readMsp2TextToModelName(Espfc::Connect::MspMessage& m, char* modelName)
{
  if(!modelName || m.remain() < 1) return;
  const uint8_t textLen = m.readU8();
  const size_t copyLen = std::min<size_t>(textLen, Espfc::MODEL_NAME_LEN);
  memset(modelName, 0, Espfc::MODEL_NAME_LEN + 1);
  for(size_t i = 0; i < copyLen && m.remain() > 0; i++)
  {
    modelName[i] = (char)m.readU8();
  }
  while(m.remain() > 0) m.readU8();
}

static uint16_t deriveLoopTimeUs(const Espfc::Model& model)
{
  uint32_t loopTimeUs = model.state.stats.loopTime();
  if(loopTimeUs == 0) loopTimeUs = model.state.loopTimer.interval;
  if(loopTimeUs == 0 && model.state.loopRate > 0) loopTimeUs = 1000000ul / model.state.loopRate;
  if(loopTimeUs == 0 && model.state.gyro.rate > 0 && model.config.loopSync > 0)
  {
    loopTimeUs = (1000000ul * (uint32_t)model.config.loopSync) / (uint32_t)model.state.gyro.rate;
  }
  if(loopTimeUs == 0) loopTimeUs = 1000; // safe fallback (1kHz)
  return (uint16_t)std::min<uint32_t>(loopTimeUs, std::numeric_limits<uint16_t>::max());
}

static bool isTransientSetCommand(uint16_t cmd)
{
  switch(cmd)
  {
    case MSP_SET_RAW_RC:
    case MSP_SET_RAW_GPS:
    case MSP_ACC_CALIBRATION:
    case MSP_MAG_CALIBRATION:
    case MSP_SET_HEADING:
    case MSP_SET_TX_INFO:
    case MSP_SET_ARMING_DISABLED:
    case MSP_SET_PASSTHROUGH:
    case MSP_SET_RTC:
      return true;
    default:
      return false;
  }
}

static bool shouldAutoPersistSetCommand(uint16_t cmd)
{
  if(cmd == MSP_EEPROM_WRITE || cmd == MSP_RESET_CONF || cmd == MSP_REBOOT) return false;
  if(isTransientSetCommand(cmd)) return false;
  return true;
}

static Espfc::SerialSpeed fromBaudIndex(SerialSpeedIndex index)
{
  using namespace Espfc;
  switch(index)
  {
    case SERIAL_SPEED_INDEX_9600:    return SERIAL_SPEED_9600;
    case SERIAL_SPEED_INDEX_19200:   return SERIAL_SPEED_19200;
    case SERIAL_SPEED_INDEX_38400:   return SERIAL_SPEED_38400;
    case SERIAL_SPEED_INDEX_57600:   return SERIAL_SPEED_57600;
    case SERIAL_SPEED_INDEX_115200:  return SERIAL_SPEED_115200;
    case SERIAL_SPEED_INDEX_230400:  return SERIAL_SPEED_230400;
    case SERIAL_SPEED_INDEX_250000:  return SERIAL_SPEED_250000;
    case SERIAL_SPEED_INDEX_400000:  return SERIAL_SPEED_400000;
    case SERIAL_SPEED_INDEX_460800:  return SERIAL_SPEED_460800;
    case SERIAL_SPEED_INDEX_500000:  return SERIAL_SPEED_500000;
    case SERIAL_SPEED_INDEX_921600:  return SERIAL_SPEED_921600;
    case SERIAL_SPEED_INDEX_1000000: return SERIAL_SPEED_1000000;
    case SERIAL_SPEED_INDEX_1500000: return SERIAL_SPEED_1500000;
    case SERIAL_SPEED_INDEX_2000000: return SERIAL_SPEED_2000000;
    case SERIAL_SPEED_INDEX_2470000: return SERIAL_SPEED_2470000;
    case SERIAL_SPEED_INDEX_AUTO:
    default:
      return SERIAL_SPEED_NONE;
  }
}

static uint8_t toFilterTypeDerivative(uint8_t t)
{
  switch(t) {
    case 0: return Espfc::FILTER_NONE;
    case 1: return Espfc::FILTER_PT3;
    case 2: return Espfc::FILTER_BIQUAD;
    default: return Espfc::FILTER_PT3;
  }
}

static uint8_t fromFilterTypeDerivative(uint8_t t)
{
  switch(t) {
    case Espfc::FILTER_NONE: return 0;
    case Espfc::FILTER_PT3: return 1;
    case Espfc::FILTER_BIQUAD: return 2;
    default: return 1;
  }
}

static char toUpperAscii(char c)
{
  return (c >= 'a' && c <= 'z') ? (char)(c - 'a' + 'A') : c;
}

static uint16_t vtxLookupFrequency(const Espfc::ModelConfig& config, uint8_t band, uint8_t channel)
{
  if(band < 1 || channel < 1) return 0;
  if(band > config.vtxTable.bands || channel > config.vtxTable.channels) return 0;
  return config.vtxTable.frequency[band - 1][channel - 1];
}

static void vtxApplyBandChannel(Espfc::ModelConfig& config, uint8_t band, uint8_t channel)
{
  config.vtx.band = band;
  config.vtx.channel = channel;
  config.vtx.freq = vtxLookupFrequency(config, band, channel);
}

struct ModeBoxMapEntry {
  uint8_t mode;
  uint8_t boxId;
};

static constexpr ModeBoxMapEntry MODE_BOX_MAP[] = {
  { Espfc::MODE_ARMED, 0 },
  { Espfc::MODE_AIRMODE, 27 },
  { Espfc::MODE_ANGLE, 1 },
  { Espfc::MODE_HORIZON, 2 },
  { Espfc::MODE_ALTHOLD, 3 },
  { Espfc::MODE_BUZZER, 13 },
  { Espfc::MODE_FAILSAFE, 26 },
  { Espfc::MODE_BLACKBOX, 22 },
  { Espfc::MODE_BLACKBOX_ERASE, 30 },
  { Espfc::MODE_MAG, 5 },
  { Espfc::MODE_HEADFREE, 6 },
  { Espfc::MODE_HEADADJ, 7 },
  { Espfc::MODE_CALIB, 16 },
  { Espfc::MODE_GPS_RESCUE, 44 },
  { Espfc::MODE_PREARM, 35 },
  { Espfc::MODE_FLIP_OVER_AFTER_CRASH, 28 },
  { Espfc::MODE_USER1, 39 },
  { Espfc::MODE_USER2, 40 },
  { Espfc::MODE_USER3, 41 },
  { Espfc::MODE_USER4, 42 },
  { Espfc::MODE_ACRO_TRAINER, 45 },
  { Espfc::MODE_LAUNCH_CONTROL, 47 },
  { Espfc::MODE_MSP_OVERRIDE, 48 },
  { Espfc::MODE_STICK_COMMANDS_DISABLE, 49 },
  { Espfc::MODE_BEEPER_MUTE, 50 },
  { Espfc::MODE_PARALYZE, 43 },
};

static constexpr const char * MODE_BOX_NAMES =
  "ARM;AIRMODE;ANGLE;HORIZON;ALTHOLD;BEEPER;FAILSAFE;BLACKBOX;BLACKBOXERASE;"
  "MAG;HEADFREE;HEADADJ;CALIB;GPS RESCUE;PREARM;FLIP OVER AFTER CRASH;"
  "USER1;USER2;USER3;USER4;ACRO TRAINER;LAUNCH CONTROL;MSP OVERRIDE;"
  "STICK COMMANDS DISABLE;BEEPER MUTE;PARALYZE;";

static int16_t boxIdToMode(uint8_t boxId)
{
  for(size_t i = 0; i < sizeof(MODE_BOX_MAP) / sizeof(MODE_BOX_MAP[0]); i++)
  {
    if(MODE_BOX_MAP[i].boxId == boxId) return MODE_BOX_MAP[i].mode;
  }
  return -1;
}

static uint8_t modeToBoxId(uint8_t mode)
{
  for(size_t i = 0; i < sizeof(MODE_BOX_MAP) / sizeof(MODE_BOX_MAP[0]); i++)
  {
    if(MODE_BOX_MAP[i].mode == mode) return MODE_BOX_MAP[i].boxId;
  }
  return mode;
}

static uint8_t fromGyroDlpf(uint8_t t)
{
  switch(t) {
    case Espfc::GYRO_DLPF_256: return 0;
    case Espfc::GYRO_DLPF_EX: return 1;
    default: return 2;
  }
}

static int8_t toVbatSource(uint8_t t)
{
  switch(t) {
    case 0: return 0; // none
    case 1: return 1; // internal adc
    default: return 0;
  }
}

static int8_t toIbatSource(uint8_t t)
{
  switch(t) {
    case 0: return 0; // none
    case 1: return 1; // internal adc
    default: return 0;
  }
}

static uint8_t toVbatVoltageLegacy(float voltage)
{
  return constrain(lrintf(voltage * 10.0f), 0, 255);
}

static uint16_t toVbatVoltage(float voltage)
{
  return constrain(lrintf(voltage * 100.0f), 0, 32000);
}

static int16_t toIbatCurrentSigned(float current)
{
  return constrain(lrintf(current * 100.0f), -32000, 32000);
}

static uint16_t toIbatCurrent(float current)
{
  return (uint16_t)toIbatCurrentSigned(current);
}

static uint16_t toIbatCurrentMeterValue(float current)
{
  return constrain(lrintf(std::max(current, 0.0f) * 1000.0f), 0, 0xffff);
}

constexpr uint8_t MSP_PASSTHROUGH_ESC_4WAY = 0xff;

}

namespace Espfc {

namespace Connect {

MspProcessor::MspProcessor(Model& model): _model(model) {}

bool MspProcessor::parse(char c, MspMessage& msg)
{
  _parser.parse(c, msg);

  if(msg.isReady()) debugMessage(msg);

  return !msg.isIdle();
}

void MspProcessor::processCommand(MspMessage& m, MspResponse& r, Device::SerialDevice& s)
{
  r.cmd = m.cmd;
  r.version = m.version;
  r.result = 1;
  switch(m.cmd)
  {
    case MSP_API_VERSION:
      r.writeU8(MSP_PROTOCOL_VERSION);
      r.writeU8(API_VERSION_MAJOR);
      r.writeU8(API_VERSION_MINOR);
      break;

    case MSP_BOARD_INFO:
      {
        r.writeData(flightControllerIdentifier, FLIGHT_CONTROLLER_IDENTIFIER_LENGTH);
        r.writeU16(1); // board version
        r.writeU8(0);   // board type
        r.writeU8(0);   // target capabilities

        // Betaflight Configurator expects length-prefixed text fields here.
        auto writeBoardInfoText = [&](const char* text) {
          const char* safeText = text ? text : "";
          const size_t textLen = std::min<size_t>(strlen(safeText), 255);
          r.writeU8((uint8_t)textLen);
          r.writeData(safeText, (int)textLen);
        };

        writeBoardInfoText(""); // target name
        writeBoardInfoText(""); // board name
        writeBoardInfoText(""); // manufacturer id
        const uint8_t boardInfoSignature[32] = {};
        r.writeData((const char*)boardInfoSignature, sizeof(boardInfoSignature));
        r.writeU8(0);            // mcu type id
        r.writeU8(0);            // configuration state
        r.writeU16((uint16_t)std::min<uint32_t>(_model.state.gyro.rate, std::numeric_limits<uint16_t>::max()));
        r.writeU32(0);           // configuration problems
      }
      break;

    case MSP_FC_VARIANT:
      r.writeData(flightControllerIdentifier, FLIGHT_CONTROLLER_IDENTIFIER_LENGTH);
      break;
    case MSP_FC_VERSION:
      r.writeU8(FC_VERSION_MAJOR);
      r.writeU8(FC_VERSION_MINOR);
      r.writeU8(FC_VERSION_PATCH_LEVEL);
      break;

    case MSP_BUILD_INFO:
      r.writeData(buildDate, BUILD_DATE_LENGTH);
      r.writeData(buildTime, BUILD_TIME_LENGTH);
      r.writeData(shortGitRevision, GIT_SHORT_REVISION_LENGTH);
      break;

    case MSP_UID:
      r.writeU32(getBoardId0());
      r.writeU32(getBoardId1());
      r.writeU32(getBoardId2());
      break;

    case MSP_STATUS_EX:
    case MSP_STATUS:
      {
        const uint16_t loopTimeUs = deriveLoopTimeUs(_model);
        r.writeU16(loopTimeUs);
      }
      r.writeU16(_model.state.i2cErrorCount); // i2c error count
      //         acc,     baro,    mag,     gps,     sonar,   gyro
      r.writeU16(_model.accelActive() | _model.baroActive() << 1 | _model.magActive() << 2 | _model.gpsActive() << 3 | 0 << 4 | _model.gyroActive() << 5);
      r.writeU32(_model.state.mode.mask); // flight mode flags
      r.writeU8(0); // pid profile
      r.writeU16(lrintf(_model.state.stats.getCpuLoad()));
      if (m.cmd == MSP_STATUS_EX) {
        r.writeU8(RATE_PROFILE_COUNT); // max profile count
        r.writeU8(_model.config.activeRateProfile); // current rate profile index
      } else {  // MSP_STATUS
        // Report actual gyro cycle time (us). Some configurator builds use this
        // to derive and display gyro update frequency.
        uint16_t gyroCycleUs = _model.state.gyro.timer.interval;
        if(gyroCycleUs == 0) gyroCycleUs = _model.state.loopTimer.interval;
        if(gyroCycleUs == 0 && _model.state.gyro.rate > 0) gyroCycleUs = (uint16_t)(1000000ul / (uint32_t)_model.state.gyro.rate);
        if(gyroCycleUs == 0) gyroCycleUs = deriveLoopTimeUs(_model);
        r.writeU16(gyroCycleUs);
        r.writeU8(_model.state.gyro.present ? 1 : 0); // detected gyro type (required by BF configurator)
      }

      // flight mode flags (above 32 bits)
      r.writeU8(0); // count

      // Write arming disable flags
      r.writeU8(ARMING_DISABLED_FLAGS_COUNT);  // 1 byte, flag count
      r.writeU32(_model.state.mode.armingDisabledFlags);  // 4 bytes, flags
      r.writeU8(0); // reboot required
      break;

    case MSP_LOOP_TIME:
      // Legacy compatibility: expose current loop interval in microseconds.
      // Prefer loopTimer because configurator maps this to displayed update rate.
      r.writeU16(deriveLoopTimeUs(_model));
      break;

    case MSP_SET_LOOP_TIME:
      {
        // Legacy compatibility: map requested looptime (us) back to loopSync.
        const uint16_t loopTimeUs = m.readU16();
        if(loopTimeUs > 0 && _model.state.gyro.rate > 0)
        {
          const uint32_t requestedLoopRate = 1000000ul / loopTimeUs;
          if(requestedLoopRate > 0)
          {
            int sync = (int)(_model.state.gyro.rate / requestedLoopRate);
            if(sync < 1) sync = 1;
            if(sync > 255) sync = 255;
            _model.config.loopSync = sync;
            _model.reload();
          }
        }
      }
      break;

    case MSP_NAME:
      r.writeString(_model.config.modelName);
      break;

    case MSP2_GET_TEXT:
      {
        const uint8_t textType = m.remain() ? m.readU8() : 0;
        switch(textType)
        {
          case MSP2TEXT_PILOT_NAME:
          case MSP2TEXT_CRAFT_NAME:
            // esp-fc stores one persisted model name; mirror it for pilot/craft text requests.
            writeMsp2Text(r, textType, _model.config.modelName);
            break;
          default:
            r.result = 0;
            break;
        }
      }
      break;

    case MSP_SET_NAME:
      memset(&_model.config.modelName, 0, MODEL_NAME_LEN + 1);
      for(size_t i = 0; i < std::min((size_t)m.received, MODEL_NAME_LEN); i++)
      {
        _model.config.modelName[i] = m.readU8();
      }
      break;

    case MSP2_SET_TEXT:
      if(m.remain() < 1)
      {
        r.result = 0;
        break;
      }
      {
        const uint8_t textType = m.readU8();
        switch(textType)
        {
          case MSP2TEXT_PILOT_NAME:
          case MSP2TEXT_CRAFT_NAME:
            readMsp2TextToModelName(m, _model.config.modelName);
            break;
          default:
            r.result = 0;
            while(m.remain() > 0) m.readU8();
            break;
        }
      }
      break;

    case MSP_BOXNAMES:
      r.writeString(MODE_BOX_NAMES);
      break;

    case MSP_BOXIDS:
      for(size_t i = 0; i < sizeof(MODE_BOX_MAP) / sizeof(MODE_BOX_MAP[0]); i++)
      {
        r.writeU8(MODE_BOX_MAP[i].boxId);
      }
      break;

    case MSP_MODE_RANGES:
      for(size_t i = 0; i < ACTUATOR_CONDITIONS; i++)
      {
        r.writeU8(modeToBoxId(_model.config.conditions[i].id));
        r.writeU8(_model.config.conditions[i].ch - AXIS_AUX_1);
        r.writeU8((_model.config.conditions[i].min - 900) / 25);
        r.writeU8((_model.config.conditions[i].max - 900) / 25);
      }
      break;

    case MSP_MODE_RANGES_EXTRA:
      r.writeU8(ACTUATOR_CONDITIONS);
      for(size_t i = 0; i < ACTUATOR_CONDITIONS; i++)
      {
        r.writeU8(modeToBoxId(_model.config.conditions[i].id));
        r.writeU8(_model.config.conditions[i].logicMode);
        r.writeU8(_model.config.conditions[i].linkId);
      }

      break;

    case MSP_SET_MODE_RANGE:
      {
        size_t i = m.readU8();
        if(i < ACTUATOR_CONDITIONS)
        {
          const uint8_t boxId = m.readU8();
          int16_t mappedMode = boxIdToMode(boxId);
          if(mappedMode < 0 && boxId < MODE_COUNT)
          {
            mappedMode = boxId;
          }
          if(mappedMode < 0)
          {
            r.result = -1;
            break;
          }
          _model.config.conditions[i].id = mappedMode;
          _model.config.conditions[i].ch = constrain(m.readU8() + AXIS_AUX_1, AXIS_AUX_1, AXIS_AUX_1 + INPUT_CHANNELS - 4 - 1);
          _model.config.conditions[i].min = constrain(m.readU8() * 25 + 900, 900, 2100);
          _model.config.conditions[i].max = constrain(m.readU8() * 25 + 900, 900, 2100);
          if(_model.config.conditions[i].min > _model.config.conditions[i].max)
          {
            std::swap(_model.config.conditions[i].min, _model.config.conditions[i].max);
          }
          if(m.remain() >= 2) {
            _model.config.conditions[i].logicMode = m.readU8(); // mode logic
            _model.config.conditions[i].linkId = m.readU8(); // link to
          }
        }
        else
        {
          r.result = -1;
        }
        _model.reload();
      }
      break;

    case MSP_ANALOG:
      r.writeU8(toVbatVoltageLegacy(_model.state.battery.voltage));  // voltage in 0.1V
      r.writeU16(std::min<uint32_t>(_model.state.battery.consumedMah, (uint32_t)std::numeric_limits<uint16_t>::max())); // mah drawn
      r.writeU16(_model.getRssi()); // rssi
      r.writeU16(toIbatCurrent(_model.state.battery.current));  // amperage in 0.01A
      r.writeU16(toVbatVoltage(_model.state.battery.voltage));  // voltage in 0.01V
      break;

    case MSP_FEATURE_CONFIG:
      r.writeU32(_model.config.featureMask);
      break;

    case MSP_SET_FEATURE_CONFIG:
      _model.config.featureMask = m.readU32();
      _model.reload();
      break;

    case MSP_BATTERY_CONFIG:
      r.writeU8(std::clamp<int>(_model.config.vbat.cellMin / 10, 0, 255));  // vbatmincellvoltage
      r.writeU8(std::clamp<int>(_model.config.vbat.cellMax / 10, 0, 255));  // vbatmaxcellvoltage
      r.writeU8((_model.config.vbat.cellWarning + 5) / 10);  // vbatwarningcellvoltage
      r.writeU16(_model.config.vbat.capacity); // batteryCapacity
      r.writeU8(_model.config.vbat.source);  // voltageMeterSource
      r.writeU8(_model.config.ibat.source);  // currentMeterSource
      r.writeU16(_model.config.vbat.cellMin); // vbatmincellvoltage
      r.writeU16(_model.config.vbat.cellMax); // vbatmaxcellvoltage
      r.writeU16(_model.config.vbat.cellWarning); // vbatwarningcellvoltage
      break;

    case MSP_SET_BATTERY_CONFIG:
      if(m.remain() >= 1) _model.config.vbat.cellMin = m.readU8() * 10;  // vbatmincellvoltage (legacy)
      if(m.remain() >= 1) _model.config.vbat.cellMax = m.readU8() * 10;  // vbatmaxcellvoltage (legacy)
      if(m.remain() >= 1) _model.config.vbat.cellWarning = m.readU8() * 10;  // vbatwarningcellvoltage (legacy)
      if(m.remain() >= 2) _model.config.vbat.capacity = m.readU16(); // batteryCapacity
      if(m.remain() >= 1) _model.config.vbat.source = toVbatSource(m.readU8());  // voltageMeterSource
      if(m.remain() >= 1) _model.config.ibat.source = toIbatSource(m.readU8());  // currentMeterSource
      if(m.remain() >= 6)
      {
        _model.config.vbat.cellMin = m.readU16(); // vbatmincellvoltage
        _model.config.vbat.cellMax = m.readU16(); // vbatmaxcellvoltage
        _model.config.vbat.cellWarning = m.readU16();
      }
      _model.reload();
      break;

    case MSP_BATTERY_STATE:
      // battery characteristics
      r.writeU8(_model.state.battery.cells); // cell count, 0 indicates battery not detected.
      r.writeU16(_model.config.vbat.capacity); // capacity in mAh

      // battery state
      r.writeU8(toVbatVoltageLegacy(_model.state.battery.voltage)); // in 0.1V steps
      r.writeU16(std::min<uint32_t>(_model.state.battery.consumedMah, (uint32_t)std::numeric_limits<uint16_t>::max())); // milliamp hours drawn from battery
      r.writeU16(toIbatCurrent(_model.state.battery.current)); // send current in 0.01 A steps, range is -320A to 320A

      // battery alerts
      r.writeU8(0);
      r.writeU16(toVbatVoltage(_model.state.battery.voltage)); // in 0.01 steps
      break;

    case MSP_VOLTAGE_METERS:
      r.writeU8(10);  // meter id (10-19 vbat adc)
      r.writeU8(toVbatVoltageLegacy(_model.state.battery.voltage));  // meter value
      break;

    case MSP_CURRENT_METERS:
      r.writeU8(10);  // meter id (10-19 ibat adc)
      r.writeU16(std::min<uint32_t>(_model.state.battery.consumedMah, (uint32_t)std::numeric_limits<uint16_t>::max())); // mah drawn
      r.writeU16(toIbatCurrentMeterValue(_model.state.battery.current));  // meter value (0.001A units, non-negative)
      break;

    case MSP_VOLTAGE_METER_CONFIG:
      // Legacy BF-compatible ADC voltage meter config payload.
      r.writeU8(_model.config.vbat.scale);   // vbatscale
      r.writeU8(_model.config.vbat.resDiv);  // vbatresdivval
      r.writeU8(_model.config.vbat.resMult); // vbatresdivmultiplier
      r.writeU8(0);                          // vbatresdivoffset (unused)
      break;

    case MSP_SET_VOLTAGE_METER_CONFIG:
      if(m.remain() >= 3)
      {
        _model.config.vbat.scale = m.readU8();
        _model.config.vbat.resDiv = m.readU8();
        _model.config.vbat.resMult = m.readU8();
      }
      if(m.remain() >= 1) m.readU8(); // optional offset
      _model.reload();
      break;

    case MSP_CURRENT_METER_CONFIG:
      // Legacy BF-compatible ADC current meter config payload.
      r.writeU16(_model.config.ibat.scale);  // currentMeterScale
      r.writeU16(_model.config.ibat.offset); // currentMeterOffset
      r.writeU8(_model.config.ibat.source);  // currentMeterType/source
      break;

    case MSP_SET_CURRENT_METER_CONFIG:
      if(m.remain() >= 4)
      {
        _model.config.ibat.scale = m.readU16();
        _model.config.ibat.offset = m.readU16();
      }
      if(m.remain() >= 1)
      {
        _model.config.ibat.source = toIbatSource(m.readU8());
      }
      _model.reload();
      break;


    case MSP_DATAFLASH_SUMMARY:
#ifdef USE_FLASHFS   
      {
        uint8_t flags = flashfsIsSupported() ? 2 : 0;
        flags |= flashfsIsReady() ? 1 : 0;
        r.writeU8(flags);
        r.writeU32(flashfsGetSectors());
        r.writeU32(flashfsGetSize());
        r.writeU32(flashfsGetOffset());
      }
#else
      r.writeU8(0);
      r.writeU32(0);
      r.writeU32(0);
      r.writeU32(0);
#endif
      break;

    case MSP_DATAFLASH_ERASE:
#ifdef USE_FLASHFS
      blackboxEraseAll();
#endif
      break;

    case MSP_DATAFLASH_READ:
#ifdef USE_FLASHFS
      {
        const unsigned int dataSize = m.remain();
        const uint32_t readAddress = m.readU32();
        uint16_t readLength;
        bool allowCompression = false;
        bool useLegacyFormat;

        if (dataSize >= sizeof(uint32_t) + sizeof(uint16_t)) {
            readLength = m.readU16();
            if (m.remain()) {
                allowCompression = m.readU8();
            }
            useLegacyFormat = false;
        } else {
            readLength = 128;
            useLegacyFormat = true;
        }
        serializeFlashData(r, readAddress, readLength, useLegacyFormat, allowCompression);
      }
#endif            
      break;

    case MSP_ACC_TRIM:
      r.writeU16(_model.config.accel.trim[0]); // pitch
      r.writeU16(_model.config.accel.trim[1]); // roll
      break;

    case MSP_SET_ACC_TRIM:
      _model.config.accel.trim[0] = std::clamp<int16_t>(m.readU16(), -300, 300); // pitch
      _model.config.accel.trim[1] = std::clamp<int16_t>(m.readU16(), -300, 300); // roll
      _model.onAccChange();
      _model.reload();
      break;

    case MSP_MIXER_CONFIG:
      r.writeU8(_model.config.mixer.type); // mixerMode, QUAD_X
      r.writeU8(_model.config.mixer.yawReverse); // yaw_motors_reversed
      break;

    case MSP_SET_MIXER_CONFIG:
      _model.config.mixer.type = m.readU8(); // mixerMode, QUAD_X
      _model.config.mixer.yawReverse = m.readU8(); // yaw_motors_reversed
      _model.reload();
      break;

    case MSP_SENSOR_CONFIG:
      // Betaflight expects gyro hardware at index 0.
      r.writeU8(_model.config.gyro.dev);  // gyro hardware
      r.writeU8(_model.config.accel.dev); // 3 acc mpu6050
      r.writeU8(_model.config.baro.dev);  // 2 baro bmp085
      r.writeU8(_model.config.mag.dev);   // 3 mag hmc5883l
      r.writeU8(_model.config.rangefinder[RANGEFINDER_BOTTOM].dev);  // Report bottom rangefinder device
      r.writeU8(_model.config.opticalFlow.dev);
      break;

    case MSP2_SENSOR_CONFIG_ACTIVE:
      r.writeU8(_model.state.gyro.present ? (uint8_t)_model.config.gyro.dev : 0xFF);
      r.writeU8(_model.state.accel.present ? (uint8_t)_model.config.accel.dev : 0xFF);
      r.writeU8(_model.state.baro.present ? (uint8_t)_model.config.baro.dev : 0xFF);
      r.writeU8(_model.state.mag.present ? (uint8_t)_model.config.mag.dev : 0xFF);
      r.writeU8(_model.state.rangefinder[RANGEFINDER_BOTTOM].present ? (uint8_t)_model.config.rangefinder[RANGEFINDER_BOTTOM].dev : 0xFF);
      r.writeU8(_model.state.opticalFlow.present ? (uint8_t)_model.config.opticalFlow.dev : 0xFF);
      break;

    case MSP2_GYRO_SENSOR_ACTIVE:
      // Keep one slot visible so configurator can render the active IMU section.
      r.writeU8(1);
      r.writeU8(_model.state.gyro.present ? (uint8_t)_model.config.gyro.dev : 0xFF);
      break;

    case MSP_SET_SENSOR_CONFIG:
      if(m.remain() >= 6)
      {
        // API 1.46+: gyro, accel, baro, mag, rangefinder, optical flow
        _model.config.gyro.dev = m.readU8();
        _model.config.accel.dev = m.readU8();
        _model.config.baro.dev = m.readU8();
        _model.config.mag.dev = m.readU8();
        _model.config.rangefinder[RANGEFINDER_BOTTOM].dev = m.readU8();
        _model.config.opticalFlow.dev = m.readU8();
      }
      else
      {
        // Legacy ordering without explicit gyro hardware.
        if(m.remain() >= 1) _model.config.accel.dev = m.readU8();
        if(m.remain() >= 1) _model.config.baro.dev = m.readU8();
        if(m.remain() >= 1) _model.config.mag.dev = m.readU8();
        if(m.remain() >= 1) _model.config.rangefinder[RANGEFINDER_BOTTOM].dev = m.readU8();
        if(m.remain() >= 1) _model.config.opticalFlow.dev = m.readU8();
      }
      _model.reload();
      break;

    case MSP_SENSOR_ALIGNMENT:
      r.writeU8(_model.config.gyro.align); // gyro align
      r.writeU8(_model.config.gyro.align); // acc align, Starting with 4.0 gyro and acc alignment are the same
      r.writeU8(_model.config.mag.align);  // mag align
      //1.41+
      r.writeU8(_model.state.gyro.present ? 1 : 0); // gyro detection mask GYRO_1_MASK
      r.writeU8(0); // gyro_to_use
      r.writeU8(_model.config.gyro.align); // gyro 1
      r.writeU8(0); // gyro 2
      break;

    case MSP_SET_SENSOR_ALIGNMENT:
      {
        uint8_t gyroAlign = m.readU8(); // gyro align
        m.readU8(); // discard deprecated acc align
        _model.config.mag.align = m.readU8(); // mag align
        // API >= 1.41 - support the gyro_to_use and alignment for gyros 1 & 2
        if(m.remain() >= 3)
        {
          m.readU8(); // gyro_to_use
          gyroAlign = m.readU8(); // gyro 1 align
          m.readU8(); // gyro 2 align
        }
        _model.config.gyro.align = gyroAlign;
        _model.reload();
      }
      break;

    case MSP2_ESPFC_SENSOR_OFFSET:
      // Read gyro bias
      r.writeU16(_model.config.gyro.bias[0]); // X
      r.writeU16(_model.config.gyro.bias[1]); // Y
      r.writeU16(_model.config.gyro.bias[2]); // Z
      // Read accel bias
      r.writeU16(_model.config.accel.bias[0]); // X
      r.writeU16(_model.config.accel.bias[1]); // Y
      r.writeU16(_model.config.accel.bias[2]); // Z
      // Read mag offset
      r.writeU16(_model.config.mag.offset[0]); // X
      r.writeU16(_model.config.mag.offset[1]); // Y
      r.writeU16(_model.config.mag.offset[2]); // Z
      break;

    case MSP2_ESPFC_SET_SENSOR_OFFSET:
      // Write gyro bias
      _model.config.gyro.bias[0] = std::clamp<int16_t>(m.readU16(), -1000, 1000);
      _model.config.gyro.bias[1] = std::clamp<int16_t>(m.readU16(), -1000, 1000);
      _model.config.gyro.bias[2] = std::clamp<int16_t>(m.readU16(), -1000, 1000);
      // Write accel bias
      if(m.remain() >= 6) {
        _model.config.accel.bias[0] = std::clamp<int16_t>(m.readU16(), -500, 500);
        _model.config.accel.bias[1] = std::clamp<int16_t>(m.readU16(), -500, 500);
        _model.config.accel.bias[2] = std::clamp<int16_t>(m.readU16(), -500, 500);
      }
      // Write mag offset
      if(m.remain() >= 6) {
        _model.config.mag.offset[0] = std::clamp<int16_t>(m.readU16(), -1000, 1000);
        _model.config.mag.offset[1] = std::clamp<int16_t>(m.readU16(), -1000, 1000);
        _model.config.mag.offset[2] = std::clamp<int16_t>(m.readU16(), -1000, 1000);
      }
      _model.onAccChange();
      break;

    case MSP2_ESPFC_SENSOR_SCALE:
      // Gyro doesn't have scale, read accel scale
      r.writeU16(1000); // Gyro scale (fixed at 1000)
      r.writeU16(1000); // Gyro scale Y
      r.writeU16(1000); // Gyro scale Z
      // Read accel scale
      r.writeU16(1000); // Accel scale X (fixed for now)
      r.writeU16(1000); // Accel scale Y
      r.writeU16(1000); // Accel scale Z
      // Read mag scale
      r.writeU16(_model.config.mag.scale[0]); // X
      r.writeU16(_model.config.mag.scale[1]); // Y
      r.writeU16(_model.config.mag.scale[2]); // Z
      break;

    case MSP2_ESPFC_SET_SENSOR_SCALE:
      // Gyro scale is not configurable (skip 3 values)
      m.readU16();
      m.readU16();
      m.readU16();
      // Accel scale not configurable (skip 3 values)
      if(m.remain() >= 6) {
        m.readU16();
        m.readU16();
        m.readU16();
      }
      // Write mag scale
      if(m.remain() >= 6) {
        _model.config.mag.scale[0] = std::clamp<int16_t>(m.readU16(), 500, 2000);
        _model.config.mag.scale[1] = std::clamp<int16_t>(m.readU16(), 500, 2000);
        _model.config.mag.scale[2] = std::clamp<int16_t>(m.readU16(), 500, 2000);
      }
      _model.onAccChange();
      break;

    case MSP2_SENSOR_RANGEFINDER:
      // Response layout follows INAV MSPv2 rangefinder message: quality + int32 distance(mm).
      // We map quality to 255 when present, otherwise 0.
      // Report bottom rangefinder (altitude hold / landing)
      r.writeU8(_model.state.rangefinder[RANGEFINDER_BOTTOM].present ? 255 : 0);
      r.writeU32((uint32_t)(_model.state.rangefinder[RANGEFINDER_BOTTOM].present ? (int32_t)_model.state.rangefinder[RANGEFINDER_BOTTOM].distance : 0));
      break;

    case MSP2_SENSOR_OPTIC_FLOW:
      // Request with empty payload: report current optical-flow telemetry.
      if(m.remain() == 0)
      {
        r.writeU8(_model.state.opticalFlow.quality);
        r.writeU32((uint32_t)_model.state.opticalFlow.motionX);
        r.writeU32((uint32_t)_model.state.opticalFlow.motionY);
      }
      // Payload layout follows INAV MSPv2 sensor message: quality + int32 motionX + int32 motionY.
      else if(m.remain() >= 9)
      {
        const uint8_t quality = m.readU8();
        const int32_t motionX = (int32_t)m.readU32();
        const int32_t motionY = (int32_t)m.readU32();

        if(_model.config.opticalFlow.dev == Device::OPFLOW_DEFAULT || _model.config.opticalFlow.dev == Device::OPFLOW_MSP)
        {
          _model.state.opticalFlow.quality = quality;
          _model.state.opticalFlow.motionX = motionX;
          _model.state.opticalFlow.motionY = motionY;
          _model.state.opticalFlow.lastUpdateUs = micros();
          _model.state.opticalFlow.rate = _model.state.opticalFlow.rate ? _model.state.opticalFlow.rate : 100;
          _model.state.opticalFlow.present = quality >= _model.config.opticalFlow.qualityThreshold;
        }
      }
      break;

    case MSP_CF_SERIAL_CONFIG:
      for(int i = 0; i < SERIAL_UART_COUNT; i++)
      {
        if(_model.config.serial[i].id >= SERIAL_ID_SOFTSERIAL_1 && !_model.isFeatureActive(FEATURE_SOFTSERIAL)) continue;
        r.writeU8(_model.config.serial[i].id); // identifier
        r.writeU16(_model.config.serial[i].functionMask); // functionMask
        r.writeU8(toBaudIndex(_model.config.serial[i].baud)); // msp_baudrateIndex
        r.writeU8(0); // gps_baudrateIndex
        r.writeU8(0); // telemetry_baudrateIndex
        r.writeU8(toBaudIndex(_model.config.serial[i].blackboxBaud)); // blackbox_baudrateIndex
      }
      break;

    case MSP2_COMMON_SERIAL_CONFIG:
      {
        uint8_t count = 0;
        for (int i = 0; i < SERIAL_UART_COUNT; i++)
        {
          if(_model.config.serial[i].id >= SERIAL_ID_SOFTSERIAL_1 && !_model.isFeatureActive(FEATURE_SOFTSERIAL)) continue;
          count++;
        }
        r.writeU8(count);
        for (int i = 0; i < SERIAL_UART_COUNT; i++)
        {
          if(_model.config.serial[i].id >= SERIAL_ID_SOFTSERIAL_1 && !_model.isFeatureActive(FEATURE_SOFTSERIAL)) continue;
          r.writeU8(_model.config.serial[i].id); // identifier
          r.writeU32(_model.config.serial[i].functionMask); // functionMask
          r.writeU8(toBaudIndex(_model.config.serial[i].baud)); // msp_baudrateIndex
          r.writeU8(0); // gps_baudrateIndex
          r.writeU8(0); // telemetry_baudrateIndex
          r.writeU8(toBaudIndex(_model.config.serial[i].blackboxBaud)); // blackbox_baudrateIndex
        }
      }
      break;

    case MSP_SET_CF_SERIAL_CONFIG:
      {
        const int packetSize = 1 + 2 + 1 + 1 + 1 + 1; // id + functionMask + 4 baud indices
        while(m.remain() >= packetSize)
        {
          int id = m.readU8();
          int k = _model.getSerialIndex((SerialPortId)id);
          if(k == -1)
          {
            m.advance(packetSize - 1);
            continue;
          }
          _model.config.serial[k].id = id;
          const uint32_t preservedHighBits = _model.config.serial[k].functionMask & 0xFFFF0000u;
          _model.config.serial[k].functionMask = preservedHighBits | (uint32_t)m.readU16(); // CFv1 supports up to 16-bit functions
          _model.config.serial[k].baud = fromBaudIndex((SerialSpeedIndex)m.readU8());
          m.readU8(); // gps_baudrateIndex (unused)
          m.readU8(); // telemetry_baudrateIndex (unused)
          _model.config.serial[k].blackboxBaud = fromBaudIndex((SerialSpeedIndex)m.readU8());
        }
      }
      _model.reload();
      break;

    case MSP2_COMMON_SET_SERIAL_CONFIG:
      {
        size_t count = m.readU8();
        (void)count; // ignore
        const int packetSize = 1 + 4 + 4;
        while(m.remain() >= packetSize)
        {
          int id = m.readU8();
          int k = _model.getSerialIndex((SerialPortId)id);
          if(k == -1)
          {
            m.advance(packetSize - 1);
            continue;
          }
          _model.config.serial[k].id = id;
          _model.config.serial[k].functionMask = m.readU32();
          _model.config.serial[k].baud = fromBaudIndex((SerialSpeedIndex)m.readU8());
          m.readU8();
          m.readU8();
          _model.config.serial[k].blackboxBaud = fromBaudIndex((SerialSpeedIndex)m.readU8());
        }
      }
      _model.reload();
      break;

    case MSP_BLACKBOX_CONFIG:
      r.writeU8(1); // Blackbox supported
      r.writeU8(_model.config.blackbox.dev); // device serial or none
      r.writeU8(1); // blackboxGetRateNum()); // unused
      r.writeU8(1); // blackboxGetRateDenom());
      r.writeU16(_model.config.blackbox.pDenom);//blackboxGetPRatio()); // p_denom
      //r.writeU8(_model.config.blackbox.pDenom); // sample_rate
      //r.writeU32(~_model.config.blackbox.fieldsMask);
      break;

    case MSP_SDCARD_SUMMARY:
      // No SD card backend in this target; report not supported/not present.
      r.writeU8(0); // flags
      r.writeU8(0); // state
      r.writeU8(0); // last error
      r.writeU32(0); // free KB
      r.writeU32(0); // total KB
      break;

    case MSP_SET_BLACKBOX_CONFIG:
      // Don't allow config to be updated while Blackbox is logging
      if (true) {
        _model.config.blackbox.dev = m.readU8();
        const int rateNum = m.readU8(); // was rate_num
        const int rateDenom = m.readU8(); // was rate_denom
        uint16_t pRatio = 0;
        if (m.remain() >= 2) {
            pRatio = m.readU16(); // p_denom specified, so use it directly
        } else {
            // p_denom not specified in MSP, so calculate it from old rateNum and rateDenom
            //pRatio = blackboxCalculatePDenom(rateNum, rateDenom);
            (void)(rateNum + rateDenom);
        }
        _model.config.blackbox.pDenom = pRatio;

        /*if (m.remain() >= 1) {
            _model.config.blackbox.pDenom = m.readU8();
        } else if(pRatio > 0) {
            _model.config.blackbox.pDenom = blackboxCalculateSampleRate(pRatio);
            //_model.config.blackbox.pDenom = pRatio;
        }
        if (m.remain() >= 4) {
          _model.config.blackbox.fieldsMask = ~m.readU32();
        }*/
      }
      break;

    case MSP_ATTITUDE:
      r.writeU16(lrintf(Utils::toDeg(_model.state.attitude.euler.x) * 10.f)); // roll  [decidegrees]
      r.writeU16(lrintf(Utils::toDeg(_model.state.attitude.euler.y) * 10.f)); // pitch [decidegrees]
      r.writeU16(lrintf(Utils::toDeg(-_model.state.attitude.euler.z)));       // yaw   [degrees]
      break;

    case MSP_ALTITUDE:
      r.writeU32(lrintf(_model.state.altitude.height * 100.f));  // alt [cm]
      r.writeU16(lrintf(_model.state.altitude.vario * 100.f));   // vario [cm/s]
      break;

    case MSP_SONAR_ALTITUDE:
      r.writeU32(_model.state.rangefinder[RANGEFINDER_BOTTOM].present ? (_model.state.rangefinder[RANGEFINDER_BOTTOM].distance / 10) : 0); // cm
      break;

    case MSP_BEEPER_CONFIG:
      r.writeU32(_model.config.beeper.beeperOffFlags); // beeper mask
      r.writeU8(_model.config.beeper.dshotBeaconTone);  // dshot beacon tone
      r.writeU32(_model.config.beeper.dshotBeaconOffFlags); // dshot beacon off flags
      break;

    case MSP_SET_BEEPER_CONFIG:
      _model.config.beeper.beeperOffFlags = m.readU32(); // beeper mask
      _model.config.buzzer.beeperMask = BUZZER_ALLOWED_MASK & ~_model.config.beeper.beeperOffFlags;
      if(m.remain() >= 1) _model.config.beeper.dshotBeaconTone = m.readU8();
      if(m.remain() >= 4) _model.config.beeper.dshotBeaconOffFlags = m.readU32();
      break;

    case MSP_BOARD_ALIGNMENT_CONFIG:
      r.writeU16(_model.config.boardAlignment[0]); // roll
      r.writeU16(_model.config.boardAlignment[1]); // pitch
      r.writeU16(_model.config.boardAlignment[2]); // yaw
      break;

    case MSP_SET_BOARD_ALIGNMENT_CONFIG:
      _model.config.boardAlignment[0] = m.readU16();
      _model.config.boardAlignment[1] = m.readU16();
      _model.config.boardAlignment[2] = m.readU16();
      _model.reload();
      break;

    case MSP_RX_MAP:
      for(size_t i = 0; i < 8; i++)
      {
        r.writeU8(_model.config.input.channel[i].map);
      }
      break;

    case MSP_SET_RX_MAP:
      for(size_t i = 0; i < 8; i++)
      {
        if(!m.remain()) break;
        _model.config.input.channel[i].map = m.readU8();
      }
      _model.reload();
      break;

    case MSP_RSSI_CONFIG:
      r.writeU8(_model.config.input.rssiChannel);
      break;

    case MSP_SET_RSSI_CONFIG:
      _model.config.input.rssiChannel = m.readU8();
      _model.reload();
      break;

    case MSP_MOTOR_CONFIG:
      r.writeU16(_model.config.output.minThrottle); // minthrottle
      r.writeU16(_model.config.output.maxThrottle); // maxthrottle
      r.writeU16(_model.config.output.minCommand);  // mincommand
      r.writeU8(_model.state.currentMixer.count);   // motor count
      // 1.42+
      r.writeU8(_model.config.output.motorPoles); // motor pole count
      r.writeU8(_model.config.output.dshotTelemetry); // dshot telemtery
      r.writeU8(0); // esc sensor
      break;

    case MSP_SET_MOTOR_CONFIG:
      _model.config.output.minThrottle = m.readU16(); // minthrottle
      _model.config.output.maxThrottle = m.readU16(); // maxthrottle
      _model.config.output.minCommand = m.readU16();  // mincommand
      if(m.remain() >= 2)
      {
        _model.config.output.motorPoles = m.readU8();
        _model.config.output.dshotTelemetry = m.readU8();
      }
      _model.reload();
      break;

    case MSP_MOTOR_3D_CONFIG:
      r.writeU16(1406); // deadband3d_low;
      r.writeU16(1514); // deadband3d_high;
      r.writeU16(1460); // neutral3d;
      break;

    case MSP_SET_MOTOR_3D_CONFIG:
      // Reversible ESC configuration is not used by this firmware yet; consume payload for compatibility.
      if(m.remain() >= 6)
      {
        m.readU16(); // deadband3d_low
        m.readU16(); // deadband3d_high
        m.readU16(); // neutral3d
      }
      _model.reload();
      break;

    case MSP_ARMING_CONFIG:
      r.writeU8(_model.config.arming.autoDisarmDelay); // auto_disarm delay
      r.writeU8(_model.config.arming.disarmKillSwitch); // reserved in BF, persisted for compatibility
      r.writeU8(_model.config.arming.smallAngle); // small angle
      r.writeU8(_model.config.arming.gyroCalOnFirstArm); // gyro cal on first arm
      break;

    case MSP_SET_ARMING_CONFIG:
      if(m.remain() >= 1) _model.config.arming.autoDisarmDelay = m.readU8(); // auto_disarm delay
      if(m.remain() >= 1) _model.config.arming.disarmKillSwitch = m.readU8(); // reserved in BF
      if(m.remain() >= 1) _model.config.arming.smallAngle = std::min<uint8_t>(180, m.readU8()); // small angle
      if(m.remain() >= 1) _model.config.arming.gyroCalOnFirstArm = m.readU8() ? 1 : 0;
      _model.reload();
      break;

    case MSP_RC_DEADBAND:
      r.writeU8(_model.config.input.deadband);
      r.writeU8(_model.config.input.deadband); // yaw deadband
      r.writeU8(0); // alt hold deadband
    #if !defined(ESP32S2)
      r.writeU16((uint16_t)std::max<int16_t>(0, (_model.config.output.deadband3dHigh - _model.config.output.deadband3dLow) / 2)); // deadband 3d throttle
    #else
      r.writeU16(0); // deadband 3d throttle
    #endif
      break;

    case MSP_SET_RC_DEADBAND:
      if(m.remain() >= 1) _model.config.input.deadband = m.readU8();
      if(m.remain() >= 1) _model.config.input.deadband = m.readU8(); // yaw deadband mapped to global deadband
      if(m.remain() >= 1) m.readU8(); // alt hold deadband
      if(m.remain() >= 2)
      {
        const uint16_t deadband3dThrottle = m.readU16();
    #if !defined(ESP32S2)
        const int16_t half = (int16_t)std::min<uint16_t>(deadband3dThrottle, 500);
        const int16_t neutral = _model.config.output.neutral3d;
        _model.config.output.deadband3dLow = std::max<int16_t>(1000, neutral - half);
        _model.config.output.deadband3dHigh = std::min<int16_t>(2000, neutral + half);
      #else
        (void)deadband3dThrottle;
    #endif
      }
      _model.reload();
      break;

    case MSP_RX_CONFIG:
      r.writeU8(_model.config.input.serialRxProvider); // serialrx_provider
      r.writeU16(_model.config.input.maxCheck); //maxcheck
      r.writeU16(_model.config.input.midRc); //midrc
      r.writeU16(_model.config.input.minCheck); //mincheck
      r.writeU8(0); // spectrum bind
      r.writeU16(_model.config.input.minRc); //min_us
      r.writeU16(_model.config.input.maxRc); //max_us
      r.writeU8(_model.config.input.interpolationMode); // rc interpolation
      r.writeU8(_model.config.input.interpolationInterval); // rc interpolation interval
      r.writeU16(1500); // airmode activate threshold
      r.writeU8(_model.config.input.rxSpiProtocol); // rx spi prot
      r.writeU32(0); // rx spi id
      r.writeU8(0); // rx spi chan count
      r.writeU8(_model.config.fpvCamAngleDegrees); // fpv camera angle
      r.writeU8(2); // rc iterpolation channels: RPYT
      r.writeU8(_model.config.input.filterType); // rc_smoothing_type
      r.writeU8(_model.config.input.filter.freq); // rc_smoothing_input_cutoff
      r.writeU8(_model.config.input.filterDerivative.freq); // rc_smoothing_throttle/feedforward_cutoff
      // NOTE: field meaning changes at API 1.47 - before: derivative_cutoff, at 1.47+: throttle_cutoff
      r.writeU8(0); // rc_smoothing_derivative_cutoff (deprecated) / rc_smoothing_throttle_cutoff (1.47+)
      r.writeU8(fromFilterTypeDerivative(_model.config.input.filterDerivative.type)); // rc_smoothing_derivative_type
      r.writeU8(0); // usb type
      // 1.42+
      r.writeU8(_model.config.input.filterAutoFactor); // rc_smoothing_auto_factor
      // 1.44+
      r.writeU8(_model.config.input.rcSmoothing); // rc_smoothing
      // 1.45+
      for(size_t i = 0; i < 6; i++) r.writeU8(_model.config.input.elrsUid[i]); // elrs uid bytes
      // 1.47+
      r.writeU8(_model.config.input.elrsModelId); // elrs model id
      break;

    case MSP2_ESPFC_LANDING_ASSIST_CONFIG:
      r.writeU8(_model.config.landingAssist.enabled);
      r.writeU8(_model.config.landingAssist.throttleIntentMargin);
      r.writeU16((uint16_t)_model.config.landingAssist.descentRateLimitCms);
      r.writeU16(_model.config.landingAssist.descentCorrectivePermille);
      r.writeU16(_model.config.landingAssist.descentCorrectiveMaxPermille);
      r.writeU16(_model.config.landingAssist.baroHeightThresholdCm);
      r.writeU16(_model.config.landingAssist.baroVarioThresholdCms);
      r.writeU16(_model.config.landingAssist.gpsDownThresholdMms);
      r.writeU16(_model.config.landingAssist.gpsGroundThresholdMms);
      r.writeU8(_model.config.landingAssist.flowQualityThreshold);
      r.writeU8(_model.config.landingAssist.flowHandQualityThreshold);
      r.writeU16(_model.config.landingAssist.flowRateThresholdMrad);
      r.writeU16(_model.config.landingAssist.flowRateHandThresholdMrad);
      r.writeU16(_model.config.landingAssist.handVarioThresholdCms);
      r.writeU16(_model.config.landingAssist.handHeightMinCm);
      r.writeU16(_model.config.landingAssist.handHeightMaxCm);
      r.writeU16(_model.config.landingAssist.touchdownHoldMs);
      r.writeU16(_model.config.landingAssist.touchdownRampPermille);
      break;

    case MSP2_ESPFC_SET_LANDING_ASSIST_CONFIG:
      if(m.remain() >= 1) _model.config.landingAssist.enabled = m.readU8();
      if(m.remain() >= 1) _model.config.landingAssist.throttleIntentMargin = m.readU8();
      if(m.remain() >= 2) _model.config.landingAssist.descentRateLimitCms = (int16_t)m.readU16();
      if(m.remain() >= 2) _model.config.landingAssist.descentCorrectivePermille = m.readU16();
      if(m.remain() >= 2) _model.config.landingAssist.descentCorrectiveMaxPermille = m.readU16();
      if(m.remain() >= 2) _model.config.landingAssist.baroHeightThresholdCm = m.readU16();
      if(m.remain() >= 2) _model.config.landingAssist.baroVarioThresholdCms = m.readU16();
      if(m.remain() >= 2) _model.config.landingAssist.gpsDownThresholdMms = m.readU16();
      if(m.remain() >= 2) _model.config.landingAssist.gpsGroundThresholdMms = m.readU16();
      if(m.remain() >= 1) _model.config.landingAssist.flowQualityThreshold = m.readU8();
      if(m.remain() >= 1) _model.config.landingAssist.flowHandQualityThreshold = m.readU8();
      if(m.remain() >= 2) _model.config.landingAssist.flowRateThresholdMrad = m.readU16();
      if(m.remain() >= 2) _model.config.landingAssist.flowRateHandThresholdMrad = m.readU16();
      if(m.remain() >= 2) _model.config.landingAssist.handVarioThresholdCms = m.readU16();
      if(m.remain() >= 2) _model.config.landingAssist.handHeightMinCm = m.readU16();
      if(m.remain() >= 2) _model.config.landingAssist.handHeightMaxCm = m.readU16();
      if(m.remain() >= 2) _model.config.landingAssist.touchdownHoldMs = m.readU16();
      if(m.remain() >= 2) _model.config.landingAssist.touchdownRampPermille = m.readU16();
      break;

    case MSP2_ESPFC_ALT_FUSION_CONFIG:
      r.writeU8(_model.config.altitudeFusion.baroHeightWeight);
      r.writeU8(_model.config.altitudeFusion.baroVarioWeight);
      r.writeU8(_model.config.altitudeFusion.gpsHeightWeight);
      r.writeU8(_model.config.altitudeFusion.gpsVarioWeight);
      r.writeU8(_model.config.altitudeFusion.rangeHeightWeight);
      r.writeU8(_model.config.altitudeFusion.flowVarioWeight);
      r.writeU16(_model.config.altitudeFusion.flowStillRate);
      r.writeU8(_model.config.altitudeFusion.gpsLossHysteresis);
      r.writeU8(_model.config.altitudeFusion.flowLossHysteresis);
      break;

    case MSP2_ESPFC_SET_ALT_FUSION_CONFIG:
      if(m.remain() >= 1) _model.config.altitudeFusion.baroHeightWeight = m.readU8();
      if(m.remain() >= 1) _model.config.altitudeFusion.baroVarioWeight = m.readU8();
      if(m.remain() >= 1) _model.config.altitudeFusion.gpsHeightWeight = m.readU8();
      if(m.remain() >= 1) _model.config.altitudeFusion.gpsVarioWeight = m.readU8();
      if(m.remain() >= 1) _model.config.altitudeFusion.rangeHeightWeight = m.readU8();
      if(m.remain() >= 1) _model.config.altitudeFusion.flowVarioWeight = m.readU8();
      if(m.remain() >= 2) _model.config.altitudeFusion.flowStillRate = (int16_t)m.readU16();
      if(m.remain() >= 1) _model.config.altitudeFusion.gpsLossHysteresis = m.readU8();
      if(m.remain() >= 1) _model.config.altitudeFusion.flowLossHysteresis = m.readU8();
      break;

    case MSP_SET_RX_CONFIG:
      _model.config.input.serialRxProvider = m.readU8(); // serialrx_provider
      _model.config.input.maxCheck = m.readU16(); //maxcheck
      _model.config.input.midRc = m.readU16(); //midrc
      _model.config.input.minCheck = m.readU16(); //mincheck
      m.readU8(); // spectrum bind
      _model.config.input.minRc = m.readU16(); //min_us
      _model.config.input.maxRc = m.readU16(); //max_us
      if (m.remain() >= 4) {
        _model.config.input.interpolationMode = m.readU8(); // rc interpolation
        _model.config.input.interpolationInterval = m.readU8(); // rc interpolation interval
        m.readU16(); // airmode activate threshold
      }
      if (m.remain() >= 6) {
        _model.config.input.rxSpiProtocol = m.readU8(); // rx spi prot
        m.readU32(); // rx spi id
        m.readU8(); // rx spi chan count
      }
      if (m.remain() >= 1) {
        _model.config.fpvCamAngleDegrees = m.readU8(); // fpv camera angle
      }
      // 1.40+
      if (m.remain() >= 6) {
        m.readU8(); // rc iterpolation channels
        _model.config.input.filterType = m.readU8(); // rc_smoothing_type
        _model.config.input.filter.freq = m.readU8(); // rc_smoothing_input_cutoff
        _model.config.input.filterDerivative.freq = m.readU8(); // rc_smoothing_throttle/feedforward_cutoff
        m.readU8(); // was rc_smoothing_derivative_cutoff (deprecated in 1.47), now throttle_cutoff unused here
        _model.config.input.filterDerivative.type = toFilterTypeDerivative(m.readU8()); // rc_smoothing_derivative_type
      }
      if (m.remain() >= 1) {
        m.readU8(); // usb type
      }
      // 1.42+
      if (m.remain() >= 1) {
        _model.config.input.filterAutoFactor = m.readU8(); // rc_smoothing_auto_factor
      }
      if (m.remain() >= 1) {
        _model.config.input.rcSmoothing = m.readU8(); // rc_smoothing
      }
      if (m.remain() >= 6) {
        for(size_t i = 0; i < 6; i++) _model.config.input.elrsUid[i] = m.readU8(); // elrs uid bytes
      }
      if (m.remain() >= 1) {
        _model.config.input.elrsModelId = m.readU8(); // elrs model id
      }

      _model.reload();
      break;

    case MSP_FAILSAFE_CONFIG:
      r.writeU8(_model.config.failsafe.delay); // failsafe_delay
      r.writeU8(_model.config.failsafe.offDelay); // failsafe_off_delay
      r.writeU16(_model.config.failsafe.throttle); //failsafe_throttle
      r.writeU8(_model.config.failsafe.killSwitch); // failsafe_kill_switch
      r.writeU16(_model.config.failsafe.throttleLowDelay); // failsafe_throttle_low_delay
      r.writeU8(_model.config.failsafe.procedure); //failsafe_procedure
      break;

    case MSP_SET_FAILSAFE_CONFIG:
      if(m.remain() >= 1) _model.config.failsafe.delay = m.readU8(); // failsafe_delay
      if(m.remain() >= 1) _model.config.failsafe.offDelay = m.readU8(); // failsafe_off_delay
      if(m.remain() >= 2) _model.config.failsafe.throttle = m.readU16(); // failsafe_throttle
      if(m.remain() >= 1) _model.config.failsafe.killSwitch = m.readU8(); // failsafe_kill_switch
      if(m.remain() >= 2) _model.config.failsafe.throttleLowDelay = m.readU16(); // failsafe_throttle_low_delay
      if(m.remain() >= 1) _model.config.failsafe.procedure = m.readU8(); // failsafe_procedure
      _model.reload();
      break;

    case MSP_RXFAIL_CONFIG:
      for(size_t i = 0; i < INPUT_CHANNELS; i++)
      {
        r.writeU8(_model.config.input.channel[i].fsMode);
        r.writeU16(_model.config.input.channel[i].fsValue);
      }
      break;

    case MSP_SET_RXFAIL_CONFIG:
      {
        if(m.remain() < 4)
        {
          r.result = 0;
          break;
        }
        size_t i = m.readU8();
        if(i < INPUT_CHANNELS)
        {
          _model.config.input.channel[i].fsMode = std::clamp<int8_t>(m.readU8(), 0, 2); // mode: auto/hold/set
          _model.config.input.channel[i].fsValue = std::clamp<int16_t>((int16_t)m.readU16(), _model.config.input.minRc, _model.config.input.maxRc); // pulse
        }
        else
        {
          r.result = 0;
        }
        _model.reload();
      }
      break;

    case MSP_RC:
      for(size_t i = 0; i < _model.state.input.channelCount; i++)
      {
        r.writeU16(lrintf(_model.state.input.us[i]));
      }
      break;

    case MSP_RC_TUNING:
      r.writeU8(_model.config.input.rate[AXIS_ROLL]);
      r.writeU8(_model.config.input.expo[AXIS_ROLL]);
      for(size_t i = 0; i < AXIS_COUNT_RPY; i++)
      {
        r.writeU8(_model.config.input.superRate[i]);
      }
      r.writeU8(_model.config.controller.tpaScale); // dyn thr pid
      r.writeU8(50); // thrMid8
      r.writeU8(_model.config.input.throttleExpo);  // thr expo
      r.writeU16(_model.config.controller.tpaBreakpoint); // tpa breakpoint
      r.writeU8(_model.config.input.expo[AXIS_YAW]); // yaw expo
      r.writeU8(_model.config.input.rate[AXIS_YAW]); // yaw rate
      r.writeU8(_model.config.input.rate[AXIS_PITCH]); // pitch rate
      r.writeU8(_model.config.input.expo[AXIS_PITCH]); // pitch expo
      // 1.41+
      r.writeU8(_model.config.output.throttleLimitType); // throttle_limit_type (off)
      r.writeU8(_model.config.output.throttleLimitPercent); // throtle_limit_percent (100%)
      //1.42+
      r.writeU16(_model.config.input.rateLimit[0]); // rate limit roll
      r.writeU16(_model.config.input.rateLimit[1]); // rate limit pitch
      r.writeU16(_model.config.input.rateLimit[2]); // rate limit yaw
      // 1.43+
      r.writeU8(_model.config.input.rateType); // rates type

      break;

    case MSP_SET_RC_TUNING:
      if(m.remain() >= 10)
      {
        const uint8_t rate = m.readU8();
        if(_model.config.input.rate[AXIS_PITCH] == _model.config.input.rate[AXIS_ROLL])
        {
          _model.config.input.rate[AXIS_PITCH] = rate;
        }
        _model.config.input.rate[AXIS_ROLL] = rate;

        const uint8_t expo = m.readU8();
        if(_model.config.input.expo[AXIS_PITCH] == _model.config.input.expo[AXIS_ROLL])
        {
          _model.config.input.expo[AXIS_PITCH] = expo;
        }
        _model.config.input.expo[AXIS_ROLL] = expo;

        for(size_t i = 0; i < AXIS_COUNT_RPY; i++)
        {
          _model.config.input.superRate[i] = m.readU8();
        }
        _model.config.controller.tpaScale = Utils::clamp(m.readU8(), (uint8_t)0, (uint8_t)90); // dyn thr pid
        m.readU8(); // thrMid8
        _model.config.input.throttleExpo = Utils::clamp(m.readU8(), (uint8_t)0, (uint8_t)100);  // thr expo
        _model.config.controller.tpaBreakpoint = Utils::clamp(m.readU16(), (uint16_t)1000, (uint16_t)2000); // tpa breakpoint
        if(m.remain() >= 1)
        {
          _model.config.input.expo[AXIS_YAW] = m.readU8(); // yaw expo
        }
        if(m.remain() >= 1)
        {
          _model.config.input.rate[AXIS_YAW]  = m.readU8(); // yaw rate
        }
        if(m.remain() >= 1)
        {
          _model.config.input.rate[AXIS_PITCH] = m.readU8(); // pitch rate
        }
        if(m.remain() >= 1)
        {
          _model.config.input.expo[AXIS_PITCH]  = m.readU8(); // pitch expo
        }
        // 1.41
        if(m.remain() >= 2)
        {
          _model.config.output.throttleLimitType = m.readU8(); // throttle_limit_type
          _model.config.output.throttleLimitPercent = m.readU8(); // throttle_limit_percent
        }
        // 1.42
        if(m.remain() >= 6)
        {
          _model.config.input.rateLimit[0] = m.readU16(); // roll
          _model.config.input.rateLimit[1] = m.readU16(); // pitch
          _model.config.input.rateLimit[2] = m.readU16(); // yaw
        }
        // 1.43
        if (m.remain() >= 1)
        {
          _model.config.input.rateType = m.readU8();
        }
        _model.syncActiveRateProfile();
        _model.reload();
      }
      else
      {
        r.result = -1;
        // error
      }
      break;

    case MSP_ADVANCED_CONFIG:
      // Report both denoms for configurator compatibility.
      // gyroSyncDenom and pidProcessDenom are both backed by loopSync.
      r.writeU8(std::max<int>(1, _model.config.loopSync));
      r.writeU8(std::max<int>(1, _model.config.loopSync));
      r.writeU8(_model.config.output.async);
      r.writeU8(_model.config.output.protocol);
      r.writeU16(_model.config.output.rate);
      r.writeU16(_model.config.output.dshotIdle);
      r.writeU8(0);    // 32k gyro
      r.writeU8(0);    // PWM inversion
      r.writeU8(0);    // gyro_to_use: {1:0, 2:1. 2:both}
      r.writeU8(0);    // gyro high fsr (flase)
      r.writeU8(48);   // gyro cal threshold
      r.writeU16(125); // gyro cal duration (1.25s)
      r.writeU16(0);   // gyro offset yaw
      r.writeU8(0);    // check overflow
      r.writeU8(_model.config.debug.mode);
      r.writeU8(DEBUG_COUNT);
      break;

    case MSP_SET_ADVANCED_CONFIG:
      {
        if(m.remain() >= 2)
        {
          const uint8_t gyroSyncDenom = m.readU8();
          const uint8_t pidProcessDenom = m.readU8();
          const uint8_t selectedDenom = pidProcessDenom ? pidProcessDenom : gyroSyncDenom;
          _model.config.loopSync = std::max<int>(1, selectedDenom);
        }
      }
      if(m.remain() >= 1) _model.config.output.async = m.readU8();
      if(m.remain() >= 1) _model.config.output.protocol = m.readU8();
      if(m.remain() >= 2) _model.config.output.rate = m.readU16();
      if(m.remain() >= 2) {
        _model.config.output.dshotIdle = m.readU16(); // dshot idle
      }
      if(m.remain()) {
        m.readU8();  // 32k gyro
      }
      if(m.remain()) {
        m.readU8();  // PWM inversion
      }
      if(m.remain() >= 8) {
        m.readU8();  // gyro_to_use
        m.readU8();  // gyro high fsr
        m.readU8();  // gyro cal threshold
        m.readU16(); // gyro cal duration
        m.readU16(); // gyro offset yaw
        m.readU8();  // check overflow
      }
      if(m.remain()) {
        _model.config.debug.mode = m.readU8();
      }
      _model.reload();
      break;

    //case MSP_COMPASS_CONFIG:
    //  r.writeU16(0); // mag_declination * 10
    //  break;

    case MSP_FILTER_CONFIG:
      r.writeU8(_model.config.gyro.filter.freq);           // gyro lpf
      r.writeU16(_model.config.dterm.filter.freq);         // dterm lpf
      r.writeU16(_model.config.yaw.filter.freq);           // yaw lpf
      r.writeU16(_model.config.gyro.notch1Filter.freq);    // gyro notch 1 hz
      r.writeU16(_model.config.gyro.notch1Filter.cutoff);  // gyro notch 1 cutoff
      r.writeU16(_model.config.dterm.notchFilter.freq);    // dterm notch hz
      r.writeU16(_model.config.dterm.notchFilter.cutoff);  // dterm notch cutoff
      r.writeU16(_model.config.gyro.notch2Filter.freq);    // gyro notch 2 hz
      r.writeU16(_model.config.gyro.notch2Filter.cutoff);  // gyro notch 2 cutoff
      r.writeU8(_model.config.dterm.filter.type);          // dterm type
      r.writeU8(fromGyroDlpf(_model.config.gyro.dlpf));
      r.writeU8(0);                                        // dlfp 32khz type
      r.writeU16(_model.config.gyro.filter.freq);          // lowpass1 freq
      r.writeU16(_model.config.gyro.filter2.freq);         // lowpass2 freq
      r.writeU8(_model.config.gyro.filter.type);           // lowpass1 type
      r.writeU8(_model.config.gyro.filter2.type);          // lowpass2 type
      r.writeU16(_model.config.dterm.filter2.freq);        // dterm lopwass2 freq
      // 1.41+
      r.writeU8(_model.config.dterm.filter2.type);         // dterm lopwass2 type
      r.writeU16(_model.config.gyro.dynLpfFilter.cutoff);  // dyn lpf gyro min
      r.writeU16(_model.config.gyro.dynLpfFilter.freq);    // dyn lpf gyro max
      r.writeU16(_model.config.dterm.dynLpfFilter.cutoff); // dyn lpf dterm min
      r.writeU16(_model.config.dterm.dynLpfFilter.freq);   // dyn lpf dterm max
      // gyro analyse
      r.writeU8(3);  // deprecated dyn notch range
      r.writeU8(_model.config.gyro.dynamicFilter.count);  // dyn_notch_width_percent
      r.writeU16(_model.config.gyro.dynamicFilter.q); // dyn_notch_q
      r.writeU16(_model.config.gyro.dynamicFilter.min_freq); // dyn_notch_min_hz
      // rpm filter
      r.writeU8(_model.config.gyro.rpmFilter.harmonics);  // gyro_rpm_notch_harmonics
      r.writeU8(_model.config.gyro.rpmFilter.minFreq);  // gyro_rpm_notch_min
      // 1.43+
      r.writeU16(_model.config.gyro.dynamicFilter.max_freq); // dyn_notch_max_hz
      break;

    case MSP_SET_FILTER_CONFIG:
      _model.config.gyro.filter.freq = m.readU8();
      _model.config.dterm.filter.freq = m.readU16();
      _model.config.yaw.filter.freq = m.readU16();
      if (m.remain() >= 8) {
          _model.config.gyro.notch1Filter.freq = m.readU16();
          _model.config.gyro.notch1Filter.cutoff = m.readU16();
          _model.config.dterm.notchFilter.freq = m.readU16();
          _model.config.dterm.notchFilter.cutoff = m.readU16();
      }
      if (m.remain() >= 4) {
          _model.config.gyro.notch2Filter.freq = m.readU16();
          _model.config.gyro.notch2Filter.cutoff = m.readU16();
      }
      if (m.remain() >= 1) {
          _model.config.dterm.filter.type = (FilterType)m.readU8();
      }
      if (m.remain() >= 10) {
        m.readU8(); // dlfp type
        m.readU8(); // 32k dlfp type
        _model.config.gyro.filter.freq = m.readU16();
        _model.config.gyro.filter2.freq = m.readU16();
        _model.config.gyro.filter.type = m.readU8();
        _model.config.gyro.filter2.type = m.readU8();
        _model.config.dterm.filter2.freq = m.readU16();
      }
      // 1.41+
      if (m.remain() >= 9) {
        _model.config.dterm.filter2.type = m.readU8();
        _model.config.gyro.dynLpfFilter.cutoff = m.readU16(); // dyn gyro lpf min
        _model.config.gyro.dynLpfFilter.freq = m.readU16();   // dyn gyro lpf max
        _model.config.dterm.dynLpfFilter.cutoff = m.readU16(); // dyn dterm lpf min
        _model.config.dterm.dynLpfFilter.freq = m.readU16();   // dyn dterm lpf min
      }
      if (m.remain() >= 8) {
        m.readU8();  // deprecated dyn_notch_range
        _model.config.gyro.dynamicFilter.count = m.readU8();  // dyn_notch_width_percent
        _model.config.gyro.dynamicFilter.q = m.readU16(); // dyn_notch_q
        _model.config.gyro.dynamicFilter.min_freq = m.readU16(); // dyn_notch_min_hz
        _model.config.gyro.rpmFilter.harmonics = m.readU8();  // gyro_rpm_notch_harmonics
        _model.config.gyro.rpmFilter.minFreq = m.readU8();  // gyro_rpm_notch_min
      }
      // 1.43+
      if (m.remain() >= 1) {
        _model.config.gyro.dynamicFilter.max_freq = m.readU16(); // dyn_notch_max_hz
      }
      _model.reload();
      break;

    case MSP_PID_CONTROLLER:
      r.writeU8(1); // betaflight controller id
      break;

    case MSP_PIDNAMES:
      r.writeString(F("ROLL;PITCH;YAW;ALT;Pos;PosR;NavR;LEVEL;MAG;VEL;"));
      break;

    case MSP_PID:
      for(size_t i = 0; i < PID_ITEM_COUNT; i++)
      {
        r.writeU8(_model.config.pid[i].P);
        r.writeU8(_model.config.pid[i].I);
        r.writeU8(_model.config.pid[i].D);
      }
      break;

    case MSP_SET_PID:
      for (int i = 0; i < PID_ITEM_COUNT; i++)
      {
        _model.config.pid[i].P = m.readU8();
        _model.config.pid[i].I = m.readU8();
        _model.config.pid[i].D = m.readU8();
      }
      _model.syncActivePidProfile();  // Update active PID profile with new values
      _model.reload();
      break;

    case MSP_PID_ADVANCED: /// !!!FINISHED HERE!!!
      r.writeU16(0);
      r.writeU16(0);
      r.writeU16(0); // was pidProfile.yaw_p_limit
      r.writeU8(0); // reserved
      r.writeU8(_model.config.dterm.vbatPidCompensation); // vbatPidCompensation;
      r.writeU8(_model.config.dterm.feedForwardTransition); // feedForwardTransition;
      r.writeU8((uint8_t)std::min(_model.config.dterm.setpointWeight, (int16_t)255)); // was low byte of dtermSetpointWeight
      r.writeU8(0); // reserved
      r.writeU8(0); // reserved
      r.writeU8(0); // reserved
      r.writeU16(0); // rateAccelLimit;
      r.writeU16(0); // yawRateAccelLimit;
      r.writeU8(_model.config.level.angleLimit); // levelAngleLimit;
      r.writeU8(_model.config.level.horizonStrength); // horizon strength
      r.writeU16(0); // itermThrottleThreshold;
      r.writeU16(1000); // itermAcceleratorGain; anti_gravity_gain, 0 in 1.45+
      r.writeU16(_model.config.dterm.setpointWeight);
      r.writeU8(_model.config.iterm.itermRotation ? 1 : 0); // iterm rotation
      r.writeU8(0); // smart feed forward
      r.writeU8(_model.config.iterm.relax); // iterm relax
      r.writeU8(1); // iterm relax type (setpoint only)
      r.writeU8(0); // abs control gain
      r.writeU8(0); // throttle boost
      r.writeU8(0); // acro trainer max angle
      r.writeU16(_model.config.pid[FC_PID_ROLL].F); //pid roll f
      r.writeU16(_model.config.pid[FC_PID_PITCH].F); //pid pitch f
      r.writeU16(_model.config.pid[FC_PID_YAW].F); //pid yaw f
      r.writeU8(0); // antigravity mode
      // 1.41+
      r.writeU8(_model.config.dterm.dMinRoll); // d min roll
      r.writeU8(_model.config.dterm.dMinPitch); // d min pitch
      r.writeU8(_model.config.dterm.dMinYaw); // d min yaw
      r.writeU8(37); // d min gain (default)
      r.writeU8(20); // d min advance (default)
      r.writeU8(_model.config.level.integratedYaw ? 1 : 0); // use_integrated_yaw
      r.writeU8(0); // integrated_yaw_relax
      // 1.42+
      r.writeU8(_model.config.iterm.relaxCutoff); // iterm_relax_cutoff
      // 1.43+
      r.writeU8(_model.config.output.motorLimit); // motor_output_limit
      r.writeU8(0); // auto_profile_cell_count
      r.writeU8(0); // idle_min_rpm
      break;

    case MSP_SET_PID_ADVANCED:
    {
      m.readU16();
      m.readU16();
      m.readU16(); // was pidProfile.yaw_p_limit
      m.readU8(); // reserved
      m.readU8();
      _model.config.dterm.feedForwardTransition = m.readU8();
      const uint8_t legacySetpointWeight = m.readU8();
      m.readU8(); // reserved
      m.readU8(); // reserved
      m.readU8(); // reserved
      m.readU16();
      m.readU16();
      if (m.remain() >= 2) {
        _model.config.level.angleLimit = m.readU8();
        _model.config.level.horizonStrength = m.readU8();
      }
      if (m.remain() >= 4) {
        m.readU16(); // itermThrottleThreshold;
        m.readU16(); // itermAcceleratorGain; anti_gravity_gain
      }
      if (m.remain() >= 2) {
        _model.config.dterm.setpointWeight = m.readU16();
      }
      else {
        _model.config.dterm.setpointWeight = legacySetpointWeight;
      }
      if (m.remain() >= 14) {
        _model.config.iterm.itermRotation = m.readU8() != 0; // iterm rotation
        m.readU8(); // smart feed forward
        _model.config.iterm.relax = m.readU8(); // iterm relax
        m.readU8(); // iterm relax type
        m.readU8(); // abs control gain
        m.readU8(); // throttle boost
        m.readU8(); // acro trainer max angle
        _model.config.pid[FC_PID_ROLL].F = m.readU16(); // pid roll f
        _model.config.pid[FC_PID_PITCH].F = m.readU16(); // pid pitch f
        _model.config.pid[FC_PID_YAW].F = m.readU16(); // pid yaw f
        m.readU8(); // antigravity mode
      }
      // 1.41+
      if (m.remain() >= 7) {
        _model.config.dterm.dMinRoll = m.readU8(); // d min roll
        _model.config.dterm.dMinPitch = m.readU8(); // d min pitch
        _model.config.dterm.dMinYaw = m.readU8(); // d min yaw
        m.readU8(); // d min gain (per-item, not yet supported)
        m.readU8(); // d min advance (per-item, not yet supported)
        _model.config.level.integratedYaw = m.readU8() != 0; // use_integrated_yaw
        m.readU8(); // integrated_yaw_relax
      }
      // 1.42+
      if (m.remain() >= 1) {
        _model.config.iterm.relaxCutoff = m.readU8(); // iterm_relax_cutoff
      }
      // 1.43+
      if (m.remain() >= 3) {
        _model.config.output.motorLimit = m.readU8(); // motor_output_limit
        m.readU8(); // auto_profile_cell_count
        m.readU8(); // idle_min_rpm
      }
      _model.syncActivePidProfile();  // Update active PID profile with new values
      _model.reload();
      break;
    }

    case MSP_RAW_IMU:
      for (int i = 0; i < AXIS_COUNT_RPY; i++)
      {
        r.writeU16(lrintf(_model.state.accel.adc[i] * ACCEL_G_INV * 2048.f));
      }
      for (int i = 0; i < AXIS_COUNT_RPY; i++)
      {
        r.writeU16(lrintf(Utils::toDeg(_model.state.gyro.adc[i])));
      }
      for (int i = 0; i < AXIS_COUNT_RPY; i++)
      {
        r.writeU16(lrintf(_model.state.mag.adc[i] * 1090));
      }
      break;

    case MSP_MOTOR:
      for (size_t i = 0; i < 8; i++)
      {
        if (i >= OUTPUT_CHANNELS || _model.config.pin[i + PIN_OUTPUT_0] == -1)
        {
          r.writeU16(0);
          continue;
        }
        r.writeU16(_model.state.output.us[i]);
      }
      break;

    case MSP_MOTOR_TELEMETRY:
      r.writeU8(OUTPUT_CHANNELS);
      for (size_t i = 0; i < OUTPUT_CHANNELS; i++)
      {
        int rpm = 0;
        uint16_t invalidPct = 0;
        uint8_t escTemperature = 0;  // degrees celcius
        uint16_t escVoltage = 0;     // 0.01V per unit
        uint16_t escCurrent = 0;     // 0.01A per unit
        uint16_t escConsumption = 0; // mAh

        if (_model.config.pin[i + PIN_OUTPUT_0] != -1)
        {
          rpm = lrintf(_model.state.output.telemetry.rpm[i]);
          invalidPct = _model.state.output.telemetry.errors[i];
          escTemperature = _model.state.output.telemetry.temperature[i];
          escVoltage = _model.state.output.telemetry.voltage[i];
          escCurrent = _model.state.output.telemetry.current[i];
        }

        r.writeU32(rpm);
        r.writeU16(invalidPct);
        r.writeU8(escTemperature);
        r.writeU16(escVoltage);
        r.writeU16(escCurrent);
        r.writeU16(escConsumption);
      }
      break;

    case MSP_ESC_SENSOR_DATA:
      // Legacy ESC telemetry command used by some clients (DJI/older tools).
      if(_model.config.output.dshotTelemetry)
      {
        const uint8_t motorCount = (uint8_t)_model.state.currentMixer.count;
        r.writeU8(motorCount);
        for (size_t i = 0; i < motorCount; i++)
        {
          r.writeU8(_model.state.output.telemetry.temperature[i]);
          r.writeU16((uint16_t)Utils::clamp<int32_t>(lrintf(_model.state.output.telemetry.rpm[i]), 0, 0xFFFF));
        }
      }
      else
      {
        r.result = 0;
      }
      break;

    case MSP_SET_MOTOR:
      if(_model.isFeatureActive(FEATURE_GPS) && _model.config.blackbox.mode > 0)
      {
        _model.setGpsHome(true);
      }
      for(size_t i = 0; i < OUTPUT_CHANNELS; i++)
      {
        _model.state.output.disarmed[i] = m.readU16();
      }
      break;

    case MSP_SERVO:
      for(size_t i = 0; i < OUTPUT_CHANNELS; i++)
      {
        if (i >= OUTPUT_CHANNELS || _model.config.pin[i + PIN_OUTPUT_0] == -1)
        {
          r.writeU16(1500);
          continue;
        }
        r.writeU16(_model.state.output.us[i]);
      }
      break;

    case MSP_SERVO_CONFIGURATIONS:
      for(size_t i = 0; i < 8; i++)
      {
        if(i < OUTPUT_CHANNELS)
        {
          r.writeU16(_model.config.output.channel[i].min);
          r.writeU16(_model.config.output.channel[i].max);
          r.writeU16(_model.config.output.channel[i].neutral);
        }
        else
        {
          r.writeU16(1000);
          r.writeU16(2000);
          r.writeU16(1500);
        }
        r.writeU8(100);
        r.writeU8(-1);
        r.writeU32(0);
      }
      break;

    case MSP_SET_SERVO_CONFIGURATION:
      {
        uint8_t i = m.readU8();
        if(i < OUTPUT_CHANNELS)
        {
          _model.config.output.channel[i].min = constrain(m.readU16(), 500, 2500);
          _model.config.output.channel[i].max = constrain(m.readU16(), 500, 2500);
          _model.config.output.channel[i].neutral = constrain(m.readU16(), 500, 2500);
          m.readU8();
          m.readU8();
          m.readU32();
          if(_model.config.output.channel[i].min > _model.config.output.channel[i].max)
            std::swap(_model.config.output.channel[i].min, _model.config.output.channel[i].max);
        }
        else
        {
          r.result = -1;
        }
        _model.reload();
      }
      break;

    case MSP_ACC_CALIBRATION:
      if(!_model.isModeActive(MODE_ARMED)) _model.calibrateGyro(); // Triggers both gyro and accel calibration
      break;

    case MSP_MAG_CALIBRATION:
      if(!_model.isModeActive(MODE_ARMED)) _model.calibrateMag();
      break;

    case MSP2_ESPFC_CALIBRATION_DATA:
      // Gyro calibration data
      r.writeU16(_model.state.gyro.bias.x * 100); // Roll bias (deg/s * 100)
      r.writeU16(_model.state.gyro.bias.y * 100); // Pitch bias (deg/s * 100)
      r.writeU16(_model.state.gyro.bias.z * 100); // Yaw bias (deg/s * 100)
      // Accel calibration data
      r.writeU16(_model.state.accel.bias.x); // Roll offset
      r.writeU16(_model.state.accel.bias.y); // Pitch offset
      r.writeU16(_model.state.accel.bias.z); // Yaw offset
      // Mag calibration data (stored in config)
      r.writeU16(_model.config.mag.offset[0]); // Roll offset
      r.writeU16(_model.config.mag.offset[1]); // Pitch offset
      r.writeU16(_model.config.mag.offset[2]); // Yaw offset
      break;

    case MSP2_ESPFC_SENSOR_STATUS:
      // Report calibration status for each sensor
      r.writeU8(_model.state.gyro.calibrationState); // Gyro: 0=idle, 1=start, 2=in_progress, 3=save, 4=done
      r.writeU8(_model.state.accel.calibrationState); // Accel calibration state
      r.writeU8(_model.state.mag.calibrationState); // Mag calibration state
      r.writeU8((_model.state.gyro.calibrationState || _model.state.accel.calibrationState || _model.state.mag.calibrationState) ? 1 : 0); // Any calibration in progress
      break;

    case MSP2_ESPFC_OBSTACLE_AVOIDANCE_CONFIG:
      // Report obstacle avoidance configuration
      {
        const auto& oac = _model.config.obstacleAvoidance;
        r.writeU8(oac.enabled);
        r.writeU16(oac.minSafeDistance);     // cm
        r.writeU16(oac.avoidanceDistance);   // cm
        r.writeU16(oac.maxAvoidanceDistance); // cm
        r.writeU8(oac.slowdownPercent);
        r.writeU8(oac.stopPercent);
        r.writeU8(oac.avoidanceMode);
        r.writeU8(oac.bypassAxis);
        r.writeU8(oac.enableInAcro);
        r.writeU8(oac.enableInHorizon);
        r.writeU8(oac.enableInAngle);
        r.writeU8(oac.enableInAltHold);
      }
      break;

    case MSP2_ESPFC_SET_OBSTACLE_AVOIDANCE_CONFIG:
      // Set obstacle avoidance configuration
      if(m.remain() >= 15)
      {
        auto& oac = _model.config.obstacleAvoidance;
        oac.enabled = m.readU8();
        oac.minSafeDistance = m.readU16();
        oac.avoidanceDistance = m.readU16();
        oac.maxAvoidanceDistance = m.readU16();
        oac.slowdownPercent = m.readU8();
        oac.stopPercent = m.readU8();
        oac.avoidanceMode = m.readU8();
        oac.bypassAxis = m.readU8();
        oac.enableInAcro = m.readU8();
        oac.enableInHorizon = m.readU8();
        oac.enableInAngle = m.readU8();
        oac.enableInAltHold = m.readU8();
        _model.onAccChange();
      }
      break;

      case MSP2_ESPFC_SENSOR_RANGEFINDER_FRONT:
        // Report front rangefinder telemetry for obstacle avoidance
        r.writeU8(_model.state.rangefinder[RANGEFINDER_FRONT].present ? 255 : 0);
        r.writeU32((uint32_t)(_model.state.rangefinder[RANGEFINDER_FRONT].present ? (int32_t)_model.state.rangefinder[RANGEFINDER_FRONT].distance : 0));
        break;

      case MSP2_ESPFC_SENSOR_RANGEFINDER_STATUS:
        // Report status of both rangefinders (bottom and front)
        r.writeU8(_model.state.rangefinder[RANGEFINDER_BOTTOM].present ? 1 : 0);  // Bottom sensor present
        r.writeU32(_model.state.rangefinder[RANGEFINDER_BOTTOM].distance);         // Bottom distance (mm)
        r.writeU8(_model.state.rangefinder[RANGEFINDER_FRONT].present ? 1 : 0);   // Front sensor present
        r.writeU32(_model.state.rangefinder[RANGEFINDER_FRONT].distance);          // Front distance (mm)
        break;

      case MSP2_ESPFC_RANGEFINDER_CONFIG:
        // Per-sensor config blocks: position, bus, dev, address, enabled
        for(uint8_t i = 0; i < RANGEFINDER_COUNT; i++)
        {
          const auto& cfg = _model.config.rangefinder[i];
          r.writeU8(i);
          r.writeU8(cfg.bus);
          r.writeU8(cfg.dev);
          r.writeU8(cfg.address);
          r.writeU8(cfg.enabled ? 1 : 0);
        }
        break;

      case MSP2_ESPFC_SET_RANGEFINDER_CONFIG:
        // Accept one or more config blocks: position, bus, dev, address, enabled
        while(m.remain() >= 5)
        {
          const uint8_t position = m.readU8();
          const uint8_t bus = m.readU8();
          const uint8_t dev = m.readU8();
          const uint8_t address = m.readU8();
          const uint8_t enabled = m.readU8();

          if(position < RANGEFINDER_COUNT)
          {
            auto& cfg = _model.config.rangefinder[position];
            cfg.position = position;
            cfg.bus = std::clamp<int>(bus, BUS_NONE, BUS_MAX - 1);
            cfg.dev = std::clamp<int>(dev, Device::RANGEFINDER_DEFAULT, Device::RANGEFINDER_MAX - 1);
            cfg.address = address;
            cfg.enabled = enabled ? 1 : 0;
          }
        }
        _model.reload();
        break;

    case MSP_VTX_CONFIG:
      if (!_model.state.vtx.active) {
        r.writeU8(0); // vtx type
        r.writeU8(0); // band
        r.writeU8(0); // channel
        r.writeU8(0); // power
        r.writeU8(0); // status
        r.writeU16(0); // freq
        r.writeU8(0); // ready
        r.writeU8(0); // low power disarm
      } else {
        r.writeU8(3 /* SMARTAUDIO */); // vtx type unknown
        r.writeU8(_model.config.vtx.band);    // band
        r.writeU8(_model.config.vtx.channel); // channel
        r.writeU8(_model.config.vtx.power);   // power
        r.writeU8(0);    // status (looks like 1 means pit mode :shrug:)
        r.writeU16(_model.config.vtx.freq);   // freq
        r.writeU8(1);    // ready
        r.writeU8(_model.config.vtx.lowPowerDisarm);    // low power disarm
      }
      // 1.42
      r.writeU16(_model.config.vtx.pitModeFreq);  // pit mode freq
      r.writeU8(1);    // vtx table available (yes)
      r.writeU8(_model.config.vtxTable.bands);
      r.writeU8(_model.config.vtxTable.channels);
      r.writeU8(_model.config.vtxTable.powerLevels);
      break;
    
    case MSP_SET_VTX_CONFIG:
      {
        uint16_t newFrequency = m.readU16();
        if(newFrequency <= VTXCOMMON_MSP_BANDCHAN_CHKVAL)
        {
          const uint8_t newBand = (newFrequency / 8) + 1;
          const uint8_t newChannel = (newFrequency % 8) + 1;
          vtxApplyBandChannel(_model.config, newBand, newChannel);
        }
        else if(newFrequency <= 5999)
        {
          _model.config.vtx.band = 0;
          _model.config.vtx.freq = newFrequency;
        }

        if (m.remain() >= 2) {
          _model.config.vtx.power =  m.readU8();
          /*const uint8_t newPitmode = */m.readU8();
        }

        if (m.remain()) {
          _model.config.vtx.lowPowerDisarm = m.readU8();
        }

        // API version 1.42 - this parameter kept separate since clients may already be supplying
        if (m.remain() >= 2) {
          _model.config.vtx.pitModeFreq = m.readU16();
        }

        // API version 1.42 - extensions for non-encoded versions of the band, channel or frequency
        if (m.remain() >= 4) {
          // Added standalone values for band, channel and frequency to move
          // away from the flawed encoded combined method originally implemented.
          uint8_t newBand = m.readU8();
          uint8_t newChannel = m.readU8();
          uint16_t newFreq = m.readU16();
          if(newBand) newFreq = vtxLookupFrequency(_model.config, newBand, newChannel);
          _model.config.vtx.band = newBand;
          _model.config.vtx.channel = newChannel;
          _model.config.vtx.freq = newFreq;
        }

        // API version 1.42 - vtx table support.
        if (m.remain() >= 4) {
          uint8_t newBandCount = m.readU8();
          uint8_t newChannelCount = m.readU8();
          uint8_t newPowerCount = m.readU8();
          uint8_t clearTable = m.readU8();

          if(newBandCount > VTX_TABLE_MAX_BANDS || newChannelCount > VTX_TABLE_MAX_CHANNELS || newPowerCount > VTX_TABLE_MAX_POWER_LEVELS)
          {
            r.result = 0;
            break;
          }

          _model.config.vtxTable.bands = newBandCount;
          _model.config.vtxTable.channels = newChannelCount;
          _model.config.vtxTable.powerLevels = newPowerCount;

          if(clearTable)
          {
            for(size_t band = 0; band < VTX_TABLE_MAX_BANDS; band++)
            {
              _model.config.vtxTable.bandLetters[band] = (char)('1' + band);
              _model.config.vtxTable.isFactoryBand[band] = 0;
              for(size_t i = 0; i < VTX_TABLE_BAND_NAME_LENGTH; i++)
              {
                _model.config.vtxTable.bandNames[band][i] = ' ';
              }
              _model.config.vtxTable.bandNames[band][VTX_TABLE_BAND_NAME_LENGTH] = '\0';
              for(size_t channel = 0; channel < VTX_TABLE_MAX_CHANNELS; channel++)
              {
                _model.config.vtxTable.frequency[band][channel] = 0;
              }
            }

            for(size_t i = 0; i < VTX_TABLE_MAX_POWER_LEVELS; i++)
            {
              _model.config.vtxTable.powerValues[i] = 0;
              _model.config.vtxTable.powerLabels[i][0] = 'L';
              _model.config.vtxTable.powerLabels[i][1] = 'V';
              _model.config.vtxTable.powerLabels[i][2] = (char)('0' + (i % 10));
              _model.config.vtxTable.powerLabels[i][VTX_TABLE_POWER_LABEL_LENGTH] = '\0';
            }
          }
        }
      }
      break;


    case MSP_SET_ARMING_DISABLED:
      {
        const uint8_t cmd = m.readU8();
        uint8_t disableRunawayTakeoff = 0;
        if(m.remain()) {
          disableRunawayTakeoff = m.readU8();
        }
        (void)disableRunawayTakeoff;
#if defined(ESPFC_DEV_PRESET_UNSAFE_ARMING)
        (void)cmd;
#warning "Danger macro used ESPFC_DEV_PRESET_UNSAFE_ARMING"
#else
      _model.setArmingDisabled(ARMING_DISABLED_MSP, cmd);
#endif
        if (_model.isModeActive(MODE_ARMED)) _model.disarm(DISARM_REASON_ARMING_DISABLED);
      }
      break;

    case MSP_SELECT_SETTING:
      if(m.remain() >= 1)
      {
        const uint8_t profileIndex = m.readU8();
        // Support extended format: if additional byte available, treat as [type, index]
        // Otherwise, maintain backwards compatibility (rate profile selection)
        if(m.remain() >= 1)
        {
          const uint8_t profileType = profileIndex;  // First byte is type
          const uint8_t index = m.readU8();
          if(profileType == 0)
          {
            _model.selectRateProfile(index);
          }
          else if(profileType == 1)
          {
            _model.selectPidProfile(index);
          }
        }
        else
        {
          // Backwards compatible: single byte = rate profile index
          _model.selectRateProfile(profileIndex);
        }
      }
      break;

    case MSP_COPY_PROFILE:
      if(m.remain() >= 3)
      {
        const uint8_t profileType = m.readU8(); // 0=rate, 1=pid
        const uint8_t destIdx = m.readU8();
        const uint8_t srcIdx = m.readU8();
        if(profileType == 0 && srcIdx < RATE_PROFILE_COUNT && destIdx < RATE_PROFILE_COUNT)
        {
          _model.config.rateProfiles[destIdx] = _model.config.rateProfiles[srcIdx];
        }
        else if(profileType == 1 && srcIdx < PID_PROFILE_COUNT && destIdx < PID_PROFILE_COUNT)
        {
          _model.config.pidProfiles[destIdx] = _model.config.pidProfiles[srcIdx];
        }
      }
      break;

    case MSP_SET_HEADING:
      if(m.remain() >= 2) m.readU16();
      break;

    case MSP_SET_RAW_RC:
      while(m.remain() >= 2) m.readU16();
      break;

    case MSP_SET_PID_CONTROLLER:
      if(m.remain()) m.readU8();
      break;

    case MSP_SET_RESET_CURR_PID:
      // Keep current tuned values; command acknowledged for compatibility.
      break;

    case MSP_SET_PASSTHROUGH:
      {
        uint8_t ptMode = MSP_PASSTHROUGH_ESC_4WAY;
        uint8_t ptArg = 0;
        if(m.remain() >= 2) {
          ptMode = m.readU8();
          ptArg = m.readU8();
        }
        switch (ptMode)
        {
          case MSP_PASSTHROUGH_ESC_4WAY:
            r.writeU8(esc4wayInit());
            serialDeviceInit(&s, 0);
            _postCommand = std::bind(&MspProcessor::processEsc4way, this);
            break;
          default:
            r.writeU8(0);
            break;
        }
        (void)ptArg;
      }
      break;

    case MSP_SET_OSD_CONFIG:
      {
        if(m.remain() < 1)
        {
          r.result = 0;
          break;
        }

        const int8_t addr = (int8_t)m.readU8();
        if(addr == -1)
        {
          if(m.remain() < 2)
          {
            r.result = 0;
            break;
          }

          _model.config.osd.enabled = 1;
          _model.config.osd.videoSystem = constrain((int)m.readU8(), 0, 3);
          _model.config.osd.units = m.readU8();

          if(m.remain() >= 1) _model.config.osd.rssiAlarm = m.readU8();
          if(m.remain() >= 2) _model.config.osd.capAlarm = m.readU16();
          if(m.remain() >= 2) m.readU16(); // legacy flight timer field
          if(m.remain() >= 2) _model.config.osd.altAlarm = m.readU16();

          if(m.remain() >= 2) _model.config.osd.enabledWarnings = m.readU16();
          if(m.remain() >= 4) _model.config.osd.enabledWarnings = m.readU32();
          if(m.remain() >= 1) _model.config.osd.profile = constrain((int)m.readU8(), 1, (int)_model.config.osd.profileCount);
          if(m.remain() >= 1) _model.config.osd.overlayRadioMode = m.readU8();
          if(m.remain() >= 2)
          {
            _model.config.osd.cameraFrameWidth = m.readU8();
            _model.config.osd.cameraFrameHeight = m.readU8();
          }
          if(m.remain() >= 2) _model.config.osd.linkQualityAlarm = m.readU16();
          if(m.remain() >= 2) _model.config.osd.rssiDbmAlarm = m.readU16();
        }
        else if(addr == -2)
        {
          if(m.remain() < 3)
          {
            r.result = 0;
            break;
          }
          const uint8_t index = m.readU8();
          const uint16_t timerValue = m.readU16();
          if(index >= ESPFC_OSD_TIMER_COUNT)
          {
            r.result = 0;
            break;
          }
          _model.config.osd.timers[index] = timerValue;
        }
        else
        {
          if(m.remain() < 2)
          {
            r.result = 0;
            break;
          }

          const uint16_t value = m.readU16();
          const uint8_t screen = m.remain() >= 1 ? m.readU8() : 1;
          if(screen == 0)
          {
            if((uint8_t)addr >= ESPFC_OSD_STAT_COUNT)
            {
              r.result = 0;
              break;
            }
            _model.config.osd.statEnabled[(uint8_t)addr] = value ? 1 : 0;
          }
          else
          {
            if((uint8_t)addr >= ESPFC_OSD_ITEM_COUNT)
            {
              r.result = 0;
              break;
            }
            _model.config.osd.itemPos[(uint8_t)addr] = value;
          }
        }
      }
      break;

    case MSP_SET_OSD_VIDEO_CONFIG:
      if(m.remain() >= 1)
      {
        _model.config.osd.videoSystem = constrain((int)m.readU8(), 0, 3);
      }
      while(m.remain() > 0) m.readU8();
      break;

    case MSP_SET_ADJUSTMENT_RANGE:
      if(m.remain() >= 7)
      {
        const uint8_t slot = m.readU8();
        if(slot >= ADJUSTMENT_RANGES_COUNT)
        {
          r.result = -1;
          while(m.remain() > 0) m.readU8();
          break;
        }

        auto& cfg = _model.config.adjustmentRanges[slot];
        cfg.stateIndex = m.readU8();
        cfg.auxChannelIndex = m.readU8();
        cfg.rangeStartStep = m.readU8();
        cfg.rangeEndStep = m.readU8();
        cfg.adjustmentFunction = m.readU8();
        cfg.auxSwitchChannelIndex = m.readU8();
        if(m.remain() >= 4)
        {
          cfg.adjustmentCenter = (int16_t)m.readU16();
          cfg.adjustmentScale = (int16_t)m.readU16();
        }
        else
        {
          cfg.adjustmentCenter = 0;
          cfg.adjustmentScale = 0;
        }
      }
      while(m.remain() > 0) m.readU8();
      break;

    case MSP_SET_SERVO_MIX_RULE:
      if(m.remain() >= 8)
      {
#if ESPFC_SERVO_MIX_RULES_STORAGE > 0
        uint8_t ruleIndex = m.readU8(); // rule slot index
        if(ruleIndex < SERVO_MIX_RULES_STORAGE)
        {
          auto& rule = _model.config.servoMixRules[ruleIndex];
          rule.targetChannel = m.readU8();
          rule.inputSource = m.readU8();
          rule.rate = (int8_t)m.readU8();
          rule.speed = m.readU8();
          rule.min = m.readU8();
          rule.max = m.readU8();
          rule.box = m.readU8();
          if(ruleIndex >= _model.config.servoMixRuleCount)
          {
            _model.config.servoMixRuleCount = ruleIndex + 1;
          }
        }
        else
        {
          // Rule index is outside persistent storage on constrained targets; consume payload and ignore.
          m.readU8();
          m.readU8();
          m.readU8();
          m.readU8();
          m.readU8();
          m.readU8();
          m.readU8();
        }
#else
        // No persistent servo mix storage on this target; consume payload for protocol compatibility.
  m.readU8();
        m.readU8();
        m.readU8();
        m.readU8();
        m.readU8();
        m.readU8();
        m.readU8();
#endif
      }
      while(m.remain() > 0) m.readU8();
      break;

    case MSP_SET_BOARD_INFO:
    case MSP_SET_NAV_CONFIG:
    case MSP_SET_RAW_GPS:
      // Manual GPS injection (for testing/simulation)
      if(m.remain() >= 1) _model.state.gps.fix = m.readU8() != 0;
      if(m.remain() >= 1) _model.state.gps.numSats = m.readU8();
      if(m.remain() >= 4) _model.state.gps.location.raw.lat = m.readU32();
      if(m.remain() >= 4) _model.state.gps.location.raw.lon = m.readU32();
      if(m.remain() >= 2) _model.state.gps.location.raw.height = (int32_t)m.readU16() * 1000;
      if(m.remain() >= 2) _model.state.gps.velocity.raw.groundSpeed = m.readU16() * 10;
      if(m.remain() >= 2) _model.state.gps.velocity.raw.heading = m.readU16() * 10000;
      break;

    case MSP_SET_SIGNATURE:
    case MSP_SET_TRANSPONDER_CONFIG:
    case MSP_SET_WP:
      // Unsupported feature groups: consume payload and ACK for compatibility.
      while(m.remain() > 0) m.readU8();
      break;

    case MSP_SET_RTC:
      // RTC is not backed by hardware timekeeping in this firmware, but payload is accepted.
      while(m.remain() > 0) m.readU8();
      break;

    case MSP_SET_LED_COLORS:
      if(m.remain() < (int)(LED_CONFIGURABLE_COLOR_COUNT * 4))
      {
        r.result = 0;
        break;
      }
      for(size_t i = 0; i < LED_CONFIGURABLE_COLOR_COUNT; i++)
      {
        _model.config.ledStrip.colors[i].h = std::min<uint16_t>(m.readU16(), 359);
        _model.config.ledStrip.colors[i].s = m.readU8();
        _model.config.ledStrip.colors[i].v = m.readU8();
      }
      _model.reload();
      break;

    case MSP_SET_LED_STRIP_CONFIG:
      if(m.remain() < 5)
      {
        r.result = 0;
        break;
      }
      {
        uint8_t i = m.readU8();
        if(i >= LED_STRIP_MAX_LENGTH)
        {
          r.result = 0;
          break;
        }
        _model.config.ledStrip.ledConfig[i] = m.readU32();
        if(m.remain() >= 1) _model.config.ledStrip.profile = m.readU8();
      }
      _model.reload();
      break;

    case MSP_SET_LED_STRIP_MODECOLOR:
      if(m.remain() < 3)
      {
        r.result = 0;
        break;
      }
      {
        uint8_t modeIdx = m.readU8();
        uint8_t modeColorIdx = m.readU8();
        uint8_t colorIdx = m.readU8();
        if(modeIdx < LED_MODE_COUNT)
        {
          if(modeColorIdx >= LED_DIRECTION_COUNT)
          {
            r.result = 0;
            break;
          }
          if(colorIdx >= LED_CONFIGURABLE_COLOR_COUNT)
          {
            r.result = 0;
            break;
          }
          _model.config.ledStrip.modeColors[modeIdx][modeColorIdx] = colorIdx;
        }
        else if(modeIdx == LED_SPECIAL_MODE_INDEX)
        {
          if(modeColorIdx >= LED_SPECIAL_COLOR_COUNT)
          {
            r.result = 0;
            break;
          }
          if(colorIdx >= LED_CONFIGURABLE_COLOR_COUNT)
          {
            r.result = 0;
            break;
          }
          _model.config.ledStrip.specialColors[modeColorIdx] = colorIdx;
        }
        else if(modeIdx == LED_AUX_CHANNEL_MODE_INDEX)
        {
          if(modeColorIdx >= 1)
          {
            r.result = 0;
            break;
          }
          _model.config.ledStrip.auxChannel = colorIdx;
        }
        else
          r.result = 0;
      }
      if(r.result) _model.reload();
      break;

    case MSP_SET_VTXTABLE_BAND:
      {
        if(m.remain() < 2)
        {
          r.result = 0;
          break;
        }
        uint8_t band = m.readU8();
        uint8_t bandNameLength = m.readU8();
        char bandName[VTX_TABLE_BAND_NAME_LENGTH] = {' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '};
        for(size_t i = 0; i < bandNameLength && m.remain() > 0; i++)
        {
          char c = (char)m.readU8();
          if(i < VTX_TABLE_BAND_NAME_LENGTH) bandName[i] = toUpperAscii(c);
        }

        if(m.remain() < 3)
        {
          r.result = 0;
          break;
        }

        char bandLetter = toUpperAscii((char)m.readU8());
        uint8_t isFactoryBand = m.readU8();
        uint8_t channelCount = m.readU8();

        if(band < 1 || band > _model.config.vtxTable.bands)
        {
          r.result = 0;
          break;
        }

        for(size_t i = 0; i < VTX_TABLE_MAX_CHANNELS; i++)
        {
          _model.config.vtxTable.frequency[band - 1][i] = 0;
        }

        for(size_t i = 0; i < channelCount && m.remain() >= 2; i++)
        {
          uint16_t freq = m.readU16();
          if(i < _model.config.vtxTable.channels)
          {
            _model.config.vtxTable.frequency[band - 1][i] = freq;
          }
        }

        for(size_t i = 0; i < VTX_TABLE_BAND_NAME_LENGTH; i++)
        {
          _model.config.vtxTable.bandNames[band - 1][i] = bandName[i];
        }
        _model.config.vtxTable.bandNames[band - 1][VTX_TABLE_BAND_NAME_LENGTH] = '\0';
        _model.config.vtxTable.bandLetters[band - 1] = bandLetter;
        _model.config.vtxTable.isFactoryBand[band - 1] = isFactoryBand ? 1 : 0;

        if(_model.config.vtx.band == band)
        {
          _model.config.vtx.freq = vtxLookupFrequency(_model.config, _model.config.vtx.band, _model.config.vtx.channel);
        }
      }
      break;

    case MSP_SET_VTXTABLE_POWERLEVEL:
      {
        if(m.remain() < 4)
        {
          r.result = 0;
          break;
        }

        uint8_t powerLevel = m.readU8();
        uint16_t powerValue = m.readU16();
        uint8_t labelLength = m.readU8();
        char label[VTX_TABLE_POWER_LABEL_LENGTH] = {' ', ' ', ' '};
        for(size_t i = 0; i < labelLength && m.remain() > 0; i++)
        {
          char c = (char)m.readU8();
          if(i < VTX_TABLE_POWER_LABEL_LENGTH) label[i] = toUpperAscii(c);
        }

        if(powerLevel < 1 || powerLevel > _model.config.vtxTable.powerLevels)
        {
          r.result = 0;
          break;
        }

        _model.config.vtxTable.powerValues[powerLevel - 1] = powerValue;
        for(size_t i = 0; i < VTX_TABLE_POWER_LABEL_LENGTH; i++)
        {
          _model.config.vtxTable.powerLabels[powerLevel - 1][i] = label[i];
        }
        _model.config.vtxTable.powerLabels[powerLevel - 1][VTX_TABLE_POWER_LABEL_LENGTH] = '\0';
      }
      break;

    case MSP2_BETAFLIGHT_BIND:
      // Binding is receiver-specific and handled externally. Keep command supported as no-op.
      break;

    case MSP2_MOTOR_OUTPUT_REORDERING:
      {
        const size_t motorCount = std::min<size_t>(_model.state.currentMixer.count, OUTPUT_CHANNELS);
        r.writeU8((uint8_t)motorCount);
        for(size_t i = 0; i < motorCount; i++)
        {
#if !defined(ESP32S2)
          r.writeU8(_model.config.output.motorOutputReordering[i]);
#else
          r.writeU8(i);
#endif
        }
      }
      break;

    case MSP2_SET_MOTOR_OUTPUT_REORDERING:
      if(m.remain() > 0)
      {
#if !defined(ESP32S2)
        const size_t motorCount = std::min<size_t>(_model.state.currentMixer.count, OUTPUT_CHANNELS);
        const size_t count = std::min<size_t>(m.readU8(), motorCount);
        bool used[OUTPUT_CHANNELS] = { false };

        for(size_t i = 0; i < count && m.remain() > 0; i++)
        {
          uint8_t mapped = m.readU8();
          if(mapped >= motorCount || used[mapped])
          {
            r.result = 0;
            continue;
          }
          used[mapped] = true;
          _model.config.output.motorOutputReordering[i] = mapped;
        }

        while(m.remain() > 0)
        {
          m.readU8();
        }

        // Keep remaining motor slots in identity order.
        for(size_t i = count; i < motorCount; i++)
        {
          _model.config.output.motorOutputReordering[i] = i;
        }

        _model.reload();
#else
        m.readU8();
        while(m.remain() > 0) m.readU8();
#endif
      }
      break;

    case MSP_DEBUG:
      for (int i = 0; i < 8; i++) {
        r.writeU16(_model.state.debug[i]);
      }
      break;

    case MSP_SET_GPS_CONFIG:
      m.readU8(); // provider
      m.readU8(); // sbas mode
      m.readU8(); // auto config
      m.readU8(); // auto baud
      if (m.remain() >= 2) {
          // Added in API version 1.43
          _model.config.gps.setHomeOnce = m.readU8(); // gps_set_home_point_once
          m.readU8(); // gps_ublox_use_galileo
      }
      _model.reload();
      break;

    case MSP_SET_GPS_RESCUE:
      if(m.remain() >= 2) _model.config.gpsRescue.maxRescueAngle = m.readU16();
      if(m.remain() >= 2) _model.config.gpsRescue.returnAltitudeM = m.readU16();
      if(m.remain() >= 2) _model.config.gpsRescue.descentDistanceM = m.readU16();
      if(m.remain() >= 2) _model.config.gpsRescue.groundSpeedCmS = m.readU16();
      if(m.remain() >= 2) _model.config.autopilot.throttleMin = m.readU16();
      if(m.remain() >= 2) _model.config.autopilot.throttleMax = m.readU16();
      if(m.remain() >= 2) _model.config.autopilot.hoverThrottle = m.readU16();
      if(m.remain() >= 1) _model.config.gpsRescue.sanityChecks = m.readU8();
      if(m.remain() >= 1) _model.config.gpsRescue.minSats = m.readU8();
      if(m.remain() >= 2) _model.config.gpsRescue.ascendRate = m.readU16();
      if(m.remain() >= 2) _model.config.gpsRescue.descendRate = m.readU16();
      if(m.remain() >= 1) _model.config.gpsRescue.allowArmingWithoutFix = m.readU8();
      if(m.remain() >= 1) _model.config.gpsRescue.altitudeMode = m.readU8();
      if(m.remain() >= 2) _model.config.gpsRescue.minStartDistM = m.readU16();
      if(m.remain() >= 2) _model.config.gpsRescue.initialClimbM = m.readU16();
      _model.reload();
      break;

    case MSP_SET_GPS_RESCUE_PIDS:
      if(m.remain() >= 2) _model.config.autopilot.altitudeP = m.readU16();
      if(m.remain() >= 2) _model.config.autopilot.altitudeI = m.readU16();
      if(m.remain() >= 2) _model.config.autopilot.altitudeD = m.readU16();
      if(m.remain() >= 2) _model.config.gpsRescue.velP = m.readU16();
      if(m.remain() >= 2) _model.config.gpsRescue.velI = m.readU16();
      if(m.remain() >= 2) _model.config.gpsRescue.velD = m.readU16();
      if(m.remain() >= 2) _model.config.gpsRescue.yawP = m.readU16();
      _model.reload();
      break;

    case MSP_SET_TX_INFO:
      if(m.remain()) m.readU8();
      break;

    case MSP_GPS_CONFIG:
      r.writeU8(1); // provider
      r.writeU8(0); // sbasMode, 0: auto
      r.writeU8(1); // autoConfig, 0: off, 1: on
      r.writeU8(1); // autoBaud, 0: off, 1: on
      // Added in API version 1.43
      r.writeU8(_model.config.gps.setHomeOnce); // gps_set_home_point_once
      r.writeU8(1); // gps_ublox_use_galileo
      break;

    case MSP_PROTOCOL_VERSION:
      r.writeU8(MSP_PROTOCOL_VERSION);
      break;

    case MSP_MULTIPLE_MSP:
      {
        if(m.remain() == 0)
        {
          r.result = 0;
          break;
        }

        const uint16_t startRead = m.read;
        const uint16_t reqCount = m.remain();
        int bytesRemaining = r.remain();
        uint8_t maxMsps = 0;

        for(uint16_t i = 0; i < reqCount && bytesRemaining > 0; i++)
        {
          const uint8_t newCmd = m.buffer[startRead + i];
          if(newCmd == MSP_MULTIPLE_MSP)
          {
            r.result = 0;
            break;
          }

          MspMessage req;
          req.cmd = newCmd;
          req.version = m.version;

          MspResponse rsp;
          processCommand(req, rsp, s);

          if(rsp.result != 1)
          {
            r.result = 0;
            break;
          }

          const int mspSize = (int)rsp.len + 1; // include 1-byte sub-payload length
          if(bytesRemaining - mspSize >= 0)
          {
            bytesRemaining -= mspSize;
            maxMsps++;
          }
        }

        if(r.result != 1) break;

        for(uint8_t i = 0; i < maxMsps; i++)
        {
          const uint8_t newCmd = m.buffer[startRead + i];

          MspMessage req;
          req.cmd = newCmd;
          req.version = m.version;

          MspResponse rsp;
          processCommand(req, rsp, s);

          r.writeU8((uint8_t)rsp.len);
          for(size_t j = 0; j < rsp.len; j++)
          {
            r.writeU8(rsp.data[j]);
          }
        }
      }
      break;

    case MSP_ADJUSTMENT_RANGES:
      for(size_t i = 0; i < ADJUSTMENT_RANGES_COUNT; i++)
      {
        const auto& cfg = _model.config.adjustmentRanges[i];
        r.writeU8(cfg.stateIndex);
        r.writeU8(cfg.auxChannelIndex);
        r.writeU8(cfg.rangeStartStep);
        r.writeU8(cfg.rangeEndStep);
        r.writeU8(cfg.adjustmentFunction);
        r.writeU8(cfg.auxSwitchChannelIndex);
        r.writeU16((uint16_t)cfg.adjustmentCenter);
        r.writeU16((uint16_t)cfg.adjustmentScale);
      }
      break;

    case MSP_OSD_CHAR_READ:
      // No font storage/display hardware; return an empty character response instead of an error.
      r.writeU8(0);
      break;

    case MSP_OSD_CONFIG:
      {
        uint8_t osdFlags = 0;
        if(_model.config.osd.enabled)
        {
          osdFlags |= OSD_FLAGS_OSD_FEATURE;
        }

        bool hasFrskyOsdSerial = false;
        for(size_t i = 0; i < SERIAL_UART_COUNT; i++)
        {
          if(_model.config.serial[i].functionMask & SERIAL_FUNCTION_FRSKY_OSD)
          {
            hasFrskyOsdSerial = true;
            break;
          }
        }

        if(hasFrskyOsdSerial)
        {
          osdFlags |= OSD_FLAGS_OSD_HARDWARE_FRSKYOSD | OSD_FLAGS_OSD_DEVICE_DETECTED;
        }
        else if(_model.config.osd.mspDisplayport)
        {
          osdFlags |= OSD_FLAGS_OSD_MSP_DEVICE | OSD_FLAGS_OSD_DEVICE_DETECTED;
        }
        else
        {
          (void)OSD_FLAGS_OSD_HARDWARE_MAX_7456;
        }

        r.writeU8(osdFlags);
        r.writeU8(_model.config.osd.videoSystem);
        r.writeU8(_model.config.osd.units);
        r.writeU8(_model.config.osd.rssiAlarm);
        r.writeU16(_model.config.osd.capAlarm);
        r.writeU8(0);
        r.writeU8(ESPFC_OSD_ITEM_COUNT);
        r.writeU16(_model.config.osd.altAlarm);

        for(size_t i = 0; i < ESPFC_OSD_ITEM_COUNT; i++)
        {
          r.writeU16(_model.config.osd.itemPos[i]);
        }

        r.writeU8(ESPFC_OSD_STAT_COUNT);
        for(size_t i = 0; i < ESPFC_OSD_STAT_COUNT; i++)
        {
          r.writeU8(_model.config.osd.statEnabled[i]);
        }

        r.writeU8(ESPFC_OSD_TIMER_COUNT);
        for(size_t i = 0; i < ESPFC_OSD_TIMER_COUNT; i++)
        {
          r.writeU16(_model.config.osd.timers[i]);
        }

        r.writeU16((uint16_t)(_model.config.osd.enabledWarnings & 0xFFFF));
        r.writeU8(ESPFC_OSD_WARNING_COUNT);
        r.writeU32(_model.config.osd.enabledWarnings);
        r.writeU8(_model.config.osd.profileCount);
        r.writeU8(_model.config.osd.profile);
        r.writeU8(_model.config.osd.overlayRadioMode);
        r.writeU8(_model.config.osd.cameraFrameWidth);
        r.writeU8(_model.config.osd.cameraFrameHeight);
        r.writeU16(_model.config.osd.linkQualityAlarm);
        r.writeU16(_model.config.osd.rssiDbmAlarm);
      }
      break;

    case MSP_OSD_VIDEO_CONFIG:
      r.writeU8(_model.config.osd.videoSystem);
      break;

    case MSP_CAMERA_CONTROL:
    case MSP_GPSSTATISTICS:
      // GPS accuracy statistics
      r.writeU16(_model.state.gps.accuracy.pDop);           // pDOP x 100
      r.writeU16(0);                                          // hdop (use pDOP proxy)
      r.writeU16(0);                                          // vdop (use pDOP proxy)
      r.writeU32(_model.state.gps.accuracy.horizontal);      // horizontal accuracy in mm
      r.writeU32(_model.state.gps.accuracy.vertical);        // vertical accuracy in mm
      r.writeU32(_model.state.gps.accuracy.speed);           // speed accuracy in mm/s
      r.writeU32(_model.state.gps.accuracy.heading);         // heading accuracy in deg x 1e5
      r.writeU32(_model.state.gps.lastMsgTs / 1000);         // lastMessageDt in ms
      r.writeU8(_model.state.gps.numSats);                   // number of satellites used
      r.writeU8(_model.state.gps.fixType);                   // fix type (0=no, 1=dead reck, 2=2D, 3=3D, 4=GNSS+DR, 5=time only)
      break;

    case MSP_TRANSPONDER_CONFIG:
    case MSP_V2_FRAME:
    case MSP_WP:
    case MSP_RESERVE_1:
    case MSP_RESERVE_2:
      // Not implemented in esp-fc: report unsupported instead of returning malformed payloads.
      r.result = 0;
      break;

    case MSP_NAV_STATUS:
      // Navigation status payload: empty/offline state (BF-compatible format)
      r.writeU8(0);       // mode: NAV_MODE_NONE
      r.writeU8(0);       // state: NAV_STATE_NONE
      r.writeU8(0);       // activeWp
      r.writeU16(0);      // flags
      r.writeU16(0);      // error
      r.writeU16(0);      // targetBearing (signed, but 0 is same as ±0)
      r.writeU16(0);      // holdTime
      break;

    case MSP_NAV_CONFIG:
      // Navigation configuration (BF-compatible format with safe defaults)
      for(size_t i = 0; i < 18; i++) r.writeU8(0);   // 6 PID controllers (P,I,D each)
      r.writeU16(300);    // max_speed
      r.writeU16(50);     // max_climb_rate
      r.writeU16(50);     // max_descent_rate
      r.writeU16(100);    // max_manual_speed
      r.writeU8(20);      // manual_baro_mode
      r.writeU16(500);    // land_descent_rate
      r.writeU16(100);    // land_slowdown_minalt
      r.writeU16(500);    // land_slowdown_maxalt
      r.writeU16(50);     // emerg_descent_rate
      r.writeU16(0);      // min_rth_distance
      r.writeU8(0);       // rth_allow_landing
      r.writeU8(30);      // rth_altitude
      r.writeU8(60);      // rth_allow_landing_after_alt_decrement
      break;

    case MSP_DEBUGMSG:
      r.writeU8(0); // empty C-string
      break;

    case MSP_SERVO_MIX_RULES:
      // Return servo mix rules in Betaflight-compatible 7-byte-per-entry format
      for(size_t i = 0; i < SERVO_MIX_RULES_MAX; i++)
      {
#if ESPFC_SERVO_MIX_RULES_STORAGE > 0
        if(i < _model.config.servoMixRuleCount && i < SERVO_MIX_RULES_STORAGE)
        {
          const auto& rule = _model.config.servoMixRules[i];
          r.writeU8(rule.targetChannel);
          r.writeU8(rule.inputSource);
          r.writeU8(rule.rate & 0xFF);       // signed, written as u8
          r.writeU8(rule.speed);
          r.writeU8(rule.min);
          r.writeU8(rule.max);
          r.writeU8(rule.box);
        }
        else
        {
          // Write empty/zeroed rule for unused slots
          r.writeU8(0);
          r.writeU8(0);
          r.writeU8(0);
          r.writeU8(0);
          r.writeU8(0);
          r.writeU8(0);
          r.writeU8(0);
        }
#else
        // No persistent storage on constrained targets; report empty rules.
        r.writeU8(0);
        r.writeU8(0);
        r.writeU8(0);
        r.writeU8(0);
        r.writeU8(0);
        r.writeU8(0);
        r.writeU8(0);
#endif
      }
      break;

    case MSP_RTC:
      // BF payload: year(u16), month, day, hour, minute, second, millis(u16)
      r.writeU16(2000);
      r.writeU8(1);
      r.writeU8(1);
      r.writeU8(0);
      r.writeU8(0);
      r.writeU8(0);
      r.writeU16(0);
      break;

    case MSP_LED_COLORS:
      for(size_t i = 0; i < LED_CONFIGURABLE_COLOR_COUNT; i++)
      {
        r.writeU16(_model.config.ledStrip.colors[i].h);
        r.writeU8(_model.config.ledStrip.colors[i].s);
        r.writeU8(_model.config.ledStrip.colors[i].v);
      }
      break;

    case MSP_LED_STRIP_CONFIG:
      for(size_t i = 0; i < LED_STRIP_MAX_LENGTH; i++)
      {
        r.writeU32(_model.config.ledStrip.ledConfig[i]);
      }
      r.writeU8(1); // advanced ledstrip available
      r.writeU8(_model.config.ledStrip.profile);
      break;

    case MSP_LED_STRIP_MODECOLOR:
      for(size_t i = 0; i < LED_MODE_COUNT; i++)
      {
        for(size_t j = 0; j < LED_DIRECTION_COUNT; j++)
        {
          r.writeU8(i);
          r.writeU8(j);
          r.writeU8(_model.config.ledStrip.modeColors[i][j]);
        }
      }
      for(size_t j = 0; j < LED_SPECIAL_COLOR_COUNT; j++)
      {
        r.writeU8(LED_SPECIAL_MODE_INDEX);
        r.writeU8(j);
        r.writeU8(_model.config.ledStrip.specialColors[j]);
      }
      r.writeU8(LED_AUX_CHANNEL_MODE_INDEX);
      r.writeU8(0);
      r.writeU8(_model.config.ledStrip.auxChannel);
      break;

    case MSP_VTXTABLE_BAND:
      {
        uint8_t band = m.remain() ? m.readU8() : 0;
        if(band < 1 || band > _model.config.vtxTable.bands)
        {
          r.result = 0;
          break;
        }
        r.writeU8(band);
        r.writeU8(VTX_TABLE_BAND_NAME_LENGTH);
        for(size_t i = 0; i < VTX_TABLE_BAND_NAME_LENGTH; i++)
        {
          r.writeU8(_model.config.vtxTable.bandNames[band - 1][i]);
        }
        r.writeU8(_model.config.vtxTable.bandLetters[band - 1]);
        r.writeU8(_model.config.vtxTable.isFactoryBand[band - 1]);
        r.writeU8(_model.config.vtxTable.channels);
        for(size_t i = 0; i < _model.config.vtxTable.channels; i++)
        {
          r.writeU16(_model.config.vtxTable.frequency[band - 1][i]);
        }
      }
      break;

    case MSP_VTXTABLE_POWERLEVEL:
      {
        uint8_t powerLevel = m.remain() ? m.readU8() : 0;
        if(powerLevel < 1 || powerLevel > _model.config.vtxTable.powerLevels)
        {
          r.result = 0;
          break;
        }
        r.writeU8(powerLevel);
        r.writeU16(_model.config.vtxTable.powerValues[powerLevel - 1]);
        r.writeU8(VTX_TABLE_POWER_LABEL_LENGTH);
        for(size_t i = 0; i < VTX_TABLE_POWER_LABEL_LENGTH; i++)
        {
          r.writeU8(_model.config.vtxTable.powerLabels[powerLevel - 1][i]);
        }
      }
      break;

    case MSP_DISPLAYPORT:
      if(m.remain() > 0)
      {
        const uint8_t subcmd = m.readU8();
        switch(subcmd)
        {
          case MSP_DP_HEARTBEAT:
          case MSP_DP_RELEASE:
          case MSP_DP_CLEAR_SCREEN:
          case MSP_DP_WRITE_STRING:
          case MSP_DP_DRAW_SCREEN:
          case MSP_DP_OPTIONS:
          case MSP_DP_SYS:
          case MSP_DP_FONTCHAR_WRITE:
            _model.config.osd.enabled = 1;
            break;
          default:
            break;
        }
      }
      while(m.remain() > 0) m.readU8();
      break;

    case MSP_OSD_CHAR_WRITE:
      _model.config.osd.enabled = 1;
      while(m.remain() > 0) m.readU8();
      break;

    case MSP_GPS_RESCUE:
      r.writeU16(_model.config.gpsRescue.maxRescueAngle);
      r.writeU16(_model.config.gpsRescue.returnAltitudeM);
      r.writeU16(_model.config.gpsRescue.descentDistanceM);
      r.writeU16(_model.config.gpsRescue.groundSpeedCmS);
      r.writeU16(_model.config.autopilot.throttleMin);
      r.writeU16(_model.config.autopilot.throttleMax);
      r.writeU16(_model.config.autopilot.hoverThrottle);
      r.writeU8(_model.config.gpsRescue.sanityChecks);
      r.writeU8(_model.config.gpsRescue.minSats);
      r.writeU16(_model.config.gpsRescue.ascendRate);
      r.writeU16(_model.config.gpsRescue.descendRate);
      r.writeU8(_model.config.gpsRescue.allowArmingWithoutFix);
      r.writeU8(_model.config.gpsRescue.altitudeMode);
      r.writeU16(_model.config.gpsRescue.minStartDistM);
      r.writeU16(_model.config.gpsRescue.initialClimbM);
      break;

    case MSP_GPS_RESCUE_PIDS:
      r.writeU16(_model.config.autopilot.altitudeP);
      r.writeU16(_model.config.autopilot.altitudeI);
      r.writeU16(_model.config.autopilot.altitudeD);
      r.writeU16(_model.config.gpsRescue.velP);
      r.writeU16(_model.config.gpsRescue.velI);
      r.writeU16(_model.config.gpsRescue.velD);
      r.writeU16(_model.config.gpsRescue.yawP);
      break;

    case MSP_TX_INFO:
      r.writeU8(0);    // rssi source
      r.writeU8(0xFF); // RTC not supported
      break;

  case MSP_RAW_GPS:
      r.writeU8(_model.state.gps.fixType > 2); // STATE(GPS_FIX));
      r.writeU8(_model.state.gps.numSats); // numSat
      r.writeU32(_model.state.gps.location.raw.lat); // lat
      r.writeU32(_model.state.gps.location.raw.lon); // lon
      r.writeU16(std::clamp((int)_model.state.gps.location.raw.height / 1000, 0, (int)std::numeric_limits<uint16_t>::max())); // height [m]
      r.writeU16(_model.state.gps.velocity.raw.groundSpeed / 10); // cm/s
      r.writeU16(_model.state.gps.velocity.raw.heading / 10000); // deg * 10
      // Added in API version 1.44
      r.writeU16(_model.state.gps.accuracy.pDop); // pDOP
      break;

  case MSP_COMP_GPS:
    r.writeU16(std::clamp((uint16_t)lrintf(_model.state.gps.distanceToHome), (uint16_t)0, (uint16_t)std::numeric_limits<uint16_t>::max())); // meters
    r.writeU16((int16_t)lrintf(Utils::toDeg(_model.state.gps.directionToHome))); // deg
    r.writeU8(_model.state.gps.homeSet ? 1 : 0);  // GPS update
    break;

  case MSP_GPSSVINFO:
      r.writeU8(_model.state.gps.numCh); // GPS_numCh
      for (size_t i = 0; i < _model.state.gps.numCh; i++) {
        r.writeU8(_model.state.gps.svinfo[i].gnssId); // GPS_svinfo_chn[i]
        r.writeU8(_model.state.gps.svinfo[i].id); // GPS_svinfo_svid[i]
        r.writeU8(static_cast<uint8_t>(_model.state.gps.svinfo[i].quality.value & 0xff)); // GPS_svinfo_quality[i]
        r.writeU8(_model.state.gps.svinfo[i].cno); // GPS_svinfo_cno[i]
      }
      break;

    case MSP_EEPROM_WRITE:
      _model.save();
      break;

    case MSP_RESET_CONF:
      if(!_model.isModeActive(MODE_ARMED))
      {
        _model.reset();
        _model.save();
      }
      break;

    case MSP_REBOOT:
      r.writeU8(0); // reboot to firmware
      _postCommand = std::bind(&MspProcessor::processRestart, this);
      break;

    default:
      r.result = 0;
      break;
  }

  // Some configurator builds do not always send explicit MSP_EEPROM_WRITE.
  // Persist successful non-transient SET commands to avoid lost settings.
  if(r.result == 1 && shouldAutoPersistSetCommand(m.cmd))
  {
    _model.save();
  }
}

void MspProcessor::processEsc4way()
{
#if defined(ESPFC_MULTI_CORE) && defined(ESPFC_FREE_RTOS)
  timer_pause(TIMER_GROUP_0, TIMER_0);
#endif
  esc4wayProcess(getSerialPort());
  processRestart();
}

void MspProcessor::processRestart()
{
  Hardware::restart(_model);
}

void MspProcessor::serializeFlashData(MspResponse& r, uint32_t address, const uint16_t size, bool useLegacyFormat, bool allowCompression)
{
#ifdef USE_FLASHFS
  (void)allowCompression; // not supported

  const uint32_t allowedToRead = r.remain() - 16;
  const uint32_t flashfsSize = flashfsGetSize();

  uint16_t readLen = std::min(std::min((uint32_t)size, allowedToRead), flashfsSize - address);

  r.writeU32(address);

  uint16_t *readLenPtr = (uint16_t*)&r.data[r.len];
  if (!useLegacyFormat)
  {
    // new format supports variable read lengths
    r.writeU16(readLen);
    r.writeU8(0); // NO_COMPRESSION
  }

  const size_t bytesRead = flashfsReadAbs(address, &r.data[r.len], readLen);
  r.advance(bytesRead);

  if (!useLegacyFormat)
  {
    // update the 'read length' with the actual amount read from flash.
    *readLenPtr = bytesRead;
  }
  else
  {
    // pad the buffer with zeros
    //for (int i = bytesRead; i < allowedToRead; i++) r.writeU8(0);
  }
#endif
}

void MspProcessor::sendResponse(MspResponse& r, Device::SerialDevice& s)
{
  debugResponse(r);
  uint8_t buff[256];
  size_t len = r.serialize(buff, sizeof(buff));
  s.write(buff, len);
}

void MspProcessor::postCommand()
{
  if(!_postCommand) return;
  std::function<void(void)> cb = _postCommand;
  _postCommand = {};
  cb();
}

bool MspProcessor::debugSkip(uint8_t cmd)
{
  //return true;
  //return false;
  if(cmd == MSP_STATUS) return true;
  if(cmd == MSP_STATUS_EX) return true;
  if(cmd == MSP_BOXNAMES) return true;
  if(cmd == MSP_ANALOG) return true;
  if(cmd == MSP_ATTITUDE) return true;
  if(cmd == MSP_ALTITUDE) return true;
  if(cmd == MSP_RC) return true;
  if(cmd == MSP_RAW_IMU) return true;
  if(cmd == MSP_MOTOR) return true;
  if(cmd == MSP_SERVO) return true;
  if(cmd == MSP_BATTERY_STATE) return true;
  if(cmd == MSP_VOLTAGE_METERS) return true;
  if(cmd == MSP_CURRENT_METERS) return true;
  return false;
}

void MspProcessor::debugMessage(const MspMessage& m)
{
  if(debugSkip(m.cmd)) return;
  Device::SerialDevice * s = _model.getSerialStream(SERIAL_FUNCTION_TELEMETRY_HOTT);
  if(!s) return;

  s->print(m.dir == MSP_TYPE_REPLY ? '>' : '<');
  s->print(m.cmd); s->print('.');
  s->print(m.expected); s->print(' ');
  for(size_t i = 0; i < m.expected; i++)
  {
    s->print(m.buffer[i], HEX); s->print(' ');
  }
  s->println();
}

void MspProcessor::debugResponse(const MspResponse& r)
{
  if(debugSkip(r.cmd)) return;
  Device::SerialDevice * s = _model.getSerialStream(SERIAL_FUNCTION_TELEMETRY_HOTT);
  if(!s) return;

  s->print(r.result == 1 ? '>' : (r.result == -1 ? '!' : '@'));
  s->print(r.cmd); s->print('.');
  s->print(r.len); s->print(' ');
  for(size_t i = 0; i < r.len; i++)
  {
    s->print(r.data[i], HEX); s->print(' ');
  }
  s->println();
}

}

}
