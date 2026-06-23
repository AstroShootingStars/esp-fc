#include "Connect/Cli.hpp"
#include <platform.h>
#include <algorithm>
#include <cstdlib>
#include "Hardware.h"
#include "Device/GyroDevice.h"
#include "Hal/Pgm.h"
#include "msp/msp_protocol.h"

#ifdef USE_FLASHFS
#include "Device/FlashDevice.h"
#endif

#if defined(ESPFC_WIFI_ALT)
#include <ESP8266WiFi.h>
#elif defined(ESPFC_WIFI)
#include <WiFi.h>
#endif

#ifdef ESPFC_FREE_RTOS
#include <freertos/task.h>
#endif

// Compile-time flag to include verbose CLI help, status, and diagnostic output.
// Disable on flash-constrained targets (esp32s2) to reduce binary size.
#ifndef ESPFC_CLI_VERBOSE_HELP
#define ESPFC_CLI_VERBOSE_HELP 0
#endif

namespace Espfc {

namespace Connect {

static inline int32_t parseIntArg(const char * v)
{
  return v ? static_cast<int32_t>(strtol(v, nullptr, 10)) : 0;
}

static inline float parseFloatArg(const char * v)
{
  return v ? strtof(v, nullptr) : 0.f;
}

void Cli::Param::print(Stream& stream) const
{
  if(!addr)
  {
    stream.print(F("UNSET"));
    return;
  }
  switch(type)
  {
    case PARAM_NONE:
      stream.print(F("NONE"));
      break;
    case PARAM_BOOL:
      stream.print(*addr != 0);
      break;
    case PARAM_BYTE:
      print(stream, *reinterpret_cast<int8_t*>(addr));
      break;
    case PARAM_BYTE_U:
      print(stream, *reinterpret_cast<uint8_t*>(addr));
      break;
    case PARAM_SHORT:
      print(stream, *reinterpret_cast<int16_t*>(addr));
      break;
    case PARAM_INT:
      print(stream, *reinterpret_cast<int32_t*>(addr));
      break;
    case PARAM_FLOAT:
      stream.print(*reinterpret_cast<float*>(addr), 4);
      break;
    case PARAM_STRING:
      stream.print(addr);
      break;
    case PARAM_BITMASK:
      stream.print((*reinterpret_cast<int32_t*>(addr) & (1ul << maxLen)) ? 1 : 0);
      break;
    case PARAM_INPUT_CHANNEL:
      print(stream, *reinterpret_cast<InputChannelConfig*>(addr));
      break;
    case PARAM_OUTPUT_CHANNEL:
      print(stream, *reinterpret_cast<OutputChannelConfig*>(addr));
      break;
    case PARAM_SCALER:
      print(stream, *reinterpret_cast<ScalerConfig*>(addr));
      break;
    case PARAM_MODE:
      print(stream, *reinterpret_cast<ActuatorCondition*>(addr));
      break;
    case PARAM_MIXER:
      print(stream, *reinterpret_cast<MixerEntry*>(addr));
      break;
    case PARAM_SERIAL:
      print(stream, *reinterpret_cast<SerialPortConfig*>(addr));
      break;
  }
}

void Cli::Param::print(Stream& stream, const OutputChannelConfig& och) const
{
  stream.print(och.servo ? 'S' : 'M');
  stream.print(' ');
  stream.print(och.reverse ? 'R' : 'N');
  stream.print(' ');
  stream.print(och.min);
  stream.print(' ');
  stream.print(och.neutral);
  stream.print(' ');
  stream.print(och.max);
}

void Cli::Param::print(Stream& stream, const InputChannelConfig& ich) const
{
  stream.print(ich.map);
  stream.print(' ');
  stream.print(ich.min);
  stream.print(' ');
  stream.print(ich.neutral);
  stream.print(' ');
  stream.print(ich.max);
  stream.print(' ');
  stream.print(ich.fsMode == 0 ? 'A' : (ich.fsMode == 1 ? 'H' : (ich.fsMode == 2 ? 'S' : '?')));
  stream.print(' ');
  stream.print(ich.fsValue);
}

void Cli::Param::print(Stream& stream, const ScalerConfig& sc) const
{
  stream.print(sc.dimension);
  stream.print(' ');
  stream.print(sc.channel);
  stream.print(' ');
  stream.print(sc.minScale);
  stream.print(' ');
  stream.print(sc.maxScale);
}

void Cli::Param::print(Stream& stream, const ActuatorCondition& ac) const
{
  stream.print(ac.id);
  stream.print(' ');
  stream.print(ac.ch);
  stream.print(' ');
  stream.print(ac.min);
  stream.print(' ');
  stream.print(ac.max);
  stream.print(' ');
  stream.print(ac.logicMode);
  stream.print(' ');
  stream.print(ac.linkId);
}

void Cli::Param::print(Stream& stream, const MixerEntry& me) const
{
  stream.print(me.src);
  stream.print(' ');
  stream.print(me.dst);
  stream.print(' ');
  stream.print(me.rate);
}

void Cli::Param::print(Stream& stream, const SerialPortConfig& sc) const
{
  stream.print(sc.functionMask);
  stream.print(' ');
  stream.print(sc.baud);
  stream.print(' ');
  stream.print(sc.blackboxBaud);
}

void Cli::Param::print(Stream& stream, int32_t v) const
{
  if(choices)
  {
    for(int32_t i = 0; choices[i]; i++)
    {
      if(i == v)
      {
        stream.print(FPSTR(choices[i]));
        return;
      }
    }
  }
  stream.print(v);
}

void Cli::Param::update(const char ** args) const
{
  const char * v = args[2];
  if(!addr) return;
  switch(type)
  {
    case PARAM_BOOL:
      if(!v) return;
      if(*v == '0') *addr = 0;
      if(*v == '1') *addr = 1;
      break;
    case PARAM_BYTE:
      if(!v) return;
      write((int8_t)parse(v));
      break;
    case PARAM_BYTE_U:
      if(!v) return;
      write((uint8_t)parse(v));
      break;
    case PARAM_SHORT:
      if(!v) return;
      write((int16_t)parse(v));
      break;
    case PARAM_INT:
      if(!v) return;
      write((int32_t)parse(v));
      break;
    case PARAM_FLOAT:
      if(!v) return;
      write(parseFloatArg(v));
      break;
    case PARAM_STRING:
      write(v ? v : "");
      break;
    case PARAM_BITMASK:
      if(!v) return;
      if(*v == '0')
      {
        *reinterpret_cast<int32_t*>(addr) &= ~(1ul << maxLen);
      }
      if(*v == '1')
      {
        *reinterpret_cast<int32_t*>(addr) |= (1ul << maxLen);
      }
      break;
    case PARAM_OUTPUT_CHANNEL:
      if(!v) return;
      write(*reinterpret_cast<OutputChannelConfig*>(addr), args);
      break;
    case PARAM_INPUT_CHANNEL:
      if(!v) return;
      write(*reinterpret_cast<InputChannelConfig*>(addr), args);
      break;
    case PARAM_SCALER:
      if(!v) return;
      write(*reinterpret_cast<ScalerConfig*>(addr), args);
      break;
    case PARAM_MODE:
      if(!v) return;
      write(*reinterpret_cast<ActuatorCondition*>(addr), args);
      break;
    case PARAM_MIXER:
      if(!v) return;
      write(*reinterpret_cast<MixerEntry*>(addr), args);
      break;
    case PARAM_SERIAL:
      if(!v) return;
      write(*reinterpret_cast<SerialPortConfig*>(addr), args);
      break;
    case PARAM_NONE:
      break;
  }
}

void Cli::Param::write(OutputChannelConfig& och, const char ** args) const
{
  if(args[2]) och.servo = *args[2] == 'S';
  if(args[3]) och.reverse = *args[3] == 'R';
  if(args[4]) och.min = parseIntArg(args[4]);
  if(args[5]) och.neutral = parseIntArg(args[5]);
  if(args[6]) och.max = parseIntArg(args[6]);
}

void Cli::Param::write(InputChannelConfig& ich, const char ** args) const
{
  if(args[2]) ich.map = parseIntArg(args[2]);
  if(args[3]) ich.min = parseIntArg(args[3]);
  if(args[4]) ich.neutral = parseIntArg(args[4]);
  if(args[5]) ich.max = parseIntArg(args[5]);
  if(args[6]) ich.fsMode = *args[6] == 'A' ? 0 : (*args[6] == 'H' ? 1 : (*args[6] == 'S' ? 2 : 0));
  if(args[7]) ich.fsValue = parseIntArg(args[7]);
}

void Cli::Param::write(ScalerConfig& sc, const char ** args) const
{
  if(args[2]) sc.dimension = (ScalerDimension)parseIntArg(args[2]);
  if(args[3]) sc.channel = parseIntArg(args[3]);
  if(args[4]) sc.minScale = parseIntArg(args[4]);
  if(args[5]) sc.maxScale = parseIntArg(args[5]);
}

void Cli::Param::write(ActuatorCondition& ac, const char ** args) const
{
  if(args[2]) ac.id = parseIntArg(args[2]);
  if(args[3]) ac.ch = parseIntArg(args[3]);
  if(args[4]) ac.min = parseIntArg(args[4]);
  if(args[5]) ac.max = parseIntArg(args[5]);
  if(args[6]) ac.logicMode = parseIntArg(args[6]);
  if(args[7]) ac.linkId = parseIntArg(args[7]);
}

void Cli::Param::write(MixerEntry& ac, const char ** args) const
{
  if(args[2]) ac.src = constrain(parseIntArg(args[2]), 0, MIXER_SOURCE_MAX - 1);
  if(args[3]) ac.dst = constrain(parseIntArg(args[3]), 0, (int)(OUTPUT_CHANNELS - 1));
  if(args[4]) ac.rate = constrain(parseIntArg(args[4]), -1000, 1000);
}

void Cli::Param::write(SerialPortConfig& sc, const char ** args) const
{
  if(args[2]) sc.functionMask = parseIntArg(args[2]);
  if(args[3]) sc.baud = parseIntArg(args[3]);
  if(args[4]) sc.blackboxBaud = parseIntArg(args[4]);
}

void Cli::Param::write(const char * v) const
{
  *addr = 0;
  strncat(addr, v ? v : "", maxLen);
}

void Cli::Param::write(const String& v) const
{
  write(v.c_str());
}

int32_t Cli::Param::parse(const char * v) const
{
  if(choices)
  {
    for(size_t i = 0; choices[i]; i++)
    {
      if(strcasecmp_P(v, choices[i]) == 0) return i;
    }
  }
  return parseIntArg(v);
}

Cli::Cli(Model& model): _model(model), _ignore(false), _active(false)
{
  _params = initialize(_model.config);
}

const Cli::Param * Cli::initialize(ModelConfig& c)
{
  const char * const * busDevChoices            = Device::BusDevice::getNames();
  const char * const * gyroDevChoices           = Device::GyroDevice::getNames();
  const char * const * baroDevChoices           = Device::BaroDevice::getNames();
  const char * const * magDevChoices            = Device::MagDevice::getNames();
  const char * const * rangefinderDevChoices    = Device::RangefinderDevice::getNames();
  const char * const * opticalFlowDevChoices    = Device::OpticalFlowDevice::getNames();
  const char * const * oledDevChoices           = Device::OledDevice::getNames();

  const char * const * fusionModeChoices        = FusionConfig::getModeNames();
  static const char * const * protocolChoices = EscDriver::getProtocolNames();

  static const char * const gyroDlpfChoices[]   = { PSTR("256Hz"), PSTR("188Hz"), PSTR("98Hz"), PSTR("42Hz"), PSTR("20Hz"), PSTR("10Hz"), PSTR("5Hz"), PSTR("EXPERIMENTAL"), NULL };
  static const char * const debugModeChoices[]  = {  PSTR("NONE"), PSTR("CYCLETIME"), PSTR("BATTERY"), PSTR("GYRO_FILTERED"), PSTR("ACCELEROMETER"), PSTR("PIDLOOP"), PSTR("GYRO_SCALED"), PSTR("RC_INTERPOLATION"),
                                              PSTR("ANGLERATE"), PSTR("ESC_SENSOR"), PSTR("SCHEDULER"), PSTR("STACK"), PSTR("ESC_SENSOR_RPM"), PSTR("ESC_SENSOR_TMP"), PSTR("ALTITUDE"), PSTR("FFT"),
                                              PSTR("FFT_TIME"), PSTR("FFT_FREQ"), PSTR("RX_FRSKY_SPI"), PSTR("RX_SFHSS_SPI"), PSTR("GYRO_RAW"), PSTR("DUAL_GYRO_RAW"), PSTR("DUAL_GYRO_DIFF"),
                                              PSTR("MAX7456_SIGNAL"), PSTR("MAX7456_SPICLOCK"), PSTR("SBUS"), PSTR("FPORT"), PSTR("RANGEFINDER"), PSTR("RANGEFINDER_QUALITY"), PSTR("LIDAR_TF"),
                                              PSTR("ADC_INTERNAL"), PSTR("RUNAWAY_TAKEOFF"), PSTR("SDIO"), PSTR("CURRENT_SENSOR"), PSTR("USB"), PSTR("SMARTAUDIO"), PSTR("RTH"), PSTR("ITERM_RELAX"),
                                              PSTR("ACRO_TRAINER"), PSTR("RC_SMOOTHING"), PSTR("RX_SIGNAL_LOSS"), PSTR("RC_SMOOTHING_RATE"), PSTR("ANTI_GRAVITY"), PSTR("DYN_LPF"), PSTR("RX_SPEKTRUM_SPI"),
                                              PSTR("DSHOT_RPM_TELEMETRY"), PSTR("RPM_FILTER"), PSTR("D_MIN"), PSTR("AC_CORRECTION"), PSTR("AC_ERROR"), PSTR("DUAL_GYRO_SCALED"), PSTR("DSHOT_RPM_ERRORS"),
                                              PSTR("CRSF_LINK_STATISTICS_UPLINK"), PSTR("CRSF_LINK_STATISTICS_PWR"), PSTR("CRSF_LINK_STATISTICS_DOWN"), PSTR("BARO"), PSTR("GPS_RESCUE_THROTTLE_PID"),
                                              PSTR("DYN_IDLE"), PSTR("FF_LIMIT"), PSTR("FF_INTERPOLATED"), PSTR("BLACKBOX_OUTPUT"), PSTR("GYRO_SAMPLE"), PSTR("RX_TIMING"), NULL };
  static const char * const filterTypeChoices[] = { PSTR("PT1"), PSTR("BIQUAD"), PSTR("PT2"), PSTR("PT3"), PSTR("NOTCH"), PSTR("NOTCH_DF1"), PSTR("BPF"), PSTR("FO"), PSTR("FIR2"), PSTR("MEDIAN3"), PSTR("NONE"), NULL };
  static const char * const alignChoices[]      = { PSTR("DEFAULT"), PSTR("CW0"), PSTR("CW90"), PSTR("CW180"), PSTR("CW270"), PSTR("CW0_FLIP"), PSTR("CW90_FLIP"), PSTR("CW180_FLIP"), PSTR("CW270_FLIP"), PSTR("CUSTOM"), NULL };
  static const char * const mixerTypeChoices[]  = { PSTR("NONE"), PSTR("TRI"), PSTR("QUADP"), PSTR("QUADX"), PSTR("BI"),
                                              PSTR("GIMBAL"), PSTR("Y6"), PSTR("HEX6"), PSTR("FWING"), PSTR("Y4"),
                                              PSTR("HEX6X"), PSTR("OCTOX8"), PSTR("OCTOFLATP"), PSTR("OCTOFLATX"), PSTR("AIRPLANE"),
                                              PSTR("HELI120"), PSTR("HELI90"), PSTR("VTAIL4"), PSTR("HEX6H"), PSTR("PPMSERVO"),
                                              PSTR("DUALCOPTER"), PSTR("SINGLECOPTER"), PSTR("ATAIL4"), PSTR("CUSTOM"), PSTR("CUSTOMAIRPLANE"),
                                              PSTR("CUSTOMTRI"), PSTR("QUADX1234"), NULL };
  static const char * const interpolChoices[]   = { PSTR("NONE"), PSTR("DEFAULT"), PSTR("AUTO"), PSTR("MANUAL"), NULL };
  static const char * const inputRateTypeChoices[] = { PSTR("BETAFLIGHT"), PSTR("RACEFLIGHT"), PSTR("KISS"), PSTR("ACTUAL"), PSTR("QUICK"), NULL };
  static const char * const throtleLimitTypeChoices[] = { PSTR("NONE"), PSTR("SCALE"), PSTR("CLIP"), NULL };
  static const char * const inputFilterChoices[] = { PSTR("INTERPOLATION"), PSTR("FILTER"), NULL };
  static const char * const inputItermRelaxChoices[] = { PSTR("OFF"), PSTR("RP"), PSTR("RPY"), PSTR("RP_INC"), PSTR("RPY_INC"), NULL };

  static const char * const voltageSourceChoices[] = { PSTR("NONE"), PSTR("ADC"), NULL };
  static const char * const currentSourceChoices[] = { PSTR("NONE"), PSTR("ADC"), NULL };
  static const char * const blackboxDevChoices[] = { PSTR("NONE"), PSTR("FLASH"), PSTR("SD_CARD"), PSTR("SERIAL"), NULL };
  static const char * const blackboxModeChoices[] = { PSTR("NORMAL"), PSTR("TEST"), PSTR("ALWAYS"), NULL };
  static const char * const ledTypeChoices[] = { PSTR("SIMPLE"), PSTR("STRIP"), NULL };

  size_t i = 0;
  static Param * params = nullptr;
  if(params) return params;

  const Param paramsTmp[] = {

    Param(PSTR("feature_gps"), &c.featureMask, 7),
    Param(PSTR("feature_dyn_notch"), &c.featureMask, 29),
    Param(PSTR("feature_motor_stop"), &c.featureMask, 4),
    Param(PSTR("feature_rx_ppm"), &c.featureMask, 0),
    Param(PSTR("feature_rx_serial"), &c.featureMask, 3),
    Param(PSTR("feature_rx_spi"), &c.featureMask, 25),
    Param(PSTR("feature_soft_serial"), &c.featureMask, 6),
    Param(PSTR("feature_telemetry"), &c.featureMask, 10),


    Param(PSTR("debug_mode"), &c.debug.mode, debugModeChoices),
    Param(PSTR("debug_axis"), &c.debug.axis),

    Param(PSTR("gyro_bus"), &c.gyro.bus, busDevChoices),
    Param(PSTR("gyro_dev"), &c.gyro.dev, gyroDevChoices),
    Param(PSTR("gyro_dlpf"), &c.gyro.dlpf, gyroDlpfChoices),
    Param(PSTR("gyro_align"), &c.gyro.align, alignChoices),
    Param(PSTR("gyro_lpf_type"), &c.gyro.filter.type, filterTypeChoices),
    Param(PSTR("gyro_lpf_freq"), &c.gyro.filter.freq),
    Param(PSTR("gyro_lpf2_type"), &c.gyro.filter2.type, filterTypeChoices),
    Param(PSTR("gyro_lpf2_freq"), &c.gyro.filter2.freq),
    Param(PSTR("gyro_lpf3_type"), &c.gyro.filter3.type, filterTypeChoices),
    Param(PSTR("gyro_lpf3_freq"), &c.gyro.filter3.freq),
    Param(PSTR("gyro_notch1_freq"), &c.gyro.notch1Filter.freq),
    Param(PSTR("gyro_notch1_cutoff"), &c.gyro.notch1Filter.cutoff),
    Param(PSTR("gyro_notch2_freq"), &c.gyro.notch2Filter.freq),
    Param(PSTR("gyro_notch2_cutoff"), &c.gyro.notch2Filter.cutoff),
    Param(PSTR("gyro_dyn_lpf_min"), &c.gyro.dynLpfFilter.cutoff),
    Param(PSTR("gyro_dyn_lpf_max"), &c.gyro.dynLpfFilter.freq),
    Param(PSTR("gyro_dyn_notch_q"), &c.gyro.dynamicFilter.q),
    Param(PSTR("gyro_dyn_notch_count"), &c.gyro.dynamicFilter.count),
    Param(PSTR("gyro_dyn_notch_min"), &c.gyro.dynamicFilter.min_freq),
    Param(PSTR("gyro_dyn_notch_max"), &c.gyro.dynamicFilter.max_freq),
    Param(PSTR("gyro_rpm_harmonics"), &c.gyro.rpmFilter.harmonics),
    Param(PSTR("gyro_rpm_q"), &c.gyro.rpmFilter.q),
    Param(PSTR("gyro_rpm_min_freq"), &c.gyro.rpmFilter.minFreq),
    Param(PSTR("gyro_rpm_fade"), &c.gyro.rpmFilter.fade),
    Param(PSTR("gyro_rpm_weight_1"), &c.gyro.rpmFilter.weights[0]),
    Param(PSTR("gyro_rpm_weight_2"), &c.gyro.rpmFilter.weights[1]),
    Param(PSTR("gyro_rpm_weight_3"), &c.gyro.rpmFilter.weights[2]),
    Param(PSTR("gyro_rpm_tlm_lpf_freq"), &c.gyro.rpmFilter.freqLpf),
    Param(PSTR("gyro_offset_x"), &c.gyro.bias[0]),
    Param(PSTR("gyro_offset_y"), &c.gyro.bias[1]),
    Param(PSTR("gyro_offset_z"), &c.gyro.bias[2]),

    Param(PSTR("accel_bus"), &c.accel.bus, busDevChoices),
    Param(PSTR("accel_dev"), &c.accel.dev, gyroDevChoices),
    Param(PSTR("accel_lpf_type"), &c.accel.filter.type, filterTypeChoices),
    Param(PSTR("accel_lpf_freq"), &c.accel.filter.freq),
    Param(PSTR("accel_offset_x"), &c.accel.bias[0]),
    Param(PSTR("accel_offset_y"), &c.accel.bias[1]),
    Param(PSTR("accel_offset_z"), &c.accel.bias[2]),
    Param(PSTR("accel_trim_roll"), &c.accel.trim[1]),
    Param(PSTR("accel_trim_pitch"), &c.accel.trim[0]),

    Param(PSTR("mag_bus"), &c.mag.bus, busDevChoices),
    Param(PSTR("mag_dev"), &c.mag.dev, magDevChoices),
    Param(PSTR("mag_align"), &c.mag.align, alignChoices),
    Param(PSTR("mag_filter_type"), &c.mag.filter.type, filterTypeChoices),
    Param(PSTR("mag_filter_lpf"), &c.mag.filter.freq),
    Param(PSTR("mag_offset_x"), &c.mag.offset[0]),
    Param(PSTR("mag_offset_y"), &c.mag.offset[1]),
    Param(PSTR("mag_offset_z"), &c.mag.offset[2]),
    Param(PSTR("mag_scale_x"), &c.mag.scale[0]),
    Param(PSTR("mag_scale_y"), &c.mag.scale[1]),
    Param(PSTR("mag_scale_z"), &c.mag.scale[2]),

    Param(PSTR("baro_bus"), &c.baro.bus, busDevChoices),
    Param(PSTR("baro_dev"), &c.baro.dev, baroDevChoices),
    Param(PSTR("baro_lpf_type"), &c.baro.filter.type, filterTypeChoices),
    Param(PSTR("baro_lpf_freq"), &c.baro.filter.freq),

    Param(PSTR("range_bottom_bus"), &c.rangefinder[RANGEFINDER_BOTTOM].bus, busDevChoices),
    Param(PSTR("range_bottom_dev"), &c.rangefinder[RANGEFINDER_BOTTOM].dev, rangefinderDevChoices),
    Param(PSTR("range_bottom_addr"), &c.rangefinder[RANGEFINDER_BOTTOM].address),
    Param(PSTR("range_bottom_enabled"), &c.rangefinder[RANGEFINDER_BOTTOM].enabled),

    Param(PSTR("range_front_bus"), &c.rangefinder[RANGEFINDER_FRONT].bus, busDevChoices),
    Param(PSTR("range_front_dev"), &c.rangefinder[RANGEFINDER_FRONT].dev, rangefinderDevChoices),
    Param(PSTR("range_front_addr"), &c.rangefinder[RANGEFINDER_FRONT].address),
    Param(PSTR("range_front_enabled"), &c.rangefinder[RANGEFINDER_FRONT].enabled),

    Param(PSTR("opflow_bus"), &c.opticalFlow.bus, busDevChoices),
    Param(PSTR("opflow_dev"), &c.opticalFlow.dev, opticalFlowDevChoices),
    Param(PSTR("opflow_quality_min"), &c.opticalFlow.qualityThreshold),

    Param(PSTR("oled_bus"), &c.oled.bus, busDevChoices),
    Param(PSTR("oled_dev"), &c.oled.dev, oledDevChoices),
    Param(PSTR("oled_height"), &c.oled.height),
    Param(PSTR("oled_page_ms"), &c.oled.pageInterval),
    Param(PSTR("oled_startup_ms"), &c.oled.startupMs),

    Param(PSTR("osd_enabled"), &c.osd.enabled),
    Param(PSTR("osd_msp_displayport"), &c.osd.mspDisplayport),
    Param(PSTR("osd_video_system"), &c.osd.videoSystem),
    Param(PSTR("osd_profiles"), &c.osd.profileCount),
    Param(PSTR("osd_profile"), &c.osd.profile),
    Param(PSTR("osd_units"), &c.osd.units),
    Param(PSTR("osd_rssi_alarm"), &c.osd.rssiAlarm),

    Param(PSTR("gps_min_sats"), &c.gps.minSats),
    Param(PSTR("gps_set_home_once"), &c.gps.setHomeOnce),
    
    Param(PSTR("gps_gnss_mode"), &c.gps.gnssMode),
    Param(PSTR("gps_enable_dual_band"), &c.gps.enableDualBand),
    Param(PSTR("gps_enable_gps"), &c.gps.enableGPS),
    Param(PSTR("gps_enable_glonass"), &c.gps.enableGLONASS),
    Param(PSTR("gps_enable_galileo"), &c.gps.enableGalileo),
    Param(PSTR("gps_enable_beidou"), &c.gps.enableBeiDou),
    Param(PSTR("gps_enable_qzss"), &c.gps.enableQZSS),
    Param(PSTR("gps_enable_sbas"), &c.gps.enableSBAS),
    
    Param(PSTR("board_align_roll"), &c.boardAlignment[0]),
    Param(PSTR("board_align_pitch"), &c.boardAlignment[1]),
    Param(PSTR("board_align_yaw"), &c.boardAlignment[2]),

    Param(PSTR("vbat_source"), &c.vbat.source, voltageSourceChoices),
    Param(PSTR("vbat_scale"), &c.vbat.scale),
    Param(PSTR("vbat_mul"), &c.vbat.resMult),
    Param(PSTR("vbat_div"), &c.vbat.resDiv),
    Param(PSTR("vbat_cell_warn"), &c.vbat.cellWarning),

    Param(PSTR("ibat_source"), &c.ibat.source, currentSourceChoices),
    Param(PSTR("ibat_scale"), &c.ibat.scale),
    Param(PSTR("ibat_offset"), &c.ibat.offset),

    Param(PSTR("fusion_mode"), &c.fusion.mode, fusionModeChoices),
    Param(PSTR("fusion_gain_p"), &c.fusion.gain),
    Param(PSTR("fusion_gain_i"), &c.fusion.gainI),

    Param(PSTR("input_rate_type"), &c.input.rateType, inputRateTypeChoices),

    Param(PSTR("input_roll_rate"), &c.input.rate[0]),
    Param(PSTR("input_roll_srate"), &c.input.superRate[0]),
    Param(PSTR("input_roll_expo"), &c.input.expo[0]),
    Param(PSTR("input_roll_limit"), &c.input.rateLimit[0]),

    Param(PSTR("input_pitch_rate"), &c.input.rate[1]),
    Param(PSTR("input_pitch_srate"), &c.input.superRate[1]),
    Param(PSTR("input_pitch_expo"), &c.input.expo[1]),
    Param(PSTR("input_pitch_limit"), &c.input.rateLimit[1]),

    Param(PSTR("input_yaw_rate"), &c.input.rate[2]),
    Param(PSTR("input_yaw_srate"), &c.input.superRate[2]),
    Param(PSTR("input_yaw_expo"), &c.input.expo[2]),
    Param(PSTR("input_yaw_limit"), &c.input.rateLimit[2]),

    Param(PSTR("input_deadband"), &c.input.deadband),

    Param(PSTR("input_min"), &c.input.minRc),
    Param(PSTR("input_mid"), &c.input.midRc),
    Param(PSTR("input_max"), &c.input.maxRc),

    Param(PSTR("input_interpolation"), &c.input.interpolationMode, interpolChoices),
    Param(PSTR("input_interpolation_interval"), &c.input.interpolationInterval),

    Param(PSTR("input_filter_type"), &c.input.filterType, inputFilterChoices),
    Param(PSTR("input_lpf_type"), &c.input.filter.type, filterTypeChoices),
    Param(PSTR("input_lpf_freq"), &c.input.filter.freq),
    Param(PSTR("input_lpf_factor"), &c.input.filterAutoFactor),
    Param(PSTR("input_ff_lpf_type"), &c.input.filterDerivative.type, filterTypeChoices),
    Param(PSTR("input_ff_lpf_freq"), &c.input.filterDerivative.freq),

    Param(PSTR("input_rssi_channel"), &c.input.rssiChannel),

    Param(PSTR("input_0"), &c.input.channel[0]),
    Param(PSTR("input_1"), &c.input.channel[1]),
    Param(PSTR("input_2"), &c.input.channel[2]),
    Param(PSTR("input_3"), &c.input.channel[3]),
    Param(PSTR("input_4"), &c.input.channel[4]),
    Param(PSTR("input_5"), &c.input.channel[5]),
    Param(PSTR("input_6"), &c.input.channel[6]),
    Param(PSTR("input_7"), &c.input.channel[7]),
    Param(PSTR("input_8"), &c.input.channel[8]),
    Param(PSTR("input_9"), &c.input.channel[9]),
    Param(PSTR("input_10"), &c.input.channel[10]),
    Param(PSTR("input_11"), &c.input.channel[11]),
    Param(PSTR("input_12"), &c.input.channel[12]),
    Param(PSTR("input_13"), &c.input.channel[13]),
    Param(PSTR("input_14"), &c.input.channel[14]),
    Param(PSTR("input_15"), &c.input.channel[15]),

    Param(PSTR("failsafe_delay"), &c.failsafe.delay),
    Param(PSTR("failsafe_off_delay"), &c.failsafe.offDelay),
    Param(PSTR("failsafe_throttle"), &c.failsafe.throttle),
    Param(PSTR("failsafe_kill_switch"), &c.failsafe.killSwitch),
    Param(PSTR("failsafe_throttle_low_delay"), &c.failsafe.throttleLowDelay),
    Param(PSTR("failsafe_procedure"), &c.failsafe.procedure),

    Param(PSTR("landing_assist_enabled"), &c.landingAssist.enabled),
    Param(PSTR("landing_assist_thr_margin"), &c.landingAssist.throttleIntentMargin),
    Param(PSTR("landing_assist_desc_rate"), &c.landingAssist.descentRateLimitCms),
    Param(PSTR("landing_assist_desc_gain"), &c.landingAssist.descentCorrectivePermille),
    Param(PSTR("landing_assist_desc_max"), &c.landingAssist.descentCorrectiveMaxPermille),
    Param(PSTR("landing_assist_baro_h"), &c.landingAssist.baroHeightThresholdCm),
    Param(PSTR("landing_assist_baro_v"), &c.landingAssist.baroVarioThresholdCms),
    Param(PSTR("landing_assist_gps_down"), &c.landingAssist.gpsDownThresholdMms),
    Param(PSTR("landing_assist_gps_ground"), &c.landingAssist.gpsGroundThresholdMms),
    Param(PSTR("landing_assist_flow_q"), &c.landingAssist.flowQualityThreshold),
    Param(PSTR("landing_assist_flow_hand_q"), &c.landingAssist.flowHandQualityThreshold),
    Param(PSTR("landing_assist_flow_rate"), &c.landingAssist.flowRateThresholdMrad),
    Param(PSTR("landing_assist_flow_hand_rate"), &c.landingAssist.flowRateHandThresholdMrad),
    Param(PSTR("landing_assist_hand_vario"), &c.landingAssist.handVarioThresholdCms),
    Param(PSTR("landing_assist_hand_h_min"), &c.landingAssist.handHeightMinCm),
    Param(PSTR("landing_assist_hand_h_max"), &c.landingAssist.handHeightMaxCm),
    Param(PSTR("landing_assist_hold_ms"), &c.landingAssist.touchdownHoldMs),
    Param(PSTR("landing_assist_ramp"), &c.landingAssist.touchdownRampPermille),

    Param(PSTR("alt_fuse_baro_h_w"), &c.altitudeFusion.baroHeightWeight),
    Param(PSTR("alt_fuse_baro_v_w"), &c.altitudeFusion.baroVarioWeight),
    Param(PSTR("alt_fuse_gps_h_w"), &c.altitudeFusion.gpsHeightWeight),
    Param(PSTR("alt_fuse_gps_v_w"), &c.altitudeFusion.gpsVarioWeight),
    Param(PSTR("alt_fuse_range_h_w"), &c.altitudeFusion.rangeHeightWeight),
    Param(PSTR("alt_fuse_flow_v_w"), &c.altitudeFusion.flowVarioWeight),
    Param(PSTR("alt_fuse_flow_still"), &c.altitudeFusion.flowStillRate),
    Param(PSTR("alt_fuse_gps_hyst"), &c.altitudeFusion.gpsLossHysteresis),
    Param(PSTR("alt_fuse_flow_hyst"), &c.altitudeFusion.flowLossHysteresis),

  #if ESPFC_EXTENDED_CONFIG_STORAGE > 0
    Param(PSTR("arming_auto_disarm_delay"), &c.arming.autoDisarmDelay),
    Param(PSTR("arming_disarm_kill_switch"), &c.arming.disarmKillSwitch),
  #endif
    Param(PSTR("arming_small_angle"), &c.arming.smallAngle),
    Param(PSTR("arming_gyro_cal_on_first_arm"), &c.arming.gyroCalOnFirstArm),

    Param(PSTR("vtx_power"), &c.vtx.power),
    Param(PSTR("vtx_channel"), &c.vtx.channel),
    Param(PSTR("vtx_band"), &c.vtx.band),
    Param(PSTR("vtx_low_power_disarm"), &c.vtx.lowPowerDisarm),

#ifdef ESPFC_SERIAL_0
    Param(PSTR("serial_0"), &c.serial[SERIAL_UART_0]),
#endif
#ifdef ESPFC_SERIAL_1
    Param(PSTR("serial_1"), &c.serial[SERIAL_UART_1]),
#endif
#ifdef ESPFC_SERIAL_2
    Param(PSTR("serial_2"), &c.serial[SERIAL_UART_2]),
#endif
#ifdef ESPFC_SERIAL_SOFT_0
    Param(PSTR("serial_soft_0"), &c.serial[SERIAL_SOFT_0]),
#endif
#ifdef ESPFC_SERIAL_USB
    Param(PSTR("serial_usb"), &c.serial[SERIAL_USB]),
#endif

    Param(PSTR("scaler_0"), &c.scaler[0]),
    Param(PSTR("scaler_1"), &c.scaler[1]),
  #if SCALER_COUNT > 2
    Param(PSTR("scaler_2"), &c.scaler[2]),
  #endif

    Param(PSTR("mode_0"), &c.conditions[0]),
    Param(PSTR("mode_1"), &c.conditions[1]),
    Param(PSTR("mode_2"), &c.conditions[2]),
    Param(PSTR("mode_3"), &c.conditions[3]),
    Param(PSTR("mode_4"), &c.conditions[4]),
    Param(PSTR("mode_5"), &c.conditions[5]),
    Param(PSTR("mode_6"), &c.conditions[6]),
    Param(PSTR("mode_7"), &c.conditions[7]),
    Param(PSTR("mode_8"), &c.conditions[8]),
    Param(PSTR("mode_9"), &c.conditions[9]),
    Param(PSTR("mode_10"), &c.conditions[10]),
    Param(PSTR("mode_11"), &c.conditions[11]),
    Param(PSTR("mode_12"), &c.conditions[12]),
    Param(PSTR("mode_13"), &c.conditions[13]),
    Param(PSTR("mode_14"), &c.conditions[14]),
    Param(PSTR("mode_15"), &c.conditions[15]),

    Param(PSTR("pid_sync"), &c.loopSync),

    Param(PSTR("pid_roll_p"), &c.pid[FC_PID_ROLL].P),
    Param(PSTR("pid_roll_i"), &c.pid[FC_PID_ROLL].I),
    Param(PSTR("pid_roll_d"), &c.pid[FC_PID_ROLL].D),
    Param(PSTR("pid_roll_f"), &c.pid[FC_PID_ROLL].F),

    Param(PSTR("pid_pitch_p"), &c.pid[FC_PID_PITCH].P),
    Param(PSTR("pid_pitch_i"), &c.pid[FC_PID_PITCH].I),
    Param(PSTR("pid_pitch_d"), &c.pid[FC_PID_PITCH].D),
    Param(PSTR("pid_pitch_f"), &c.pid[FC_PID_PITCH].F),

    Param(PSTR("pid_yaw_p"), &c.pid[FC_PID_YAW].P),
    Param(PSTR("pid_yaw_i"), &c.pid[FC_PID_YAW].I),
    Param(PSTR("pid_yaw_d"), &c.pid[FC_PID_YAW].D),
    Param(PSTR("pid_yaw_f"), &c.pid[FC_PID_YAW].F),

    Param(PSTR("pid_level_p"), &c.pid[FC_PID_LEVEL].P),
    Param(PSTR("pid_level_i"), &c.pid[FC_PID_LEVEL].I),
    Param(PSTR("pid_level_d"), &c.pid[FC_PID_LEVEL].D),
    Param(PSTR("pid_level_f"), &c.pid[FC_PID_LEVEL].F),

    Param(PSTR("pid_level_angle_limit"), &c.level.angleLimit),
    Param(PSTR("pid_level_horizon_strength"), &c.level.horizonStrength),
    Param(PSTR("pid_level_integrated_yaw"), &c.level.integratedYaw),
    Param(PSTR("pid_level_antigravity"), &c.level.antiGravityGain),
    Param(PSTR("pid_level_rate_limit"), &c.level.rateLimit),
    Param(PSTR("pid_level_lpf_type"), &c.level.ptermFilter.type, filterTypeChoices),
    Param(PSTR("pid_level_lpf_freq"), &c.level.ptermFilter.freq),

    Param(PSTR("pid_althold_vel_p"), &c.pid[FC_PID_VEL].P),
    Param(PSTR("pid_althold_vel_i"), &c.pid[FC_PID_VEL].I),
    Param(PSTR("pid_althold_vel_d"), &c.pid[FC_PID_VEL].D),
    Param(PSTR("pid_althold_vel_f"), &c.pid[FC_PID_VEL].F),
    Param(PSTR("pid_althold_iterm_center"), &c.altHold.itermCenter),
    Param(PSTR("pid_althold_iterm_range"), &c.altHold.itermRange),

    Param(PSTR("pid_yaw_lpf_type"), &c.yaw.filter.type, filterTypeChoices),
    Param(PSTR("pid_yaw_lpf_freq"), &c.yaw.filter.freq),

    Param(PSTR("pid_dterm_lpf_type"), &c.dterm.filter.type, filterTypeChoices),
    Param(PSTR("pid_dterm_lpf_freq"), &c.dterm.filter.freq),
    Param(PSTR("pid_dterm_lpf2_type"), &c.dterm.filter2.type, filterTypeChoices),
    Param(PSTR("pid_dterm_lpf2_freq"), &c.dterm.filter2.freq),
    Param(PSTR("pid_dterm_notch_freq"), &c.dterm.notchFilter.freq),
    Param(PSTR("pid_dterm_notch_cutoff"), &c.dterm.notchFilter.cutoff),
    Param(PSTR("pid_dterm_dyn_lpf_min"), &c.dterm.dynLpfFilter.cutoff),
    Param(PSTR("pid_dterm_dyn_lpf_max"), &c.dterm.dynLpfFilter.freq),
    Param(PSTR("pid_feedforward_transition"), &c.dterm.feedForwardTransition),
    Param(PSTR("pid_dmin_roll"), &c.dterm.dMinRoll),
    Param(PSTR("pid_dmin_pitch"), &c.dterm.dMinPitch),
    Param(PSTR("pid_dmin_yaw"), &c.dterm.dMinYaw),
    Param(PSTR("pid_vbat_compensation"), &c.dterm.vbatPidCompensation),

    Param(PSTR("pid_dterm_weight"), &c.dterm.setpointWeight),
    Param(PSTR("pid_iterm_limit"), &c.iterm.limit),
    Param(PSTR("pid_iterm_zero"), &c.iterm.lowThrottleZeroIterm),
    Param(PSTR("pid_iterm_relax"), &c.iterm.relax, inputItermRelaxChoices),
    Param(PSTR("pid_iterm_relax_cutoff"), &c.iterm.relaxCutoff),
    Param(PSTR("pid_iterm_rotation"), &c.iterm.itermRotation),
    Param(PSTR("pid_tpa_scale"), &c.controller.tpaScale),
    Param(PSTR("pid_tpa_breakpoint"), &c.controller.tpaBreakpoint),

    Param(PSTR("mixer_sync"), &c.mixerSync),
    Param(PSTR("mixer_type"), &c.mixer.type, mixerTypeChoices),
    Param(PSTR("mixer_yaw_reverse"), &c.mixer.yawReverse),
    Param(PSTR("mixer_throttle_limit_type"), &c.output.throttleLimitType, throtleLimitTypeChoices),
    Param(PSTR("mixer_throttle_limit_percent"), &c.output.throttleLimitPercent),
    Param(PSTR("mixer_output_limit"), &c.output.motorLimit),

    Param(PSTR("output_motor_protocol"), &c.output.protocol, protocolChoices),
    Param(PSTR("output_motor_async"), &c.output.async),
    Param(PSTR("output_motor_rate"), &c.output.rate),
    Param(PSTR("output_motor_poles"), &c.output.motorPoles),
    Param(PSTR("output_servo_rate"), &c.output.servoRate),

    Param(PSTR("output_min_command"), &c.output.minCommand),
    Param(PSTR("output_min_throttle"), &c.output.minThrottle),
    Param(PSTR("output_max_throttle"), &c.output.maxThrottle),
    Param(PSTR("output_dshot_idle"), &c.output.dshotIdle),
  #if ESPFC_EXTENDED_CONFIG_STORAGE > 0
    Param(PSTR("output_3d_low"), &c.output.deadband3dLow),
    Param(PSTR("output_3d_high"), &c.output.deadband3dHigh),
    Param(PSTR("output_3d_neutral"), &c.output.neutral3d),
  #endif
#ifdef ESPFC_DSHOT_TELEMETRY
    Param(PSTR("output_dshot_telemetry"), &c.output.dshotTelemetry),
#endif
    Param(PSTR("output_0"), &c.output.channel[0]),
    Param(PSTR("output_1"), &c.output.channel[1]),
    Param(PSTR("output_2"), &c.output.channel[2]),
    Param(PSTR("output_3"), &c.output.channel[3]),
#if ESPFC_OUTPUT_COUNT > 4
    Param(PSTR("output_4"), &c.output.channel[4]),
#endif
#if ESPFC_OUTPUT_COUNT > 5
    Param(PSTR("output_5"), &c.output.channel[5]),
#endif
#if ESPFC_OUTPUT_COUNT > 6
    Param(PSTR("output_6"), &c.output.channel[6]),
#endif
#if ESPFC_OUTPUT_COUNT > 7
    Param(PSTR("output_7"), &c.output.channel[7]),
#endif
#ifdef ESPFC_INPUT
    Param(PSTR("pin_input_rx"), &c.pin[PIN_INPUT_RX]),
#endif
    Param(PSTR("pin_output_0"), &c.pin[PIN_OUTPUT_0]),
    Param(PSTR("pin_output_1"), &c.pin[PIN_OUTPUT_1]),
    Param(PSTR("pin_output_2"), &c.pin[PIN_OUTPUT_2]),
    Param(PSTR("pin_output_3"), &c.pin[PIN_OUTPUT_3]),
#if ESPFC_OUTPUT_COUNT > 4
    Param(PSTR("pin_output_4"), &c.pin[PIN_OUTPUT_4]),
#endif
#if ESPFC_OUTPUT_COUNT > 5
    Param(PSTR("pin_output_5"), &c.pin[PIN_OUTPUT_5]),
#endif
#if ESPFC_OUTPUT_COUNT > 6
    Param(PSTR("pin_output_6"), &c.pin[PIN_OUTPUT_6]),
#endif
#if ESPFC_OUTPUT_COUNT > 7
    Param(PSTR("pin_output_7"), &c.pin[PIN_OUTPUT_7]),
#endif
    Param(PSTR("pin_button"), &c.pin[PIN_BUTTON]),
    Param(PSTR("pin_buzzer"), &c.pin[PIN_BUZZER]),
    Param(PSTR("pin_led"), &c.pin[PIN_LED_BLINK]),
#if defined(ESPFC_SERIAL_0) && defined(ESPFC_SERIAL_REMAP_PINS)
    Param(PSTR("pin_serial_0_tx"), &c.pin[PIN_SERIAL_0_TX]),
    Param(PSTR("pin_serial_0_rx"), &c.pin[PIN_SERIAL_0_RX]),
#endif
#if defined(ESPFC_SERIAL_1) && defined(ESPFC_SERIAL_REMAP_PINS)
    Param(PSTR("pin_serial_1_tx"), &c.pin[PIN_SERIAL_1_TX]),
    Param(PSTR("pin_serial_1_rx"), &c.pin[PIN_SERIAL_1_RX]),
#endif
#if defined(ESPFC_SERIAL_2) && defined(ESPFC_SERIAL_REMAP_PINS)
    Param(PSTR("pin_serial_2_tx"), &c.pin[PIN_SERIAL_2_TX]),
    Param(PSTR("pin_serial_2_rx"), &c.pin[PIN_SERIAL_2_RX]),
#endif
#ifdef ESPFC_I2C_0
    Param(PSTR("pin_i2c_scl"), &c.pin[PIN_I2C_0_SCL]),
    Param(PSTR("pin_i2c_sda"), &c.pin[PIN_I2C_0_SDA]),
#endif
#ifdef ESPFC_ADC_0
    Param(PSTR("pin_input_adc_0"), &c.pin[PIN_INPUT_ADC_0]),
#endif
#ifdef ESPFC_ADC_1
    Param(PSTR("pin_input_adc_1"), &c.pin[PIN_INPUT_ADC_1]),
#endif
#ifdef ESPFC_SPI_0
    Param(PSTR("pin_spi_0_sck"), &c.pin[PIN_SPI_0_SCK]),
    Param(PSTR("pin_spi_0_mosi"), &c.pin[PIN_SPI_0_MOSI]),
    Param(PSTR("pin_spi_0_miso"), &c.pin[PIN_SPI_0_MISO]),
    Param(PSTR("pin_spi_cs_0"), &c.pin[PIN_SPI_CS0]),
    Param(PSTR("pin_spi_cs_1"), &c.pin[PIN_SPI_CS1]),
    Param(PSTR("pin_spi_cs_2"), &c.pin[PIN_SPI_CS2]),
#endif
    Param(PSTR("pin_buzzer_invert"), &c.buzzer.inverted),
    Param(PSTR("pin_led_invert"), &c.led.invert),
    Param(PSTR("pin_led_type"), &c.led.type, ledTypeChoices),

#ifdef ESPFC_I2C_0
    Param(PSTR("i2c_speed"), &c.i2cSpeed),
#endif
    Param(PSTR("rescue_config_delay"), &c.rescueConfigDelay),

    //Param(PSTR("telemetry"), &c.telemetry),
    Param(PSTR("telemetry_interval"), &c.telemetryInterval),

    Param(PSTR("blackbox_dev"), &c.blackbox.dev, blackboxDevChoices),
    Param(PSTR("blackbox_mode"), &c.blackbox.mode, blackboxModeChoices),
    Param(PSTR("blackbox_rate"), &c.blackbox.pDenom),
    Param(PSTR("blackbox_log_acc"), &c.blackbox.fieldsMask, BLACKBOX_FIELD_ACC),
    Param(PSTR("blackbox_log_alt"), &c.blackbox.fieldsMask, BLACKBOX_FIELD_ALTITUDE),
    Param(PSTR("blackbox_log_bat"), &c.blackbox.fieldsMask, BLACKBOX_FIELD_BATTERY),
    Param(PSTR("blackbox_log_debug"), &c.blackbox.fieldsMask, BLACKBOX_FIELD_DEBUG_LOG),
    Param(PSTR("blackbox_log_gps"), &c.blackbox.fieldsMask, BLACKBOX_FIELD_GPS),
    Param(PSTR("blackbox_log_gyro"), &c.blackbox.fieldsMask, BLACKBOX_FIELD_GYRO),
    Param(PSTR("blackbox_log_gyro_raw"), &c.blackbox.fieldsMask, BLACKBOX_FIELD_GYROUNFILT),
    Param(PSTR("blackbox_log_mag"), &c.blackbox.fieldsMask, BLACKBOX_FIELD_MAG),
    Param(PSTR("blackbox_log_motor"), &c.blackbox.fieldsMask, BLACKBOX_FIELD_MOTOR),
    Param(PSTR("blackbox_log_pid"), &c.blackbox.fieldsMask, BLACKBOX_FIELD_PID),
    Param(PSTR("blackbox_log_rc"), &c.blackbox.fieldsMask, BLACKBOX_FIELD_RC_COMMANDS),
    Param(PSTR("blackbox_log_rpm"), &c.blackbox.fieldsMask, BLACKBOX_FIELD_RPM),
    Param(PSTR("blackbox_log_rssi"), &c.blackbox.fieldsMask, BLACKBOX_FIELD_RSSI),
    Param(PSTR("blackbox_log_sp"), &c.blackbox.fieldsMask, BLACKBOX_FIELD_SETPOINT),

    Param(PSTR("model_name"), PARAM_STRING, &c.modelName[0], NULL, MODEL_NAME_LEN),

#ifdef ESPFC_SERIAL_SOFT_0_WIFI
    Param(PSTR("wifi_ssid"), PARAM_STRING, &c.wireless.ssid[0], NULL, WirelessConfig::MAX_LEN),
    Param(PSTR("wifi_pass"), PARAM_STRING, &c.wireless.pass[0], NULL, WirelessConfig::MAX_LEN),
    Param(PSTR("wifi_tcp_port"), &c.wireless.port),
  Param(PSTR("wifi_ota"), &c.wireless.otaEnabled),
  Param(PSTR("wifi_ota_port"), &c.wireless.otaPort),
  Param(PSTR("wifi_ota_pass"), PARAM_STRING, &c.wireless.otaPass[0], NULL, WirelessConfig::MAX_LEN),
  Param(PSTR("bt_ota"), &c.wireless.btOtaEnabled),
#endif

    Param(PSTR("mix_outputs"), &c.customMixerCount),
#if ESPFC_SERVO_MIX_RULES_STORAGE > 0
  Param(PSTR("servo_mix_rules"), &c.servoMixRuleCount),
#endif
    Param(PSTR("mix_0"), &c.customMixes[i++]),
    Param(PSTR("mix_1"), &c.customMixes[i++]),
    Param(PSTR("mix_2"), &c.customMixes[i++]),
    Param(PSTR("mix_3"), &c.customMixes[i++]),
    Param(PSTR("mix_4"), &c.customMixes[i++]),
    Param(PSTR("mix_5"), &c.customMixes[i++]),
    Param(PSTR("mix_6"), &c.customMixes[i++]),
    Param(PSTR("mix_7"), &c.customMixes[i++]),
    Param(PSTR("mix_8"), &c.customMixes[i++]),
    Param(PSTR("mix_9"), &c.customMixes[i++]),
    Param(PSTR("mix_10"), &c.customMixes[i++]),
    Param(PSTR("mix_11"), &c.customMixes[i++]),
    Param(PSTR("mix_12"), &c.customMixes[i++]),
    Param(PSTR("mix_13"), &c.customMixes[i++]),
    Param(PSTR("mix_14"), &c.customMixes[i++]),
    Param(PSTR("mix_15"), &c.customMixes[i++]),
    Param(PSTR("mix_16"), &c.customMixes[i++]),
    Param(PSTR("mix_17"), &c.customMixes[i++]),
    Param(PSTR("mix_18"), &c.customMixes[i++]),
    Param(PSTR("mix_19"), &c.customMixes[i++]),
    Param(PSTR("mix_20"), &c.customMixes[i++]),
    Param(PSTR("mix_21"), &c.customMixes[i++]),
    Param(PSTR("mix_22"), &c.customMixes[i++]),
    Param(PSTR("mix_23"), &c.customMixes[i++]),
    Param(PSTR("mix_24"), &c.customMixes[i++]),
    Param(PSTR("mix_25"), &c.customMixes[i++]),
    Param(PSTR("mix_26"), &c.customMixes[i++]),
    Param(PSTR("mix_27"), &c.customMixes[i++]),
    Param(PSTR("mix_28"), &c.customMixes[i++]),
    Param(PSTR("mix_29"), &c.customMixes[i++]),
    Param(PSTR("mix_30"), &c.customMixes[i++]),
    Param(PSTR("mix_31"), &c.customMixes[i++]),
    Param(PSTR("mix_32"), &c.customMixes[i++]),
    Param(PSTR("mix_33"), &c.customMixes[i++]),
    Param(PSTR("mix_34"), &c.customMixes[i++]),
    Param(PSTR("mix_35"), &c.customMixes[i++]),
    Param(PSTR("mix_36"), &c.customMixes[i++]),
    Param(PSTR("mix_37"), &c.customMixes[i++]),
    Param(PSTR("mix_38"), &c.customMixes[i++]),
    Param(PSTR("mix_39"), &c.customMixes[i++]),
    Param(PSTR("mix_40"), &c.customMixes[i++]),
    Param(PSTR("mix_41"), &c.customMixes[i++]),
    Param(PSTR("mix_42"), &c.customMixes[i++]),
    Param(PSTR("mix_43"), &c.customMixes[i++]),
    Param(PSTR("mix_44"), &c.customMixes[i++]),
    Param(PSTR("mix_45"), &c.customMixes[i++]),
    Param(PSTR("mix_46"), &c.customMixes[i++]),
    Param(PSTR("mix_47"), &c.customMixes[i++]),
    Param(PSTR("mix_48"), &c.customMixes[i++]),
    Param(PSTR("mix_49"), &c.customMixes[i++]),
    Param(PSTR("mix_50"), &c.customMixes[i++]),
    Param(PSTR("mix_51"), &c.customMixes[i++]),
    Param(PSTR("mix_52"), &c.customMixes[i++]),
    Param(PSTR("mix_53"), &c.customMixes[i++]),
    Param(PSTR("mix_54"), &c.customMixes[i++]),
    Param(PSTR("mix_55"), &c.customMixes[i++]),
    Param(PSTR("mix_56"), &c.customMixes[i++]),
    Param(PSTR("mix_57"), &c.customMixes[i++]),
    Param(PSTR("mix_58"), &c.customMixes[i++]),
    Param(PSTR("mix_59"), &c.customMixes[i++]),
    Param(PSTR("mix_60"), &c.customMixes[i++]),
    Param(PSTR("mix_61"), &c.customMixes[i++]),
    Param(PSTR("mix_62"), &c.customMixes[i++]),
    Param(PSTR("mix_63"), &c.customMixes[i++]),

    Param() // terminate
  };

  const size_t paramsCount = sizeof(paramsTmp) / sizeof(Param);
  params = new Param[paramsCount];
  for(size_t p = 0; p < paramsCount; ++p) params[p] = paramsTmp[p];
  return params;
}
  bool Cli::process(const char c, CliCmd& cmd, Stream& stream)
{
  // configurator handshake
  if(!_active && c == '#')
  {
    //FIXME: detect disconnection
    _active = true;
    stream.println();
    stream.println(F("Entering CLI Mode, type 'exit' to return, or 'help'"));
    stream.print(F("# "));
    printVersion(stream);
    stream.println();
    _model.setArmingDisabled(ARMING_DISABLED_CLI, true);
    cmd = CliCmd();
    return true;
  }
  if(_active && c == 4) // CTRL-D
  {
    stream.println();
    stream.println(F(" #leaving CLI mode, unsaved changes lost"));
    _active = false;
    cmd = CliCmd();
    return true;
  }

  bool endl = c == '\n' || c == '\r';
  if(cmd.index && endl)
  {
    parse(cmd);
    execute(cmd, stream);
    cmd = CliCmd();
    return true;
  }

  if(c == '#') _ignore = true;
  else if(endl) _ignore = false;

  // don't put characters into buffer in specific conditions
  if(_ignore || endl || cmd.index >= CLI_BUFF_SIZE - 1) return false;

  if(c == '\b') // handle backspace
  {
    cmd.buff[--cmd.index] = '\0';
  }
  else
  {
    cmd.buff[cmd.index] = c;
    cmd.buff[++cmd.index] = '\0';
  }
  return false;
}

void Cli::parse(CliCmd& cmd)
{
  const char * DELIM = " \t";
  char * pch = strtok(cmd.buff, DELIM);
  size_t count = 0;
  while(pch)
  {
    cmd.args[count++] = pch;
    pch = strtok(NULL, DELIM);
  }
}

void Cli::execute(CliCmd& cmd, Stream& s)
{
  if(cmd.args[0]) s.print(F("# "));
  for(size_t i = 0; i < CLI_ARGS_SIZE; ++i)
  {
    if(!cmd.args[i]) break;
    s.print(cmd.args[i]);
    s.print(' ');
  }
  s.println();

  if(!cmd.args[0]) return;

  if(strcmp_P(cmd.args[0], PSTR("help")) == 0)
  {
#if ESPFC_CLI_VERBOSE_HELP
    static const char * const helps[] = {
      PSTR("available commands:"),
      PSTR(" help"), PSTR(" dump"), PSTR(" get param"), PSTR(" set param value ..."), PSTR(" cal [gyro]"),
      PSTR(" range_bottom [bus dev addr enabled]"), PSTR(" range_front [bus dev addr enabled]"),
      PSTR(" defaults"), PSTR(" save"), PSTR(" reboot"), PSTR(" scaler"), PSTR(" mixer"),
      PSTR(" stats"), PSTR(" status"), PSTR(" devinfo"), PSTR(" version"), PSTR(" logs"), PSTR(" gps [set_home|clear_home]"),
      //PSTR(" load"), PSTR(" eeprom"),
      //PSTR(" fsinfo"), PSTR(" fsformat"), PSTR(" log"),
      nullptr
    };
    for(const char * const * ptr = helps; *ptr; ptr++)
    {
      s.println(FPSTR(*ptr));
    }
#endif
  }
  else if(strcmp_P(cmd.args[0], PSTR("version")) == 0)
  {
    printVersion(s);
    s.println();
  }
#if defined(ESPFC_WIFI) || defined(ESPFC_WIFI_ALT)
  else if(strcmp_P(cmd.args[0], PSTR("wifi")) == 0)
  {
#if ESPFC_CLI_VERBOSE_HELP
    s.print(F("ST IP4: tcp://"));
    s.print(WiFi.localIP());
    s.print(F(":"));
    s.println(_model.config.wireless.port);
    s.print(F("ST MAC: "));
    s.println(WiFi.macAddress());
    s.print(F("AP IP4: tcp://"));
    s.print(WiFi.softAPIP());
    s.print(F(":"));
    s.println(_model.config.wireless.port);
    s.print(F("AP MAC: "));
    s.println(WiFi.softAPmacAddress());
    s.print(F("STATUS: "));
    s.println(WiFi.status());
    s.print(F("  MODE: "));
    s.println(WiFi.getMode());
    s.print(F("CHANNEL: "));
    s.println(WiFi.channel());
    //WiFi.printDiag(s);
#endif
  }
#endif
#if defined(ESPFC_FREE_RTOS)
  else if(strcmp_P(cmd.args[0], PSTR("tasks")) == 0)
  {
#if ESPFC_CLI_VERBOSE_HELP
    printVersion(s);
    s.println();

    size_t numTasks = uxTaskGetNumberOfTasks();

    s.print(F("num tasks: "));
    s.print(numTasks);
    s.println();
#endif
  }
#endif
  else if(strcmp_P(cmd.args[0], PSTR("devinfo")) == 0)
  {
#if ESPFC_CLI_VERBOSE_HELP
    printVersion(s);
    s.println();

    s.print(F("cpu freq: "));
    s.print(targetCpuFreq());
    s.println(F(" MHz"));

    s.print(F("  memory: "));
    s.print(sizeof(ModelConfig));
    s.print(F(", "));
    s.print(sizeof(ModelState));
    s.print(F(", "));
    s.println(targetFreeHeap());
#endif
  }
  else if(strcmp_P(cmd.args[0], PSTR("get")) == 0)
  {
    bool found = false;
    for(size_t i = 0; _params[i].name; ++i)
    {
      String ts = FPSTR(_params[i].name);
      if(!cmd.args[1] || ts.indexOf(cmd.args[1]) >= 0)
      {
        print(_params[i], s);
        found = true;
      }
    }
    if(!found)
    {
      s.print(F("param not found: "));
      s.print(cmd.args[1]);
    }
    s.println();
  }
  else if(strcmp_P(cmd.args[0], PSTR("set")) == 0)
  {
    if(!cmd.args[1])
    {
      s.println(F("param required"));
      s.println();
      return;
    }
    bool found = false;
    for(size_t i = 0; _params[i].name; ++i)
    {
      if(strcmp_P(cmd.args[1], _params[i].name) == 0)
      {
        _params[i].update(cmd.args);
        print(_params[i], s);
        found = true;
        break;
      }
    }
    if(!found)
    {
      s.print(F("param not found: "));
      s.println(cmd.args[1]);
    }
  }
  else if(strcmp_P(cmd.args[0], PSTR("range_bottom")) == 0 || strcmp_P(cmd.args[0], PSTR("range_front")) == 0)
  {
    const bool isFront = strcmp_P(cmd.args[0], PSTR("range_front")) == 0;
    const uint8_t idx = isFront ? RANGEFINDER_FRONT : RANGEFINDER_BOTTOM;
    auto& cfg = _model.config.rangefinder[idx];

    auto parseChoice = [](const char* value, const char * const * choices) -> int {
      if(!value || !choices) return -1;
      for(int i = 0; choices[i]; i++)
      {
        if(strcasecmp_P(value, choices[i]) == 0) return i;
      }
      return -1;
    };

    const char ** busChoices = Device::BusDevice::getNames();
    const char ** devChoices = Device::RangefinderDevice::getNames();

    if(!cmd.args[1])
    {
#if ESPFC_CLI_VERBOSE_HELP
      s.print(F("# "));
      s.print(isFront ? F("range_front") : F("range_bottom"));
      s.print(F(" "));
      s.print(cfg.bus);
      s.print(F(" "));
      s.print(cfg.dev);
      s.print(F(" "));
      s.print(cfg.address);
      s.print(F(" "));
      s.println(cfg.enabled ? 1 : 0);
      s.println(F("# usage: <cmd> [bus dev addr enabled]"));
      s.println(F("# bus/dev accepts index or enum name"));
#endif
      return;
    }

    bool ok = true;

    if(cmd.args[1])
    {
      int bus = parseChoice(cmd.args[1], busChoices);
      if(bus < 0) bus = parseIntArg(cmd.args[1]);
      if(bus >= BUS_NONE && bus < BUS_MAX) cfg.bus = bus;
      else ok = false;
    }

    if(cmd.args[2])
    {
      int dev = parseChoice(cmd.args[2], devChoices);
      if(dev < 0) dev = parseIntArg(cmd.args[2]);
      if(dev >= Device::RANGEFINDER_DEFAULT && dev < Device::RANGEFINDER_MAX) cfg.dev = dev;
      else ok = false;
    }

    if(cmd.args[3])
    {
      int addr = parseIntArg(cmd.args[3]);
      if(addr >= 0 && addr < 128) cfg.address = addr;
      else ok = false;
    }

    if(cmd.args[4])
    {
      int en = parseIntArg(cmd.args[4]);
      if(en == 0 || en == 1) cfg.enabled = en;
      else ok = false;
    }

    cfg.position = idx;
    _model.reload();

    if(!ok)
    {
#if ESPFC_CLI_VERBOSE_HELP
      s.println(F("NOT OK: invalid argument"));
      s.println(F("# usage: range_bottom|range_front [bus dev addr enabled]"));
#endif
    }

    s.print(F("OK "));
    s.print(isFront ? F("range_front") : F("range_bottom"));
    s.print(F(" "));
    s.print(cfg.bus);
    s.print(F(" "));
    s.print(cfg.dev);
    s.print(F(" "));
    s.print(cfg.address);
    s.print(F(" "));
    s.println(cfg.enabled ? 1 : 0);
    s.println(F("# tip: save"));
  }
  else if(strcmp_P(cmd.args[0], PSTR("dump")) == 0)
  {
    s.println(F("defaults"));
    for(size_t i = 0; _params[i].name; ++i)
    {
      print(_params[i], s);
    }
    s.println(F("save"));
  }
  else if(strcmp_P(cmd.args[0], PSTR("cal")) == 0)
  {
    if(!cmd.args[1])
    {
      s.print(F(" gyro offset: "));
      s.print(_model.config.gyro.bias[0]); s.print(' ');
      s.print(_model.config.gyro.bias[1]); s.print(' ');
      s.print(_model.config.gyro.bias[2]); s.print(F(" ["));
      s.print(Utils::toDeg(_model.state.gyro.bias[0])); s.print(' ');
      s.print(Utils::toDeg(_model.state.gyro.bias[1])); s.print(' ');
      s.print(Utils::toDeg(_model.state.gyro.bias[2])); s.println(F("]"));

      s.print(F("accel offset: "));
      s.print(_model.config.accel.bias[0]); s.print(' ');
      s.print(_model.config.accel.bias[1]); s.print(' ');
      s.print(_model.config.accel.bias[2]); s.print(F(" ["));
      s.print(_model.state.accel.bias[0]); s.print(' ');
      s.print(_model.state.accel.bias[1]); s.print(' ');
      s.print(_model.state.accel.bias[2]); s.println(F("]"));

      s.print(F("  mag offset: "));
      s.print(_model.config.mag.offset[0]); s.print(' ');
      s.print(_model.config.mag.offset[1]); s.print(' ');
      s.print(_model.config.mag.offset[2]); s.print(F(" ["));
      s.print(_model.state.mag.calibrationOffset[0]); s.print(' ');
      s.print(_model.state.mag.calibrationOffset[1]); s.print(' ');
      s.print(_model.state.mag.calibrationOffset[2]); s.println(F("]"));

      s.print(F("   mag scale: "));
      s.print(_model.config.mag.scale[0]); s.print(' ');
      s.print(_model.config.mag.scale[1]); s.print(' ');
      s.print(_model.config.mag.scale[2]); s.print(F(" ["));
      s.print(_model.state.mag.calibrationScale[0]); s.print(' ');
      s.print(_model.state.mag.calibrationScale[1]); s.print(' ');
      s.print(_model.state.mag.calibrationScale[2]); s.println(F("]"));
    }
    else if(strcmp_P(cmd.args[1], PSTR("gyro")) == 0)
    {
      if(!_model.isModeActive(MODE_ARMED)) _model.calibrateGyro();
      s.println(F("OK"));
    }
    else if(strcmp_P(cmd.args[1], PSTR("mag")) == 0)
    {
      if(!_model.isModeActive(MODE_ARMED)) _model.calibrateMag();
      s.println(F("OK"));
    }
    else if(strcmp_P(cmd.args[1], PSTR("reset_accel")) == 0 || strcmp_P(cmd.args[1], PSTR("reset_all")) == 0)
    {
      _model.state.accel.bias = VectorFloat();
      s.println(F("OK"));
    }
    else if(strcmp_P(cmd.args[1], PSTR("reset_gyro")) == 0 || strcmp_P(cmd.args[1], PSTR("reset_all")) == 0)
    {
      _model.state.gyro.bias = VectorFloat();
      s.println(F("OK"));
    }
    else if(strcmp_P(cmd.args[1], PSTR("reset_mag")) == 0 || strcmp_P(cmd.args[1], PSTR("reset_all")) == 0)
    {
      _model.state.mag.calibrationOffset = VectorFloat();
      _model.state.mag.calibrationScale = VectorFloat(1.f, 1.f, 1.f);
      s.println(F("OK"));
    }
  }
  else if(strcmp_P(cmd.args[0], PSTR("gps")) == 0)
  {
    if(cmd.args[1] && strcmp_P(cmd.args[1], PSTR("set_home")) == 0)
    {
      _model.setGpsHome(true);
      s.println(_model.state.gps.homeSet ? F("Home position set") : F("No GPS fix"));
    }
    else if(cmd.args[1] && strcmp_P(cmd.args[1], PSTR("clear_home")) == 0)
    {
      _model.state.gps.homeSet = false;
      s.println(F("Home position cleared"));
    }
    else
    {
      printGpsStatus(s, true);
    }
  }
  else if(strcmp_P(cmd.args[0], PSTR("preset")) == 0)
  {
    if(!cmd.args[1])
    {
      s.println(F("Available presets: scaler, modes, micrus, brobot,"));
      s.println(F("  landing_indoor, landing_outdoor, landing_freestyle,"));
      s.println(F("  alt_fusion_indoor, alt_fusion_outdoor"));
    }
    else if(strcmp_P(cmd.args[1], PSTR("scaler")) == 0)
    {
      _model.config.scaler[0].dimension = (ScalerDimension)(ACT_INNER_P | ACT_AXIS_PITCH | ACT_AXIS_ROLL);
      _model.config.scaler[0].channel = 5;
      _model.config.scaler[0].minScale = 25; //%
      _model.config.scaler[0].maxScale = 400;

      _model.config.scaler[1].dimension = (ScalerDimension)(ACT_INNER_I | ACT_AXIS_PITCH | ACT_AXIS_ROLL);
      _model.config.scaler[1].channel = 6;
      _model.config.scaler[1].minScale = 25; //%
      _model.config.scaler[1].maxScale = 400;

#if SCALER_COUNT > 2
      _model.config.scaler[2].dimension = (ScalerDimension)(ACT_INNER_D | ACT_AXIS_PITCH | ACT_AXIS_ROLL);
      _model.config.scaler[2].channel = 7;
      _model.config.scaler[2].minScale = 25; //%
      _model.config.scaler[2].maxScale = 400;
#endif

      s.println(F("OK"));
    }
    else if(strcmp_P(cmd.args[1], PSTR("modes")) == 0)
    {
      _model.config.conditions[0].id = MODE_ARMED;
      _model.config.conditions[0].ch = AXIS_AUX_1 + 0;
      _model.config.conditions[0].min = 1700;
      _model.config.conditions[0].max = 2100;

      _model.config.conditions[1].id = MODE_ANGLE;
      _model.config.conditions[1].ch = AXIS_AUX_1 + 0; // aux1
      _model.config.conditions[1].min = 1900;
      _model.config.conditions[1].max = 2100;

      _model.config.conditions[2].id = MODE_AIRMODE;
      _model.config.conditions[2].ch = 0; // aux1
      _model.config.conditions[2].min = (1700 - 900) / 25;
      _model.config.conditions[2].max = (2100 - 900) / 25;

      s.println(F("OK"));
    }
    else if(strcmp_P(cmd.args[1], PSTR("micrus")) == 0)
    {
      s.println(F("OK"));
    }
    else if(strcmp_P(cmd.args[1], PSTR("brobot")) == 0)
    {
      s.println(F("OK"));
    }
    else if(strcmp_P(cmd.args[1], PSTR("landing_indoor")) == 0)
    {
      // Indoor / hand-catch: prioritize optical flow confidence and gentler touchdown confirmation.
      _model.config.landingAssist.enabled = 1;
      _model.config.landingAssist.throttleIntentMargin = 45;
      _model.config.landingAssist.descentRateLimitCms = -70;
      _model.config.landingAssist.descentCorrectivePermille = 320;
      _model.config.landingAssist.descentCorrectiveMaxPermille = 420;
      _model.config.landingAssist.baroHeightThresholdCm = 35;
      _model.config.landingAssist.baroVarioThresholdCms = 25;
      _model.config.landingAssist.gpsDownThresholdMms = 300;
      _model.config.landingAssist.gpsGroundThresholdMms = 700;
      _model.config.landingAssist.flowQualityThreshold = 25;
      _model.config.landingAssist.flowHandQualityThreshold = 40;
      _model.config.landingAssist.flowRateThresholdMrad = 220;
      _model.config.landingAssist.flowRateHandThresholdMrad = 160;
      _model.config.landingAssist.handVarioThresholdCms = 20;
      _model.config.landingAssist.handHeightMinCm = 12;
      _model.config.landingAssist.handHeightMaxCm = 160;
      _model.config.landingAssist.touchdownHoldMs = 550;
      _model.config.landingAssist.touchdownRampPermille = 15;

      _model.config.altitudeFusion.baroHeightWeight = 60;
      _model.config.altitudeFusion.baroVarioWeight = 60;
      _model.config.altitudeFusion.gpsHeightWeight = 20;
      _model.config.altitudeFusion.gpsVarioWeight = 20;
      _model.config.altitudeFusion.rangeHeightWeight = 100;
      _model.config.altitudeFusion.flowVarioWeight = 35;
      _model.config.altitudeFusion.flowStillRate = 180;
      _model.config.altitudeFusion.gpsLossHysteresis = 4;
      _model.config.altitudeFusion.flowLossHysteresis = 8;
      s.println(F("OK landing_indoor"));
      s.println(F("# tip: save"));
    }
    else if(strcmp_P(cmd.args[1], PSTR("landing_outdoor")) == 0)
    {
      // Outdoor ground landing: balanced fusion between baro and GPS with moderate descent.
      _model.config.landingAssist.enabled = 1;
      _model.config.landingAssist.throttleIntentMargin = 35;
      _model.config.landingAssist.descentRateLimitCms = -90;
      _model.config.landingAssist.descentCorrectivePermille = 250;
      _model.config.landingAssist.descentCorrectiveMaxPermille = 350;
      _model.config.landingAssist.baroHeightThresholdCm = 30;
      _model.config.landingAssist.baroVarioThresholdCms = 30;
      _model.config.landingAssist.gpsDownThresholdMms = 250;
      _model.config.landingAssist.gpsGroundThresholdMms = 500;
      _model.config.landingAssist.flowQualityThreshold = 20;
      _model.config.landingAssist.flowHandQualityThreshold = 30;
      _model.config.landingAssist.flowRateThresholdMrad = 250;
      _model.config.landingAssist.flowRateHandThresholdMrad = 200;
      _model.config.landingAssist.handVarioThresholdCms = 25;
      _model.config.landingAssist.handHeightMinCm = 15;
      _model.config.landingAssist.handHeightMaxCm = 180;
      _model.config.landingAssist.touchdownHoldMs = 450;
      _model.config.landingAssist.touchdownRampPermille = 20;

      _model.config.altitudeFusion.baroHeightWeight = 65;
      _model.config.altitudeFusion.baroVarioWeight = 70;
      _model.config.altitudeFusion.gpsHeightWeight = 35;
      _model.config.altitudeFusion.gpsVarioWeight = 30;
      _model.config.altitudeFusion.rangeHeightWeight = 95;
      _model.config.altitudeFusion.flowVarioWeight = 20;
      _model.config.altitudeFusion.flowStillRate = 220;
      _model.config.altitudeFusion.gpsLossHysteresis = 5;
      _model.config.altitudeFusion.flowLossHysteresis = 5;
      s.println(F("OK landing_outdoor"));
      s.println(F("# tip: save"));
    }
    else if(strcmp_P(cmd.args[1], PSTR("landing_freestyle")) == 0)
    {
      // Freestyle quick-disarm: faster touchdown confirmation and steeper throttle ramp.
      _model.config.landingAssist.enabled = 1;
      _model.config.landingAssist.throttleIntentMargin = 25;
      _model.config.landingAssist.descentRateLimitCms = -120;
      _model.config.landingAssist.descentCorrectivePermille = 180;
      _model.config.landingAssist.descentCorrectiveMaxPermille = 280;
      _model.config.landingAssist.baroHeightThresholdCm = 20;
      _model.config.landingAssist.baroVarioThresholdCms = 35;
      _model.config.landingAssist.gpsDownThresholdMms = 350;
      _model.config.landingAssist.gpsGroundThresholdMms = 900;
      _model.config.landingAssist.flowQualityThreshold = 15;
      _model.config.landingAssist.flowHandQualityThreshold = 25;
      _model.config.landingAssist.flowRateThresholdMrad = 320;
      _model.config.landingAssist.flowRateHandThresholdMrad = 280;
      _model.config.landingAssist.handVarioThresholdCms = 35;
      _model.config.landingAssist.handHeightMinCm = 10;
      _model.config.landingAssist.handHeightMaxCm = 220;
      _model.config.landingAssist.touchdownHoldMs = 250;
      _model.config.landingAssist.touchdownRampPermille = 35;

      _model.config.altitudeFusion.baroHeightWeight = 70;
      _model.config.altitudeFusion.baroVarioWeight = 75;
      _model.config.altitudeFusion.gpsHeightWeight = 25;
      _model.config.altitudeFusion.gpsVarioWeight = 20;
      _model.config.altitudeFusion.rangeHeightWeight = 90;
      _model.config.altitudeFusion.flowVarioWeight = 10;
      _model.config.altitudeFusion.flowStillRate = 260;
      _model.config.altitudeFusion.gpsLossHysteresis = 3;
      _model.config.altitudeFusion.flowLossHysteresis = 3;
      s.println(F("OK landing_freestyle"));
      s.println(F("# tip: save"));
    }
    else if(strcmp_P(cmd.args[1], PSTR("alt_fusion_indoor")) == 0)
    {
      _model.config.altitudeFusion.baroHeightWeight = 60;
      _model.config.altitudeFusion.baroVarioWeight = 60;
      _model.config.altitudeFusion.gpsHeightWeight = 15;
      _model.config.altitudeFusion.gpsVarioWeight = 15;
      _model.config.altitudeFusion.rangeHeightWeight = 100;
      _model.config.altitudeFusion.flowVarioWeight = 40;
      _model.config.altitudeFusion.flowStillRate = 170;
      _model.config.altitudeFusion.gpsLossHysteresis = 4;
      _model.config.altitudeFusion.flowLossHysteresis = 8;
      s.println(F("OK alt_fusion_indoor"));
      s.println(F("# tip: save"));
    }
    else if(strcmp_P(cmd.args[1], PSTR("alt_fusion_outdoor")) == 0)
    {
      _model.config.altitudeFusion.baroHeightWeight = 65;
      _model.config.altitudeFusion.baroVarioWeight = 70;
      _model.config.altitudeFusion.gpsHeightWeight = 40;
      _model.config.altitudeFusion.gpsVarioWeight = 35;
      _model.config.altitudeFusion.rangeHeightWeight = 90;
      _model.config.altitudeFusion.flowVarioWeight = 20;
      _model.config.altitudeFusion.flowStillRate = 220;
      _model.config.altitudeFusion.gpsLossHysteresis = 6;
      _model.config.altitudeFusion.flowLossHysteresis = 5;
      s.println(F("OK alt_fusion_outdoor"));
      s.println(F("# tip: save"));
    }
    else
    {
      s.println(F("NOT OK"));
    }
  }
  else if(strcmp_P(cmd.args[0], PSTR("load")) == 0)
  {
    _model.load();
    s.println(F("OK"));
  }
  else if(strcmp_P(cmd.args[0], PSTR("save")) == 0)
  {
    _model.save();
    s.println(F("# Saved, type reboot to apply changes"));
    s.println();
  }
  else if(strcmp_P(cmd.args[0], PSTR("eeprom")) == 0)
  {
    /*
    int start = 0;
    if(cmd.args[1])
    {
      start = std::max(String(cmd.args[1]).toInt(), 0L);
    }

    for(int i = start; i < start + 32; ++i)
    {
      uint8_t v = EEPROM.read(i);
      if(v <= 0xf) s.print('0');
      s.print(v, HEX);
      s.print(' ');
    }
    s.println();

    for(int i = start; i < start + 32; ++i)
    {
      s.print((int8_t)EEPROM.read(i));
      s.print(' ');
    }
    s.println();
    */
  }
  else if(strcmp_P(cmd.args[0], PSTR("scaler")) == 0)
  {
    for(size_t i = 0; i < SCALER_COUNT; i++)
    {
      uint32_t mode = _model.config.scaler[i].dimension;
      if(!mode) continue;
      short c = _model.config.scaler[i].channel;
      float v = _model.state.input.ch[c];
      float min = _model.config.scaler[i].minScale * 0.01f;
      float max = _model.config.scaler[i].maxScale * 0.01f;
      float scale = Utils::map3(v, -1.f, 0.f, 1.f, min, min < 0 ? 0.f : 1.f, max);
      s.print(F("scaler: "));
      s.print(i);
      s.print(' ');
      s.print(mode);
      s.print(' ');
      s.print(min);
      s.print(' ');
      s.print(max);
      s.print(' ');
      s.print(v);
      s.print(' ');
      s.println(scale);
    }
  }
  else if(strcmp_P(cmd.args[0], PSTR("mixer")) == 0)
  {
    const MixerConfig& mixer = _model.state.currentMixer;
    s.print(F("set mix_outputs "));
    s.println(mixer.count);
    Param p;
    for(size_t i = 0; i < MIXER_RULE_MAX; i++)
    {
      s.print(F("set mix_"));
      s.print(i);
      s.print(' ');
      p.print(s, mixer.mixes[i]);
      s.println();
      if(mixer.mixes[i].src == MIXER_SOURCE_NULL) break;
    }
  }
  else if(strcmp_P(cmd.args[0], PSTR("status")) == 0)
  {
#if ESPFC_CLI_VERBOSE_HELP
    printVersion(s);
    s.println();
    s.println(F("STATUS: "));
    printStats(s);
    s.println();

    Device::GyroDevice * gyro = _model.state.gyro.dev;
    Device::BaroDevice * baro = _model.state.baro.dev;
    Device::MagDevice  * mag  = _model.state.mag.dev;
    Device::RangefinderDevice * rangeBottom = _model.state.rangefinder[RANGEFINDER_BOTTOM].dev;
    Device::RangefinderDevice * rangeFront  = _model.state.rangefinder[RANGEFINDER_FRONT].dev;
    s.print(F("     devices: "));
    if(gyro)
    {
      s.print(FPSTR(Device::GyroDevice::getName(gyro->getType())));
      s.print('/');
      s.print(FPSTR(Device::BusDevice::getName(gyro->getBus()->getType())));
    }
    else
    {
      s.print(F("NO GYRO"));
    }

    if(baro)
    {
      s.print(F(", "));
      s.print(FPSTR(Device::BaroDevice::getName(baro->getType())));
      s.print('/');
      s.print(FPSTR(Device::BusDevice::getName(baro->getBus()->getType())));
    }

    if(mag)
    {
      s.print(F(", "));
      s.print(FPSTR(Device::MagDevice::getName(mag->getType())));
      s.print('/');
      s.print(FPSTR(Device::BusDevice::getName(mag->getBus()->getType())));
    }

    if(_model.state.gps.present)
    {
      s.print(F(", GPS"));
    }

    if(_model.state.opticalFlow.dev)
    {
      s.print(F(", "));
      s.print(FPSTR(Device::OpticalFlowDevice::getName(_model.state.opticalFlow.dev->getType())));
      s.print('/');
      s.print(FPSTR(Device::BusDevice::getName(_model.state.opticalFlow.dev->getBus()->getType())));
    }
    else if(_model.config.opticalFlow.dev == Device::OPFLOW_MSP || _model.config.opticalFlow.dev == Device::OPFLOW_DEFAULT)
    {
      s.print(F(", OPFLOW/MSP"));
    }

    if(rangeBottom)
    {
      s.print(F(", RNG-B:"));
      s.print(FPSTR(Device::RangefinderDevice::getName(rangeBottom->getType())));
      s.print('/');
      s.print(FPSTR(Device::BusDevice::getName(rangeBottom->getBus()->getType())));
    }

    if(rangeFront)
    {
      s.print(F(", RNG-F:"));
      s.print(FPSTR(Device::RangefinderDevice::getName(rangeFront->getType())));
      s.print('/');
      s.print(FPSTR(Device::BusDevice::getName(rangeFront->getBus()->getType())));
    }

    if(_model.state.oled.dev)
    {
      s.print(F(", "));
      s.print(FPSTR(Device::OledDevice::getName(_model.state.oled.dev->getType())));
      s.print('/');
      s.print(FPSTR(Device::BusDevice::getName(_model.state.oled.dev->getBus()->getType())));
    }
    s.println();

    s.print(F("       input: "));
    s.print(_model.state.input.frameRate);
    s.print(F(" Hz, "));
    s.print(_model.state.input.autoFreq);
    s.print(F(" Hz, "));
    s.println(_model.state.input.autoFactor);

    s.print(F("      opflow: "));
    s.print(_model.state.opticalFlow.present ? F("OK") : F("NO DATA"));
    s.print(F(", q="));
    s.print(_model.state.opticalFlow.quality);
    s.print(F(", dx="));
    s.print(_model.state.opticalFlow.motionX);
    s.print(F(", dy="));
    s.print(_model.state.opticalFlow.motionY);
    s.print(F(", vx="));
    s.print(_model.state.opticalFlow.flowRateX, 2);
    s.print(F(", vy="));
    s.print(_model.state.opticalFlow.flowRateY, 2);
    if(_model.state.opticalFlow.lastUpdateUs)
    {
      s.print(F(", age="));
      s.print((micros() - _model.state.opticalFlow.lastUpdateUs) * 0.001f, 1);
      s.print(F(" ms"));
    }
    s.println();

    s.print(F("      rangeB: "));
    s.print(_model.state.rangefinder[RANGEFINDER_BOTTOM].present ? F("OK") : F("NO DATA"));
    s.print(F(", dev="));
    s.print(_model.config.rangefinder[RANGEFINDER_BOTTOM].dev);
    s.print(F(", bus="));
    s.print(_model.config.rangefinder[RANGEFINDER_BOTTOM].bus);
    s.print(F(", addr="));
    s.print(_model.config.rangefinder[RANGEFINDER_BOTTOM].address);
    s.print(F(", en="));
    s.print(_model.config.rangefinder[RANGEFINDER_BOTTOM].enabled);
    s.print(F(", mm="));
    s.println(_model.state.rangefinder[RANGEFINDER_BOTTOM].distance);

    s.print(F("      rangeF: "));
    s.print(_model.state.rangefinder[RANGEFINDER_FRONT].present ? F("OK") : F("NO DATA"));
    s.print(F(", dev="));
    s.print(_model.config.rangefinder[RANGEFINDER_FRONT].dev);
    s.print(F(", bus="));
    s.print(_model.config.rangefinder[RANGEFINDER_FRONT].bus);
    s.print(F(", addr="));
    s.print(_model.config.rangefinder[RANGEFINDER_FRONT].address);
    s.print(F(", en="));
    s.print(_model.config.rangefinder[RANGEFINDER_FRONT].enabled);
    s.print(F(", mm="));
    s.println(_model.state.rangefinder[RANGEFINDER_FRONT].distance);

    s.print(F("        oled: "));
    s.println(_model.state.oled.present ? F("OK") : F("MISSING"));

    static const char* armingDisableNames[] = {
      PSTR("NO_GYRO"), PSTR("FAILSAFE"), PSTR("RX_FAILSAFE"), PSTR("BAD_RX_RECOVERY"),
      PSTR("BOXFAILSAFE"), PSTR("RUNAWAY_TAKEOFF"), PSTR("CRASH_DETECTED"), PSTR("THROTTLE"),
      PSTR("ANGLE"), PSTR("BOOT_GRACE_TIME"), PSTR("NOPREARM"), PSTR("LOAD"),
      PSTR("CALIBRATING"), PSTR("CLI"), PSTR("CMS_MENU"), PSTR("BST"),
      PSTR("MSP"), PSTR("PARALYZE"), PSTR("GPS"), PSTR("RESC"),
      PSTR("RPMFILTER"), PSTR("REBOOT_REQUIRED"), PSTR("DSHOT_BITBANG"), PSTR("ACC_CALIBRATION"),
      PSTR("MOTOR_PROTOCOL"), PSTR("ARM_SWITCH")
    };
    const size_t armingDisableNamesLength = sizeof(armingDisableNames) / sizeof(armingDisableNames[0]);

    s.print(F("   arm flags:"));
    for(size_t i = 0; i < armingDisableNamesLength; i++)
    {
      if(_model.state.mode.armingDisabledFlags & (1 << i)) {
        s.print(' ');
        s.print(armingDisableNames[i]);
      }
    }
    s.println();
    s.print(F(" rescue mode: "));
    s.print(_model.state.mode.rescueConfigMode);
    s.println();

    s.print(F("      uptime: "));
    s.print(millis() * 0.001, 1);
    s.println();
#endif
  }
  else if(strcmp_P(cmd.args[0], PSTR("stats")) == 0)
  {
#if ESPFC_CLI_VERBOSE_HELP
    printVersion(s);
    s.println();
    printStats(s);
    s.println();
    for(int i = 0; i < COUNTER_COUNT; ++i)
    {
      StatCounter c = (StatCounter)i;
      int time = lrintf(_model.state.stats.getTime(c));
      float load = _model.state.stats.getLoad(c);
      int freq = lrintf(_model.state.stats.getFreq(c));
      int real = lrintf(_model.state.stats.getReal(c));
      if(freq == 0) continue;

      s.print(FPSTR(_model.state.stats.getName(c)));
      s.print(": ");
      if(time < 100) s.print(' ');
      if(time < 10) s.print(' ');
      s.print(time);
      s.print("us,  ");

      if(real < 100) s.print(' ');
      if(real < 10) s.print(' ');
      s.print(real);
      s.print("us/i,  ");

      if(load < 10) s.print(' ');
      s.print(load, 1);
      s.print("%,  ");

      if(freq < 1000) s.print(' ');
      if(freq < 100) s.print(' ');
      if(freq < 10) s.print(' ');
      s.print(freq);
      s.print(" Hz");
      s.println();
    }
    s.print(F("  TOTAL: "));
    s.print((int)(_model.state.stats.getCpuTime()));
    s.print(F("us, "));
    s.print(_model.state.stats.getCpuLoad(), 1);
    s.print(F("%"));
    s.println();
#endif
  }
  else if(strcmp_P(cmd.args[0], PSTR("reboot")) == 0 || strcmp_P(cmd.args[0], PSTR("exit")) == 0)
  {
    _active = false;
    Hardware::restart(_model);
  }
  else if(strcmp_P(cmd.args[0], PSTR("defaults")) == 0)
  {
    _model.reset();
  }
  else if(strcmp_P(cmd.args[0], PSTR("motors")) == 0)
  {
    s.print(PSTR("count: "));
    s.println(getMotorCount());
    for (size_t i = 0; i < 8; i++)
    {
      s.print(i);
      s.print(PSTR(": "));
      if (i >= OUTPUT_CHANNELS || _model.config.pin[i + PIN_OUTPUT_0] == -1)
      {
        s.print(-1);
        s.print(' ');
        s.println(0);
      } else {
        s.print(_model.config.pin[i + PIN_OUTPUT_0]);
        s.print(' ');
        s.println(_model.state.output.us[i]);
      }
    }
  }
  else if(strcmp_P(cmd.args[0], PSTR("logs")) == 0)
  {
    s.print(_model.logger.c_str());
    s.print(PSTR("usage: "));
    s.println(_model.logger.length());
  }
#ifdef USE_FLASHFS
  else if(strcmp_P(cmd.args[0], PSTR("flash")) == 0)
  {
    if(!cmd.args[1])
    {
      size_t total = flashfsGetSize();
      size_t used = flashfsGetOffset();
      s.printf("total: %d\r\n", total);
      s.printf(" used: %d\r\n", used);
      s.printf(" free: %d\r\n", total - used);
    }
    else if(strcmp_P(cmd.args[1], PSTR("partitions")) == 0)
    {
      Device::FlashDevice::partitions(s);
    }
    else if(strcmp_P(cmd.args[1], PSTR("journal")) == 0)
    {
      const FlashfsRuntime* flashfs = flashfsGetRuntime();
      FlashfsJournalItem journal[16];
      flashfsJournalLoad(journal, 0, 16);
      for(size_t i = 0; i < 16; i++)
      {
        const auto& it = journal[i];
        const auto& itr = flashfs->journal[i];
        s.printf("%02d: %08X : %08X / %08X : %08X\r\n",i , it.logBegin, it.logEnd, itr.logBegin, itr.logEnd);
      }
      s.printf("current: %d\r\n", flashfs->journalIdx);
    }
    else if(strcmp_P(cmd.args[1], PSTR("erase")) == 0)
    {
      flashfsEraseCompletely();
      s.println("OK");
    }
    else if(strcmp_P(cmd.args[1], PSTR("test")) == 0)
    {
      const char * data = "flashfs-test";
      flashfsWrite((const uint8_t*)data, strlen(data), true);
      flashfsFlushAsync(true);
      flashfsClose();
      s.println("OK");
    }
    else if(strcmp_P(cmd.args[1], PSTR("print")) == 0)
    {
      size_t addr = 0;
      if(cmd.args[2])
      {
        addr = parseIntArg(cmd.args[2]);
      }
      size_t size = 0;
      if(cmd.args[3])
      {
        size = parseIntArg(cmd.args[3]);
      }
      size = Utils::clamp(size, 8u, 128 * 1024u);
      size_t chunk_size = 256;

      uint8_t* data = new uint8_t[chunk_size];
      while(size)
      {
        size_t len = std::min(size, chunk_size);
        flashfsReadAbs(addr, data, len);
        s.write(data, len);

        if(size > chunk_size)
        {
          size -= chunk_size;
          addr += chunk_size;
        }
        else break;
      }
      s.println();
      delete[] data;
    }
    else
    {
      s.println(F("wrong param!"));
    }
  }
#endif
  else
  {
    s.print(F("unknown command: "));
    s.println(cmd.args[0]);
  }
  s.println();
}

void Cli::print(const Param& param, Stream& s) const
{
  s.print(F("set "));
  s.print(FPSTR(param.name));
  s.print(' ');
  param.print(s);
  s.println();
}

static constexpr const char * const gnssNames[] = {" GPS", "SBAS", "GALI", "BEID", "IMES", "QZSS", "GLON"};
static constexpr const char * const qualityNames[] = {"no_signal", "searching", "acquired", "unusable", "locked", "fully_locked", "fully_locked", "fully_locked"};
static constexpr const char * const usedNames[] = {" No", "Yes"};

static const char * const getGnssName(size_t num)
{
  constexpr size_t gnssNamesMax = sizeof(gnssNames) / sizeof(gnssNames[0]);
  if(num < gnssNamesMax) return gnssNames[num];
  return "?";
}

static const char * const getQualityName(size_t num)
{
  constexpr size_t qualityNamesMax = sizeof(qualityNames) / sizeof(qualityNames[0]);
  if(num < qualityNamesMax) return qualityNames[num];
  return "?";
}

static const char * const getUsedName(size_t num)
{
  constexpr size_t usedNamesMax = sizeof(usedNames) / sizeof(usedNames[0]);
  if(num < usedNamesMax) return usedNames[num];
  return "?";
}

void Cli::printGpsStatus(Stream& s, bool full) const
{
#ifndef UNIT_TEST
  s.println(F("GPS STATUS:"));

  s.print(F("   Fix: "));
  s.print(_model.state.gps.fix);
  s.print(F(" ("));
  s.print(_model.state.gps.fixType);
  s.println(F(")"));

  s.print(F("   Lat: "));
  s.print(_model.state.gps.location.raw.lat);
  s.print(F(" ("));
  s.print(_model.state.gps.location.raw.lat * 1e-7f, 7);
  s.print(F(" deg)"));
  s.println();

  s.print(F("   Lon: "));
  s.print(_model.state.gps.location.raw.lon);
  s.print(F(" ("));
  s.print(_model.state.gps.location.raw.lon * 1e-7f, 7);
  s.print(F(" deg)"));
  s.println();

  s.print(F("Height: "));
  s.print(_model.state.gps.location.raw.height);
  s.print(F(" ("));
  s.print(_model.state.gps.location.raw.height * 0.001f);
  s.print(F(" m)"));
  s.println();

  s.print(F(" Speed: "));
  s.print(_model.state.gps.velocity.raw.groundSpeed);
  s.print(F(" ("));
  s.print(_model.state.gps.velocity.raw.groundSpeed * 0.001f);
  s.print(F(" m/s, "));
  s.print(_model.state.gps.velocity.raw.groundSpeed * 0.0036f);
  s.print(F(" km/h)"));
  s.println();

  s.print(F("  Head: "));
  s.print(_model.state.gps.velocity.raw.heading);
  s.print(F(" ("));
  s.print(_model.state.gps.velocity.raw.heading * 0.00001f);
  s.print(F(" deg)"));
  s.println();

  s.print(F("  Accu: "));
  s.print(_model.state.gps.accuracy.horizontal * 0.001f);
  s.print(F(" m, "));
  s.print(_model.state.gps.accuracy.vertical * 0.001f);
  s.print(F(" m, "));
  s.print(_model.state.gps.accuracy.speed * 0.001f);
  s.print(F(" m/s, "));
  s.print(_model.state.gps.accuracy.heading * 0.00001f);
  s.print(F(" deg, pDOP: "));
  s.print(_model.state.gps.accuracy.pDop * 0.01f);
  s.println();

  const GpsDateTime& gdt = _model.state.gps.dateTime;
  s.printf("  Time: %04d-%02d-%02d %02d:%02d:%02d.%03d UTC", gdt.year, gdt.month, gdt.day, gdt.hour, gdt.minute, gdt.second, gdt.msec);
  s.println();

  s.print(F("  Rate: "));
  s.print(1000000.0f / _model.state.gps.interval, 1);
  s.println(F(" Hz"));

  s.print(F("  Sats: "));
  s.print(_model.state.gps.numSats);
  s.print(F(" ("));
  s.print(_model.state.gps.numCh);
  s.println(F(" ch)"));

  s.printf("GNSS  ID Sig Used Quality");
  s.println();
  for (size_t i = 0; i < _model.state.gps.numCh; i++)
  {
    const GpsSatelite& sv = _model.state.gps.svinfo[i];
    s.printf("%s %3d %3d  %s %s", getGnssName(sv.gnssId), sv.id, sv.cno, getUsedName(sv.quality.svUsed), getQualityName(sv.quality.qualityInd));
    s.println();
  }
  s.println(F("Home:"));
  if (_model.state.gps.homeSet)
  {
    s.print(F("  Lat:  "));
    s.print(_model.state.gps.location.home.lat);
    s.print(F(" ("));
    s.print(_model.state.gps.location.home.lat * 1e-7f, 7);
    s.println(F(")"));

    s.print(F("  Lon:  "));
    s.print(_model.state.gps.location.home.lon);
    s.print(F(" ("));
    s.print(_model.state.gps.location.home.lon * 1e-7f, 7);
    s.println(F(")"));

    s.print(F("  Dist: "));
    s.print(Utils::toDeg(_model.state.gps.distanceToHome), 2);
    s.println(F(" m"));

    s.print(F("  Bear: "));
    s.print(Utils::toDeg(_model.state.gps.directionToHome), 2);
    s.println(F(" deg"));
  }
  else
  {
    s.println(F("  Not set"));
  }
#endif
}

void Cli::printVersion(Stream& s) const
{
  s.print(boardIdentifier);
  s.print(' ');
  s.print(targetName);
  s.print(' ');
  s.print(targetVersion);
  s.print(' ');
  s.print(shortGitRevision);
  s.print(' ');
  s.print(buildDate);
  s.print(' ');
  s.print(buildTime);
  s.print(" api=");
  s.print(API_VERSION_MAJOR);
  s.print('.');
  s.print(API_VERSION_MINOR);
  s.print(" gcc=");
  s.print(__VERSION__);
  s.print(" std=");
  s.print(__cplusplus);
}

void Cli::printStats(Stream& s) const
{
  s.print(F("    cpu freq: "));
  s.print(targetCpuFreq());
  s.println(F(" MHz"));

  s.print(F("  gyro clock: "));
  s.print(_model.state.gyro.clock);
  s.println(F(" Hz"));

  s.print(F("   gyro rate: "));
  s.print(_model.state.gyro.timer.rate);
  s.println(F(" Hz"));

  s.print(F("   loop rate: "));
  s.print(_model.state.loopTimer.rate);
  s.println(F(" Hz"));

  s.print(F("  mixer rate: "));
  s.print(_model.state.mixer.timer.rate);
  s.println(F(" Hz"));

  s.print(F("  accel rate: "));
  s.print(_model.state.accel.timer.rate);
  s.println(F(" Hz"));

  s.print(F("   baro rate: "));
  s.print(_model.state.baro.rate);
  s.println(F(" Hz"));

  s.print(F("    mag rate: "));
  s.print(_model.state.mag.timer.rate);
  s.println(F(" Hz"));
}

}

}
