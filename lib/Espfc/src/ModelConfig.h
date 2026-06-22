#ifndef _ESPFC_MODEL_CONFIG_H_
#define _ESPFC_MODEL_CONFIG_H_

#include "Target/Target.h"
#include "EscDriver.h"
#include "Utils/Filter.h"
#include "Device/BusDevice.hpp"
#include "Device/GyroDevice.h"
#include "Device/MagDevice.hpp"
#include "Device/BaroDevice.hpp"
#include "Device/RangefinderDevice.hpp"
#include "Device/OpticalFlowDevice.hpp"
#include "Device/OledDevice.hpp"
#include "Device/SerialDevice.h"
#include "Device/InputPPM.h"
#include "Output/Mixers.h"
#include "Control/Pid.h"

namespace Espfc {

enum GyroDlpf {
  GYRO_DLPF_256 = 0x00,
  GYRO_DLPF_188 = 0x01,
  GYRO_DLPF_98  = 0x02,
  GYRO_DLPF_42  = 0x03,
  GYRO_DLPF_20  = 0x04,
  GYRO_DLPF_10  = 0x05,
  GYRO_DLPF_5   = 0x06,
  GYRO_DLPF_EX  = 0x07,
};

enum AccelMode {
  ACCEL_DELAYED   = 0x00,
  ACCEL_GYRO      = 0x01,
};

enum SensorAlign {
  ALIGN_DEFAULT        = 0,
  ALIGN_CW0_DEG        = 1,
  ALIGN_CW90_DEG       = 2,
  ALIGN_CW180_DEG      = 3,
  ALIGN_CW270_DEG      = 4,
  ALIGN_CW0_DEG_FLIP   = 5,
  ALIGN_CW90_DEG_FLIP  = 6,
  ALIGN_CW180_DEG_FLIP = 7,
  ALIGN_CW270_DEG_FLIP = 8,
  ALIGN_CUSTOM         = 9,
};

enum FusionMode {
  FUSION_NONE,
  FUSION_MADGWICK,
  FUSION_MAHONY,
  FUSION_COMPLEMENTARY,
  FUSION_KALMAN,
  FUSION_RTQF,
  FUSION_LERP,
  FUSION_SIMPLE,
  FUSION_EXPERIMENTAL,
  FUSION_MAX,
};

struct FusionConfig
{
  int8_t mode = FUSION_MAHONY;
  uint8_t gain = 50;
  uint8_t gainI = 5;

  static const char * getModeName(FusionMode mode)
  {
    if(mode >= FUSION_MAX) return PSTR("?");
    return getModeNames()[mode];
  }

  static const char ** getModeNames()
  {
    static const char* modeChoices[] = {
      PSTR("NONE"), PSTR("MADGWICK"), PSTR("MAHONY"), PSTR("COMPLEMENTARY"), PSTR("KALMAN"),
      PSTR("RTQF"), PSTR("LERP"), PSTR("SIMPLE"), PSTR("EXPERIMENTAL"),
      NULL };
    return modeChoices;
  }
};

enum FlightMode {
  MODE_ARMED,
  MODE_AIRMODE,
  MODE_ANGLE,
  MODE_HORIZON,
  MODE_ALTHOLD,
  MODE_BUZZER,
  MODE_FAILSAFE,
  MODE_BLACKBOX,
  MODE_BLACKBOX_ERASE,
  MODE_MAG,
  MODE_HEADFREE,
  MODE_HEADADJ,
  MODE_CALIB,
  MODE_GPS_RESCUE,
  MODE_PREARM,
  MODE_FLIP_OVER_AFTER_CRASH,
  MODE_USER1,
  MODE_USER2,
  MODE_USER3,
  MODE_USER4,
  MODE_ACRO_TRAINER,
  MODE_LAUNCH_CONTROL,
  MODE_MSP_OVERRIDE,
  MODE_STICK_COMMANDS_DISABLE,
  MODE_BEEPER_MUTE,
  MODE_PARALYZE,
  MODE_COUNT,
};

enum RangefinderPosition {
  RANGEFINDER_BOTTOM = 0,  // Downward facing - altitude hold, landing
  RANGEFINDER_FRONT = 1,   // Forward facing - obstacle avoidance
  RANGEFINDER_COUNT = 2,
};

enum ScalerDimension {
  ACT_INNER_P     = 1 << 0,  // 1
  ACT_INNER_I     = 1 << 1,  // 2
  ACT_INNER_D     = 1 << 2,  // 4
  ACT_INNER_F     = 1 << 3,  // 8
  ACT_OUTER_P     = 1 << 4,  // 16
  ACT_OUTER_I     = 1 << 5,  // 32
  ACT_OUTER_D     = 1 << 6,  // 64
  ACT_OUTER_F     = 1 << 7,  // 128
  ACT_AXIS_ROLL   = 1 << 8,  // 256
  ACT_AXIS_PITCH  = 1 << 9,  // 512
  ACT_AXIS_YAW    = 1 << 10, // 1024
  ACT_AXIS_THRUST = 1 << 11, // 2048
  ACT_GYRO_THRUST = 1 << 12, // 4096
};

#if defined(ESP32S2)
constexpr size_t SCALER_COUNT = 2;
#else
constexpr size_t SCALER_COUNT = 3;
#endif

struct ScalerConfig {
  uint32_t dimension = 0;
  int16_t minScale = 20;
  int16_t maxScale = 400;
  int8_t channel = 0;
};

enum DebugMode {
  DEBUG_NONE,
  DEBUG_CYCLETIME,
  DEBUG_BATTERY,
  DEBUG_GYRO_FILTERED,
  DEBUG_ACCELEROMETER,
  DEBUG_PIDLOOP,
  DEBUG_GYRO_SCALED,
  DEBUG_RC_INTERPOLATION,
  DEBUG_ANGLERATE,
  DEBUG_ESC_SENSOR,
  DEBUG_SCHEDULER,
  DEBUG_STACK,
  DEBUG_ESC_SENSOR_RPM,
  DEBUG_ESC_SENSOR_TMP,
  DEBUG_ALTITUDE,
  DEBUG_FFT,
  DEBUG_FFT_TIME,
  DEBUG_FFT_FREQ,
  DEBUG_RX_FRSKY_SPI,
  DEBUG_RX_SFHSS_SPI,
  DEBUG_GYRO_RAW,
  DEBUG_DUAL_GYRO_RAW,
  DEBUG_DUAL_GYRO_DIFF,
  DEBUG_MAX7456_SIGNAL,
  DEBUG_MAX7456_SPICLOCK,
  DEBUG_SBUS,
  DEBUG_FPORT,
  DEBUG_RANGEFINDER,
  DEBUG_RANGEFINDER_QUALITY,
  DEBUG_LIDAR_TF,
  DEBUG_ADC_INTERNAL,
  DEBUG_RUNAWAY_TAKEOFF,
  DEBUG_SDIO,
  DEBUG_CURRENT_SENSOR,
  DEBUG_USB,
  DEBUG_SMARTAUDIO,
  DEBUG_RTH,
  DEBUG_ITERM_RELAX,
  DEBUG_ACRO_TRAINER,
  DEBUG_RC_SMOOTHING,
  DEBUG_RX_SIGNAL_LOSS,
  DEBUG_RC_SMOOTHING_RATE,
  DEBUG_ANTI_GRAVITY,
  DEBUG_DYN_LPF,
  DEBUG_RX_SPEKTRUM_SPI,
  DEBUG_DSHOT_RPM_TELEMETRY,
  DEBUG_RPM_FILTER,
  DEBUG_D_MIN,
  DEBUG_AC_CORRECTION,
  DEBUG_AC_ERROR,
  DEBUG_DUAL_GYRO_SCALED,
  DEBUG_DSHOT_RPM_ERRORS,
  DEBUG_CRSF_LINK_STATISTICS_UPLINK,
  DEBUG_CRSF_LINK_STATISTICS_PWR,
  DEBUG_CRSF_LINK_STATISTICS_DOWN,
  DEBUG_BARO,
  DEBUG_GPS_RESCUE_THROTTLE_PID,
  DEBUG_DYN_IDLE,
  DEBUG_FF_LIMIT,
  DEBUG_FF_INTERPOLATED,
  DEBUG_BLACKBOX_OUTPUT,
  DEBUG_GYRO_SAMPLE,
  DEBUG_RX_TIMING,
  DEBUG_COUNT,
};

enum Axis {
  AXIS_ROLL,    // x
  AXIS_PITCH,   // y
  AXIS_YAW,     // z
  AXIS_THRUST,  // throttle channel index
  AXIS_AUX_1,
  AXIS_AUX_2,
  AXIS_AUX_3,
  AXIS_AUX_4,
  AXIS_AUX_5,
  AXIS_AUX_6,
  AXIS_AUX_7,
  AXIS_AUX_8,
  AXIS_AUX_9,
  AXIS_AUX_10,
  AXIS_AUX_11,
  AXIS_AUX_12,
  AXIS_COUNT,
  AXIS_COUNT_RP = AXIS_YAW,      // RP axis count
  AXIS_COUNT_RPY = AXIS_THRUST,  // RPY axis count
  AXIS_COUNT_RPYT = AXIS_AUX_1,  // RPYT axis count
};

enum Feature {
  FEATURE_RX_PPM     = 1 << 0,
  FEATURE_RX_SERIAL  = 1 << 3,
  FEATURE_MOTOR_STOP = 1 << 4,
  FEATURE_SOFTSERIAL = 1 << 6,
  FEATURE_GPS        = 1 << 7,
  FEATURE_TELEMETRY  = 1 << 10,
  FEATURE_AIRMODE    = 1 << 22,
  FEATURE_RX_SPI     = 1 << 25,
  FEATURE_DYNAMIC_FILTER = 1 << 29,
};

enum InputInterpolation {
  INPUT_INTERPOLATION_OFF,
  INPUT_INTERPOLATION_DEFAULT,
  INPUT_INTERPOLATION_AUTO,
  INPUT_INTERPOLATION_MANUAL,
};

enum InputFilterType : uint8_t {
  INPUT_INTERPOLATION,
  INPUT_FILTER,
};

constexpr size_t MODEL_NAME_LEN  = 16;
constexpr size_t INPUT_CHANNELS  = AXIS_COUNT;
constexpr size_t OUTPUT_CHANNELS = ESC_CHANNEL_COUNT;
static_assert(ESC_CHANNEL_COUNT == ESPFC_OUTPUT_COUNT, "ESC_CHANNEL_COUNT and ESPFC_OUTPUT_COUNT must be equal");

constexpr size_t RPM_FILTER_MOTOR_MAX = 4;
constexpr size_t RPM_FILTER_HARMONICS_MAX = 3;

enum PinFunction {
#ifdef ESPFC_INPUT
  PIN_INPUT_RX,
#endif
  PIN_OUTPUT_0,
  PIN_OUTPUT_1,
  PIN_OUTPUT_2,
  PIN_OUTPUT_3,
#if ESPFC_OUTPUT_COUNT > 4
  PIN_OUTPUT_4,
#endif
#if ESPFC_OUTPUT_COUNT > 5
  PIN_OUTPUT_5,
#endif
#if ESPFC_OUTPUT_COUNT > 6
  PIN_OUTPUT_6,
#endif
#if ESPFC_OUTPUT_COUNT > 7
  PIN_OUTPUT_7,
#endif
  PIN_BUTTON,
  PIN_BUZZER,
  PIN_LED_BLINK,
#ifdef ESPFC_SERIAL_0
  PIN_SERIAL_0_TX,
  PIN_SERIAL_0_RX,
#endif
#ifdef ESPFC_SERIAL_1
  PIN_SERIAL_1_TX,
  PIN_SERIAL_1_RX,
#endif
#ifdef ESPFC_SERIAL_2
  PIN_SERIAL_2_TX,
  PIN_SERIAL_2_RX,
#endif
#ifdef ESPFC_I2C_0
  PIN_I2C_0_SCL,
  PIN_I2C_0_SDA,
#endif
#ifdef ESPFC_ADC_0
  PIN_INPUT_ADC_0,
#endif
#ifdef ESPFC_ADC_1
  PIN_INPUT_ADC_1,
#endif
#ifdef ESPFC_SPI_0
  PIN_SPI_0_SCK,
  PIN_SPI_0_MOSI,
  PIN_SPI_0_MISO,
#endif
#ifdef ESPFC_SPI_0
  PIN_SPI_CS0,
  PIN_SPI_CS1,
  PIN_SPI_CS2,
#endif
  PIN_COUNT,
};

constexpr size_t ACTUATOR_CONDITIONS = 16;
constexpr size_t ADJUSTMENT_RANGES_COUNT = 16;

struct ActuatorCondition
{
  uint8_t id = 0;
  uint8_t ch = AXIS_AUX_1;
  int16_t min = 900;
  int16_t max = 900;
  uint8_t logicMode = 0;
  uint8_t linkId = 0;
};

struct AdjustmentRangeConfig
{
  uint8_t stateIndex = 0;
  uint8_t auxChannelIndex = 0;
  uint8_t rangeStartStep = 0;
  uint8_t rangeEndStep = 0;
  uint8_t adjustmentFunction = 0;
  uint8_t auxSwitchChannelIndex = 0;
  int16_t adjustmentCenter = 0;
  int16_t adjustmentScale = 0;
};

struct SerialPortConfig
{
  int8_t id;
  int32_t functionMask;
  int32_t baud;
  int32_t blackboxBaud;
};

constexpr size_t BUZZER_MAX_EVENTS = 8;

enum BuzzerEvent {
  BUZZER_SILENCE = 0,             // Silence, see beeperSilence()
  BUZZER_GYRO_CALIBRATED,
  BUZZER_RX_LOST,                 // Beeps when TX is turned off or signal lost (repeat until TX is okay)
  BUZZER_RX_LOST_LANDING,         // Beeps SOS when armed and TX is turned off or signal lost (autolanding/autodisarm)
  BUZZER_DISARMING,               // Beep when disarming the board
  BUZZER_ARMING,                  // Beep when arming the board
  BUZZER_ARMING_GPS_FIX,          // Beep a special tone when arming the board and GPS has fix
  BUZZER_BAT_CRIT_LOW,            // Longer warning beeps when battery is critically low (repeats)
  BUZZER_BAT_LOW,                 // Warning beeps when battery is getting low (repeats)
  BUZZER_GPS_STATUS,              // FIXME **** Disable beeper when connected to USB ****
  BUZZER_RX_SET,                  // Beeps when aux channel is set for beep or beep sequence how many satellites has found if GPS enabled
  BUZZER_ACC_CALIBRATION,         // ACC inflight calibration completed confirmation
  BUZZER_ACC_CALIBRATION_FAIL,    // ACC inflight calibration failed
  BUZZER_READY_BEEP,              // Ring a tone when GPS is locked and ready
  BUZZER_MULTI_BEEPS,             // Internal value used by 'beeperConfirmationBeeps()'.
  BUZZER_DISARM_REPEAT,           // Beeps sounded while stick held in disarm position
  BUZZER_ARMED,                   // Warning beeps when board is armed (repeats until board is disarmed or throttle is increased)
  BUZZER_SYSTEM_INIT,             // Initialisation beeps when board is powered on
  BUZZER_USB,                     // Some boards have beeper powered USB connected
  BUZZER_BLACKBOX_ERASE,          // Beep when blackbox erase completes
  BUZZER_CRASH_FLIP_MODE,         // Crash flip mode is active
  BUZZER_CAM_CONNECTION_OPEN,     // When the 5 key simulation stated
  BUZZER_CAM_CONNECTION_CLOSE,    // When the 5 key simulation stop
  BUZZER_ARMING_GPS_NO_FIX,       // Beep a special tone when arming and GPS has no fix
  BUZZER_ALL,                     // Turn ON or OFF all beeper conditions
  BUZZER_PREFERENCE,              // Save preferred beeper configuration
  // BUZZER_ALL and BUZZER_PREFERENCE must remain at the bottom of this enum
};

constexpr uint32_t buzzerEventFlag(BuzzerEvent mode)
{
  return mode > BUZZER_SILENCE ? (1u << (mode - 1)) : 0u;
}

constexpr uint32_t BUZZER_ALLOWED_MASK =
  buzzerEventFlag(BUZZER_GYRO_CALIBRATED) |
  buzzerEventFlag(BUZZER_RX_LOST) |
  buzzerEventFlag(BUZZER_RX_LOST_LANDING) |
  buzzerEventFlag(BUZZER_DISARMING) |
  buzzerEventFlag(BUZZER_ARMING) |
  buzzerEventFlag(BUZZER_ARMING_GPS_FIX) |
  buzzerEventFlag(BUZZER_BAT_CRIT_LOW) |
  buzzerEventFlag(BUZZER_BAT_LOW) |
  buzzerEventFlag(BUZZER_GPS_STATUS) |
  buzzerEventFlag(BUZZER_RX_SET) |
  buzzerEventFlag(BUZZER_ACC_CALIBRATION) |
  buzzerEventFlag(BUZZER_ACC_CALIBRATION_FAIL) |
  buzzerEventFlag(BUZZER_READY_BEEP) |
  buzzerEventFlag(BUZZER_MULTI_BEEPS) |
  buzzerEventFlag(BUZZER_DISARM_REPEAT) |
  buzzerEventFlag(BUZZER_ARMED) |
  buzzerEventFlag(BUZZER_SYSTEM_INIT) |
  buzzerEventFlag(BUZZER_USB) |
  buzzerEventFlag(BUZZER_BLACKBOX_ERASE) |
  buzzerEventFlag(BUZZER_CRASH_FLIP_MODE) |
  buzzerEventFlag(BUZZER_CAM_CONNECTION_OPEN) |
  buzzerEventFlag(BUZZER_CAM_CONNECTION_CLOSE) |
  buzzerEventFlag(BUZZER_ARMING_GPS_NO_FIX);

struct BuzzerConfig
{
  int8_t inverted = true;
  int32_t beeperMask = (int32_t)BUZZER_ALLOWED_MASK;
};

enum PidIndex {
  FC_PID_ROLL,
  FC_PID_PITCH,
  FC_PID_YAW,
  FC_PID_ALT,
  FC_PID_POS,
  FC_PID_POSR,
  FC_PID_NAVR,
  FC_PID_LEVEL,
  FC_PID_MAG,
  FC_PID_VEL,
  FC_PID_ITEM_COUNT,
};

enum BlacboxLogField { // no more than 32, sync with FlightLogFieldSelect_e
  BLACKBOX_FIELD_PID = 0,
  BLACKBOX_FIELD_RC_COMMANDS,
  BLACKBOX_FIELD_SETPOINT,
  BLACKBOX_FIELD_BATTERY,
  BLACKBOX_FIELD_MAG,
  BLACKBOX_FIELD_ALTITUDE,
  BLACKBOX_FIELD_RSSI,
  BLACKBOX_FIELD_GYRO,
  BLACKBOX_FIELD_ACC,
  BLACKBOX_FIELD_DEBUG_LOG,
  BLACKBOX_FIELD_MOTOR,
  BLACKBOX_FIELD_GPS,
  BLACKBOX_FIELD_RPM,
  BLACKBOX_FIELD_GYROUNFILT,
  BLACKBOX_FIELD_COUNT,
};

enum BlackboxLogDevice {
  BLACKBOX_DEV_NONE = 0,
  BLACKBOX_DEV_FLASH = 1,
  BLACKBOX_DEV_SDCARD = 2,
  BLACKBOX_DEV_SERIAL = 3,
};

struct PidConfig
{
  uint8_t P;
  uint8_t I;
  uint8_t D;
  int16_t F;
  uint8_t dMin = 0;  // D-term minimum value (percent of D)
};

struct InputChannelConfig
{
  int16_t min = 1000;
  int16_t neutral = 1500;
  int16_t max = 2000;
  int8_t map = 0;
  int8_t fsMode = 0;
  int16_t fsValue = 1500;
};

struct InputConfig
{
  int8_t ppmMode = PPM_MODE_NORMAL;
  uint8_t serialRxProvider = SERIALRX_SBUS;

  int16_t minCheck = 1050;
  int16_t maxCheck = 1900;
  int16_t minRc = 885;
  int16_t midRc = 1500;
  int16_t maxRc = 2115;

  int8_t interpolationMode = INPUT_INTERPOLATION_AUTO;
  int8_t interpolationInterval = 26;
  int8_t deadband = 3;

  int8_t filterType = INPUT_FILTER;
  int8_t filterAutoFactor = 50;
  uint8_t rcSmoothing = 1;
  uint8_t rxSpiProtocol = 0;
  uint8_t elrsUid[6] = { 0, 0, 0, 0, 0, 0 };
  uint8_t elrsModelId = 0;
  FilterConfig filter{FILTER_PT3, 0};
  FilterConfig filterDerivative{FILTER_PT3, 0};

  uint8_t expo[3] = { 0, 0, 0 };
  uint8_t throttleExpo = 0;
  uint8_t rate[3] = { 20, 20, 30 };
  uint8_t superRate[3] = { 40, 40,  36 };
  int16_t rateLimit[3] = { 1998, 1998, 1998 };
  int8_t rateType = 3;

  uint8_t rssiChannel = 0;

  InputChannelConfig channel[INPUT_CHANNELS];
};

constexpr size_t RATE_PROFILE_COUNT = 1;

struct RateProfileConfig
{
  uint8_t expo[3] = { 0, 0, 0 };
  uint8_t throttleExpo = 0;
  uint8_t rate[3] = { 20, 20, 30 };
  uint8_t superRate[3] = { 40, 40, 36 };
  int16_t rateLimit[3] = { 1998, 1998, 1998 };
  int8_t rateType = 3;
};

constexpr size_t PID_PROFILE_COUNT = 1;

struct PidProfileConfig
{
  PidConfig pid[FC_PID_ITEM_COUNT] = {
    [FC_PID_ROLL]  = { .P = 42, .I = 85, .D = 24, .F = 72 },  // ROLL
    [FC_PID_PITCH] = { .P = 46, .I = 90, .D = 26, .F = 76 },  // PITCH
    [FC_PID_YAW]   = { .P = 45, .I = 90, .D =  0, .F = 72 },  // YAW
    [FC_PID_ALT]   = { .P =  0, .I =  0, .D =  0, .F =  0 },  // ALTHOLD POS
    [FC_PID_POS]   = { .P =  0, .I =  0, .D =  0, .F =  0 },  // POSHOLD_P * 100, POSHOLD_I * 100,
    [FC_PID_POSR]  = { .P =  0, .I =  0, .D =  0, .F =  0 },  // POSHOLD_RATE_P * 10, POSHOLD_RATE_I * 100, POSHOLD_RATE_D * 1000,
    [FC_PID_NAVR]  = { .P =  0, .I =  0, .D =  0, .F =  0 },  // NAV_P * 10, NAV_I * 100, NAV_D * 1000
    [FC_PID_LEVEL] = { .P = 45, .I =  0, .D =  0, .F =  0 },  // ANGLE/LEVEL
    [FC_PID_MAG]   = { .P =  0, .I =  0, .D =  0, .F =  0 },  // MAG
    [FC_PID_VEL]   = { .P = 80, .I = 60, .D = 40, .F = 20 },  // ALTHOLD VEL
  };
};

struct OutputChannelConfig
{
  int16_t min = 1000;
  int16_t neutral = 1500;
  int16_t max = 2000;
  bool reverse = false;
  bool servo = false;
};

struct OutputConfig
{
  int8_t protocol = ESC_PROTOCOL_DISABLED;
  int8_t async = false;
  int8_t dshotTelemetry = false;
  int8_t motorPoles = 14;
  int16_t rate = 480;
  int16_t servoRate = 0;

  int16_t minCommand = 1000;
  int16_t minThrottle = 1070;
  int16_t maxThrottle = 2000;
  int16_t dshotIdle = 550;
#if !defined(ESP32S2)
  int16_t deadband3dLow = 1406;
  int16_t deadband3dHigh = 1514;
  int16_t neutral3d = 1460;
#endif

  int8_t throttleLimitType = 0;
  int8_t throttleLimitPercent = 100;
  int8_t motorLimit = 100;

#if !defined(ESP32S2)
  uint8_t motorOutputReordering[OUTPUT_CHANNELS] = {0};
#endif

  OutputChannelConfig channel[ESPFC_OUTPUT_COUNT];
};

enum DisarmReason {
  DISARM_REASON_ARMING_DISABLED   = 0,
  DISARM_REASON_FAILSAFE          = 1,
  DISARM_REASON_THROTTLE_TIMEOUT  = 2,
  DISARM_REASON_STICKS            = 3,
  DISARM_REASON_SWITCH            = 4,
  DISARM_REASON_CRASH_PROTECTION  = 5,
  DISARM_REASON_RUNAWAY_TAKEOFF   = 6,
  DISARM_REASON_GPS_RESCUE        = 7,
  DISARM_REASON_SERIAL_COMMAND    = 8,
  DISARM_REASON_SYSTEM            = 255,
};

enum ArmingDisabledFlags {
  ARMING_DISABLED_NO_GYRO         = (1 << 0),
  ARMING_DISABLED_FAILSAFE        = (1 << 1),
  ARMING_DISABLED_RX_FAILSAFE     = (1 << 2),
  ARMING_DISABLED_BAD_RX_RECOVERY = (1 << 3),
  ARMING_DISABLED_BOXFAILSAFE     = (1 << 4),
  ARMING_DISABLED_RUNAWAY_TAKEOFF = (1 << 5),
  ARMING_DISABLED_CRASH_DETECTED  = (1 << 6),
  ARMING_DISABLED_THROTTLE        = (1 << 7),
  ARMING_DISABLED_ANGLE           = (1 << 8),
  ARMING_DISABLED_BOOT_GRACE_TIME = (1 << 9),
  ARMING_DISABLED_NOPREARM        = (1 << 10),
  ARMING_DISABLED_LOAD            = (1 << 11),
  ARMING_DISABLED_CALIBRATING     = (1 << 12),
  ARMING_DISABLED_CLI             = (1 << 13),
  ARMING_DISABLED_CMS_MENU        = (1 << 14),
  ARMING_DISABLED_BST             = (1 << 15),
  ARMING_DISABLED_MSP             = (1 << 16),
  ARMING_DISABLED_PARALYZE        = (1 << 17),
  ARMING_DISABLED_GPS             = (1 << 18),
  ARMING_DISABLED_RESC            = (1 << 19),
  ARMING_DISABLED_RPMFILTER       = (1 << 20),
  ARMING_DISABLED_REBOOT_REQUIRED = (1 << 21),
  ARMING_DISABLED_DSHOT_BITBANG   = (1 << 22),
  ARMING_DISABLED_ACC_CALIBRATION = (1 << 23),
  ARMING_DISABLED_MOTOR_PROTOCOL  = (1 << 24),
  ARMING_DISABLED_ARM_SWITCH      = (1 << 25), // Needs to be the last element, since it's always activated if one of the others is active when arming
};

static constexpr size_t ARMING_DISABLED_FLAGS_COUNT = 25;

struct WirelessConfig
{
  static constexpr size_t MAX_LEN = 32;
  int16_t port = 1111;
  char ssid[MAX_LEN + 1];
  char pass[MAX_LEN + 1];
  uint8_t otaEnabled = 1;
  int16_t otaPort = 3232;
  char otaPass[MAX_LEN + 1];
  uint8_t btOtaEnabled = 0;
};

struct FailsafeConfig
{
  uint8_t delay = 4;
  uint8_t offDelay = 10;
  int16_t throttle = 1000;
  uint8_t killSwitch = 0;
  int16_t throttleLowDelay = 100;
  uint8_t procedure = 0;
};

struct BlackboxConfig
{
  int8_t dev = 0;
  int16_t pDenom = 32; // 1k loop rate: log every 32 samples (1000/32 ≈ 31Hz)
  // Enable all available blackbox fields for comprehensive sensor logging:
  // GYRO | GYRO_RAW | ACC | MAG | BARO | PID | RC | MOTOR | BATTERY | GPS | DEBUG | RPM | ALTITUDE
  int32_t fieldsMask = 0xffffffff; // Log all available sensor fields
  int8_t mode = 0;
};

struct DebugConfig
{
  int8_t mode = DEBUG_NONE;
  uint8_t axis = 1;
};

struct RpmFilterConfig
{
  uint8_t harmonics = 3;
  uint8_t minFreq = 100;
  int16_t q = 500;
  uint8_t freqLpf = 150;
  uint8_t weights[RPM_FILTER_HARMONICS_MAX] = {100, 100, 100};
  uint8_t fade = 30;
};

struct VBatConfig
{
  int16_t cellWarning = 350;
  uint8_t scale = 100;
  uint8_t resDiv = 10;
  uint8_t resMult = 1;
  int8_t source = 0;
};

struct IBatConfig
{
  int8_t source = 0;
  int16_t scale = 100;
  int16_t offset = 0;
};

struct GyroConfig
{
  int8_t bus = BUS_AUTO;
  int8_t dev = GYRO_AUTO;
  int8_t dlpf = GYRO_DLPF_256;
  int8_t align = ALIGN_DEFAULT;
  int16_t bias[3] = { 0, 0, 0 };
  FilterConfig filter{FILTER_PT1, 100};
  FilterConfig filter2{FILTER_PT1, 213};
  FilterConfig filter3{FILTER_FO, 150};
  FilterConfig notch1Filter{FILTER_NOTCH, 0, 0};
  FilterConfig notch2Filter{FILTER_NOTCH, 0, 0};
  FilterConfig dynLpfFilter{FILTER_PT1, 425, 170};
  DynamicFilterConfig dynamicFilter;
  RpmFilterConfig rpmFilter;
};

struct AccelConfig
{
  int8_t bus = BUS_AUTO;
  int8_t dev = GYRO_AUTO;
  int16_t bias[3] = { 0, 0, 0 };
  int16_t trim[2] = { 0, 0 };
  FilterConfig filter{FILTER_BIQUAD, 15};
};

struct BaroConfig
{
  int8_t bus = BUS_AUTO;
  int8_t dev = BARO_DEFAULT;
  FilterConfig filter{FILTER_BIQUAD, 3};
};

struct MagConfig
{
  int8_t bus = BUS_AUTO;
  int8_t dev = MAG_DEFAULT;
  int8_t align = ALIGN_DEFAULT;
  int16_t offset[3] = { 0, 0, 0 };
  int16_t scale[3] = { 1000, 1000, 1000 };
  FilterConfig filter{FILTER_BIQUAD, 10};
};

struct RangefinderConfig
{
  int8_t bus = BUS_AUTO;
  int8_t dev = Device::RANGEFINDER_DEFAULT;
  FilterConfig filter{FILTER_BIQUAD, 50};
  uint8_t address = 0;                    // 0=auto/default I2C address
  uint8_t position = RANGEFINDER_BOTTOM;  // BOTTOM or FRONT
  uint8_t enabled = 1;                    // Individual enable flag
};

struct OpticalFlowConfig
{
  int8_t bus = BUS_AUTO;
  int8_t dev = Device::OPFLOW_DEFAULT;
  int16_t qualityThreshold = 10;
};

struct ObstacleAvoidanceConfig
{
  uint8_t enabled = 0;              // Enable obstacle avoidance
  uint16_t minSafeDistance = 100;   // Minimum safe distance in cm
  uint16_t avoidanceDistance = 150; // Start avoidance at this distance in cm
  uint16_t maxAvoidanceDistance = 300; // Max rangefinder valid distance in cm
  uint8_t slowdownPercent = 50;     // Reduce throttle by this % when obstacle detected
  uint8_t stopPercent = 0;          // Reduce throttle by this % to stop forward motion
  uint8_t avoidanceMode = 0;        // 0=slow down, 1=stop, 2=bypass (move up/down), 3=auto
  uint8_t bypassAxis = 0;           // 0=yaw, 1=pitch, 2=roll (for bypass mode)
  uint8_t enableInAcro = 1;         // Enable in acro mode
  uint8_t enableInHorizon = 1;      // Enable in horizon mode
  uint8_t enableInAngle = 1;        // Enable in angle mode
  uint8_t enableInAltHold = 1;      // Enable in altitude hold mode
  uint8_t debugMode = 0;            // Debug output mode
};

struct OledConfig
{
  int8_t bus = BUS_AUTO;
  int8_t dev = Device::OLED_DEFAULT;
  uint8_t height = 0;      // 0=auto, 32 or 64
  int16_t pageInterval = 3000; // ms between page auto-scroll
  int16_t startupMs = 1500; // ms startup info page duration (0 disables)
};

constexpr uint8_t ESPFC_OSD_ITEM_COUNT = 32;
constexpr uint8_t ESPFC_OSD_STAT_COUNT = 8;
constexpr uint8_t ESPFC_OSD_TIMER_COUNT = 2;
constexpr uint8_t ESPFC_OSD_PROFILE_COUNT = 1;

struct OsdConfig
{
  uint8_t enabled = 1;
  uint8_t mspDisplayport = 1;
  uint8_t videoSystem = 2; // 0=AUTO, 1=PAL, 2=NTSC, 3=HD
  uint8_t units = 0;
  uint8_t rssiAlarm = 20;
  uint16_t capAlarm = 2200;
  uint16_t altAlarm = 100;
  uint16_t itemPos[ESPFC_OSD_ITEM_COUNT] = {0};
  uint8_t statEnabled[ESPFC_OSD_STAT_COUNT] = {0};
  uint16_t timers[ESPFC_OSD_TIMER_COUNT] = {0};
  uint32_t enabledWarnings = 0;
  uint8_t profileCount = ESPFC_OSD_PROFILE_COUNT;
  uint8_t profile = 1;
  uint8_t overlayRadioMode = 0;
  uint8_t cameraFrameWidth = 24;
  uint8_t cameraFrameHeight = 11;
  uint16_t linkQualityAlarm = 70;
  uint16_t rssiDbmAlarm = 60;
};

struct YawConfig
{
  FilterConfig filter{FILTER_PT1, 90};
};

struct DtermConfig
{
  FilterConfig filter{FILTER_PT1, 128};
  FilterConfig filter2{FILTER_PT1, 128};
  FilterConfig notchFilter{FILTER_NOTCH, 0, 0};
  FilterConfig dynLpfFilter{FILTER_PT1, 145, 60};
  uint8_t feedForwardTransition = 0;
  int16_t setpointWeight = 30;
  uint8_t dMinRoll = 0;  // D-min for roll axis (percent)
  uint8_t dMinPitch = 0;  // D-min for pitch axis (percent)
  uint8_t dMinYaw = 0;  // D-min for yaw axis (percent)
  uint8_t dMinGain = 37;  // Gain applied when D-min reduces D-term
  uint8_t dMinAdvance = 20;  // Advance time for D-min
  uint8_t vbatPidCompensation = 0;  // Battery voltage PID compensation (0-100)
};

struct ItermConfig
{
  int8_t limit = 30;
  int8_t relax = ITERM_RELAX_RP;
  int8_t relaxCutoff = 15;
  bool lowThrottleZeroIterm = true;
  bool itermRotation = false;  // Enable iterm rotation
};

struct LevelConfig
{
  FilterConfig ptermFilter{FILTER_PT1, 90};
  int8_t angleLimit = 55;
  int16_t rateLimit = 300;
  uint8_t horizonStrength = 100;
  bool integratedYaw = false;  // Use integrated yaw
  uint8_t antiGravityGain = 0;  // Anti-gravity mode gain (0 = disabled)
};

struct AltHoldConfig
{
  uint8_t itermCenter = 50;
  uint8_t itermRange = 50;
};

struct MixerConfiguration
{
  int8_t type = FC_MIXER_QUADX;
  bool yawReverse = 0;
};

struct ControllerConfig
{
  int8_t tpaScale = 10;
  int16_t tpaBreakpoint = 1650;
};

constexpr size_t SERVO_MIX_RULES_MAX = 16; // Protocol-visible rule slots

#if defined(ESP32S2)
#define ESPFC_SERVO_MIX_RULES_STORAGE 0
#define ESPFC_EXTENDED_CONFIG_STORAGE 0
#elif defined(ARDUINO_ARCH_ESP8266)
#define ESPFC_SERVO_MIX_RULES_STORAGE 8
#define ESPFC_EXTENDED_CONFIG_STORAGE 1
#else
#define ESPFC_SERVO_MIX_RULES_STORAGE SERVO_MIX_RULES_MAX
#define ESPFC_EXTENDED_CONFIG_STORAGE 1
#endif

constexpr size_t SERVO_MIX_RULES_STORAGE = ESPFC_SERVO_MIX_RULES_STORAGE;

struct ServoMixRule
{
  uint8_t targetChannel = 0;  // output channel
  uint8_t inputSource = 0;    // input source (roll, pitch, yaw, thrust, etc.)
  int8_t rate = 0;            // rate/scale (-127 to 127)
  uint8_t speed = 0;          // servo speed
  uint8_t min = 0;            // min value
  uint8_t max = 255;          // max value
  uint8_t box = 0;            // logic condition box index
};

constexpr size_t LED_STRIP_MAX_LENGTH = 32;
constexpr size_t LED_CONFIGURABLE_COLOR_COUNT = 16;
constexpr size_t LED_MODE_COUNT = 6;
constexpr size_t LED_DIRECTION_COUNT = 6;
constexpr size_t LED_SPECIAL_COLOR_COUNT = 11;
constexpr uint8_t LED_SPECIAL_MODE_INDEX = 6;
constexpr uint8_t LED_AUX_CHANNEL_MODE_INDEX = 7;

struct LedHsvConfig
{
  uint16_t h = 0;
  uint8_t s = 0;
  uint8_t v = 0;
};

struct LedStripConfig
{
  uint32_t ledConfig[LED_STRIP_MAX_LENGTH] = {0};
  LedHsvConfig colors[LED_CONFIGURABLE_COLOR_COUNT] = {
    {  0,   0,   0 }, // BLACK
    {  0, 255, 255 }, // WHITE
    {  0,   0, 255 }, // RED
    { 15,   0, 255 }, // ORANGE
    { 50,   0, 255 }, // YELLOW
    {100,   0, 255 }, // LIME_GREEN
    {115,   0, 255 }, // GREEN
    {125,   0, 255 }, // MINT_GREEN
    {180,   0, 255 }, // CYAN
    {210,   0, 255 }, // LIGHT_BLUE
    {240,   0, 255 }, // BLUE
    {270,   0, 255 }, // DARK_VIOLET
    {300,   0, 255 }, // MAGENTA
    {330,   0, 255 }, // DEEP_PINK
    {  0,   0,   0 },
    {  0,   0,   0 },
  };
  uint8_t modeColors[LED_MODE_COUNT][LED_DIRECTION_COUNT] = {
    {1, 11, 2, 13, 10, 3}, // ORIENTATION
    {5, 11, 3, 13, 10, 3}, // HEADFREE
    {10, 11, 4, 13, 10, 3}, // HORIZON
    {8, 11, 4, 13, 10, 3}, // ANGLE
    {7, 11, 3, 13, 10, 3}, // MAG
    {1, 11, 2, 13, 10, 3}, // BARO
  };
  uint8_t specialColors[LED_SPECIAL_COLOR_COUNT] = {
    6,  // DISARMED
    10, // ARMED
    1,  // ANIMATION
    0,  // BACKGROUND
    0,  // BLINKBACKGROUND
    2,  // GPSNOSATS
    3,  // GPSNOLOCK
    6,  // GPSLOCKED
    0,
    0,
    0,
  };
  uint8_t auxChannel = AXIS_THRUST;
  uint8_t profile = 2; // STATUS
};

constexpr size_t VTX_TABLE_MAX_BANDS = 8;
constexpr size_t VTX_TABLE_MAX_CHANNELS = 8;
constexpr size_t VTX_TABLE_MAX_POWER_LEVELS = 8;
constexpr size_t VTX_TABLE_BAND_NAME_LENGTH = 8;
constexpr size_t VTX_TABLE_POWER_LABEL_LENGTH = 3;

struct VtxTableConfig
{
  uint8_t bands = 5;
  uint8_t channels = 8;
  uint8_t powerLevels = 5;
  uint16_t frequency[VTX_TABLE_MAX_BANDS][VTX_TABLE_MAX_CHANNELS] = {
    {5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725}, // BOSCAM A
    {5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866}, // BOSCAM B
    {5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945}, // BOSCAM E
    {5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880}, // FATSHARK
    {5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917}, // RACEBAND
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0},
  };
  char bandNames[VTX_TABLE_MAX_BANDS][VTX_TABLE_BAND_NAME_LENGTH + 1] = {
    "BOSCAM A",
    "BOSCAM B",
    "BOSCAM E",
    "FATSHARK",
    "RACEBAND",
    "BAND6",
    "BAND7",
    "BAND8",
  };
  char bandLetters[VTX_TABLE_MAX_BANDS] = {'A', 'B', 'E', 'F', 'R', '6', '7', '8'};
  uint8_t isFactoryBand[VTX_TABLE_MAX_BANDS] = {1, 1, 1, 1, 1, 0, 0, 0};
  uint16_t powerValues[VTX_TABLE_MAX_POWER_LEVELS] = {25, 100, 200, 400, 600, 0, 0, 0};
  char powerLabels[VTX_TABLE_MAX_POWER_LEVELS][VTX_TABLE_POWER_LABEL_LENGTH + 1] = {
    "25 ",
    "100",
    "200",
    "400",
    "600",
    "LV5",
    "LV6",
    "LV7",
  };
};

struct VtxConfig
{
  uint8_t channel = 1;
  uint8_t band = 4;
  uint8_t power = 1;
  uint8_t lowPowerDisarm = 0;
  uint16_t freq = 5740;
  uint16_t pitModeFreq = 0;
};

struct GpsConfig
{
  uint8_t minSats = 8;
  uint8_t setHomeOnce = 1;

  // GNSS Constellation Configuration (M10 multi-band support)
  uint8_t gnssMode = 0;          // 0=Auto, 1=GPS only, 2=GPS+GLO, 3=GPS+GAL, 4=GPS+BDS, 5=All
  uint8_t enableDualBand = 1;    // Enable L1+L5 dual-band on M10 (0=L1 only, 1=Auto/M10 dual-band)
  uint8_t enableGPS = 1;         // Enable GPS constellation
  uint8_t enableGLONASS = 0;     // Enable GLONASS constellation
  uint8_t enableGalileo = 1;     // Enable Galileo constellation
  uint8_t enableBeiDou = 0;      // Enable BeiDou constellation
  uint8_t enableQZSS = 1;        // Enable QZSS (Asia-Pacific)
  uint8_t enableSBAS = 1;        // Enable SBAS (WAAS/EGNOS)
};

struct LedConfig
{
  uint8_t invert = 0;
  int8_t type = 0;
};

struct ArmingConfig
{
#if !defined(ESP32S2)
  uint8_t autoDisarmDelay = 5;
  uint8_t disarmKillSwitch = 0;
#endif
  uint8_t smallAngle = 25;
};

struct LandingAssistConfig
{
  uint8_t enabled = 1;
  uint8_t throttleIntentMargin = 35;     // us above min_check
  int16_t descentRateLimitCms = -90;     // cm/s
  int16_t descentCorrectivePermille = 250; // gain * 1000
  int16_t descentCorrectiveMaxPermille = 350; // max additive throttle * 1000

  int16_t baroHeightThresholdCm = 30;
  int16_t baroVarioThresholdCms = 30;

  int16_t gpsDownThresholdMms = 250;
  int16_t gpsGroundThresholdMms = 500;

  uint8_t flowQualityThreshold = 20;
  uint8_t flowHandQualityThreshold = 30;
  int16_t flowRateThresholdMrad = 250;
  int16_t flowRateHandThresholdMrad = 200;

  int16_t handVarioThresholdCms = 25;
  int16_t handHeightMinCm = 15;
  int16_t handHeightMaxCm = 180;

  int16_t touchdownHoldMs = 450;
  int16_t touchdownRampPermille = 20;   // throttle decrement per loop * 1000
};

struct AltitudeFusionConfig
{
  // Weights are in percent-like units [0..100], normalized at runtime across available sensors.
  uint8_t baroHeightWeight = 65;
  uint8_t baroVarioWeight = 70;
  uint8_t gpsHeightWeight = 35;
  uint8_t gpsVarioWeight = 30;
  uint8_t rangeHeightWeight = 95;
  uint8_t flowVarioWeight = 20;

  int16_t flowStillRate = 220;  // optical-flow rate threshold where stillness cue reaches zero

  // Sensor-loss hysteresis in update cycles to smooth brief dropouts.
  uint8_t gpsLossHysteresis = 5;
  uint8_t flowLossHysteresis = 5;
};

struct AutopilotConfig
{
  uint8_t landingAltitudeM = 4;
  uint16_t hoverThrottle = 1275;
  uint16_t throttleMin = 1100;
  uint16_t throttleMax = 1900;
  uint8_t altitudeP = 30;
  uint8_t altitudeI = 30;
  uint8_t altitudeD = 30;
  uint8_t altitudeF = 30;
};

struct GpsRescueConfig
{
  uint16_t maxRescueAngle = 45;
  uint16_t returnAltitudeM = 30;
  uint16_t descentDistanceM = 20;
  uint16_t groundSpeedCmS = 750;
  uint8_t yawP = 20;
  uint8_t minSats = 8;
  uint8_t velP = 8;
  uint8_t velI = 40;
  uint8_t velD = 12;
  uint16_t minStartDistM = 15;
  uint8_t sanityChecks = 2;
  uint8_t allowArmingWithoutFix = 0;
  uint8_t useMag = 1;
  uint8_t altitudeMode = 0;
  uint16_t ascendRate = 750;
  uint16_t descendRate = 150;
  uint16_t initialClimbM = 10;
  uint8_t rollMix = 150;
  uint8_t disarmThreshold = 30;
  uint8_t pitchCutoffHz = 75;
  uint8_t imuYawGain = 10;
};

// persistent data
class ModelConfig
{
  public:
    // inputs and sensors
    GyroConfig gyro;
    AccelConfig accel;
    BaroConfig baro;
    MagConfig mag;
    RangefinderConfig rangefinder[RANGEFINDER_COUNT];  // [BOTTOM] and [FRONT]
    ObstacleAvoidanceConfig obstacleAvoidance;
    OpticalFlowConfig opticalFlow;
    OledConfig oled;
    OsdConfig osd;
    InputConfig input;
    FailsafeConfig failsafe;
    FusionConfig fusion;
    VBatConfig vbat;
    IBatConfig ibat;
    VtxConfig vtx;
    VtxTableConfig vtxTable;
    GpsConfig gps;
    ArmingConfig arming;
    LandingAssistConfig landingAssist;
    AltitudeFusionConfig altitudeFusion;
    AutopilotConfig autopilot;
    GpsRescueConfig gpsRescue;

    ActuatorCondition conditions[ACTUATOR_CONDITIONS];
    AdjustmentRangeConfig adjustmentRanges[ADJUSTMENT_RANGES_COUNT];
    ScalerConfig scaler[SCALER_COUNT];
    uint8_t activeRateProfile = 0;
    RateProfileConfig rateProfiles[RATE_PROFILE_COUNT];
    uint8_t activePidProfile = 0;
    PidProfileConfig pidProfiles[PID_PROFILE_COUNT];

    // pid controller
    PidConfig pid[FC_PID_ITEM_COUNT] = {
      [FC_PID_ROLL]  = { .P = 42, .I = 85, .D = 24, .F = 72 },  // ROLL
      [FC_PID_PITCH] = { .P = 46, .I = 90, .D = 26, .F = 76 },  // PITCH
      [FC_PID_YAW]   = { .P = 45, .I = 90, .D =  0, .F = 72 },  // YAW
      [FC_PID_ALT]   = { .P =  0, .I =  0, .D =  0, .F =  0 },  // ALTHOLD POS
      [FC_PID_POS]   = { .P =  0, .I =  0, .D =  0, .F =  0 },  // POSHOLD_P * 100, POSHOLD_I * 100,
      [FC_PID_POSR]  = { .P =  0, .I =  0, .D =  0, .F =  0 },  // POSHOLD_RATE_P * 10, POSHOLD_RATE_I * 100, POSHOLD_RATE_D * 1000,
      [FC_PID_NAVR]  = { .P =  0, .I =  0, .D =  0, .F =  0 },  // NAV_P * 10, NAV_I * 100, NAV_D * 1000
      [FC_PID_LEVEL] = { .P = 45, .I =  0, .D =  0, .F =  0 },  // ANGLE/LEVEL
      [FC_PID_MAG]   = { .P =  0, .I =  0, .D =  0, .F =  0 },  // MAG
      [FC_PID_VEL]   = { .P = 80, .I = 60, .D = 40, .F = 20 },  // ALTHOLD VEL
    };
    YawConfig yaw;
    LevelConfig level;
    DtermConfig dterm;
    ItermConfig iterm;
    AltHoldConfig altHold;
    ControllerConfig controller;
    // hardware
    int8_t pin[PIN_COUNT] = {
#ifdef ESPFC_INPUT
      [PIN_INPUT_RX] = ESPFC_INPUT_PIN,
#endif
      [PIN_OUTPUT_0] = ESPFC_OUTPUT_0,
      [PIN_OUTPUT_1] = ESPFC_OUTPUT_1,
      [PIN_OUTPUT_2] = ESPFC_OUTPUT_2,
      [PIN_OUTPUT_3] = ESPFC_OUTPUT_3,
#if ESPFC_OUTPUT_COUNT > 4
      [PIN_OUTPUT_4] = ESPFC_OUTPUT_4,
#endif
#if ESPFC_OUTPUT_COUNT > 5
      [PIN_OUTPUT_5] = ESPFC_OUTPUT_5,
#endif
#if ESPFC_OUTPUT_COUNT > 6
      [PIN_OUTPUT_6] = ESPFC_OUTPUT_6,
#endif
#if ESPFC_OUTPUT_COUNT > 7
      [PIN_OUTPUT_7] = ESPFC_OUTPUT_7,
#endif
      [PIN_BUTTON] = ESPFC_BUTTON_PIN,
      [PIN_BUZZER] = ESPFC_BUZZER_PIN,
      [PIN_LED_BLINK] = ESPFC_LED_PIN,
#ifdef ESPFC_SERIAL_0
      [PIN_SERIAL_0_TX] = ESPFC_SERIAL_0_TX,
      [PIN_SERIAL_0_RX] = ESPFC_SERIAL_0_RX,
#endif
#ifdef ESPFC_SERIAL_1
      [PIN_SERIAL_1_TX] = ESPFC_SERIAL_1_TX,
      [PIN_SERIAL_1_RX] = ESPFC_SERIAL_1_RX,
#endif
#ifdef ESPFC_SERIAL_2
      [PIN_SERIAL_2_TX] = ESPFC_SERIAL_2_TX,
      [PIN_SERIAL_2_RX] = ESPFC_SERIAL_2_RX,
#endif
#ifdef ESPFC_I2C_0
      [PIN_I2C_0_SCL] = ESPFC_I2C_0_SCL,
      [PIN_I2C_0_SDA] = ESPFC_I2C_0_SDA,
#endif
#ifdef ESPFC_ADC_0
      [PIN_INPUT_ADC_0] = ESPFC_ADC_0_PIN,
#endif
#ifdef ESPFC_ADC_1
      [PIN_INPUT_ADC_1] = ESPFC_ADC_1_PIN,
#endif
#ifdef ESPFC_SPI_0
      [PIN_SPI_0_SCK] = ESPFC_SPI_0_SCK,
      [PIN_SPI_0_MOSI] = ESPFC_SPI_0_MOSI,
      [PIN_SPI_0_MISO] = ESPFC_SPI_0_MISO,
      [PIN_SPI_CS0] = ESPFC_SPI_CS_GYRO,
      [PIN_SPI_CS1] = ESPFC_SPI_CS_BARO,
      [PIN_SPI_CS2] = -1,
#endif
    };
    SerialPortConfig serial[SERIAL_UART_COUNT] = {
#ifdef ESPFC_SERIAL_USB
      [SERIAL_USB]    = { .id = SERIAL_ID_USB_VCP, .functionMask = ESPFC_SERIAL_USB_FN, .baud = SERIAL_SPEED_115200, .blackboxBaud = SERIAL_SPEED_NONE },
#endif
#ifdef ESPFC_SERIAL_0
      [SERIAL_UART_0] = { .id = SERIAL_ID_UART_1, .functionMask = ESPFC_SERIAL_0_FN, .baud = ESPFC_SERIAL_0_BAUD, .blackboxBaud = ESPFC_SERIAL_0_BBAUD },
#endif
#ifdef ESPFC_SERIAL_1
      [SERIAL_UART_1] = { .id = SERIAL_ID_UART_2, .functionMask = ESPFC_SERIAL_1_FN, .baud = ESPFC_SERIAL_1_BAUD, .blackboxBaud = ESPFC_SERIAL_1_BBAUD },
#endif
#ifdef ESPFC_SERIAL_2
      [SERIAL_UART_2] = { .id = SERIAL_ID_UART_3, .functionMask = ESPFC_SERIAL_2_FN, .baud = ESPFC_SERIAL_2_BAUD, .blackboxBaud = ESPFC_SERIAL_2_BBAUD },
#endif
#ifdef ESPFC_SERIAL_SOFT_0
      [SERIAL_SOFT_0] = { .id = SERIAL_ID_SOFTSERIAL_1, .functionMask = ESPFC_SERIAL_SOFT_0_FN, .baud = SERIAL_SPEED_115200, .blackboxBaud = SERIAL_SPEED_NONE },
#endif
    };

    LedConfig led;
    LedStripConfig ledStrip;
    BuzzerConfig buzzer;
    WirelessConfig wireless;

    // mixer and outputs
    int8_t customMixerCount = 0;
    MixerEntry customMixes[MIXER_RULE_MAX];
    MixerConfiguration mixer;
  #if ESPFC_SERVO_MIX_RULES_STORAGE > 0
    uint8_t servoMixRuleCount = 0;
    ServoMixRule servoMixRules[SERVO_MIX_RULES_STORAGE];
  #endif
    OutputConfig output;
    BlackboxConfig blackbox;
    DebugConfig debug;

    // not classified yet
    int16_t i2cSpeed = 800;
    int8_t loopSync = 8; // MPU 1000Hz
    int8_t mixerSync = 1;
    int32_t featureMask = ESPFC_FEATURE_MASK;
    bool telemetry = 0;
    int32_t telemetryInterval = 1000;
    uint8_t rescueConfigDelay = 30;
    int16_t boardAlignment[3] = {0, 0, 0};
    char modelName[MODEL_NAME_LEN + 1];

    ModelConfig()
    {
      for(size_t i = 0; i < INPUT_CHANNELS; i++)
      {
        input.channel[i].map = i;
        input.channel[i].neutral = input.midRc;
        input.channel[i].fsMode = i <= AXIS_THRUST ? 0 : 1; // auto, hold, set
        input.channel[i].fsValue = i != AXIS_THRUST ? input.midRc : 1000;
      }
      // swap yaw and throttle for AETR
      input.channel[2].map = 3; // replace input 2 with rx channel 3, yaw
      input.channel[3].map = 2; // replace input 3 with rx channel 2, throttle

#if !defined(ESP32S2)
      for(size_t i = 0; i < OUTPUT_CHANNELS; i++)
      {
        output.motorOutputReordering[i] = i;
      }
#endif

      // PID controller config (BF default)
      //pid[FC_PID_ROLL]  = { .P = 42, .I = 85, .D = 30, .F = 90 };
      //pid[FC_PID_PITCH] = { .P = 46, .I = 90, .D = 32, .F = 95 };
      //pid[FC_PID_YAW]   = { .P = 45, .I = 90, .D =  0, .F = 90 };
      //pid[FC_PID_LEVEL] = { .P = 55, .I =  0, .D =  0, .F = 0 };

      wireless.ssid[0] = 0;
      wireless.pass[0] = 0;
      modelName[0] = 0;

      // Keep default vtx frequency in sync with the default vtx table selection.
      if(vtx.band >= 1 && vtx.band <= vtxTable.bands && vtx.channel >= 1 && vtx.channel <= vtxTable.channels)
      {
        vtx.freq = vtxTable.frequency[vtx.band - 1][vtx.channel - 1];
      }

      // Dual rangefinder defaults: keep bottom enabled, front opt-in.
      rangefinder[RANGEFINDER_BOTTOM].position = RANGEFINDER_BOTTOM;
      rangefinder[RANGEFINDER_BOTTOM].enabled = 1;
      rangefinder[RANGEFINDER_FRONT].position = RANGEFINDER_FRONT;
      rangefinder[RANGEFINDER_FRONT].enabled = 0;
      rangefinder[RANGEFINDER_FRONT].dev = Device::RANGEFINDER_NONE;

// only local development settings
#if !defined(ESPFC_REVISION)
      devPreset();
#endif
    }

    void devPreset()
    {
#ifdef ESPFC_DEV_PRESET_BLACKBOX_SERIAL
      blackbox.dev = BLACKBOX_DEV_SERIAL; // serial
      debug.mode = DEBUG_GYRO_SCALED;
      serial[ESPFC_DEV_PRESET_BLACKBOX_SERIAL].functionMask |= SERIAL_FUNCTION_BLACKBOX;
      serial[ESPFC_DEV_PRESET_BLACKBOX_SERIAL].blackboxBaud = SERIAL_SPEED_250000;
      serial[ESPFC_DEV_PRESET_BLACKBOX_SERIAL].baud = SERIAL_SPEED_250000;
#endif

#ifdef ESPFC_DEV_PRESET_BLACKBOX_FLASH
      blackbox.dev = BLACKBOX_DEV_FLASH; // flash
      blackbox.pDenom = 16; // 500Hz
#endif

#ifdef ESPFC_DEV_PRESET_MODES
      conditions[0].id = MODE_ARMED;
      conditions[0].ch = AXIS_AUX_1 + 0; // aux1
      conditions[0].min = 1300;
      conditions[0].max = 2100;
      conditions[0].logicMode = 0;
      conditions[0].linkId = 0;

      conditions[1].id = MODE_AIRMODE;
      conditions[1].ch = AXIS_AUX_1 + 0; // aux1
      conditions[1].min = 1300;
      conditions[1].max = 2100;
      conditions[1].logicMode = 0;
      conditions[1].linkId = 0;

      conditions[2].id = MODE_ANGLE;
      conditions[2].ch = AXIS_AUX_1 + 1; // aux2
      conditions[2].min = 1300;
      conditions[2].max = 2100;
      conditions[2].logicMode = 0;
      conditions[2].linkId = 0;

      conditions[3].id = MODE_ALTHOLD;
      conditions[3].ch = AXIS_AUX_1 + 1; // aux2
      conditions[3].min = 1700;
      conditions[3].max = 2100;
      conditions[3].logicMode = 0;
      conditions[3].linkId = 0;
#endif

#ifdef ESPFC_DEV_PRESET_SCALER
      scaler[0].dimension = (ACT_INNER_P | ACT_AXIS_ROLL | ACT_AXIS_PITCH);
      scaler[0].channel = AXIS_AUX_1 + 1;
      scaler[1].dimension = (ACT_INNER_I | ACT_AXIS_ROLL | ACT_AXIS_PITCH);
      scaler[1].channel = AXIS_AUX_1 + 2;
      scaler[2].dimension = (ACT_INNER_D | ACT_AXIS_ROLL | ACT_AXIS_PITCH);
      scaler[2].channel = AXIS_AUX_1 + 3;
#endif

#ifdef ESPFC_DEV_PRESET_DSHOT
      output.protocol = ESC_PROTOCOL_DSHOT300;
#endif

#ifdef ESPFC_DEV_PRESET_BRUSHED
      output.protocol = ESC_PROTOCOL_BRUSHED;
      output.async = true;
      output.rate = 3000;
#endif
    }

    void brobot()
    {
      mixer.type = FC_MIXER_GIMBAL;

      pin[PIN_OUTPUT_0] = 14;    // D5 // ROBOT
      pin[PIN_OUTPUT_1] = 12;    // D6 // ROBOT
      pin[PIN_OUTPUT_2] = 15;    // D8 // ROBOT
      pin[PIN_OUTPUT_3] = 0;     // D3 // ROBOT

      //fusionMode = FUSION_SIMPLE;    // ROBOT
      //fusionMode = FUSION_COMPLEMENTARY; // ROBOT
      //accelFilter.freq = 30;        // ROBOT

      iterm.lowThrottleZeroIterm = false; // ROBOT
      iterm.limit = 10; // ROBOT
      dterm.setpointWeight = 0;      // ROBOT
      level.angleLimit = 10;       // deg // ROBOT

      output.protocol = ESC_PROTOCOL_PWM; // ROBOT
      output.rate = 100;    // ROBOT
      output.async = true;  // ROBOT

      output.channel[0].servo = true;   // ROBOT
      output.channel[1].servo = true;   // ROBOT
      output.channel[0].reverse = true; // ROBOT

      scaler[0].dimension = (ACT_INNER_P | ACT_AXIS_PITCH); // ROBOT
      //scaler[1].dimension = (ACT_INNER_P | ACT_AXIS_YAW); // ROBOT
      scaler[1].dimension = (ACT_INNER_I | ACT_AXIS_PITCH); // ROBOT
      scaler[2].dimension = (ACT_INNER_D | ACT_AXIS_PITCH); // ROBOT
    }
};

}

#endif