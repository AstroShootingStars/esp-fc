#pragma once

#include "Model.h"
#include "Utils/Math.hpp"
#include "Device/InputDevice.h"
#include "Device/InputPPM.h"
#include "Device/InputIBUS.hpp"
#include "Device/InputSBUS.h"
#include "Device/InputCRSF.h"
#include "Device/InputSpektrum.h"
#include "Device/InputSUMD.h"
#include "Device/InputSUMH.h"
#include "Device/InputFPort.h"
#include "TelemetryManager.h"
#if defined(ESPFC_ESPNOW)
#include "Device/InputEspNow.h"
#endif

namespace Espfc {

enum FailsafeChannelMode {
  FAILSAFE_MODE_AUTO,
  FAILSAFE_MODE_HOLD,
  FAILSAFE_MODE_SET,
  FAILSAFE_MODE_INVALID
};

enum InputPwmRange {
  PWM_RANGE_MIN = 1000,
  PWM_RANGE_MID = 1500,
  PWM_RANGE_MAX = 2000
};

enum FailsafeProcedure {
  FAILSAFE_PROCEDURE_DROP = 0,
  FAILSAFE_PROCEDURE_LAND = 1,
  FAILSAFE_PROCEDURE_GPS_RESCUE = 2,
};

class Input
{
  public:
    Input(Model& model, TelemetryManager& telemetry);

    int begin();
    int update();

    int16_t getFailsafeValue(uint8_t c);
    void setInput(Axis i, float v, bool newFrame, bool noFilter = false);

    InputStatus readInputs();
    void processInputs();

    bool failsafe(InputStatus status);
    void failsafeIdle();
    void failsafeStage1();
    void failsafeStage2();
    void failsafeLanding();
    void filterInputs(InputStatus status);

    void updateFrameRate();
    Device::InputDevice * getInputDevice();

  private:
    inline float _interpolate(float left, float right, float step)
    {
      return (left * (1.f - step) + right * step);
    }

    Model& _model;
    TelemetryManager& _telemetry;
    Device::InputDevice * _device;
    Utils::Filter _filter[INPUT_CHANNELS];
    float _step;
    Device::InputPPM _ppm;
    Device::InputIBUS _ibus;
    Device::InputSBUS _sbus;
    Device::InputCRSF _crsf;
    Device::InputSpektrum _spektrum;
    Device::InputSUMD _sumd;
    Device::InputSUMH _sumh;
    Device::InputFPort _fport;
#if defined(ESPFC_ESPNOW)
    Device::InputEspNow _espnow;
#endif

    static constexpr uint32_t TENTH_TO_US = 100000UL;  // 1_000_000 / 10;
    static constexpr uint32_t FRAME_TIME_DEFAULT_US = 23000; // 23 ms
    static constexpr uint8_t FAILSAFE_RECOVERY_FRAMES = 3;

    uint8_t _failsafeRecoveryFrames = 0;
};

}
