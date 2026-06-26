#include "Control/Actuator.h"
#include "Utils/Math.hpp"
#include <Arduino.h>
#include <algorithm>
#include <cmath>

namespace {

constexpr uint8_t ADJUSTMENT_FUNCTION_NONE = 0;
constexpr uint8_t ADJUSTMENT_FUNCTION_RC_RATE = 1;
constexpr uint8_t ADJUSTMENT_FUNCTION_RC_EXPO = 2;
constexpr uint8_t ADJUSTMENT_FUNCTION_THROTTLE_EXPO = 3;
constexpr uint8_t ADJUSTMENT_FUNCTION_PITCH_ROLL_RATE = 4;
constexpr uint8_t ADJUSTMENT_FUNCTION_YAW_RATE = 5;
constexpr uint8_t ADJUSTMENT_FUNCTION_PITCH_ROLL_P = 6;
constexpr uint8_t ADJUSTMENT_FUNCTION_PITCH_ROLL_I = 7;
constexpr uint8_t ADJUSTMENT_FUNCTION_PITCH_ROLL_D = 8;
constexpr uint8_t ADJUSTMENT_FUNCTION_YAW_P = 9;
constexpr uint8_t ADJUSTMENT_FUNCTION_YAW_I = 10;
constexpr uint8_t ADJUSTMENT_FUNCTION_YAW_D = 11;
constexpr uint8_t ADJUSTMENT_FUNCTION_RATE_PROFILE = 12;
constexpr uint8_t ADJUSTMENT_FUNCTION_PITCH_RATE = 13;
constexpr uint8_t ADJUSTMENT_FUNCTION_ROLL_RATE = 14;
constexpr uint8_t ADJUSTMENT_FUNCTION_PITCH_P = 15;
constexpr uint8_t ADJUSTMENT_FUNCTION_PITCH_I = 16;
constexpr uint8_t ADJUSTMENT_FUNCTION_PITCH_D = 17;
constexpr uint8_t ADJUSTMENT_FUNCTION_ROLL_P = 18;
constexpr uint8_t ADJUSTMENT_FUNCTION_ROLL_I = 19;
constexpr uint8_t ADJUSTMENT_FUNCTION_ROLL_D = 20;
constexpr uint8_t ADJUSTMENT_FUNCTION_RC_RATE_YAW = 21;
constexpr uint8_t ADJUSTMENT_FUNCTION_PITCH_ROLL_F = 22;
constexpr uint8_t ADJUSTMENT_FUNCTION_FEEDFORWARD_TRANSITION = 23;
constexpr uint8_t ADJUSTMENT_FUNCTION_HORIZON_STRENGTH = 24;
constexpr uint8_t ADJUSTMENT_FUNCTION_PITCH_F = 26;
constexpr uint8_t ADJUSTMENT_FUNCTION_ROLL_F = 27;
constexpr uint8_t ADJUSTMENT_FUNCTION_YAW_F = 28;
constexpr uint8_t ADJUSTMENT_FUNCTION_OSD_PROFILE = 29;
constexpr uint8_t ADJUSTMENT_FUNCTION_LED_PROFILE = 30;
constexpr uint8_t ADJUSTMENT_FUNCTION_SLIDER_MASTER_MULTIPLIER = 31;

constexpr uint32_t ADJUSTMENT_REPEAT_INTERVAL_MS = 500;

template<typename T>
bool adjustValue(T& value, int delta, T minValue, T maxValue)
{
  const T next = std::clamp<T>((T)(value + delta), minValue, maxValue);
  if(next == value) return false;
  value = next;
  return true;
}

template<typename T>
bool adjustPair(T& left, T& right, int delta, T minValue, T maxValue)
{
  bool changed = adjustValue(left, delta, minValue, maxValue);
  changed = adjustValue(right, delta, minValue, maxValue) || changed;
  return changed;
}

static uint16_t stepToUs(uint8_t step)
{
  return (uint16_t)(step * 25u + 900u);
}

static bool isConditionConfigured(const Espfc::ActuatorCondition& c)
{
  return c.min < c.max && c.ch >= Espfc::AXIS_AUX_1 && c.ch < Espfc::AXIS_COUNT && c.id < Espfc::MODE_COUNT;
}

static bool isConditionInRange(const Espfc::ActuatorCondition& c, const Espfc::ModelState& state)
{
  if(!isConditionConfigured(c)) return false;
  const int16_t val = state.input.us[c.ch];
  return val > c.min && val < c.max;
}

}

namespace Espfc::Control {

Actuator::Actuator(Model& model): _model(model) {}

int Actuator::begin()
{
  _model.state.mode.mask = 0;
  _model.state.mode.maskPrev = 0;
  _model.state.mode.maskPresent = 0;
  _model.state.mode.maskSwitch = 0;
  for(size_t i = 0; i < ACTUATOR_CONDITIONS; i++)
  {
    const auto &c = _model.config.conditions[i];
    if(!(c.min < c.max)) continue; // inactive
    if(c.ch < AXIS_AUX_1 || c.ch >= AXIS_COUNT) continue; // invalid channel
    if(c.id >= MODE_COUNT) continue;
    _model.state.mode.maskPresent |= 1u << c.id;
  }
  _model.state.mode.airmodeAllowed = false;
  _model.state.mode.rescueConfigMode = RESCUE_CONFIG_PENDING;
  return 1;
}

int Actuator::update()
{
  uint32_t startTime = micros();
  Utils::Stats::Measure(_model.state.stats, COUNTER_ACTUATOR);
  updateArmingDisabled();
  updateModeMask();
  updateArmed();
  updateAirMode();
  updateAdjustments();
  updateScaler();
  updateBuzzer();
  updateDynLpf();
  updateRescueConfig();
  updateLed();

  if(_model.config.debug.mode == DEBUG_PIDLOOP)
  {
    _model.state.debug[4] = micros() - startTime;
  }

  return 1;
}

void Actuator::updateAdjustments()
{
  const uint32_t nowMs = millis();
  const int16_t midRc = _model.config.input.midRc;
  const auto& input = _model.state.input;

  for(size_t i = 0; i < ADJUSTMENT_RANGES_COUNT; i++)
  {
    const auto& cfg = _model.config.adjustmentRanges[i];
    if(cfg.adjustmentFunction == ADJUSTMENT_FUNCTION_NONE || cfg.rangeStartStep == cfg.rangeEndStep)
    {
      if(cfg.stateIndex < 4 && cfg.adjustmentFunction == ADJUSTMENT_FUNCTION_SLIDER_MASTER_MULTIPLIER)
      {
        _sliderMasterSnapshot[cfg.stateIndex].valid = false;
      }
      _adjustmentPosition[i] = 0;
      continue;
    }

    const size_t rangeChannel = AXIS_AUX_1 + cfg.auxChannelIndex;
    const size_t adjustChannel = AXIS_AUX_1 + cfg.auxSwitchChannelIndex;
    if(rangeChannel >= AXIS_COUNT || adjustChannel >= AXIS_COUNT)
    {
      if(cfg.stateIndex < 4 && cfg.adjustmentFunction == ADJUSTMENT_FUNCTION_SLIDER_MASTER_MULTIPLIER)
      {
        _sliderMasterSnapshot[cfg.stateIndex].valid = false;
      }
      _adjustmentPosition[i] = 0;
      continue;
    }

    const uint16_t rangeStartUs = stepToUs(cfg.rangeStartStep);
    const uint16_t rangeEndUs = stepToUs(cfg.rangeEndStep);
    const int16_t rangeValueUs = input.us[rangeChannel];
    if(!(rangeValueUs > (int16_t)rangeStartUs && rangeValueUs < (int16_t)rangeEndUs))
    {
      if(cfg.stateIndex < 4 && cfg.adjustmentFunction == ADJUSTMENT_FUNCTION_SLIDER_MASTER_MULTIPLIER)
      {
        _sliderMasterSnapshot[cfg.stateIndex].valid = false;
      }
      _adjustmentPosition[i] = 0;
      continue;
    }

    const int16_t adjustValueUs = input.us[adjustChannel];
    const int8_t position = adjustValueUs > midRc + 200 ? 1 : (adjustValueUs < midRc - 200 ? -1 : 0);

    bool changed = false;
    const bool absoluteMode = cfg.adjustmentCenter != 0 || cfg.adjustmentScale != 0;
    if(absoluteMode)
    {
      changed = applyAdjustmentAbsolute(cfg.stateIndex, cfg.adjustmentFunction, input.ch[adjustChannel], cfg.adjustmentCenter, cfg.adjustmentScale);
      _adjustmentPosition[i] = position;
    }
    else if(cfg.adjustmentFunction == ADJUSTMENT_FUNCTION_OSD_PROFILE || cfg.adjustmentFunction == ADJUSTMENT_FUNCTION_LED_PROFILE || cfg.adjustmentFunction == ADJUSTMENT_FUNCTION_RATE_PROFILE)
    {
      if(position != _adjustmentPosition[i])
      {
        changed = applyAdjustmentSelection(cfg.adjustmentFunction, position);
        _adjustmentPosition[i] = position;
      }
    }
    else if(position != 0)
    {
      if(position != _adjustmentPosition[i] || nowMs - _adjustmentRepeatMs[i] >= ADJUSTMENT_REPEAT_INTERVAL_MS)
      {
        changed = applyAdjustmentStep(cfg.adjustmentFunction, position);
        _adjustmentRepeatMs[i] = nowMs;
        _adjustmentPosition[i] = position;
      }
    }
    else
    {
      _adjustmentPosition[i] = 0;
    }

    if(changed)
    {
      _model.syncActiveRateProfile();
      _model.state.tuningUpdatePending = true;
    }
  }
}

bool Actuator::applyAdjustmentStep(uint8_t function, int8_t direction)
{
  switch(function)
  {
    case ADJUSTMENT_FUNCTION_RC_RATE:
      return adjustPair(_model.config.input.rate[AXIS_ROLL], _model.config.input.rate[AXIS_PITCH], direction, (uint8_t)1, (uint8_t)255);
    case ADJUSTMENT_FUNCTION_RC_EXPO:
      return adjustPair(_model.config.input.expo[AXIS_ROLL], _model.config.input.expo[AXIS_PITCH], direction, (uint8_t)0, (uint8_t)100);
    case ADJUSTMENT_FUNCTION_THROTTLE_EXPO:
      return adjustValue(_model.config.input.throttleExpo, direction, (uint8_t)0, (uint8_t)100);
    case ADJUSTMENT_FUNCTION_PITCH_ROLL_RATE:
      return adjustPair(_model.config.input.superRate[AXIS_ROLL], _model.config.input.superRate[AXIS_PITCH], direction, (uint8_t)0, (uint8_t)255);
    case ADJUSTMENT_FUNCTION_YAW_RATE:
      return adjustValue(_model.config.input.superRate[AXIS_YAW], direction, (uint8_t)0, (uint8_t)255);
    case ADJUSTMENT_FUNCTION_PITCH_ROLL_P:
      return adjustPair(_model.config.pid[FC_PID_ROLL].P, _model.config.pid[FC_PID_PITCH].P, direction, (uint8_t)0, (uint8_t)255);
    case ADJUSTMENT_FUNCTION_PITCH_ROLL_I:
      return adjustPair(_model.config.pid[FC_PID_ROLL].I, _model.config.pid[FC_PID_PITCH].I, direction, (uint8_t)0, (uint8_t)255);
    case ADJUSTMENT_FUNCTION_PITCH_ROLL_D:
      return adjustPair(_model.config.pid[FC_PID_ROLL].D, _model.config.pid[FC_PID_PITCH].D, direction, (uint8_t)0, (uint8_t)255);
    case ADJUSTMENT_FUNCTION_YAW_P:
      return adjustValue(_model.config.pid[FC_PID_YAW].P, direction, (uint8_t)0, (uint8_t)255);
    case ADJUSTMENT_FUNCTION_YAW_I:
      return adjustValue(_model.config.pid[FC_PID_YAW].I, direction, (uint8_t)0, (uint8_t)255);
    case ADJUSTMENT_FUNCTION_YAW_D:
      return adjustValue(_model.config.pid[FC_PID_YAW].D, direction, (uint8_t)0, (uint8_t)255);
    case ADJUSTMENT_FUNCTION_PITCH_RATE:
      return adjustValue(_model.config.input.superRate[AXIS_PITCH], direction, (uint8_t)0, (uint8_t)255);
    case ADJUSTMENT_FUNCTION_ROLL_RATE:
      return adjustValue(_model.config.input.superRate[AXIS_ROLL], direction, (uint8_t)0, (uint8_t)255);
    case ADJUSTMENT_FUNCTION_PITCH_P:
      return adjustValue(_model.config.pid[FC_PID_PITCH].P, direction, (uint8_t)0, (uint8_t)255);
    case ADJUSTMENT_FUNCTION_PITCH_I:
      return adjustValue(_model.config.pid[FC_PID_PITCH].I, direction, (uint8_t)0, (uint8_t)255);
    case ADJUSTMENT_FUNCTION_PITCH_D:
      return adjustValue(_model.config.pid[FC_PID_PITCH].D, direction, (uint8_t)0, (uint8_t)255);
    case ADJUSTMENT_FUNCTION_ROLL_P:
      return adjustValue(_model.config.pid[FC_PID_ROLL].P, direction, (uint8_t)0, (uint8_t)255);
    case ADJUSTMENT_FUNCTION_ROLL_I:
      return adjustValue(_model.config.pid[FC_PID_ROLL].I, direction, (uint8_t)0, (uint8_t)255);
    case ADJUSTMENT_FUNCTION_ROLL_D:
      return adjustValue(_model.config.pid[FC_PID_ROLL].D, direction, (uint8_t)0, (uint8_t)255);
    case ADJUSTMENT_FUNCTION_RC_RATE_YAW:
      return adjustValue(_model.config.input.rate[AXIS_YAW], direction, (uint8_t)1, (uint8_t)255);
    case ADJUSTMENT_FUNCTION_PITCH_ROLL_F:
      return adjustPair(_model.config.pid[FC_PID_ROLL].F, _model.config.pid[FC_PID_PITCH].F, direction, (int16_t)0, (int16_t)2000);
    case ADJUSTMENT_FUNCTION_FEEDFORWARD_TRANSITION:
      return adjustValue(_model.config.dterm.feedForwardTransition, direction, (uint8_t)0, (uint8_t)100);
    case ADJUSTMENT_FUNCTION_HORIZON_STRENGTH:
      return adjustValue(_model.config.level.horizonStrength, direction, (uint8_t)0, (uint8_t)200);
    case ADJUSTMENT_FUNCTION_PITCH_F:
      return adjustValue(_model.config.pid[FC_PID_PITCH].F, direction, (int16_t)0, (int16_t)2000);
    case ADJUSTMENT_FUNCTION_ROLL_F:
      return adjustValue(_model.config.pid[FC_PID_ROLL].F, direction, (int16_t)0, (int16_t)2000);
    case ADJUSTMENT_FUNCTION_YAW_F:
      return adjustValue(_model.config.pid[FC_PID_YAW].F, direction, (int16_t)0, (int16_t)2000);
    default:
      return false;
  }
}

bool Actuator::applyAdjustmentSelection(uint8_t function, int8_t position)
{
  switch(function)
  {
    case ADJUSTMENT_FUNCTION_OSD_PROFILE:
    {
      const uint8_t selected = std::clamp<int>(position + 2, 1, _model.config.osd.profileCount);
      if(selected == _model.config.osd.profile) return false;
      _model.config.osd.profile = selected;
      return true;
    }
    case ADJUSTMENT_FUNCTION_RATE_PROFILE:
    {
      const uint8_t selected = std::clamp<int>(position + 1, 0, RATE_PROFILE_COUNT - 1);
      return _model.selectRateProfile(selected);
    }
    case ADJUSTMENT_FUNCTION_LED_PROFILE:
    {
      const uint8_t selected = std::clamp<int>(position + 1, 0, 2);
      if(selected == _model.config.ledStrip.profile) return false;
      _model.config.ledStrip.profile = selected;
      return true;
    }
    default:
      return false;
  }
}

bool Actuator::applyAdjustmentAbsolute(uint8_t stateIndex, uint8_t function, float input, int16_t center, int16_t scale)
{
  const int value = lrintf((float)center + input * (float)scale);

  switch(function)
  {
    case ADJUSTMENT_FUNCTION_RC_RATE:
      return adjustPair(_model.config.input.rate[AXIS_ROLL], _model.config.input.rate[AXIS_PITCH], 0, (uint8_t)std::clamp(value, 1, 255), (uint8_t)std::clamp(value, 1, 255));
    case ADJUSTMENT_FUNCTION_RC_EXPO:
      return adjustPair(_model.config.input.expo[AXIS_ROLL], _model.config.input.expo[AXIS_PITCH], 0, (uint8_t)std::clamp(value, 0, 100), (uint8_t)std::clamp(value, 0, 100));
    case ADJUSTMENT_FUNCTION_THROTTLE_EXPO:
      return adjustValue(_model.config.input.throttleExpo, 0, (uint8_t)std::clamp(value, 0, 100), (uint8_t)std::clamp(value, 0, 100));
    case ADJUSTMENT_FUNCTION_PITCH_ROLL_RATE:
      return adjustPair(_model.config.input.superRate[AXIS_ROLL], _model.config.input.superRate[AXIS_PITCH], 0, (uint8_t)std::clamp(value, 0, 255), (uint8_t)std::clamp(value, 0, 255));
    case ADJUSTMENT_FUNCTION_YAW_RATE:
      return adjustValue(_model.config.input.superRate[AXIS_YAW], 0, (uint8_t)std::clamp(value, 0, 255), (uint8_t)std::clamp(value, 0, 255));
    case ADJUSTMENT_FUNCTION_PITCH_ROLL_P:
      return adjustPair(_model.config.pid[FC_PID_ROLL].P, _model.config.pid[FC_PID_PITCH].P, 0, (uint8_t)std::clamp(value, 0, 255), (uint8_t)std::clamp(value, 0, 255));
    case ADJUSTMENT_FUNCTION_PITCH_ROLL_I:
      return adjustPair(_model.config.pid[FC_PID_ROLL].I, _model.config.pid[FC_PID_PITCH].I, 0, (uint8_t)std::clamp(value, 0, 255), (uint8_t)std::clamp(value, 0, 255));
    case ADJUSTMENT_FUNCTION_PITCH_ROLL_D:
      return adjustPair(_model.config.pid[FC_PID_ROLL].D, _model.config.pid[FC_PID_PITCH].D, 0, (uint8_t)std::clamp(value, 0, 255), (uint8_t)std::clamp(value, 0, 255));
    case ADJUSTMENT_FUNCTION_YAW_P:
      return adjustValue(_model.config.pid[FC_PID_YAW].P, 0, (uint8_t)std::clamp(value, 0, 255), (uint8_t)std::clamp(value, 0, 255));
    case ADJUSTMENT_FUNCTION_YAW_I:
      return adjustValue(_model.config.pid[FC_PID_YAW].I, 0, (uint8_t)std::clamp(value, 0, 255), (uint8_t)std::clamp(value, 0, 255));
    case ADJUSTMENT_FUNCTION_YAW_D:
      return adjustValue(_model.config.pid[FC_PID_YAW].D, 0, (uint8_t)std::clamp(value, 0, 255), (uint8_t)std::clamp(value, 0, 255));
    case ADJUSTMENT_FUNCTION_PITCH_RATE:
      return adjustValue(_model.config.input.superRate[AXIS_PITCH], 0, (uint8_t)std::clamp(value, 0, 255), (uint8_t)std::clamp(value, 0, 255));
    case ADJUSTMENT_FUNCTION_ROLL_RATE:
      return adjustValue(_model.config.input.superRate[AXIS_ROLL], 0, (uint8_t)std::clamp(value, 0, 255), (uint8_t)std::clamp(value, 0, 255));
    case ADJUSTMENT_FUNCTION_PITCH_P:
      return adjustValue(_model.config.pid[FC_PID_PITCH].P, 0, (uint8_t)std::clamp(value, 0, 255), (uint8_t)std::clamp(value, 0, 255));
    case ADJUSTMENT_FUNCTION_PITCH_I:
      return adjustValue(_model.config.pid[FC_PID_PITCH].I, 0, (uint8_t)std::clamp(value, 0, 255), (uint8_t)std::clamp(value, 0, 255));
    case ADJUSTMENT_FUNCTION_PITCH_D:
      return adjustValue(_model.config.pid[FC_PID_PITCH].D, 0, (uint8_t)std::clamp(value, 0, 255), (uint8_t)std::clamp(value, 0, 255));
    case ADJUSTMENT_FUNCTION_ROLL_P:
      return adjustValue(_model.config.pid[FC_PID_ROLL].P, 0, (uint8_t)std::clamp(value, 0, 255), (uint8_t)std::clamp(value, 0, 255));
    case ADJUSTMENT_FUNCTION_ROLL_I:
      return adjustValue(_model.config.pid[FC_PID_ROLL].I, 0, (uint8_t)std::clamp(value, 0, 255), (uint8_t)std::clamp(value, 0, 255));
    case ADJUSTMENT_FUNCTION_ROLL_D:
      return adjustValue(_model.config.pid[FC_PID_ROLL].D, 0, (uint8_t)std::clamp(value, 0, 255), (uint8_t)std::clamp(value, 0, 255));
    case ADJUSTMENT_FUNCTION_RC_RATE_YAW:
      return adjustValue(_model.config.input.rate[AXIS_YAW], 0, (uint8_t)std::clamp(value, 1, 255), (uint8_t)std::clamp(value, 1, 255));
    case ADJUSTMENT_FUNCTION_PITCH_ROLL_F:
      return adjustPair(_model.config.pid[FC_PID_ROLL].F, _model.config.pid[FC_PID_PITCH].F, 0, (int16_t)std::clamp(value, 0, 2000), (int16_t)std::clamp(value, 0, 2000));
    case ADJUSTMENT_FUNCTION_FEEDFORWARD_TRANSITION:
      return adjustValue(_model.config.dterm.feedForwardTransition, 0, (uint8_t)std::clamp(value, 0, 100), (uint8_t)std::clamp(value, 0, 100));
    case ADJUSTMENT_FUNCTION_HORIZON_STRENGTH:
      return adjustValue(_model.config.level.horizonStrength, 0, (uint8_t)std::clamp(value, 0, 200), (uint8_t)std::clamp(value, 0, 200));
    case ADJUSTMENT_FUNCTION_PITCH_F:
      return adjustValue(_model.config.pid[FC_PID_PITCH].F, 0, (int16_t)std::clamp(value, 0, 2000), (int16_t)std::clamp(value, 0, 2000));
    case ADJUSTMENT_FUNCTION_ROLL_F:
      return adjustValue(_model.config.pid[FC_PID_ROLL].F, 0, (int16_t)std::clamp(value, 0, 2000), (int16_t)std::clamp(value, 0, 2000));
    case ADJUSTMENT_FUNCTION_YAW_F:
      return adjustValue(_model.config.pid[FC_PID_YAW].F, 0, (int16_t)std::clamp(value, 0, 2000), (int16_t)std::clamp(value, 0, 2000));
    case ADJUSTMENT_FUNCTION_SLIDER_MASTER_MULTIPLIER:
    {
      if(stateIndex >= 4) return false;
      auto& snapshot = _sliderMasterSnapshot[stateIndex];
      if(!snapshot.valid)
      {
        for(size_t axis = 0; axis < AXIS_COUNT_RPY; axis++)
        {
          snapshot.rate[axis] = _model.config.input.rate[axis];
          snapshot.superRate[axis] = _model.config.input.superRate[axis];
          snapshot.pidP[axis] = _model.config.pid[axis].P;
          snapshot.pidI[axis] = _model.config.pid[axis].I;
          snapshot.pidD[axis] = _model.config.pid[axis].D;
          snapshot.pidF[axis] = _model.config.pid[axis].F;
        }
        snapshot.throttleExpo = _model.config.input.throttleExpo;
        snapshot.feedForwardTransition = _model.config.dterm.feedForwardTransition;
        snapshot.horizonStrength = _model.config.level.horizonStrength;
        snapshot.valid = true;
      }

      const int effectiveCenter = center != 0 ? center : 100;
      const int effectiveScale = scale != 0 ? std::abs(scale) : 125;
      const int multiplier = (int)std::clamp<long>(lrintf((float)effectiveCenter + input * (float)effectiveScale), 20l, 200l);
      bool changed = false;
      for(size_t axis = 0; axis < AXIS_COUNT_RPY; axis++)
      {
        changed = adjustValue(_model.config.input.rate[axis], 0, (uint8_t)std::clamp(lrintf(snapshot.rate[axis] * multiplier * 0.01f), 1l, 255l), (uint8_t)std::clamp(lrintf(snapshot.rate[axis] * multiplier * 0.01f), 1l, 255l)) || changed;
        changed = adjustValue(_model.config.input.superRate[axis], 0, (uint8_t)std::clamp(lrintf(snapshot.superRate[axis] * multiplier * 0.01f), 0l, 255l), (uint8_t)std::clamp(lrintf(snapshot.superRate[axis] * multiplier * 0.01f), 0l, 255l)) || changed;
        changed = adjustValue(_model.config.pid[axis].P, 0, (uint8_t)std::clamp(lrintf(snapshot.pidP[axis] * multiplier * 0.01f), 0l, 255l), (uint8_t)std::clamp(lrintf(snapshot.pidP[axis] * multiplier * 0.01f), 0l, 255l)) || changed;
        changed = adjustValue(_model.config.pid[axis].I, 0, (uint8_t)std::clamp(lrintf(snapshot.pidI[axis] * multiplier * 0.01f), 0l, 255l), (uint8_t)std::clamp(lrintf(snapshot.pidI[axis] * multiplier * 0.01f), 0l, 255l)) || changed;
        changed = adjustValue(_model.config.pid[axis].D, 0, (uint8_t)std::clamp(lrintf(snapshot.pidD[axis] * multiplier * 0.01f), 0l, 255l), (uint8_t)std::clamp(lrintf(snapshot.pidD[axis] * multiplier * 0.01f), 0l, 255l)) || changed;
        changed = adjustValue(_model.config.pid[axis].F, 0, (int16_t)std::clamp(lrintf(snapshot.pidF[axis] * multiplier * 0.01f), 0l, 2000l), (int16_t)std::clamp(lrintf(snapshot.pidF[axis] * multiplier * 0.01f), 0l, 2000l)) || changed;
      }
      changed = adjustValue(_model.config.input.throttleExpo, 0, (uint8_t)std::clamp(lrintf(snapshot.throttleExpo * multiplier * 0.01f), 0l, 100l), (uint8_t)std::clamp(lrintf(snapshot.throttleExpo * multiplier * 0.01f), 0l, 100l)) || changed;
      changed = adjustValue(_model.config.dterm.feedForwardTransition, 0, (uint8_t)std::clamp(lrintf(snapshot.feedForwardTransition * multiplier * 0.01f), 0l, 100l), (uint8_t)std::clamp(lrintf(snapshot.feedForwardTransition * multiplier * 0.01f), 0l, 100l)) || changed;
      changed = adjustValue(_model.config.level.horizonStrength, 0, (uint8_t)std::clamp(lrintf(snapshot.horizonStrength * multiplier * 0.01f), 0l, 200l), (uint8_t)std::clamp(lrintf(snapshot.horizonStrength * multiplier * 0.01f), 0l, 200l)) || changed;
      return changed;
    }
    default:
      return false;
  }
}

void Actuator::updateScaler()
{
  for(size_t i = 0; i < SCALER_COUNT; i++)
  {
    uint32_t mode = _model.config.scaler[i].dimension;
    if(!mode) continue;

    short c = _model.config.scaler[i].channel;
    if(c < AXIS_AUX_1) continue;

    float v = _model.state.input.ch[c];
    float min = _model.config.scaler[i].minScale * 0.01f;
    float max = _model.config.scaler[i].maxScale * 0.01f;
    float scale = Utils::map3(v, -1.f, 0.f, 1.f, min, min < 0 ? 0.f : 1.f, max);
    for(size_t x = 0; x < AXIS_COUNT_RPYT; x++)
    {
      if(
        (x == AXIS_ROLL   && (mode & ACT_AXIS_ROLL))  ||
        (x == AXIS_PITCH  && (mode & ACT_AXIS_PITCH)) ||
        (x == AXIS_YAW    && (mode & ACT_AXIS_YAW))   ||
        (x == AXIS_THRUST && (mode & ACT_AXIS_THRUST))
      )
      {

        if(mode & ACT_INNER_P) _model.state.innerPid[x].pScale = scale;
        if(mode & ACT_INNER_I) _model.state.innerPid[x].iScale = scale;
        if(mode & ACT_INNER_D) _model.state.innerPid[x].dScale = scale;
        if(mode & ACT_INNER_F) _model.state.innerPid[x].fScale = scale;

        if(mode & ACT_OUTER_P) _model.state.outerPid[x].pScale = scale;
        if(mode & ACT_OUTER_I) _model.state.outerPid[x].iScale = scale;
        if(mode & ACT_OUTER_D) _model.state.outerPid[x].dScale = scale;
        if(mode & ACT_OUTER_F) _model.state.outerPid[x].fScale = scale;

      }
    }
  }
}

void Actuator::updateArmingDisabled()
{
  int errors = _model.state.i2cErrorDelta;
  _model.state.i2cErrorDelta = 0;

  bool gyroBusI2c = false;
  if(_model.state.gyro.dev && _model.state.gyro.dev->getBus())
  {
    gyroBusI2c = _model.state.gyro.dev->getBus()->getType() == BUS_I2C;
  }

  const bool gyroCommError = errors && (gyroBusI2c || !_model.state.gyro.present);

  _model.setArmingDisabled(ARMING_DISABLED_NO_GYRO,        !_model.state.gyro.present || gyroCommError);
  _model.setArmingDisabled(ARMING_DISABLED_FAILSAFE,        _model.state.failsafe.phase != FC_FAILSAFE_IDLE);
  _model.setArmingDisabled(ARMING_DISABLED_RX_FAILSAFE,     _model.state.input.rxLoss || _model.state.input.rxFailSafe);
  _model.setArmingDisabled(ARMING_DISABLED_THROTTLE,       !_model.isThrottleLow());
  _model.setArmingDisabled(ARMING_DISABLED_CALIBRATING,     _model.calibrationActive());
  _model.setArmingDisabled(ARMING_DISABLED_MOTOR_PROTOCOL,  _model.config.output.protocol == ESC_PROTOCOL_DISABLED);
  _model.setArmingDisabled(ARMING_DISABLED_REBOOT_REQUIRED, _model.state.mode.rescueConfigMode == RESCUE_CONFIG_ACTIVE);
  _model.setArmingDisabled(ARMING_DISABLED_NOPREARM,
    (_model.state.mode.maskPresent & (1 << MODE_PREARM)) && !_model.isSwitchActive(MODE_PREARM));
  _model.setArmingDisabled(ARMING_DISABLED_PARALYZE, _model.isSwitchActive(MODE_PARALYZE));

  // Check small angle - prevent arming if tilted beyond configured angle
  if(_model.config.arming.smallAngle < 180.0f && _model.accelActive())
  {
    const float maxTiltRad = Utils::toRad(_model.config.arming.smallAngle);
    const float roll = _model.state.attitude.euler[AXIS_ROLL];
    const float pitch = _model.state.attitude.euler[AXIS_PITCH];
    const float currentTilt = std::max(std::fabs(roll), std::fabs(pitch));
    _model.setArmingDisabled(ARMING_DISABLED_ANGLE, currentTilt > maxTiltRad);
  }
  else
  {
    _model.setArmingDisabled(ARMING_DISABLED_ANGLE, false);
  }
  if(_model.isFeatureActive(FEATURE_GPS))
  {
    _model.setArmingDisabled(ARMING_DISABLED_GPS, !_model.state.gps.present || _model.state.gps.numSats < _model.config.gps.minSats);
  }
}

void Actuator::updateModeMask()
{
  bool rangeActive[ACTUATOR_CONDITIONS] = { false };
  bool conditionActive[ACTUATOR_CONDITIONS] = { false };

  for(size_t i = 0; i < ACTUATOR_CONDITIONS; i++)
  {
    const ActuatorCondition& c = _model.config.conditions[i];
    rangeActive[i] = isConditionInRange(c, _model.state);
  }

  // Apply Betaflight-style linked mode logic from MSP_MODE_RANGES_EXTRA.
  // logicMode: 0 = direct range match, 1 = range match AND linked condition active.
  for(size_t i = 0; i < ACTUATOR_CONDITIONS; i++)
  {
    const ActuatorCondition& c = _model.config.conditions[i];
    if(!isConditionConfigured(c)) continue;
    if(c.logicMode == 1 && c.linkId < ACTUATOR_CONDITIONS && c.linkId != i)
    {
      conditionActive[i] = rangeActive[i] && rangeActive[c.linkId];
    }
    else
    {
      conditionActive[i] = rangeActive[i];
    }
  }

  uint32_t newMask = 0;
  for(size_t i = 0; i < ACTUATOR_CONDITIONS; i++)
  {
    const ActuatorCondition& c = _model.config.conditions[i];
    if(!conditionActive[i]) continue;
    newMask |= 1u << c.id;
  }

  _model.updateSwitchActive(newMask);

  _model.setArmingDisabled(ARMING_DISABLED_FAILSAFE,    _model.state.failsafe.phase != FC_FAILSAFE_IDLE);
  _model.setArmingDisabled(ARMING_DISABLED_BOXFAILSAFE, _model.isSwitchActive(MODE_FAILSAFE));
  _model.setArmingDisabled(ARMING_DISABLED_ARM_SWITCH,  _model.armingDisabled() && _model.isSwitchActive(MODE_ARMED));

  const uint32_t failsafeBit = (1u << MODE_FAILSAFE);
  if(_model.state.failsafe.phase != FC_FAILSAFE_IDLE)
  {
    newMask |= failsafeBit;
  }

  // Only modes transitioning from OFF->ON need activation gate checks.
  uint32_t activating = newMask & ~_model.state.mode.mask;
  while(activating)
  {
    const uint32_t bit = activating & (~activating + 1u);
    const uint8_t mode = (uint8_t)__builtin_ctz(activating);
    if(!canActivateMode((FlightMode)mode))
    {
      newMask &= ~bit; // block activation, clear bit
    }
    activating &= ~bit;
  }

  _model.updateModes(newMask);
}

bool Actuator::canActivateMode(FlightMode mode)
{
  switch(mode)
  {
    case MODE_ARMED:
      return !_model.armingDisabled()
        && _model.isThrottleLow()
        && !_model.state.input.rxLoss
        && !_model.state.input.rxFailSafe
        && _model.state.input.channelsValid
        && _model.state.input.frameCount >= 5;
    case MODE_ANGLE:
    case MODE_HORIZON:
      return _model.accelActive();
    case MODE_AIRMODE:
      return _model.state.mode.airmodeAllowed;
    case MODE_ALTHOLD:
      return _model.state.baro.dev;
    case MODE_MAG:
      return _model.magActive();
    case MODE_HEADFREE:
      return _model.accelActive() && _model.magActive();
    case MODE_HEADADJ:
      return _model.magActive();
    case MODE_GPS_RESCUE:
      return _model.gpsActive() && _model.state.gps.numSats >= _model.config.gps.minSats;
    case MODE_FLIP_OVER_AFTER_CRASH:
      return _model.isModeActive(MODE_ARMED);
    default:
      return true;
  }
}

void Actuator::updateArmed()
{
  if(_model.hasChanged(MODE_ARMED))
  {
    bool armed = _model.isModeActive(MODE_ARMED);
    if(armed)
    {
      _model.state.mode.disarmReason = DISARM_REASON_SYSTEM;
      _model.state.mode.rescueConfigMode = RESCUE_CONFIG_DISABLED;
    }
    else if(!armed && _model.state.mode.disarmReason == DISARM_REASON_SYSTEM)
    {
      _model.state.mode.disarmReason = DISARM_REASON_SWITCH;
    }
    if(armed) _model.setGpsHome();
  }
}

void Actuator::updateAirMode()
{
  bool armed = _model.isModeActive(MODE_ARMED);
  if(!armed)
  {
    _model.state.mode.airmodeAllowed = false;
  }
  if(armed && !_model.state.mode.airmodeAllowed && _model.state.input.us[AXIS_THRUST] > 1400) // activate airmode in the air
  {
    _model.state.mode.airmodeAllowed = true;
  }
}

void Actuator::updateBuzzer()
{
  if(_model.isModeActive(MODE_FAILSAFE))
  {
    _model.state.buzzer.play(BUZZER_RX_LOST);
  }
  if(_model.state.battery.warn(_model.config.vbat.cellWarning))
  {
    _model.state.buzzer.play(BUZZER_BAT_LOW);
  }
  if(_model.isModeActive(MODE_BUZZER))
  {
    _model.state.buzzer.play(BUZZER_RX_SET);
  }
  if((_model.hasChanged(MODE_ARMED)))
  {
    if(_model.isModeActive(MODE_ARMED))
    {
      BuzzerEvent event = BUZZER_ARMING;
      if(_model.isFeatureActive(FEATURE_GPS))
      {
        event = _model.state.gps.fix ? BUZZER_ARMING_GPS_FIX : BUZZER_ARMING_GPS_NO_FIX;
      }
      _model.state.buzzer.push(event);
    }
    else
    {
      _model.state.buzzer.push(BUZZER_DISARMING);
    }
  }
  if(!_model.state.gps.wasLocked && _model.state.gps.numSats >= _model.config.gps.minSats)
  {
    _model.state.buzzer.play(BUZZER_READY_BEEP);
    _model.state.gps.wasLocked = true;
  }
}

void Actuator::updateDynLpf()
{
  return; // temporary disable
  int scale = Utils::clamp((int)_model.state.input.us[AXIS_THRUST], 1000, 2000);
  if(_model.config.gyro.dynLpfFilter.cutoff > 0) {
    int gyroFreq = Utils::map(scale, 1000, 2000, _model.config.gyro.dynLpfFilter.cutoff, _model.config.gyro.dynLpfFilter.freq);
    for(size_t i = 0; i < AXIS_COUNT_RPY; i++) {
      _model.state.gyro.filter[i].reconfigure(gyroFreq);
    }
  }
  if(_model.config.dterm.dynLpfFilter.cutoff > 0) {
    int dtermFreq = Utils::map(scale, 1000, 2000, _model.config.dterm.dynLpfFilter.cutoff, _model.config.dterm.dynLpfFilter.freq);
    for(size_t i = 0; i < AXIS_COUNT_RPY; i++) {
      _model.state.innerPid[i].dtermFilter.reconfigure(dtermFreq);
    }
  }
}

void Actuator::updateRescueConfig()
{
  switch(_model.state.mode.rescueConfigMode)
  {
    case RESCUE_CONFIG_PENDING:
      // if some rc frames are received, disable to prevent activate later
      if(_model.state.input.frameCount > 100)
      {
        _model.state.mode.rescueConfigMode = RESCUE_CONFIG_DISABLED;
      }
      if(_model.state.failsafe.phase != FC_FAILSAFE_IDLE && _model.config.rescueConfigDelay > 0 && millis() > _model.config.rescueConfigDelay * 1000)
      {
        _model.state.mode.rescueConfigMode = RESCUE_CONFIG_ACTIVE;
      }
      break;
    case RESCUE_CONFIG_ACTIVE:
    case RESCUE_CONFIG_DISABLED:
      // nothing to do here
      break;
  }
}

void Actuator::updateLed()
{
  const bool gyroReady = _model.state.gyro.present && _model.state.gyro.dev != nullptr;

  if(!gyroReady)
  {
    static uint32_t ledSensorDiagTs = 0;
    const uint32_t now = millis();
    if(now - ledSensorDiagTs > 2000)
    {
      ledSensorDiagTs = now;
      _model.logger.info()
          .log(F("LED ERR g/a/b"))
          .log((int)_model.state.gyro.present)
          .log((int)_model.state.accel.present)
          .log((int)_model.state.baro.present)
          .log(F(" arming"))
          .log((int)_model.state.mode.armingDisabledFlags)
          .log(F(" i2c"))
          .logln((int)_model.state.i2cErrorCount);
    }
    _model.state.led.setStatus(Connect::LED_ERROR);
    return;
  }

  if(_model.isModeActive(MODE_ARMED) || _model.state.mode.isLongClickActive())
  {
    if(_model.state.mode.isLongClickActive()) _model.setGpsHome();
    _model.state.led.setStatus(Connect::LED_ON);
  }
  else
  {
    _model.state.led.setStatus(Connect::LED_OK);
  }
}

}
