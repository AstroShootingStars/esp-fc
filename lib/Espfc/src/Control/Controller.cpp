#include "Control/Controller.h"
#include "Utils/Math.hpp"
#include <algorithm>

namespace Espfc::Control {

Controller::Controller(Model& model): _model(model), _rates{}, _obstacleAvoidance(model) {}

int Controller::begin()
{
  _rates.begin(_model.config.input);
  _speedFilter.begin(FilterConfig(FILTER_BIQUAD, 10), _model.state.loopTimer.rate);
  _obstacleAvoidance.begin();

  beginInnerLoop(AXIS_ROLL);
  beginInnerLoop(AXIS_PITCH);
  beginInnerLoop(AXIS_YAW);
  beginOuterLoop(AXIS_ROLL);
  beginOuterLoop(AXIS_PITCH);
  beginAltHold();

  return 1;
}

int FAST_CODE_ATTR Controller::update()
{
  auto& state = _model.state;
  const bool robotMixer = _model.config.mixer.type == FC_MIXER_GIMBAL;

  if(_model.config.pidAdvanced.autoProfileCellCount > 0 && state.battery.cells > 0)
  {
    const int profileIndex = std::clamp(
      (int)state.battery.cells - (int)_model.config.pidAdvanced.autoProfileCellCount,
      0,
      (int)PID_PROFILE_COUNT - 1);
    if(_model.selectPidProfile((uint8_t)profileIndex))
    {
      state.tuningUpdatePending = true;
    }
  }

  if(_model.state.tuningUpdatePending)
  {
    refreshRuntimeTunings();
    state.tuningUpdatePending = false;
  }

  uint32_t startTime = 0;
  if (_model.config.debug.mode == DEBUG_PIDLOOP)
  {
    startTime = micros();
    state.debug[0] = startTime - state.loopTimer.last;
  }

  {
    Utils::Stats::Measure(state.stats, COUNTER_OUTER_PID);
    resetIterm();
    if(robotMixer) outerLoopRobot();
    else outerLoop();
  }

  {
    Utils::Stats::Measure(state.stats, COUNTER_INNER_PID);
    if(robotMixer) innerLoopRobot();
    else innerLoop();
  }

  applyLandingAssist();

  // Update obstacle avoidance and apply corrections
  _obstacleAvoidance.update();
  if(_model.config.obstacleAvoidance.enabled && state.obstacleAvoidance.active)
  {
    // Apply throttling in stop/slowdown and bypass modes.
    const uint8_t mode = _model.config.obstacleAvoidance.avoidanceMode;
    if(mode <= 2)
    {
      float correction = _obstacleAvoidance.getThrottleCorrection();
      state.output.ch[AXIS_THRUST] *= correction;
    }
  }

  if (_model.config.debug.mode == DEBUG_PIDLOOP)
  {
    state.debug[2] = micros() - startTime;
  }

  return 1;
}

void Controller::refreshRuntimeTunings()
{
  _rates.begin(_model.config.input);

  const float pidScale[] = {
    1.f,
    _model.config.mixer.type == FC_MIXER_GIMBAL ? 20.f : 1.f,
    _model.config.mixer.type == FC_MIXER_GIMBAL ? 0.2f : 1.f,
  };

  for(size_t axis = AXIS_ROLL; axis <= AXIS_YAW; axis++)
  {
    const auto& pc = _model.config.pid[axis];
    auto& pid = _model.state.innerPid[axis];
    pid.Kp = (float)pc.P * PTERM_SCALE * pidScale[axis];
    pid.Ki = (float)pc.I * ITERM_SCALE * pidScale[axis];
    pid.Kd = (float)pc.D * DTERM_SCALE * pidScale[axis];
    pid.Kf = (float)pc.F * FTERM_SCALE * pidScale[axis];
    pid.ffBoost = std::max<uint8_t>(_model.config.pidAdvanced.throttleBoost, _model.config.pidAdvanced.ffBoost44);
    pid.ffMaxRateLimit = _model.config.pidAdvanced.ffMaxRateLimit44;
    pid.ffJitterFactor = std::max<uint8_t>(
      _model.config.pidAdvanced.feedforwardSmoothness > 0
        ? (uint8_t)std::min<uint16_t>(_model.config.pidAdvanced.feedforwardSmoothness, 100)
        : 0,
      _model.config.pidAdvanced.ffJitterFactor44);
    pid.ffAveraging = _model.config.pidAdvanced.ffAveraging44 > 0
      ? _model.config.pidAdvanced.ffAveraging44
      : (uint8_t)std::min<uint16_t>(_model.config.pidAdvanced.feedforwardAveraging, 100);
    pid.ffSmoothness = _model.config.pidAdvanced.ffSmoothness44 > 0
      ? _model.config.pidAdvanced.ffSmoothness44
      : (uint8_t)std::min<uint16_t>(_model.config.pidAdvanced.feedforwardSmoothness, 100);
    pid.pLimit = (axis == AXIS_YAW && _model.config.pidAdvanced.yawPLimit > 0)
      ? (float)_model.config.pidAdvanced.yawPLimit * 0.001f
      : 0.0f;
    pid.itermRelaxMode = _model.config.pidAdvanced.itermRelaxType > 0 ? 1 : 0;
  }
}

void FAST_CODE_ATTR Controller::applyLandingAssist()
{
  const auto& lac = _model.config.landingAssist;
  auto& state = _model.state;
  if(!lac.enabled || !_model.landingAssistReady())
  {
    _landingTouchdownPending = false;
    _landingTouchdownStartMs = 0;
    return;
  }

  if(!_model.isModeActive(MODE_ARMED))
  {
    _landingTouchdownPending = false;
    _landingTouchdownStartMs = 0;
    return;
  }

  const bool lowThrottleIntent = state.input.us[AXIS_THRUST] <= (_model.config.input.minCheck + lac.throttleIntentMargin);
  const bool failsafeLandingIntent = state.failsafe.phase == FC_FAILSAFE_LANDING;
  const bool landingIntent = lowThrottleIntent || failsafeLandingIntent;

  if(!landingIntent)
  {
    _landingTouchdownPending = false;
    _landingTouchdownStartMs = 0;
    return;
  }

  float& thrustOut = state.output.ch[AXIS_THRUST];

  // Smooth descent limiter from barometer to avoid hard drop near touchdown.
  if(_model.baroActive())
  {
    const float descentRateLimit = lac.descentRateLimitCms * 0.01f;
    const float vario = state.altitude.vario;
    if(vario < descentRateLimit)
    {
      const float gain = std::clamp((float)lac.descentCorrectivePermille * 0.001f, 0.0f, 1.0f);
      const float maxCorr = std::clamp((float)lac.descentCorrectiveMaxPermille * 0.001f, 0.0f, 1.0f);
      const float corrective = std::clamp((descentRateLimit - vario) * gain, 0.0f, maxCorr);
      thrustOut = std::clamp(thrustOut + corrective, -1.0f, 1.0f);
    }
  }

  bool baroTouchdown = false;
  bool gpsTouchdown = false;
  bool flowTouchdown = false;
  bool handTouchdown = false;

  const bool gpsActive = _model.gpsActive();
  const bool flowActive = _model.opticalFlowActive();

  if(_model.baroActive())
  {
    const float hThr = lac.baroHeightThresholdCm * 0.01f;
    const float vThr = lac.baroVarioThresholdCms * 0.01f;
    baroTouchdown = std::fabs(state.altitude.height) < hThr && std::fabs(state.altitude.vario) < vThr;
  }

  if(gpsActive && state.gps.fix && state.gps.numSats >= _model.config.gps.minSats)
  {
    const int gpsDownAbs = state.gps.velocity.raw.down < 0 ? -state.gps.velocity.raw.down : state.gps.velocity.raw.down;
    const int gpsGroundAbs = state.gps.velocity.raw.groundSpeed < 0 ? -state.gps.velocity.raw.groundSpeed : state.gps.velocity.raw.groundSpeed;
    gpsTouchdown = gpsDownAbs < lac.gpsDownThresholdMms
      && gpsGroundAbs < lac.gpsGroundThresholdMms;
  }

  if(flowActive)
  {
    const float flowThr = std::max(0.0f, (float)lac.flowRateThresholdMrad * 0.001f);
    const float flowHandThr = std::max(0.0f, (float)lac.flowRateHandThresholdMrad * 0.001f);
    const float handVarioThr = std::max(0.0f, (float)lac.handVarioThresholdCms * 0.01f);
    const float handMinH = std::max(0.0f, (float)lac.handHeightMinCm * 0.01f);
    const float handMaxH = std::max(handMinH, (float)lac.handHeightMaxCm * 0.01f);

    flowTouchdown = state.opticalFlow.quality >= lac.flowQualityThreshold
      && std::fabs(state.opticalFlow.flowRateX) < flowThr
      && std::fabs(state.opticalFlow.flowRateY) < flowThr;

    handTouchdown = state.opticalFlow.quality >= lac.flowHandQualityThreshold
      && std::fabs(state.opticalFlow.flowRateX) < flowHandThr
      && std::fabs(state.opticalFlow.flowRateY) < flowHandThr
      && std::fabs(state.altitude.vario) < handVarioThr
      && state.altitude.height > handMinH
      && state.altitude.height < handMaxH;
  }

  const int touchdownVotes = (baroTouchdown ? 1 : 0) + (gpsTouchdown ? 1 : 0) + (flowTouchdown ? 1 : 0);
  const bool multiSensorTouchdown = touchdownVotes >= 2;
  const bool singleSensorFallback = baroTouchdown && !gpsActive && !flowActive;
  const bool touchdownDetected = handTouchdown || multiSensorTouchdown || singleSensorFallback;

  const uint32_t nowMs = millis();
  if(touchdownDetected)
  {
    if(!_landingTouchdownPending)
    {
      _landingTouchdownPending = true;
      _landingTouchdownStartMs = nowMs;
    }

    // Ramp down thrust smoothly while touchdown is confirmed.
    const float ramp = std::clamp((float)lac.touchdownRampPermille * 0.001f, 0.0f, 1.0f);
    thrustOut = std::max(thrustOut - ramp, -1.0f);

    if(nowMs - _landingTouchdownStartMs > (uint32_t)std::max(0, (int)lac.touchdownHoldMs))
    {
      _model.disarm(DISARM_REASON_THROTTLE_TIMEOUT);
    }
  }
  else
  {
    _landingTouchdownPending = false;
    _landingTouchdownStartMs = 0;
  }
}

void Controller::outerLoopRobot()
{
  const float speedScale = 2.f;
  const float gyroScale = 0.1f;
  const float speed = _speedFilter.update(_model.state.output.ch[AXIS_PITCH] * speedScale +
                                          _model.state.gyro.adc[AXIS_PITCH] * gyroScale);
  const auto& input = _model.state.input;
  const auto& levelConf = _model.config.level;
  const float angleLimitRad = Utils::toRad(levelConf.angleLimit);
  const float angle = input.ch[AXIS_PITCH] * angleLimitRad;
  _model.state.setpoint.angle.set(AXIS_PITCH, angle);
  _model.state.setpoint.rate[AXIS_YAW] = input.ch[AXIS_YAW] * Utils::toRad(levelConf.rateLimit);

  if (_model.config.debug.mode == DEBUG_ANGLERATE)
  {
    _model.state.debug[0] = speed * 1000;
    _model.state.debug[1] = lrintf(Utils::toDeg(angle) * 10);
  }
}

void Controller::innerLoopRobot()
{
  // VectorFloat v(0.f, 0.f, 1.f);
  // v.rotate(_model.state.attitude.quaternion);
  // const float angle = acos(v.z);

  const auto& attitude = _model.state.attitude;
  const auto& setpoint = _model.state.setpoint;

  auto& output = _model.state.output;
  auto& innerPid = _model.state.innerPid;

  const float angle = std::max(abs(attitude.euler[AXIS_PITCH]), abs(attitude.euler[AXIS_ROLL]));
  const float angleLimitRad = Utils::toRad(_model.config.level.angleLimit);
  const bool stabilize = angle < angleLimitRad;
  if (stabilize)
  {
    output.ch[AXIS_PITCH] = innerPid[AXIS_PITCH].update(setpoint.angle[AXIS_PITCH], attitude.euler[AXIS_PITCH]);
    output.ch[AXIS_YAW] = innerPid[AXIS_YAW].update(setpoint.rate[AXIS_YAW], _model.state.gyro.adc[AXIS_YAW]);
  }
  else
  {
    resetIterm();
    output.ch[AXIS_PITCH] = 0.f;
    output.ch[AXIS_YAW] = 0.f;
  }

  if (_model.config.debug.mode == DEBUG_ANGLERATE)
  {
    _model.state.debug[2] = lrintf(Utils::toDeg(attitude.euler[AXIS_PITCH]) * 10);
    _model.state.debug[3] = lrintf(output.ch[AXIS_PITCH] * 1000);
  }
}

void FAST_CODE_ATTR Controller::outerLoop()
{
  auto& state = _model.state;
  const auto& input = state.input;

  const float ffFloorBase = 1.0f - (_model.config.dterm.feedForwardTransition * 0.01f);
  const float smartFf = std::clamp((float)_model.config.pidAdvanced.smartFeedForward * 0.01f, 0.0f, 1.0f);
  const float setpointWeight = std::clamp((float)_model.config.dterm.setpointWeight * (1.0f / 255.0f), 0.0f, 1.0f);
  const float ffFloor = std::clamp(ffFloorBase + smartFf * 0.25f + setpointWeight * 0.2f, 0.0f, 1.0f);
  auto updateFeedforwardFactor = [&](size_t axis, float stickMagnitude, bool enabled) {
    const float stick = std::clamp(stickMagnitude, 0.0f, 1.0f);
    state.innerPid[axis].ffTransitionFactor = enabled ? std::max(stick, ffFloor) : 0.0f;
  };
  auto applyAcroTrainerLimit = [&](size_t axis, float command, float stickInput) {
    const uint8_t limitDeg = _model.config.pidAdvanced.acroTrainerAngleLimit;
    if(limitDeg == 0) return command;
    const float limitRad = Utils::toRad((float)limitDeg);
    const float angle = state.attitude.euler[axis];
    const bool pushingOut = (angle > 0.0f && stickInput > 0.0f) || (angle < 0.0f && stickInput < 0.0f);
    if(!pushingOut) return command;
    const float overflow = std::max(0.0f, std::fabs(angle) - limitRad);
    if(overflow <= 0.0f) return command;
    const float reduction = std::clamp(overflow / Utils::toRad(20.0f), 0.0f, 1.0f);
    return command * (1.0f - reduction);
  };

  const float angleLimitRad = Utils::toRad(_model.config.level.angleLimit);
  const float horizonStrength = _model.config.level.horizonStrength * 0.01f;
  const bool angleMode = _model.isModeActive(MODE_ANGLE);
  const bool horizonMode = _model.isModeActive(MODE_HORIZON);

  // Roll/Pitch rates control
  if (angleMode)
  {
    for (size_t i = 0; i < AXIS_COUNT_RP; i++)
    {
      const float stickInput = input.ch[i];
      const float angleSetpoint = angleLimitRad * stickInput;
      state.setpoint.rate[i] = state.outerPid[i].update(angleSetpoint, state.attitude.euler[i]);
      updateFeedforwardFactor(i, std::fabs(stickInput), false);
    }
  }
  else if (horizonMode)
  {
    for (size_t i = 0; i < AXIS_COUNT_RP; i++)
    {
      const float stickInput = input.ch[i];
      const float angleSetpoint = angleLimitRad * stickInput;
      const float angleRate = state.outerPid[i].update(angleSetpoint, state.attitude.euler[i]);
      const float acroRate = calculateSetpointRate(i, stickInput);
      const float stick = std::fabs(stickInput);
      const float angleWeight = std::clamp((1.f - std::clamp(stick, 0.f, 1.f)) * horizonStrength, 0.0f, 1.0f);
      state.setpoint.rate[i] = acroRate * (1.f - angleWeight) + angleRate * angleWeight;
      state.setpoint.rate[i] = applyAcroTrainerLimit(i, state.setpoint.rate[i], stickInput);
      updateFeedforwardFactor(i, stick, true);
    }
  }
  else
  {
    for (size_t i = 0; i < AXIS_COUNT_RP; i++)
    {
      const float stickInput = input.ch[i];
      state.setpoint.rate[i] = calculateSetpointRate(i, stickInput);
      state.setpoint.rate[i] = applyAcroTrainerLimit(i, state.setpoint.rate[i], stickInput);
      updateFeedforwardFactor(i, std::fabs(stickInput), true);
    }
  }

  // Yaw rates control
  state.setpoint.rate[AXIS_YAW] = calculateSetpointRate(AXIS_YAW, input.ch[AXIS_YAW]);
  if((angleMode || horizonMode) && _model.config.level.integratedYaw)
  {
    const float relax = std::clamp((float)_model.config.pidAdvanced.integratedYawRelax * 0.01f, 0.0f, 1.0f);
    const float stick = std::clamp(std::fabs(input.ch[AXIS_YAW]), 0.0f, 1.0f);
    state.setpoint.rate[AXIS_YAW] *= std::clamp(stick + relax, 0.0f, 1.0f);
  }
  updateFeedforwardFactor(AXIS_YAW, std::fabs(input.ch[AXIS_YAW]), true);

  const uint8_t dMinByAxis[AXIS_COUNT_RPY] = {
    _model.config.dterm.dMinRoll,
    _model.config.dterm.dMinPitch,
    _model.config.dterm.dMinYaw
  };
  const float dMinGain = std::clamp((float)_model.config.dterm.dMinGain * 0.01f, 0.0f, 2.55f);
  const float dMinAdvance = std::clamp((float)_model.config.dterm.dMinAdvance * 0.01f, 0.0f, 2.55f);
  for(size_t i = 0; i < AXIS_COUNT_RPY; i++)
  {
    const float dCurrent = std::max(1.0f, (float)_model.config.pid[i].D);
    const float dMinBase = std::clamp((float)dMinByAxis[i] / dCurrent, 0.0f, 1.0f);
    const float stickActivity = std::clamp(std::fabs(input.ch[i]), 0.0f, 1.0f);
    const float rateActivity = std::clamp(std::fabs(state.setpoint.rate[i]) / Utils::toRad(720.0f), 0.0f, 1.0f);
    const float activity = std::clamp(stickActivity + rateActivity * (0.5f + dMinAdvance * 0.25f), 0.0f, 1.0f);
    const float ramp = std::clamp(activity * (0.5f + dMinGain * 0.25f), 0.0f, 1.0f);
    state.innerPid[i].dMinFactor = dMinBase + (1.0f - dMinBase) * ramp;
  }

  // thrust control
  if (_model.isModeActive(MODE_ALTHOLD))
  {
    state.setpoint.rate[AXIS_THRUST] = calcualteAltHoldSetpoint();
  }
  else
  {
    float thrust = input.ch[AXIS_THRUST];
    const float linearization = std::clamp((float)_model.config.pidAdvanced.thrustLinearization44 * 0.01f, 0.0f, 1.0f);
    if(linearization > 0.0f)
    {
      float normalized = std::clamp((thrust + 1.0f) * 0.5f, 0.0f, 1.0f);
      normalized += linearization * (normalized - normalized * normalized);
      thrust = std::clamp(normalized * 2.0f - 1.0f, -1.0f, 1.0f);
    }
    state.setpoint.rate[AXIS_THRUST] = thrust;
  }

  // debug
  if (_model.config.debug.mode == DEBUG_ANGLERATE)
  {
    for (size_t i = 0; i < AXIS_COUNT_RPY; ++i)
    {
      state.debug[i] = lrintf(Utils::toDeg(state.setpoint.rate[i]));
    }
  }
}

void FAST_CODE_ATTR Controller::innerLoop()
{
  // Roll/Pitch/Yaw rates control
  const float tpaFactor = getTpaFactor();
  const auto& setpoint = _model.state.setpoint;
  const auto& altitude = _model.state.altitude;
  const float throttleDeltaUs = std::fabs(_model.state.input.us[AXIS_THRUST] - _prevThrottleInputUs);
  _prevThrottleInputUs = _model.state.input.us[AXIS_THRUST];
  const float itermThreshold = std::max(1.0f, (float)_model.config.pidAdvanced.itermThrottleThreshold);
  const float antiGravityGain = std::clamp((float)_model.config.level.antiGravityGain * 0.01f, 0.0f, 2.55f);
  const float antiGravityExcess = std::clamp((throttleDeltaUs - itermThreshold) / std::max(50.0f, itermThreshold), 0.0f, 1.0f);
  const float antiGravityBoost = 1.0f + antiGravityGain * antiGravityExcess;
  const uint8_t antiGravityMode = _model.config.pidAdvanced.antiGravityMode;
  const float absControlGain = std::clamp((float)_model.config.pidAdvanced.absControlGain * 0.00001f, 0.0f, 0.01f);

  auto& innerPid = _model.state.innerPid;
  auto& output = _model.state.output;

  for (size_t i = 0; i < AXIS_COUNT_RPY; ++i)
  {
    const bool applyAntiGravity = antiGravityBoost > 1.0f && (i < AXIS_COUNT_RP || antiGravityMode > 0);
    innerPid[i].iScale = applyAntiGravity ? antiGravityBoost : 1.0f;
    output.ch[i] = innerPid[i].update(setpoint.rate[i], _model.state.gyro.adc[i]) * tpaFactor;
    if(absControlGain > 0.0f)
    {
      output.ch[i] -= _model.state.gyro.adc[i] * absControlGain;
    }
    output.ch[i] = std::clamp(output.ch[i], -1.0f, 1.0f);
  }

  // thrust control
  if (_model.isModeActive(MODE_ALTHOLD))
  {
    output.ch[AXIS_THRUST] = innerPid[AXIS_THRUST].update(setpoint.rate[AXIS_THRUST], altitude.vario);
  }
  else
  {
    innerPid[AXIS_THRUST].update(0, altitude.vario);
    // follow iTerm from rc input for smooth mid-air transition
    innerPid[AXIS_THRUST].iTerm = _model.state.input.ch[AXIS_THRUST];
    output.ch[AXIS_THRUST] = setpoint.rate[AXIS_THRUST];
  }

  if (_model.config.debug.mode == DEBUG_STACK)
  {
    _model.state.debug[0] = std::clamp(lrintf(setpoint.rate[AXIS_THRUST] * 1000.0f), -3000l, 3000l);    // hi mem
    _model.state.debug[1] = std::clamp(lrintf(altitude.vario * 1000.0f), -30000l, 30000l);              // lo mem
    _model.state.debug[2] = std::clamp(lrintf(altitude.height * 100.0f), -30000l, 30000l);              // curr
    _model.state.debug[3] = std::clamp(lrintf(innerPid[AXIS_THRUST].error * 1000.0f), -30000l, 30000l); // p
    _model.state.debug[4] = std::clamp(lrintf(innerPid[AXIS_THRUST].pTerm * 1000.0f), -3000l, 3000l);
    _model.state.debug[5] = std::clamp(lrintf(innerPid[AXIS_THRUST].iTerm * 1000.0f), -3000l, 3000l);
    _model.state.debug[6] = std::clamp(lrintf(innerPid[AXIS_THRUST].dTerm * 1000.0f), -3000l, 3000l);
    _model.state.debug[7] = std::clamp(lrintf(innerPid[AXIS_THRUST].fTerm * 1000.0f), -3000l, 3000l);
  }

  // debug
  if (_model.config.debug.mode == DEBUG_ITERM_RELAX)
  {
    _model.state.debug[0] = lrintf(Utils::toDeg(innerPid[AXIS_ROLL].itermRelaxBase));
    _model.state.debug[1] = lrintf(innerPid[AXIS_ROLL].itermRelaxFactor * 100.0f);
    _model.state.debug[2] = lrintf(Utils::toDeg(innerPid[AXIS_ROLL].iTermError));
    _model.state.debug[3] = lrintf(innerPid[AXIS_ROLL].iTerm * 1000.0f);
  }
}

float Controller::calcualteAltHoldSetpoint() const
{
  const float hover = std::clamp(_model.config.input.throttleHover * 0.02f - 1.0f, -1.0f, 1.0f);
  float thrust = _model.state.input.ch[AXIS_THRUST] - hover;

  // if(_model.isThrottleLow()) thrust = 0.0f; // stick below min check, no command

  thrust = Utils::deadband(thrust, 0.1f); // +/- 12.5% deadband

  return Utils::map3(thrust, -1.f, 0.f, 1.f, -2.0f, 0.f, 4.f); // climb rate 5ms, descend rate 2 m/s
}

float Controller::getTpaFactor() const
{
  const uint8_t tpaMode = _model.config.pidAdvanced.tpaMode45;
  const uint8_t tpaRate = _model.config.pidAdvanced.tpaRate45 > 0
    ? _model.config.pidAdvanced.tpaRate45
    : _model.config.controller.tpaScale;
  const uint16_t tpaBreakpoint = _model.config.pidAdvanced.tpaBreakpoint45 > 0
    ? _model.config.pidAdvanced.tpaBreakpoint45
    : _model.config.controller.tpaBreakpoint;

  if(tpaRate == 0 || tpaMode > 1) return 1.f;

  float t = Utils::clamp(_model.state.input.us[AXIS_THRUST], (float)tpaBreakpoint, 2000.f);
  float factor = Utils::map(t, (float)tpaBreakpoint, 2000.f, 1.f,
    1.f - ((float)tpaRate * 0.01f));

  const uint8_t vbatComp = std::max<uint8_t>(_model.config.dterm.vbatPidCompensation, _model.config.pidAdvanced.vbatSagCompensation44);
  if(vbatComp > 0 && _model.state.battery.cellVoltage > 0.1f)
  {
    const float cellMax = std::max(3.0f, _model.config.vbat.cellMax * 0.01f);
    const float sag = std::clamp((cellMax - _model.state.battery.cellVoltage) / cellMax, 0.0f, 0.35f);
    factor *= 1.0f + sag * ((float)vbatComp * 0.01f);
  }

  return std::clamp(factor, 0.5f, 1.5f);
}

void Controller::resetIterm()
{
  if (!_model.isModeActive(MODE_ARMED) // when not armed
      || (!_model.isAirModeActive() && _model.config.iterm.lowThrottleZeroIterm &&
          _model.isThrottleLow()) // on low throttle (not in air mode)
  )
  {
    for (size_t i = 0; i < AXIS_COUNT_RPY; i++)
    {
      _model.state.innerPid[i].resetIterm();
      _model.state.outerPid[i].resetIterm();
    }
  }
  if (!_model.isModeActive(MODE_ARMED))
  {
    //_model.state.innerPid[AXIS_THRUST].resetIterm();
  }
}

float Controller::calculateSetpointRate(int axis, float input)
{
  if (axis == AXIS_YAW) input *= -1.f;
  const float target = _rates.getSetpoint(axis, input);

  if(axis < AXIS_ROLL || axis > AXIS_YAW)
  {
    return target;
  }

  const uint16_t accelLimit = axis == AXIS_YAW
    ? _model.config.pidAdvanced.yawRateAccelLimit
    : _model.config.pidAdvanced.rateAccelLimit;

  if(accelLimit == 0)
  {
    _setpointRatePrev[axis] = target;
    _setpointRatePrevValid[axis] = true;
    return target;
  }

  if(!_setpointRatePrevValid[axis])
  {
    _setpointRatePrev[axis] = target;
    _setpointRatePrevValid[axis] = true;
    return target;
  }

  const float maxStep = Utils::toRad((float)accelLimit) / std::max(1.0f, (float)_model.state.loopTimer.rate);
  const float delta = std::clamp(target - _setpointRatePrev[axis], -maxStep, maxStep);
  _setpointRatePrev[axis] += delta;
  return _setpointRatePrev[axis];
}

void Controller::beginInnerLoop(size_t axis)
{
  const int pidFilterRate = _model.state.loopTimer.rate;
  float pidScale[] = {1.f, 1.f, 1.f};
  if (_model.config.mixer.type == FC_MIXER_GIMBAL)
  {
    pidScale[AXIS_YAW] = 0.2f;   // ROBOT
    pidScale[AXIS_PITCH] = 20.f; // ROBOT
  }

  const auto& pc = _model.config.pid[axis];
  const auto& dtermConf = _model.config.dterm;

  auto& pid = _model.state.innerPid[axis];
  pid.Kp = (float)pc.P * PTERM_SCALE * pidScale[axis];
  pid.Ki = (float)pc.I * ITERM_SCALE * pidScale[axis];
  pid.Kd = (float)pc.D * DTERM_SCALE * pidScale[axis];
  pid.Kf = (float)pc.F * FTERM_SCALE * pidScale[axis];
  pid.iLimitLow = -_model.config.iterm.limit * 0.01f;
  pid.iLimitHigh = _model.config.iterm.limit * 0.01f;
  pid.oLimitLow = -0.66f;
  pid.oLimitHigh = 0.66f;
  pid.rate = pidFilterRate;
  pid.dtermNotchFilter.begin(dtermConf.notchFilter, pidFilterRate);
  if (dtermConf.dynLpfFilter.cutoff > 0)
  {
    pid.dtermFilter.begin(FilterConfig((FilterType)dtermConf.filter.type, dtermConf.dynLpfFilter.cutoff),
                          pidFilterRate);
  }
  else
  {
    pid.dtermFilter.begin(dtermConf.filter, pidFilterRate);
  }
  pid.dtermFilter2.begin(dtermConf.filter2, pidFilterRate);
  pid.ftermFilter.begin(_model.config.input.filterDerivative, pidFilterRate);
  pid.ffBoost = std::max<uint8_t>(_model.config.pidAdvanced.throttleBoost, _model.config.pidAdvanced.ffBoost44);
  pid.ffMaxRateLimit = _model.config.pidAdvanced.ffMaxRateLimit44;
  pid.ffJitterFactor = std::max<uint8_t>(
    _model.config.pidAdvanced.feedforwardSmoothness > 0
      ? (uint8_t)std::min<uint16_t>(_model.config.pidAdvanced.feedforwardSmoothness, 100)
      : 0,
    _model.config.pidAdvanced.ffJitterFactor44);
  pid.ffAveraging = _model.config.pidAdvanced.ffAveraging44 > 0
    ? _model.config.pidAdvanced.ffAveraging44
    : (uint8_t)std::min<uint16_t>(_model.config.pidAdvanced.feedforwardAveraging, 100);
  pid.ffSmoothness = _model.config.pidAdvanced.ffSmoothness44 > 0
    ? _model.config.pidAdvanced.ffSmoothness44
    : (uint8_t)std::min<uint16_t>(_model.config.pidAdvanced.feedforwardSmoothness, 100);
  pid.pLimit = (axis == AXIS_YAW && _model.config.pidAdvanced.yawPLimit > 0)
    ? (float)_model.config.pidAdvanced.yawPLimit * 0.001f
    : 0.0f;
  pid.itermRelaxMode = _model.config.pidAdvanced.itermRelaxType > 0 ? 1 : 0;
  pid.ffDeltaFiltered = 0.0f;
  pid.itermRelaxFilter.begin(FilterConfig(FILTER_PT1, _model.config.iterm.relaxCutoff), pidFilterRate);
  if (axis == AXIS_YAW)
  {
    pid.itermRelax = (_model.config.iterm.relax == ITERM_RELAX_RPY || _model.config.iterm.relax == ITERM_RELAX_RPY_INC)
                         ? _model.config.iterm.relax
                         : ITERM_RELAX_OFF;
    pid.ptermFilter.begin(_model.config.yaw.filter, pidFilterRate);
  }
  else
  {
    pid.itermRelax = _model.config.iterm.relax;
  }
  pid.begin();
}

void Controller::beginOuterLoop(size_t axis)
{
  const int pidFilterRate = _model.state.loopTimer.rate;
  const auto& pc = _model.config.pid[FC_PID_LEVEL];

  auto& pid = _model.state.outerPid[axis];
  pid.Kp = (float)pc.P * LEVEL_PTERM_SCALE;
  pid.Ki = (float)pc.I * LEVEL_ITERM_SCALE;
  pid.Kd = (float)pc.D * LEVEL_DTERM_SCALE;
  pid.Kf = (float)pc.F * LEVEL_FTERM_SCALE;
  pid.iLimitHigh = Utils::toRad(_model.config.level.rateLimit * 0.1f);
  pid.iLimitLow = -pid.iLimitHigh;
  pid.oLimitHigh = Utils::toRad(_model.config.level.rateLimit);
  pid.oLimitLow = -pid.oLimitHigh;
  pid.rate = pidFilterRate;
  pid.ptermFilter.begin(_model.config.level.ptermFilter, pidFilterRate);
  // pid.iLimit = 0.3f; // ROBOT
  // pid.oLimit = 1.f;  // ROBOT
  pid.begin();
}

void Controller::beginAltHold()
{
  float itermCenter = std::clamp((int)_model.config.altHold.itermCenter, 10, 60) * 0.01f;
  float itermRange = itermCenter * std::clamp((int)_model.config.altHold.itermRange, 10, 60) * 0.01f;
  const auto& pc = _model.config.pid[FC_PID_VEL];

  auto& pid = _model.state.innerPid[AXIS_THRUST];
  pid.Kp = (float)pc.P * VEL_PTERM_SCALE;
  pid.Ki = (float)pc.I * VEL_ITERM_SCALE;
  pid.Kd = (float)pc.D * VEL_DTERM_SCALE;
  pid.Kf = (float)pc.F * VEL_FTERM_SCALE;
  pid.iLimitLow = -1.0f + 2.0f * (itermCenter - itermRange);
  pid.iLimitHigh = -1.0f + 2.0f * (itermCenter + itermRange);
  pid.iReset = pid.iLimitLow;
  pid.rate = _model.state.loopTimer.rate;
  pid.dtermFilter.begin(FilterConfig(FILTER_PT1, 10), _model.state.loopTimer.rate);
  pid.ftermDerivative = false;
  pid.begin();
}

} // namespace Espfc::Control
