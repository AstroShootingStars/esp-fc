#include "Rates.h"
#include "Utils/MemoryHelper.h"

namespace Espfc::Control {

constexpr float SETPOINT_RATE_LIMIT = 1998.0f;
constexpr float RC_RATE_INCREMENTAL = 14.54f;

void Rates::begin(const InputConfig& config)
{
  _config = &config;
  rateType = (RateType)config.rateType;
}

float FAST_CODE_ATTR Rates::getSetpoint(const int axis, float input) const
{
  if(!_config) return 0.f;
  input = Utils::clamp(input, -0.995f, 0.995f); // limit input
  const float inputAbs = fabsf(input);
  const RateType rateType = (RateType)_config->rateType;
  float result = 0;
  switch (rateType)
  {
    case RATES_TYPE_BETAFLIGHT: result = betaflight(axis, input, inputAbs);
    case RATES_TYPE_RACEFLIGHT: result = raceflight(axis, input, inputAbs);
    case RATES_TYPE_KISS: result = kiss(axis, input, inputAbs);
    case RATES_TYPE_ACTUAL: result = actual(axis, input, inputAbs);
    case RATES_TYPE_QUICK: result = quick(axis, input, inputAbs);
  }
  return Utils::toRad(Utils::clamp(result, -(float)_config->rateLimit[axis], (float)_config->rateLimit[axis]));
}

float FAST_CODE_ATTR Rates::betaflight(const int axis, float rcCommandf, const float rcCommandfAbs) const
{
  if (_config->expo[axis])
  {
    const float expof = _config->expo[axis] / 100.0f;
    rcCommandf = rcCommandf * power3(rcCommandfAbs) * expof + rcCommandf * (1 - expof);
  }

  float rcRate = _config->rate[axis] / 100.0f;
  if (rcRate > 2.0f)
  {
    rcRate += RC_RATE_INCREMENTAL * (rcRate - 2.0f);
  }
  float angleRate = 200.0f * rcRate * rcCommandf;
  if (_config->superRate[axis])
  {
    const float rcSuperfactor =
        1.0f / (constrainf(1.0f - (rcCommandfAbs * (_config->superRate[axis] / 100.0f)), 0.01f, 1.00f));
    angleRate *= rcSuperfactor;
  }

  return angleRate;
}

float FAST_CODE_ATTR Rates::raceflight(const int axis, float rcCommandf, const float rcCommandfAbs) const
{
  // -1.0 to 1.0 ranged and curved
  rcCommandf = ((1.0f + 0.01f * _config->expo[axis] * (rcCommandf * rcCommandf - 1.0f)) * rcCommandf);
  // convert to -2000 to 2000 range using acro+ modifier
  float angleRate = 10.0f * _config->rate[axis] * rcCommandf;
  angleRate = angleRate * (1 + rcCommandfAbs * (float)_config->superRate[axis] * 0.01f);

  return angleRate;
}

float FAST_CODE_ATTR Rates::kiss(const int axis, float rcCommandf, const float rcCommandfAbs) const
{
    const float rcCurvef = _config->expo[axis] / 100.0f;

    float kissRpyUseRates = 1.0f / (constrainf(1.0f - (rcCommandfAbs * (_config->superRate[axis] / 100.0f)), 0.01f, 1.00f));
  float kissRcCommandf =
      (power3(rcCommandf) * rcCurvef + rcCommandf * (1 - rcCurvef)) * (_config->rate[axis] / 1000.0f);
  float kissAngle =
      constrainf(((2000.0f * kissRpyUseRates) * kissRcCommandf), -SETPOINT_RATE_LIMIT, SETPOINT_RATE_LIMIT);

  return kissAngle;
}

float FAST_CODE_ATTR Rates::actual(const int axis, float rcCommandf, const float rcCommandfAbs) const
{
  float expof = _config->expo[axis] / 100.0f;
  expof = rcCommandfAbs * (power5(rcCommandf) * expof + rcCommandf * (1 - expof));

  const float centerSensitivity = _config->rate[axis] * 10.0f;
  const float stickMovement = std::max(0.f, _config->superRate[axis] * 10.0f - centerSensitivity);
  const float angleRate = rcCommandf * centerSensitivity + stickMovement * expof;

  return angleRate;
}

float FAST_CODE_ATTR Rates::quick(const int axis, float rcCommandf, const float rcCommandfAbs) const
{
  const float rcRate = _config->rate[axis] * 2;
  const float maxDPS = std::max(_config->superRate[axis] * 10.f, rcRate);
  const float linearity = _config->expo[axis] / 100.0f;
  const float superFactorConfig = (maxDPS / rcRate - 1) / (maxDPS / rcRate);

  float curve = power3(rcCommandfAbs) * linearity + rcCommandfAbs * (1 - linearity);
  float superfactor = 1.0f / (constrainf(1.0f - (curve * superFactorConfig), 0.01f, 1.00f));
  float angleRate = constrainf(rcCommandf * rcRate * superfactor, -SETPOINT_RATE_LIMIT, SETPOINT_RATE_LIMIT);

  return angleRate;
}

} // namespace Espfc::Control
