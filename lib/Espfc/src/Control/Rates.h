#pragma once

#include "ModelConfig.h"
#include "Utils/Math.hpp"

namespace Espfc {

enum RateType
{
  RATES_TYPE_BETAFLIGHT = 0,
  RATES_TYPE_RACEFLIGHT,
  RATES_TYPE_KISS,
  RATES_TYPE_ACTUAL,
  RATES_TYPE_QUICK,
};

namespace Control {

class Rates
{
public:
  void begin(const InputConfig& config);
  float getSetpoint(const int axis, float input) const;

private:
  float betaflight(const int axis, float rcCommandf, const float rcCommandfAbs) const;
  float raceflight(const int axis, float rcCommandf, const float rcCommandfAbs) const;
  float kiss(const int axis, float rcCommandf, const float rcCommandfAbs) const;
  float actual(const int axis, float rcCommandf, const float rcCommandfAbs) const;
  float quick(const int axis, float rcCommandf, const float rcCommandfAbs) const;

  inline float power3(float x) const
  {
    return x * x * x;
  }

  inline float power5(float x) const
  {
    return x * x * x * x * x;
  }

  inline float constrainf(float x, float l, float h) const
  {
    return Utils::clamp(x, l, h);
  }

private:
  const InputConfig* _config = nullptr;
  RateType rateType;
};

} // namespace Control

} // namespace Espfc
