#include "Pid.h"
#include "Utils/Math.hpp"
#include "Utils/MemoryHelper.h"
#include <algorithm>

namespace Espfc {

namespace Control {

Pid::Pid():
  rate(1.0f), dt(1.0f), Kp(0.1), Ki(0.f), Kd(0.f), Kf(0.0f),
  iLimitLow(-0.3f), iLimitHigh(0.3f), iReset(0.0f), oLimitLow(-1.f), oLimitHigh(1.f),
  pScale(1.f), iScale(1.f), dScale(1.f), fScale(1.f), pLimit(0.f), ffTransitionFactor(1.f), dMinFactor(1.f),
  ffBoost(0), ffMaxRateLimit(0), ffJitterFactor(0), ffAveraging(0), ffSmoothness(0), ffDeltaFiltered(0.f),
  error(0.f), iTermError(0.f),
  pTerm(0.f), iTerm(0.f), dTerm(0.f), fTerm(0.f),
  prevMeasurement(0.f), prevError(0.f), prevSetpoint(0.f),
  ftermDerivative(true), outputSaturated(false),
  itermRelax(ITERM_RELAX_OFF), itermRelaxMode(0), itermRelaxFactor(1.0f), itermRelaxBase(0.f)
  {}

void Pid::begin()
{
  dt = 1.f / rate;
}

void Pid::resetIterm()
{
  iTerm = iReset;
}

float FAST_CODE_ATTR Pid::update(float setpoint, float measurement)
{
  const bool iEnabled = Ki > 0.f && iScale > 0.f;
  const bool dEnabled = Kd > 0.f && dScale > 0.f;
  const bool fEnabled = Kf > 0.f && fScale > 0.f;

  error = setpoint - measurement;

  // P-term
  pTerm = Kp * error * pScale;
  pTerm = ptermFilter.update(pTerm);
  if(pLimit > 0.0f)
  {
    pTerm = std::clamp(pTerm, -pLimit, pLimit);
  }

  // I-term
  iTermError = error;
  if(iEnabled)
  {
    if(!outputSaturated)
    {
      // I-term relax
      if(itermRelax)
      {
        const bool increasing = (iTerm > 0 && iTermError > 0) || (iTerm < 0 && iTermError < 0);
        const bool incrementOnly = itermRelax == ITERM_RELAX_RP_INC || itermRelax == ITERM_RELAX_RPY_INC;
        const float relaxSource = itermRelaxMode == 0 ? setpoint : error;
        itermRelaxBase = relaxSource - itermRelaxFilter.update(relaxSource);
        itermRelaxFactor = std::max(0.0f, 1.0f - std::abs(Utils::toDeg(itermRelaxBase)) * 0.025f); // (itermRelaxBase / 40)
        if(!incrementOnly || increasing) iTermError *= itermRelaxFactor;
      }
      const float iGainDt = Ki * iScale * dt;
      iTerm += iGainDt * iTermError;
      iTerm = std::clamp(iTerm, iLimitLow, iLimitHigh);
    }
  }
  else
  {
    iTerm = 0; // zero integral
  }

  // D-term
  if(dEnabled)
  {
    //dTerm = (Kd * dScale * (((error - prevError) * dGamma) + (prevMeasurement - measure) * (1.f - dGamma)) / dt);
    const float dGainRate = Kd * dScale * rate;
    dTerm = dGainRate * (prevMeasurement - measurement);
    dTerm = dtermNotchFilter.update(dTerm);
    dTerm = dtermFilter.update(dTerm);
    dTerm = dtermFilter2.update(dTerm);
    dTerm *= dMinFactor;  // Apply D-min scaling
  }
  else
  {
    dTerm = 0;
  }

  // F-term
  if(fEnabled)
  {
    const float fGain = Kf * fScale * ffTransitionFactor;
    if(ftermDerivative)
    {
      const float rawDelta = setpoint - prevSetpoint;

      // Extra smoothing for feedforward derivative to reduce high-frequency jitter.
      const float avg = std::clamp((float)ffAveraging * 0.01f, 0.0f, 1.0f);
      const float smooth = std::clamp((float)ffSmoothness * 0.01f, 0.0f, 1.0f);
      const float alpha = std::clamp(1.0f - (0.45f * avg + 0.55f * smooth), 0.05f, 1.0f);
      ffDeltaFiltered += alpha * (rawDelta - ffDeltaFiltered);

      float ffDelta = ffDeltaFiltered;
      const float deltaRate = std::abs(ffDelta) * rate;

      if(ffJitterFactor > 0)
      {
        const float jitter = std::clamp((float)ffJitterFactor * 0.01f, 0.0f, 2.0f);
        const float atten = 1.0f / (1.0f + jitter * deltaRate * 0.02f);
        ffDelta *= atten;
      }

      float boostFactor = 1.0f;
      if(ffBoost > 0)
      {
        const float accelNorm = std::clamp(deltaRate / Utils::toRad(1200.0f), 0.0f, 1.5f);
        boostFactor += std::clamp((float)ffBoost * 0.01f, 0.0f, 2.0f) * accelNorm;
      }

      if(ffMaxRateLimit > 0)
      {
        const float rateNorm = std::clamp(std::abs(setpoint) / Utils::toRad(1200.0f), 0.0f, 1.0f);
        const float maxLimit = std::clamp((float)ffMaxRateLimit * 0.01f, 0.0f, 1.0f);
        const float limitScale = std::clamp(1.0f - rateNorm * maxLimit, 0.1f, 1.0f);
        boostFactor *= limitScale;
      }

      fTerm = fGain * ffDelta * rate * boostFactor;
    }
    else
    {
      fTerm = fGain * setpoint;
    }
    fTerm = ftermFilter.update(fTerm);
  }
  else
  {
    fTerm = 0;
  }

  prevMeasurement = measurement;
  prevError = error;
  prevSetpoint = setpoint;

  const float out = pTerm + iTerm + dTerm + fTerm;
  return std::clamp(out, oLimitLow, oLimitHigh);
}

}

}
