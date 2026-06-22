#pragma once

#include "Model.h"
#include "Utils/Filter.h"
#include <algorithm>
#include <cmath>

namespace Espfc::Control {

class ObstacleAvoidance
{
public:
  ObstacleAvoidance(Model& model): _model(model) {}

  int begin()
  {
    _model.state.obstacleAvoidance.active = 0;
    _model.state.obstacleAvoidance.detectedDistance = 0;
    _model.state.obstacleAvoidance.throttleCorrection = 1.0f;
    _model.state.obstacleAvoidance.avoidanceFlag = 0;
    _model.state.obstacleAvoidance.lastUpdateMs = millis();

    const int rate = _model.state.loopTimer.rate > 0 ? _model.state.loopTimer.rate : 1000;
    _distanceFilter.begin(FilterConfig(FILTER_BIQUAD, 5), rate);

    return 1;
  }

  int update()
  {
    const auto& oac = _model.config.obstacleAvoidance;
    auto& oas = _model.state.obstacleAvoidance;

    // Check if obstacle avoidance is enabled and rangefinder is present
    if(!oac.enabled || !_model.state.rangefinder[RANGEFINDER_FRONT].present)
    {
      oas.active = 0;
      oas.throttleCorrection = 1.0f;
      oas.avoidanceFlag = 0;
      return 1;
    }

    // Check if obstacle avoidance is enabled for current flight mode
    if(!isEnabledInMode(oac))
    {
      oas.active = 0;
      oas.throttleCorrection = 1.0f;
      oas.avoidanceFlag = 0;
      return 1;
    }

    // Get filtered distance from rangefinder (convert mm to cm)
    uint16_t distance = _distanceFilter.update(_model.state.rangefinder[RANGEFINDER_FRONT].distance / 10.0f);
    oas.detectedDistance = distance;
    oas.lastUpdateMs = millis();

    // Check for valid distance
    if(distance > oac.maxAvoidanceDistance)
    {
      oas.active = 0;
      oas.throttleCorrection = 1.0f;
      oas.avoidanceFlag = 0;
      return 1;
    }

    // Determine obstacle status
    if(distance <= oac.minSafeDistance)
    {
      // Danger: obstacle too close
      oas.avoidanceFlag = 2;
      oas.active = 1;
      oas.throttleCorrection = 0.0f; // Complete stop
    }
    else if(distance <= oac.avoidanceDistance)
    {
      // Warning: obstacle detected, apply avoidance
      oas.avoidanceFlag = 1;
      oas.active = 1;

      // Calculate throttle correction based on avoidance mode
      switch(oac.avoidanceMode)
      {
        case 0: // Slow down
          oas.throttleCorrection = 1.0f - (oac.slowdownPercent / 100.0f);
          break;

        case 1: // Stop
          oas.throttleCorrection = 1.0f - (oac.stopPercent / 100.0f);
          break;

        case 2: // Bypass (move up/down or side)
          oas.throttleCorrection = 1.0f - (oac.slowdownPercent / 100.0f);
          // Bypass adjustment is handled by the controller (pitch/roll/yaw input)
          break;

        case 3: // Auto (adjust based on distance)
          {
            float ratio = (distance - oac.avoidanceDistance) / (float)(oac.maxAvoidanceDistance - oac.avoidanceDistance);
            ratio = std::clamp(ratio, 0.0f, 1.0f);
            oas.throttleCorrection = ratio;
            break;
          }

        default:
          oas.throttleCorrection = 1.0f;
          break;
      }
    }
    else
    {
      // Clear: no obstacle
      oas.avoidanceFlag = 0;
      oas.active = 0;
      oas.throttleCorrection = 1.0f;
    }

    return 1;
  }

  // Get throttle correction factor (0.0 = stop, 1.0 = no adjustment)
  float getThrottleCorrection() const
  {
    if(!_model.config.obstacleAvoidance.enabled)
      return 1.0f;

    return _model.state.obstacleAvoidance.throttleCorrection;
  }

  // Check if obstacle avoidance should apply input bypass (for mode 2)
  bool shouldBypass() const
  {
    const auto& oac = _model.config.obstacleAvoidance;
    if(oac.avoidanceMode != 2 || oac.enabled == 0)
      return false;

    return _model.state.obstacleAvoidance.active;
  }

  // Get bypass axis (0=yaw, 1=pitch, 2=roll)
  uint8_t getBypassAxis() const
  {
    return std::clamp<uint8_t>(_model.config.obstacleAvoidance.bypassAxis, 0, 2);
  }

private:
  bool isEnabledInMode(const ObstacleAvoidanceConfig& oac)
  {
    // Must be armed
    if(!_model.isModeActive(MODE_ARMED))
      return false;

    // Check if allowed in current flight mode
    if(_model.isModeActive(MODE_ANGLE) && oac.enableInAngle) return true;
    if(_model.isModeActive(MODE_HORIZON) && oac.enableInHorizon) return true;
    if(_model.isModeActive(MODE_ALTHOLD) && oac.enableInAltHold) return true;
    
    // For acro mode, check if acro trainer is enabled or just allow it more broadly
    if(oac.enableInAcro) return true; // Allow in all other armed modes when acro is enabled

    return false;
  }

  Model& _model;
  Utils::Filter _distanceFilter;
};

} // namespace Espfc::Control
