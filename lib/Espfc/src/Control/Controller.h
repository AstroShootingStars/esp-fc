#pragma once

#include "Control/Altitude.hpp"
#include "Control/ObstacleAvoidance.hpp"
#include "Control/Rates.h"
#include "Model.h"

namespace Espfc::Control {

class Controller
{
public:
  Controller(Model& model);
  int begin();
  int update();

  void outerLoopRobot();
  void innerLoopRobot();
  void outerLoop();
  void innerLoop();
  void applyLandingAssist();

  inline float getTpaFactor() const;
  inline void resetIterm();
  float calculateSetpointRate(int axis, float input) const;
  float calcualteAltHoldSetpoint() const;

private:
  void beginAltHold();
  void beginInnerLoop(size_t axis);
  void beginOuterLoop(size_t axis);

  Model& _model;
  Rates _rates;
  Utils::Filter _speedFilter;
  ObstacleAvoidance _obstacleAvoidance;

  uint32_t _landingTouchdownStartMs = 0;
  bool _landingTouchdownPending = false;
};

} // namespace Espfc::Control
