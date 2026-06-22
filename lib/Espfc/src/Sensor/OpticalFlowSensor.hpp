#pragma once

#include "BaseSensor.h"
#include "Model.h"
#include "Device/OpticalFlowDevice.hpp"

namespace Espfc::Sensor {

class OpticalFlowSensor : public BaseSensor
{
public:
  OpticalFlowSensor(Model& model);

  int begin();
  int update();

private:
  int updateFromDevice();
  int updateFromMsp();
  void updateRates(uint32_t nowUs, int32_t motionX, int32_t motionY);

  static constexpr uint32_t MSP_TIMEOUT_US = 300000;

  Model& _model;
  Device::OpticalFlowDevice* _flow;
  uint32_t _lastSampleUs;
};

} // namespace Espfc::Sensor
