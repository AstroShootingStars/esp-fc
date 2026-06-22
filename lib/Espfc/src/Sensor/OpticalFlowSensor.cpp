#include "Sensor/OpticalFlowSensor.hpp"
#include <Arduino.h>

namespace Espfc::Sensor {

OpticalFlowSensor::OpticalFlowSensor(Model& model): _model(model), _flow(nullptr), _lastSampleUs(0) {}

int OpticalFlowSensor::begin()
{
  if (_model.config.opticalFlow.dev == Device::OPFLOW_NONE) return 0;

  _flow = _model.state.opticalFlow.dev;
  if (_flow)
  {
    _model.state.opticalFlow.rate = _flow->getRate();
    _model.logger.info().log(F("OPFLOW INIT")).log(FPSTR(Device::OpticalFlowDevice::getName(_flow->getType()))).logln(_flow->getRate());
    return 1;
  }

  if (_model.config.opticalFlow.dev == Device::OPFLOW_MSP || _model.config.opticalFlow.dev == Device::OPFLOW_DEFAULT)
  {
    _model.state.opticalFlow.rate = 100;
    _model.logger.info().logln(F("OPFLOW INIT MSP"));
    return 1;
  }

  return 0;
}

int OpticalFlowSensor::update()
{
  if (_flow) return updateFromDevice();

  if (_model.config.opticalFlow.dev == Device::OPFLOW_MSP || _model.config.opticalFlow.dev == Device::OPFLOW_DEFAULT)
  {
    return updateFromMsp();
  }

  return 0;
}

int OpticalFlowSensor::updateFromDevice()
{
  if (!_flow) return 0;

  Device::OpticalFlowData data;
  if (!_flow->readMotion(data)) return 0;

  auto& flow = _model.state.opticalFlow;
  flow.motionX = data.motionX;
  flow.motionY = data.motionY;
  flow.quality = data.quality;
  flow.lastUpdateUs = data.timeUs;
  flow.present = (flow.quality >= _model.config.opticalFlow.qualityThreshold);

  updateRates(flow.lastUpdateUs, flow.motionX, flow.motionY);

  return 1;
}

int OpticalFlowSensor::updateFromMsp()
{
  auto& flow = _model.state.opticalFlow;
  if (!flow.lastUpdateUs)
  {
    flow.present = false;
    return 0;
  }

  const uint32_t nowUs = micros();
  if ((nowUs - flow.lastUpdateUs) > MSP_TIMEOUT_US)
  {
    flow.present = false;
    return 0;
  }

  flow.present = (flow.quality >= _model.config.opticalFlow.qualityThreshold);

  if (flow.lastUpdateUs != _lastSampleUs)
  {
    updateRates(flow.lastUpdateUs, flow.motionX, flow.motionY);
    return 1;
  }

  return 0;
}

void OpticalFlowSensor::updateRates(uint32_t nowUs, int32_t motionX, int32_t motionY)
{
  auto& flow = _model.state.opticalFlow;
  if (_lastSampleUs == 0 || nowUs <= _lastSampleUs)
  {
    _lastSampleUs = nowUs;
    return;
  }

  const float dt = (nowUs - _lastSampleUs) * 1e-6f;
  if (dt > 0.0f)
  {
    flow.flowRateX = motionX / dt;
    flow.flowRateY = motionY / dt;
  }

  _lastSampleUs = nowUs;
}

} // namespace Espfc::Sensor
