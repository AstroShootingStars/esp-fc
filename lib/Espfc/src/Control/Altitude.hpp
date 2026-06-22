#pragma once

#include "Model.h"
#include "Utils/Filter.h"
#include <algorithm>
#include <cmath>

namespace Espfc::Control {

class Altitude
{
public:
  Altitude(Model& model): _model(model) {}

  int begin()
  {
    _model.state.altitude.height = 0.0f;
    _model.state.altitude.vario = 0.0f;

    _gpsBiasReady = false;
    _gpsHeightBias = 0.0f;
    _gpsAvailable = false;
    _flowAvailable = false;
    _gpsLossCount = 0;
    _flowLossCount = 0;

    const int rate = _model.state.loopTimer.rate > 0 ? _model.state.loopTimer.rate : 1000;
    _altitudeFilter.begin(FilterConfig(FILTER_PT3, 5), rate);
    _varioFilter.begin(FilterConfig(FILTER_PT3, 5), rate);

    return 1;
  }

  int update()
  {
    const auto& afc = _model.config.altitudeFusion;

    const bool hasBaro = _model.baroActive();
    const bool gpsSignal = _model.gpsActive() && _model.state.gps.fix && _model.state.gps.numSats >= _model.config.gps.minSats;
    const bool hasRange = _model.state.rangefinder[RANGEFINDER_BOTTOM].present;
    const bool flowSignal = _model.opticalFlowActive() && _model.state.opticalFlow.quality >= _model.config.opticalFlow.qualityThreshold;

    if(gpsSignal)
    {
      _gpsAvailable = true;
      _gpsLossCount = 0;
    }
    else if(_gpsAvailable)
    {
      if(_gpsLossCount < 255) _gpsLossCount++;
      if(_gpsLossCount >= std::max<uint8_t>(1, afc.gpsLossHysteresis))
      {
        _gpsAvailable = false;
        _gpsBiasReady = false;
      }
    }

    if(flowSignal)
    {
      _flowAvailable = true;
      _flowLossCount = 0;
    }
    else if(_flowAvailable)
    {
      if(_flowLossCount < 255) _flowLossCount++;
      if(_flowLossCount >= std::max<uint8_t>(1, afc.flowLossHysteresis))
      {
        _flowAvailable = false;
      }
    }

    const bool hasGps = _gpsAvailable;
    const bool hasFlow = _flowAvailable;

    float heightWeighted = 0.0f;
    float heightWeightSum = 0.0f;
    float varioWeighted = 0.0f;
    float varioWeightSum = 0.0f;

    const float baroHeightWeight = std::clamp<float>(afc.baroHeightWeight * 0.01f, 0.0f, 2.0f);
    const float baroVarioWeight = std::clamp<float>(afc.baroVarioWeight * 0.01f, 0.0f, 2.0f);
    const float gpsHeightWeight = std::clamp<float>(afc.gpsHeightWeight * 0.01f, 0.0f, 2.0f);
    const float gpsVarioWeight = std::clamp<float>(afc.gpsVarioWeight * 0.01f, 0.0f, 2.0f);
    const float rangeHeightWeight = std::clamp<float>(afc.rangeHeightWeight * 0.01f, 0.0f, 2.0f);
    const float flowVarioMaxWeight = std::clamp<float>(afc.flowVarioWeight * 0.01f, 0.0f, 2.0f);

    if(hasBaro)
    {
      heightWeighted += _model.state.baro.altitudeGround * baroHeightWeight;
      heightWeightSum += baroHeightWeight;
      varioWeighted += _model.state.baro.vario * baroVarioWeight;
      varioWeightSum += baroVarioWeight;
    }

    if(hasRange)
    {
      // Rangefinder is most reliable close to ground, so give it strong authority for height.
      const float rangeHeight = _model.state.rangefinder[RANGEFINDER_BOTTOM].distance * 0.001f; // mm -> m
      heightWeighted += rangeHeight * rangeHeightWeight;
      heightWeightSum += rangeHeightWeight;
    }

    if(hasGps)
    {
      const float gpsHeight = _model.state.gps.location.raw.height * 0.001f; // mm -> m
      if(!_gpsBiasReady)
      {
        _gpsHeightBias = gpsHeight - (hasBaro ? _model.state.baro.altitudeGround : _model.state.altitude.height);
        _gpsBiasReady = true;
      }

      const float gpsHeightGround = gpsHeight - _gpsHeightBias;
      const float gpsVario = -_model.state.gps.velocity.raw.down * 0.001f; // mm/s -> m/s, invert NED down axis

      heightWeighted += gpsHeightGround * gpsHeightWeight;
      heightWeightSum += gpsHeightWeight;
      varioWeighted += gpsVario * gpsVarioWeight;
      varioWeightSum += gpsVarioWeight;
    }

    if(hasFlow)
    {
      // Optical flow does not measure vertical speed directly, but high-quality low-motion observations
      // are a strong cue that vertical velocity should be near zero (helps reduce baro/GPS noise).
      const float flowMag = std::sqrt(
        _model.state.opticalFlow.flowRateX * _model.state.opticalFlow.flowRateX +
        _model.state.opticalFlow.flowRateY * _model.state.opticalFlow.flowRateY);

      const float flowStillRate = std::max<float>(1.0f, afc.flowStillRate);

      const float flowStill = std::clamp((flowStillRate - flowMag) / flowStillRate, 0.0f, 1.0f);
      const float qualityRange = std::max(1.0f, 100.0f - _model.config.opticalFlow.qualityThreshold);
      const float flowQuality = std::clamp(
        (_model.state.opticalFlow.quality - _model.config.opticalFlow.qualityThreshold) /
          qualityRange,
        0.0f,
        1.0f);
      const float flowWeight = flowVarioMaxWeight * flowStill * flowQuality;

      varioWeighted += 0.0f * flowWeight;
      varioWeightSum += flowWeight;
    }

    if(heightWeightSum > 0.0f)
    {
      _model.state.altitude.height = _altitudeFilter.update(heightWeighted / heightWeightSum);
    }

    if(varioWeightSum > 0.0f)
    {
      _model.state.altitude.vario = _varioFilter.update(varioWeighted / varioWeightSum);
    }

    if(_model.config.debug.mode == DEBUG_ALTITUDE)
    {
      const float dbgHeightSrc = hasRange ? (_model.state.rangefinder[RANGEFINDER_BOTTOM].distance * 0.1f) : (hasBaro ? _model.state.baro.altitudeGround * 100.0f : 0.0f);
      const float dbgVarioSrc = hasBaro ? _model.state.baro.vario * 100.0f : (hasGps ? (-_model.state.gps.velocity.raw.down * 0.1f) : 0.0f);
      _model.state.debug[0] = std::clamp(lrintf(dbgHeightSrc), -32000l, 32000l);                           // primary src height [cm]
      _model.state.debug[1] = std::clamp(lrintf(dbgVarioSrc), -32000l, 32000l);                            // primary src vario [cm/s]
      _model.state.debug[2] = std::clamp(lrintf(_model.state.altitude.height * 100.0f), -32000l, 32000l);  // fused height [cm]
      _model.state.debug[3] = std::clamp(lrintf(_model.state.altitude.vario * 100.0f), -32000l, 32000l);   // fused vario [cm/s]
    }

    return 1;
  }

private:
  Model& _model;
  Utils::Filter _altitudeFilter;
  Utils::Filter _varioFilter;
  bool _gpsBiasReady = false;
  float _gpsHeightBias = 0.0f;
  bool _gpsAvailable = false;
  bool _flowAvailable = false;
  uint8_t _gpsLossCount = 0;
  uint8_t _flowLossCount = 0;
};

}
