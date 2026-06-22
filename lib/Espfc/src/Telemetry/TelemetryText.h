#pragma once

#include "Model.h"

namespace Espfc {

namespace Telemetry {

class TelemetryText
{
  public:
    TelemetryText(Model& model): _model(model) {}

    int process(Stream& s) const
    {
      s.print(F("TEL "));
      s.print(F("armed="));
      s.print(_model.isModeActive(MODE_ARMED) ? 1 : 0);

      s.print(F(" bat_v="));
      s.print(_model.state.battery.voltage, 2);
      s.print(F(" bat_a="));
      s.print(_model.state.battery.current, 2);
      s.print(F(" bat_%="));
      s.print(_model.state.battery.percentage, 1);

      s.print(F(" rpy="));
      s.print(_model.state.attitude.euler.x, 3);
      s.print(',');
      s.print(_model.state.attitude.euler.y, 3);
      s.print(',');
      s.print(_model.state.attitude.euler.z, 3);

      s.print(F(" gyro_hz="));
      s.print(_model.state.gyro.timer.rate);
      s.print(F(" loop_hz="));
      s.print(_model.state.loopTimer.rate);

      s.print(F(" alt_m="));
      if(_model.baroActive()) s.print(_model.state.baro.altitude, 2);
      else s.print(F("NA"));

      s.print(F(" vario_ms="));
      if(_model.baroActive()) s.print(_model.state.baro.vario, 2);
      else s.print(F("NA"));

      s.print(F(" rng_mm="));
      if(_model.state.rangefinder[RANGEFINDER_BOTTOM].present) s.print(_model.state.rangefinder[RANGEFINDER_BOTTOM].distance);
      else s.print(F("NA"));

      s.print(F(" flow_q="));
      s.print(_model.state.opticalFlow.quality);
      s.print(F(" flow_dx="));
      s.print(_model.state.opticalFlow.motionX);
      s.print(F(" flow_dy="));
      s.print(_model.state.opticalFlow.motionY);
      s.print(F(" flow_v="));
      s.print(_model.state.opticalFlow.flowRateX, 2);
      s.print(',');
      s.print(_model.state.opticalFlow.flowRateY, 2);

      s.print(F(" gps_fix="));
      s.print(_model.state.gps.fix);
      s.print(F(" sats="));
      s.print(_model.state.gps.numSats);

      s.print(F(" rpm0="));
      s.print(_model.state.output.telemetry.rpm[0], 0);
      s.print(F(" rpm1="));
      s.print(_model.state.output.telemetry.rpm[1], 0);

      s.print(F(" oled="));
      s.print(_model.state.oled.present ? 1 : 0);

      println(s);

      return 1;
    }

  private:
    template<typename T>
    void print(Stream& s, const T& v) const
    {
      s.print(v);
      s.print(' ');
    }

    void print(Stream& s, const long& v) const
    {
      s.print(v);
      s.print(' ');
    }

    void print(Stream& s, float& v, int len) const
    {
      s.print(v, len);
      s.print(' ');
    }

    void print(Stream& s, const VectorFloat& v) const
    {
      print(s, v.x);
      print(s, v.y);
      print(s, v.z);
    }

    void print(Stream& s, const VectorInt16& v) const
    {
      print(s, v.x);
      print(s, v.y);
      print(s, v.z);
    }

    void print(Stream& s, const Quaternion& v) const
    {
      print(s, v.w);
      print(s, v.x);
      print(s, v.y);
      print(s, v.z);
    }

    void println(Stream& s) const
    {
      s.println();
    }

    Model& _model;
};

}

}
