#pragma once

#include "Model.h"
#include "Device/SerialDevice.h"
#include "Rc/Crsf.h"
#include "Utils/Math.hpp"
#include <algorithm>

// https://github.com/crsf-wg/crsf/wiki/CRSF_FRAMETYPE_MSP_REQ - not correct
// https://github.com/betaflight/betaflight/blob/2525be9a3369fa666d8ce1485ec5ad344326b085/src/main/telemetry/crsf.c#L664
// https://github.com/betaflight/betaflight/blob/2525be9a3369fa666d8ce1485ec5ad344326b085/src/main/telemetry/msp_shared.c#L46

namespace Espfc {

namespace Telemetry {

enum TelemetryState {
  CRSF_TELEMETRY_STATE_ATTI,      // Attitude (roll, pitch, yaw)
  CRSF_TELEMETRY_STATE_BAT,       // Battery (voltage, current, capacity, percentage)
  CRSF_TELEMETRY_STATE_FM,        // Flight mode status
  CRSF_TELEMETRY_STATE_GPS,       // GPS (position, velocity, altitude, satellites)
  CRSF_TELEMETRY_STATE_BARO,      // Barometer (altitude, vertical speed)
  CRSF_TELEMETRY_STATE_GYRO,      // Gyroscope (roll, pitch, yaw rates)
  CRSF_TELEMETRY_STATE_ACCEL,     // Accelerometer (x, y, z acceleration)
  CRSF_TELEMETRY_STATE_MAG,       // Magnetometer (x, y, z magnetic field)
  CRSF_TELEMETRY_STATE_LINK_TX,   // Link statistics TX (signal quality metrics)
  CRSF_TELEMETRY_STATE_HB,        // Heartbeat
};

class TelemetryCRSF
{
public:
  TelemetryCRSF(Model& model): _model(model) {}

  int begin()
  {
    return 1;
  }

  int process(Device::SerialDevice& s) const
  {
    Rc::CrsfMessage f;
    switch(_current)
    {
      case CRSF_TELEMETRY_STATE_ATTI:
        attitude(f);
        send(f, s);
        _current = CRSF_TELEMETRY_STATE_BAT;
        break;
      case CRSF_TELEMETRY_STATE_BAT:
        battery(f);
        send(f, s);
        _current = CRSF_TELEMETRY_STATE_FM;
        break;
      case CRSF_TELEMETRY_STATE_FM:
        flightMode(f);
        send(f, s);
        _current = CRSF_TELEMETRY_STATE_GYRO;
        break;
      case CRSF_TELEMETRY_STATE_GYRO:
        if(_model.gyroActive()) {
          gyroscope(f);
          send(f, s);
        }
        _current = CRSF_TELEMETRY_STATE_ACCEL;
        break;
      case CRSF_TELEMETRY_STATE_ACCEL:
        if(_model.accelActive()) {
          accelerometer(f);
          send(f, s);
        }
        _current = CRSF_TELEMETRY_STATE_MAG;
        break;
      case CRSF_TELEMETRY_STATE_MAG:
        if(_model.magActive()) {
          magnetometer(f);
          send(f, s);
        }
        _current = CRSF_TELEMETRY_STATE_GPS;
        break;
      case CRSF_TELEMETRY_STATE_GPS:
        if(_model.gpsActive()) {
          gps(f);
          send(f, s);
        }
        _current = CRSF_TELEMETRY_STATE_BARO;
        break;
      case CRSF_TELEMETRY_STATE_BARO:
        if(_model.baroActive()) {
          vario(f);
          send(f, s);
        }
        _current = CRSF_TELEMETRY_STATE_LINK_TX;
        break;
      case CRSF_TELEMETRY_STATE_LINK_TX:
        linkStatistics(f);
        send(f, s);
        _current = CRSF_TELEMETRY_STATE_HB;
        break;
      default:    // Heartbeat and cycle back
        heartbeat(f);
        send(f, s);
        _current = CRSF_TELEMETRY_STATE_ATTI;
        break;
    }

    return 1;
  }

  int sendMsp(Device::SerialDevice& s, Connect::MspResponse resp, uint8_t origin)
  {
    size_t size = resp.serialize(_buff, sizeof(_buff));
    const uint8_t* beg = _buff + 3;        // skip msp header
    const uint8_t* end = _buff + size - 1; // skip crc
    uint8_t version = resp.version == Connect::MSP_V1 ? 1 : 2;
    size_t iter = 0;
    Rc::CrsfMessage frame;
    do
    {
      beg = Rc::Crsf::encodeMspData(frame, origin, version, _seq++, !iter, beg, end);
      send(frame, s);
      iter++;
    } while(beg != end && iter < 4);

    return iter;
  }

  void send(const Rc::CrsfMessage& msg, Device::SerialDevice& s) const
  {
    s.write((uint8_t*)&msg, msg.size + 2);
  }

  int16_t toAngle(float angle) const
  {
    if(angle < -Utils::pi()) angle += Utils::twoPi();
    if(angle >  Utils::pi()) angle -= Utils::twoPi();
    return lrintf(angle * 10000);
  }

  void attitude(Rc::CrsfMessage& msg) const
  {
    msg.prepare(Rc::CRSF_FRAMETYPE_ATTITUDE);

    int16_t r = toAngle(_model.state.attitude.euler.x);
    int16_t p = toAngle(_model.state.attitude.euler.y);
    int16_t y = toAngle(_model.state.attitude.euler.z);

    msg.writeU16(Utils::toBigEndian16(r));
    msg.writeU16(Utils::toBigEndian16(p));
    msg.writeU16(Utils::toBigEndian16(y));

    msg.finalize();
  }

  void battery(Rc::CrsfMessage& msg) const
  {
    msg.prepare(Rc::CRSF_FRAMETYPE_BATTERY_SENSOR);

    uint16_t voltage = Utils::clamp(lrintf(_model.state.battery.voltage * 10.0f), 0l, 32000l);
    uint16_t current = Utils::clamp(lrintf(_model.state.battery.current * 10.0f), 0l, 32000l);
    uint32_t mahDrawn = 0;
    uint8_t remainPerc = lrintf(_model.state.battery.percentage);

    msg.writeU16(Utils::toBigEndian16(voltage));
    msg.writeU16(Utils::toBigEndian16(current));
    msg.writeU8(mahDrawn >> 16);
    msg.writeU8(mahDrawn >> 8);
    msg.writeU8(mahDrawn);
    msg.writeU8(remainPerc);

    msg.finalize();
  }

  void flightMode(Rc::CrsfMessage& msg) const
  {
    msg.prepare(Rc::CRSF_FRAMETYPE_FLIGHT_MODE);

    if(_model.armingDisabled()) msg.writeString("!DIS");
    if(_model.isModeActive(MODE_FAILSAFE)) msg.writeString("!FS,");
    if(_model.isModeActive(MODE_ARMED)) msg.writeString("ARM,");
    if(_model.isModeActive(MODE_AIRMODE)) msg.writeString("AIR,");
    if(_model.isModeActive(MODE_ANGLE)) msg.writeString("STAB,");
    msg.writeU8(0);

    msg.finalize();
  }

  void gps(Rc::CrsfMessage& msg) const
  {
    msg.prepare(Rc::CRSF_FRAMETYPE_GPS);

    msg.writeU32(Utils::toBigEndian32(_model.state.gps.location.raw.lat)); // deg * 1e7
    msg.writeU32(Utils::toBigEndian32(_model.state.gps.location.raw.lon)); // deg * 1e7
    msg.writeU16(Utils::toBigEndian16((_model.state.gps.velocity.raw.groundSpeed * 36 + 500) / 1000)); // in km/h * 10
    msg.writeU16(Utils::toBigEndian16((_model.state.gps.velocity.raw.heading + 500) / 1000)); // deg * 10
    uint16_t altitude = std::clamp((_model.state.gps.location.raw.height + 500) / 1000, (int32_t)-900, (int32_t)5000) + 1000; // m
    msg.writeU16(Utils::toBigEndian16(altitude));
    msg.writeU8(_model.state.gps.numSats);

    msg.finalize();
  }

  void vario(Rc::CrsfMessage& msg) const
  {
    // https://github.com/crsf-wg/crsf/wiki/CRSF_FRAMETYPE_BARO_ALTITUDE
    msg.prepare(Rc::CRSF_FRAMETYPE_BARO_ALTITUDE);

    // Send barometer data
    msg.writeU16(Utils::toBigEndian16((_model.state.baro.altitude * 10.0f) + 10000 )); // (dm + 10000) or (m + 0 | 0x8000)
    msg.writeU16(Utils::toBigEndian16(_model.state.baro.vario * 100.0f)); // cm

    msg.finalize();
  }

  void heartbeat(Rc::CrsfMessage& msg) const
  {
    msg.prepare(Rc::CRSF_FRAMETYPE_HEARTBEAT);

    msg.writeU16(Utils::toBigEndian16(Rc::CRSF_ADDRESS_FLIGHT_CONTROLLER));

    msg.finalize();
  }

  void gyroscope(Rc::CrsfMessage& msg) const
  {
    // Send gyroscope data: roll, pitch, yaw angular rates in deg/s × 100
    msg.prepare(Rc::CRSF_FRAMETYPE_FLIGHT_MODE);

    int16_t roll = lrintf(Utils::toDeg(_model.state.gyro.adc[0]) * 100.0f);
    int16_t pitch = lrintf(Utils::toDeg(_model.state.gyro.adc[1]) * 100.0f);
    int16_t yaw = lrintf(Utils::toDeg(_model.state.gyro.adc[2]) * 100.0f);

    msg.writeU16(Utils::toBigEndian16((uint16_t)roll));
    msg.writeU16(Utils::toBigEndian16((uint16_t)pitch));
    msg.writeU16(Utils::toBigEndian16((uint16_t)yaw));

    msg.finalize();
  }

  void accelerometer(Rc::CrsfMessage& msg) const
  {
    // Send accelerometer data in m/s² × 100
    msg.prepare(Rc::CRSF_FRAMETYPE_FLIGHT_MODE);

    // Scale from native ADC to m/s² (typically: raw_accel / 2048 * 9.81)
    // Using standard conversion factor for typical accelerometers
    float scale = 9.81f * 100.0f / 2048.0f; // m/s² × 100, for 2048 LSB/g sensor
    int16_t accelX = lrintf(_model.state.accel.adc[0] * scale);
    int16_t accelY = lrintf(_model.state.accel.adc[1] * scale);
    int16_t accelZ = lrintf(_model.state.accel.adc[2] * scale);

    msg.writeU16(Utils::toBigEndian16((uint16_t)accelX));
    msg.writeU16(Utils::toBigEndian16((uint16_t)accelY));
    msg.writeU16(Utils::toBigEndian16((uint16_t)accelZ));

    msg.finalize();
  }

  void magnetometer(Rc::CrsfMessage& msg) const
  {
    // Send magnetometer data (raw ADC values in Gauss × 100)
    msg.prepare(Rc::CRSF_FRAMETYPE_FLIGHT_MODE);

    int16_t magX = lrintf(_model.state.mag.adc[0] * 100.0f);
    int16_t magY = lrintf(_model.state.mag.adc[1] * 100.0f);
    int16_t magZ = lrintf(_model.state.mag.adc[2] * 100.0f);

    msg.writeU16(Utils::toBigEndian16((uint16_t)magX));
    msg.writeU16(Utils::toBigEndian16((uint16_t)magY));
    msg.writeU16(Utils::toBigEndian16((uint16_t)magZ));

    msg.finalize();
  }

  void linkStatistics(Rc::CrsfMessage& msg) const
  {
    // Send link statistics (TX side - FC perspective)
    msg.prepare(Rc::CRSF_FRAMETYPE_LINK_STATISTICS_TX);

    // TX power and link quality metrics
    uint8_t txPower = 0;     // 0mW by default
    uint8_t rssi = 0;        // RSSI placeholder (would need RX input)
    uint8_t rfMode = 0;      // RF mode/rate (0 = 4Hz, 1 = 50Hz, 2 = 150Hz)
    uint8_t modelMatch = 1;  // Model match (1 = matched)
    uint16_t flags = 0;      // Flags

    msg.writeU8(txPower);
    msg.writeU8(rssi);
    msg.writeU8(rfMode);
    msg.writeU8(modelMatch);
    msg.writeU16(Utils::toBigEndian16(flags));

    msg.finalize();
  }

private:
  Model& _model;
  mutable TelemetryState _current;
  uint8_t _buff[256] = {0};
  uint8_t _seq = 0;
};

}

}
