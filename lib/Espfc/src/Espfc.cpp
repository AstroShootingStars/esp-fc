#include "Espfc.h"
#include "Hal/Gpio.h"
#include "Debug_Espfc.h"
#include <Arduino.h>

namespace Espfc {

namespace {

void applyDefaultLedConfig(Model& model)
{
#if defined(ESPFC_LED_PIN) && (ESPFC_LED_PIN >= 0)
#if defined(ESPFC_LED_FORCE_DEFAULT) && (ESPFC_LED_FORCE_DEFAULT)
  model.config.pin[PIN_LED_BLINK] = ESPFC_LED_PIN;
#if defined(ESPFC_LED_TYPE)
  model.config.led.type = ESPFC_LED_TYPE;
#endif
#else
  if(model.config.pin[PIN_LED_BLINK] < 0)
  {
    model.config.pin[PIN_LED_BLINK] = ESPFC_LED_PIN;
#if defined(ESPFC_LED_TYPE)
    model.config.led.type = ESPFC_LED_TYPE;
#endif
  }
#endif
#endif
}

void runBootLedAnimation(Model& model)
{
  // Show explicit boot state (red blink) on both simple and strip LEDs.
  model.state.led.setStatus(Connect::LED_BOOT, true);
  for(size_t i = 0; i < 2; i++)
  {
    delay(20);
    model.state.led.update();
  }
}

void updateBootLedStatus(Model& model)
{
  const bool gyroDetected = model.state.gyro.present && model.state.gyro.dev != nullptr;
  const bool accelRequired = model.config.accel.dev != GYRO_NONE;
  const bool accelDetected = !accelRequired || model.state.accel.present;
  const bool baroRequired = model.config.baro.dev != BARO_NONE;
  const bool baroDetected = !baroRequired || model.state.baro.present;

  // Requested LED policy:
  // - green blink on successful boot
  // - orange blink when gyro is missing
  const bool bootSensorsDetected = gyroDetected;
  if(model.config.led.type == Connect::LED_STRIP || model.config.led.type == Connect::LED_SIMPLE)
  {
    model.state.led.setStatus(bootSensorsDetected ? Connect::LED_OK : Connect::LED_ERROR, true);
  }

  model.logger.info()
      .log(F("BOOT SENSORS g/a/b"))
      .log((int)gyroDetected)
      .log((int)accelDetected)
      .logln((int)baroDetected);
}

}

Espfc::Espfc():
  _hardware{_model}, _controller{_model}, _telemetry{_model}, _input{_model, _telemetry}, _actuator{_model}, _sensor{_model},
  _mixer{_model}, _blackbox{_model}, _buzzer{_model}, _serial{_model, _telemetry}
  {}

int Espfc::load()
{
  PIN_DEBUG_INIT();
  _model.load();
  _model.state.appQueue.begin();
  return 1;
}

int Espfc::begin()
{
  applyDefaultLedConfig(_model);

  _model.state.led.begin(_model.config.pin[PIN_LED_BLINK], _model.config.led.type, _model.config.led.invert);

  // Keep startup responsive on power cycle while still giving peripherals a brief settle window.
  delay(50);

  // Bring up serial/MSP first so configurator handshakes are available immediately after reset.
  _serial.begin();      // requires _model.load()

  // Short boot animation for RGB onboard LED while subsystems initialize.
  runBootLedAnimation(_model);

  //_model.logStorageResult();
  _hardware.begin();    // requires _model.load()
  _model.begin();       // requires _hardware.begin()
  _mixer.begin();
  _sensor.begin();      // requires _hardware.begin()
  _input.begin();       // requires _serial.begin()
  _actuator.begin();    // requires _model.begin()
  _controller.begin();
  _blackbox.begin();    // requires _serial.begin(), _actuator.begin()
  _buzzer.begin();

  updateBootLedStatus(_model);

  _model.state.buzzer.push(BUZZER_SYSTEM_INIT);

  return 1;
}

int FAST_CODE_ATTR Espfc::update(bool externalTrigger)
{
  if(externalTrigger)
  {
    _model.state.gyro.timer.update();
  }
  else
  {
    if(!_model.state.gyro.timer.check()) return 0;
  }
  Utils::Stats::Measure measure(_model.state.stats, COUNTER_CPU_0);

#if defined(ESPFC_MULTI_CORE)

  _sensor.read();
  if(_model.state.input.timer.syncTo(_model.state.gyro.timer, 1u))
  {
    _input.update();
  }
  if(_model.state.actuatorTimer.check())
  {
    _actuator.update();
  }

#else

  _sensor.update();
  if(_model.state.loopTimer.syncTo(_model.state.gyro.timer))
  {
    _controller.update();
    if(_model.state.mixer.timer.syncTo(_model.state.loopTimer))
    {
      _mixer.update();
    }
    _blackbox.update();
    if(_model.state.input.timer.syncTo(_model.state.gyro.timer, 1u))
    {
      _input.update();
    }
    if(_model.state.actuatorTimer.check())
    {
      _actuator.update();
    }
  }
  _sensor.updateDelayed();

#endif

  _serial.update();
  _buzzer.update();
  _hardware.updateOled();
  _model.state.led.update(_model);
  _model.state.stats.update();

#if defined(ESPFC_STAB_DEBUG)
  // WARNING: This prints text over the active serial stream.
  // Do not enable while using Betaflight configurator over MSP on the same port.
  static uint32_t stabilityCheckTs = 0;
  const uint32_t nowStab = millis();
  if(nowStab - stabilityCheckTs >= 100) // Every 100ms (10 Hz)
  {
    stabilityCheckTs = nowStab;

    const float rpyScale = 57.2957795f;
    const float yawDeg = _model.state.attitude.euler.z * rpyScale;
    const float pitchDeg = _model.state.attitude.euler.y * rpyScale;
    const float rollDeg = _model.state.attitude.euler.x * rpyScale;

    const float gyroX = _model.state.gyro.adc.x;
    const float gyroY = _model.state.gyro.adc.y;
    const float gyroZ = _model.state.gyro.adc.z;
    const float gyroNorm = sqrtf(gyroX * gyroX + gyroY * gyroY + gyroZ * gyroZ);

    // Emit only at rest/slow movement.
    if(gyroNorm < 300.f)
    {
      D("STAB",
        "yaw", yawDeg,
        "pitch", pitchDeg,
        "roll", rollDeg,
        "alt", _model.state.baro.altitudeGround,
        "gnorm", gyroNorm
      );
    }
  }
#endif

  return 1;
}

// other task
int FAST_CODE_ATTR Espfc::updateOther()
{
#if defined(ESPFC_MULTI_CORE)
  if(_model.state.appQueue.isEmpty())
  {
    return 0;
  }
  Event e = _model.state.appQueue.receive();

  Utils::Stats::Measure measure(_model.state.stats, COUNTER_CPU_1);

  switch(e.type)
  {
    case EVENT_GYRO_READ:
      _sensor.preLoop();
      _controller.update();
      // skip mixer and bb if earlier than half cycle, possible delay in previous iteration, 
      // to keep space to receive dshot erpm frame, but process rest
      if(_loop_next < micros())
      {
        _loop_next = micros() + _model.state.loopTimer.interval / 2;
        _mixer.update();
        _blackbox.update();
      }
      _sensor.postLoop();
      break;
    case EVENT_ACCEL_READ:
      _sensor.fusion();
      break;
    default:
      break;
      // nothing
  }
#endif

  return 1;
}

}

