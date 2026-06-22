#include "Buzzer.hpp"
#include "Hal/Gpio.h"
#include <cstddef>

namespace Espfc {

namespace Connect {

namespace {
constexpr uint8_t BEEPER_COMMAND_REPEAT = 0xFE;
constexpr uint8_t BEEPER_COMMAND_STOP = 0xFF;
}

struct Buzzer::BeeperEntry {
  BuzzerEvent mode;
  uint8_t priority; // 0 = highest priority
  const uint8_t* sequence;
};

Buzzer::Buzzer(Model& model): _model(model), _status(BUZZER_STATUS_IDLE), _wait(0), _scheme(NULL), _pos(0), _e(BUZZER_SILENCE) {}

int Buzzer::begin()
{
  if(_model.config.pin[PIN_BUZZER] == -1) return 0;
  Hal::Gpio::pinMode(_model.config.pin[PIN_BUZZER], OUTPUT);
  Hal::Gpio::digitalWrite(_model.config.pin[PIN_BUZZER], (pin_status_t)_model.config.buzzer.inverted);
  _model.state.buzzer.timer.setRate(100);
  _silence();

  return 1;
}

int Buzzer::update()
{
  if(_model.config.pin[PIN_BUZZER] == -1) return 0;
  if(!_model.state.buzzer.timer.check()) return 0;

  while(!_model.state.buzzer.empty())
  {
    _setMode(_model.state.buzzer.pop());
  }

  if(_status != BUZZER_STATUS_ACTIVE || !_scheme) return 1;
  if(_wait > millis()) return 1;

  _advanceSequence();

  return 1;
}

void Buzzer::_setMode(BuzzerEvent mode)
{
  if(mode == BUZZER_SILENCE)
  {
    _silence();
    return;
  }

  const BeeperEntry* candidate = findEntry(mode);
  if(!candidate || !candidate->sequence) return;

  const BeeperEntry* current = findEntry(_e);
  if(current && current->sequence && candidate->priority >= current->priority)
  {
    return;
  }

  _e = mode;
  _scheme = candidate->sequence;
  _pos = 0;
  _wait = 0;
  _status = BUZZER_STATUS_ACTIVE;
}

void Buzzer::_silence()
{
  _write(false);
  _status = BUZZER_STATUS_IDLE;
  _wait = 0;
  _scheme = NULL;
  _pos = 0;
  _e = BUZZER_SILENCE;
}

void Buzzer::_advanceSequence()
{
  if(!_scheme)
  {
    _silence();
    return;
  }

  bool wasRepeat = false;
  while(true)
  {
    const uint8_t cmd = _scheme[_pos];
    if(cmd == BEEPER_COMMAND_REPEAT)
    {
      if(wasRepeat)
      {
        _silence();
        return;
      }
      wasRepeat = true;
      _pos = 0;
      continue;
    }

    if(cmd == BEEPER_COMMAND_STOP)
    {
      _silence();
      return;
    }

    if(cmd == 0)
    {
      _pos++;
      continue;
    }

    _pos++;
    const bool on = (_pos % 2) == 1;
    _write(on);
    _wait = millis() + _delayMs(cmd);
    return;
  }
}

void Buzzer::_write(bool v)
{
  Hal::Gpio::digitalWrite(_model.config.pin[PIN_BUZZER], (pin_status_t)(_model.config.buzzer.inverted ? !v : v));
}

uint32_t Buzzer::_delayMs(uint8_t ticks) const
{
  return (uint32_t)ticks * 10u;
}

const Buzzer::BeeperEntry* Buzzer::entries()
{
  static const uint8_t beep_short[] = { 10, 10, BEEPER_COMMAND_STOP };
  static const uint8_t beep_arming[] = { 30, 5, 5, 5, BEEPER_COMMAND_STOP };
  static const uint8_t beep_armingGpsFix[] = { 5, 5, 15, 5, 5, 5, 15, 30, BEEPER_COMMAND_STOP };
  static const uint8_t beep_armingGpsNoFix[] = { 30, 5, 30, 5, 30, 5, BEEPER_COMMAND_STOP };
  static const uint8_t beep_armed[] = { 0, 245, 10, 5, BEEPER_COMMAND_REPEAT };
  static const uint8_t beep_disarm[] = { 15, 5, 15, 5, BEEPER_COMMAND_STOP };
  static const uint8_t beep_disarmRepeat[] = { 0, 100, 10, BEEPER_COMMAND_REPEAT };
  static const uint8_t beep_batLow[] = { 25, 50, BEEPER_COMMAND_REPEAT };
  static const uint8_t beep_batCrit[] = { 50, 2, BEEPER_COMMAND_REPEAT };
  static const uint8_t beep_rxLost[] = { 50, 50, BEEPER_COMMAND_REPEAT };
  static const uint8_t beep_sos[] = { 10, 10, 10, 10, 10, 40, 40, 10, 40, 10, 40, 40, 10, 10, 10, 10, 10, 70, BEEPER_COMMAND_REPEAT };
  static const uint8_t beep_ready[] = { 4, 5, 4, 5, 8, 5, 15, 5, 8, 5, 4, 5, 4, 5, BEEPER_COMMAND_STOP };
  static const uint8_t beep_2short[] = { 5, 5, 5, 5, BEEPER_COMMAND_STOP };
  static const uint8_t beep_2long[] = { 20, 15, 35, 5, BEEPER_COMMAND_STOP };
  static const uint8_t beep_gyroCalibrated[] = { 20, 10, 20, 10, 20, 10, BEEPER_COMMAND_STOP };
  static const uint8_t beep_camOpen[] = { 5, 15, 10, 15, 20, BEEPER_COMMAND_STOP };
  static const uint8_t beep_camClose[] = { 10, 8, 5, BEEPER_COMMAND_STOP };

  static const BeeperEntry beeperTable[] = {
    { BUZZER_GYRO_CALIBRATED, 0, beep_gyroCalibrated },
    { BUZZER_RX_LOST, 1, beep_rxLost },
    { BUZZER_RX_LOST_LANDING, 2, beep_sos },
    { BUZZER_DISARMING, 3, beep_disarm },
    { BUZZER_ARMING, 4, beep_arming },
    { BUZZER_ARMING_GPS_FIX, 5, beep_armingGpsFix },
    { BUZZER_ARMING_GPS_NO_FIX, 6, beep_armingGpsNoFix },
    { BUZZER_BAT_CRIT_LOW, 7, beep_batCrit },
    { BUZZER_BAT_LOW, 8, beep_batLow },
    { BUZZER_GPS_STATUS, 9, beep_short },
    { BUZZER_RX_SET, 10, beep_short },
    { BUZZER_ACC_CALIBRATION, 11, beep_2short },
    { BUZZER_ACC_CALIBRATION_FAIL, 12, beep_2long },
    { BUZZER_READY_BEEP, 13, beep_ready },
    { BUZZER_MULTI_BEEPS, 14, beep_short },
    { BUZZER_DISARM_REPEAT, 15, beep_disarmRepeat },
    { BUZZER_ARMED, 16, beep_armed },
    { BUZZER_SYSTEM_INIT, 17, beep_short },
    { BUZZER_USB, 18, beep_short },
    { BUZZER_BLACKBOX_ERASE, 19, beep_2short },
    { BUZZER_CRASH_FLIP_MODE, 20, beep_2long },
    { BUZZER_CAM_CONNECTION_OPEN, 21, beep_camOpen },
    { BUZZER_CAM_CONNECTION_CLOSE, 22, beep_camClose },
    { BUZZER_ALL, 23, NULL },
    { BUZZER_PREFERENCE, 24, NULL },
  };

  return beeperTable;
}

size_t Buzzer::entryCount()
{
  return (size_t)BUZZER_PREFERENCE;
}

const Buzzer::BeeperEntry* Buzzer::findEntry(BuzzerEvent mode)
{
  const BeeperEntry* table = entries();
  for(size_t i = 0; i < entryCount(); i++)
  {
    if(table[i].mode == mode)
    {
      return &table[i];
    }
  }
  return NULL;
}

}

}
