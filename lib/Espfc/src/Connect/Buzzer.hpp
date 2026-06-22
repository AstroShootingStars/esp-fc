#pragma once

#include "Model.h"

namespace Espfc {

namespace Connect {

enum BuzzerPlayStatus
{
  BUZZER_STATUS_IDLE,
  BUZZER_STATUS_ACTIVE
};

class Buzzer
{
public:
  Buzzer(Model& model);
  int begin();
  int update();

private:
  struct BeeperEntry;

  void _setMode(BuzzerEvent mode);
  void _silence();
  void _advanceSequence();
  void _write(bool v);
  uint32_t _delayMs(uint8_t ticks) const;

  static const BeeperEntry* entries();
  static size_t entryCount();
  static const BeeperEntry* findEntry(BuzzerEvent mode);

  Model& _model;

  BuzzerPlayStatus _status;
  uint32_t _wait;
  const uint8_t * _scheme;
  uint8_t _pos;
  BuzzerEvent _e;
};

}

}
