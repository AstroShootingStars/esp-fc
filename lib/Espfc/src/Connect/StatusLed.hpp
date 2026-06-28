#pragma once
#include <cstdint>
#include <cstddef>

namespace Espfc { class Model; }

namespace Espfc::Connect
{

enum LedType
{
  LED_SIMPLE,
  LED_STRIP,
};

enum LedStatus
{
  LED_OFF,
  LED_BOOT,
  LED_OK,
  LED_ERROR,
  LED_ARMED,
  LED_ON,
};

class StatusLed
{

public:
  StatusLed();
  void begin(int8_t pin, uint8_t type, uint8_t invert);
  void update();
  void update(const Espfc::Model& model);
  void setStatus(LedStatus newStatus, bool force = false);

private:
  void _write(uint8_t val);
  int8_t _pin;
  uint8_t _type;
  uint8_t _invert;
  LedStatus _status;
  uint32_t _next;
  bool _state;
  size_t _step;
  int * _pattern;
};

}