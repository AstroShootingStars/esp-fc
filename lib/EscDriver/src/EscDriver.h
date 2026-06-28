#pragma once

#include "EscDriverBase.hpp"

#if defined(ESP8266)

  #define ESC_CHANNEL_COUNT 4
  #include "EscDriverEsp8266.h"
  #define EscDriver EscDriverEsp8266

  #define ESC_DRIVER_MOTOR_TIMER ESC_DRIVER_TIMER1
  #define ESC_DRIVER_SERVO_TIMER ESC_DRIVER_TIMER2

#elif defined(ESP32C3)

  #define ESC_CHANNEL_COUNT 4
  #include "EscDriverEsp32c3.h"
  #define EscDriver EscDriverEsp32c3

  #define ESC_DRIVER_MOTOR_TIMER ESC_DRIVER_TIMER0
  #define ESC_DRIVER_SERVO_TIMER ESC_DRIVER_TIMER1

#elif defined(ESP32S3) || defined(ESP32S2)

  #define ESC_CHANNEL_COUNT 4
  #include "EscDriverEsp32.h"
  #define EscDriver EscDriverEsp32

  #define ESC_DRIVER_MOTOR_TIMER 0
  #define ESC_DRIVER_SERVO_TIMER 1

#elif defined(ESP32)

  #define ESC_CHANNEL_COUNT RMT_CHANNEL_MAX
  #include "EscDriverEsp32.h"
  #define EscDriver EscDriverEsp32

  #define ESC_DRIVER_MOTOR_TIMER 0
  #define ESC_DRIVER_SERVO_TIMER 1

#elif defined(ARCH_RP2040)

  #define ESC_CHANNEL_COUNT 8
  #include "EscDriverRP2040.h"
  #define EscDriver EscDriverRP2040

  #define ESC_DRIVER_MOTOR_TIMER ESC_DRIVER_TIMER0
  #define ESC_DRIVER_SERVO_TIMER ESC_DRIVER_TIMER1

#elif defined(STM32F7xx) || defined(STM32F7) || defined(STM32H7xx) || defined(STM32H7)

  #define ESC_CHANNEL_COUNT 4
  class EscDriverStm32: public EscDriverBase
  {
    public:
      int begin(const EscConfig&) { return 1; }
      void end() {}
      int attach(size_t, int pin, int) { return pin != -1 ? 1 : 0; }
      int write(size_t, int) { return 1; }
      void apply() {}
      int pin(size_t) const { return -1; }
      uint32_t telemetry(size_t) const { return 0; }
  };
  #define EscDriver EscDriverStm32

  #define ESC_DRIVER_MOTOR_TIMER 0
  #define ESC_DRIVER_SERVO_TIMER 1

#elif defined(UNIT_TEST)

  #define ESC_CHANNEL_COUNT 4
  #define EscDriver EscDriverBase

  #define ESC_DRIVER_MOTOR_TIMER 0
  #define ESC_DRIVER_SERVO_TIMER 1

#else

  #error "Unsupported platform"

#endif
