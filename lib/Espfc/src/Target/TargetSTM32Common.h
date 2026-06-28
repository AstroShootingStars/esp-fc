#pragma once

#include <Arduino.h>
#include "Device/SerialDevice.h"

namespace Espfc {

template<typename T>
inline int targetSerialInit(T& dev, const SerialDeviceConfig& conf)
{
  (void)conf;
  dev.begin(conf.baud);
  return 1;
}

template<typename T>
inline int targetSPIInit(T& dev, int8_t sck, int8_t mosi, int8_t miso, int8_t ss)
{
  (void)sck;
  (void)mosi;
  (void)miso;
  (void)ss;
  dev.begin();
  return 1;
}

template<typename T>
inline int targetI2CInit(T& dev, int8_t sda, int8_t scl, uint32_t speed)
{
  if(sda >= 0 && scl >= 0)
  {
    dev.begin((uint8_t)sda, (uint8_t)scl);
  }
  else
  {
    dev.begin();
  }
  dev.setClock(speed);
  return 1;
}

inline uint32_t getBoardId0()
{
#if defined(HAL_GetUIDw0)
  return HAL_GetUIDw0();
#else
  return 0;
#endif
}

inline uint32_t getBoardId1()
{
#if defined(HAL_GetUIDw1)
  return HAL_GetUIDw1();
#else
  return 0;
#endif
}

inline uint32_t getBoardId2()
{
#if defined(HAL_GetUIDw2)
  return HAL_GetUIDw2();
#else
  return 0;
#endif
}

inline void targetReset()
{
  NVIC_SystemReset();
}

inline uint32_t targetCpuFreq()
{
#if defined(HAL_RCC_GetSysClockFreq)
  return HAL_RCC_GetSysClockFreq() / 1000000u;
#else
  return F_CPU / 1000000u;
#endif
}

inline uint32_t targetFreeHeap()
{
  return 0;
}

}
