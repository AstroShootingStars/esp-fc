#include "Device/RangefinderVL53L1X.hpp"
#include <Arduino.h>
#include <Wire.h>

namespace Espfc::Device {

int RangefinderVL53L1X::begin(BusDevice* bus)
{
  return begin(bus, DEVICE_ADDRESS);
}

int RangefinderVL53L1X::begin(BusDevice* bus, uint8_t addr)
{
  if (!bus) return 0;
  setBus(bus, addr);

  if (!testConnection()) return 0;

  init();
  return 1;
}

RangefinderDeviceType RangefinderVL53L1X::getType() const
{
  return RANGEFINDER_VL53L1X;
}

uint16_t RangefinderVL53L1X::readDistance()
{
  // Trigger a single ranging cycle.
  writeReg(SYSTEM_START, 0x01);

  uint8_t status = 0;
  uint16_t timeout = 100;
  while (timeout-- > 0)
  {
    readReg(GPIO_TIO_HV_STATUS, status);
    if ((status & 0x01) == 0x01) break;
    delay(1);
  }

  uint16_t distance = 0;
  readReg16(RANGE_RESULT, distance);
  writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

  return distance;
}

bool RangefinderVL53L1X::testConnection()
{
  uint8_t whoAmI = 0;
  readReg(WHO_AM_I_REG, whoAmI);
  return whoAmI == WHO_AM_I_VALUE;
}

int8_t RangefinderVL53L1X::writeReg(uint16_t reg, uint8_t val)
{
  Wire.beginTransmission(_addr);
  Wire.write((uint8_t)(reg >> 8));
  Wire.write((uint8_t)(reg & 0xFF));
  Wire.write(val);
  return Wire.endTransmission() == 0 ? 1 : 0;
}

int8_t RangefinderVL53L1X::readReg(uint16_t reg, uint8_t& val)
{
  Wire.beginTransmission(_addr);
  Wire.write((uint8_t)(reg >> 8));
  Wire.write((uint8_t)(reg & 0xFF));
  if (Wire.endTransmission(false) != 0) return 0;

  if (Wire.requestFrom((int)_addr, 1) != 1) return 0;
  val = Wire.read();
  return 1;
}

int8_t RangefinderVL53L1X::readReg16(uint16_t reg, uint16_t& val)
{
  Wire.beginTransmission(_addr);
  Wire.write((uint8_t)(reg >> 8));
  Wire.write((uint8_t)(reg & 0xFF));
  if (Wire.endTransmission(false) != 0) return 0;

  if (Wire.requestFrom((int)_addr, 2) != 2) return 0;
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  val = ((uint16_t)hi << 8) | lo;
  return 1;
}

void RangefinderVL53L1X::init()
{
  // Set continuous ranging by default for simple polling use.
  writeReg(SYSTEM_START, 0x40);
}

} // namespace Espfc::Device
