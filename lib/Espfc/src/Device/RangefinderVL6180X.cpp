#include "Device/RangefinderVL6180X.hpp"
#include "Debug_Espfc.h"

namespace Espfc::Device {

int RangefinderVL6180X::begin(BusDevice* bus)
{
  return begin(bus, DEVICE_ADDRESS);
}

int RangefinderVL6180X::begin(BusDevice* bus, uint8_t addr)
{
  if (!bus) return 0;
  setBus(bus, addr);

  // Verify device connection
  if (!testConnection()) return 0;

  init();
  return 1;
}

RangefinderDeviceType RangefinderVL6180X::getType() const
{
  return RANGEFINDER_VL6180X;
}

uint16_t RangefinderVL6180X::readDistance()
{
  // Start measurement
  writeReg(SYSRANGE_START, 0x01);
  
  // Poll for measurement completion
  uint8_t status = 0;
  uint16_t timeout = 1000;
  while ((status & 0x04) == 0 && timeout-- > 0) {
    delay(1);
    readReg(RESULT_INTERRUPT_STATUS_GPIO, status);
  }
  
  // Read distance result
  uint8_t distance = 0;
  readReg(SYSRANGE_RESULT_RANGE, distance);
  
  // Clear interrupt
  writeReg(SYSTEM_INTERRUPT_CLEAR, 0x07);
  
  return (uint16_t)distance;
}

bool RangefinderVL6180X::testConnection()
{
  uint8_t whoAmI = 0;
  readReg(WHO_AM_I_REG, whoAmI);
  return whoAmI == WHO_AM_I_VALUE;
}

int8_t RangefinderVL6180X::writeReg(uint16_t reg, uint8_t val)
{
  uint8_t regAddr[2] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF)};
  // Write register address then value
  _bus->write(_addr, reg >> 8, 1, regAddr);
  return _bus->write(_addr, reg & 0xFF, 1, &val);
}

int8_t RangefinderVL6180X::readReg(uint16_t reg, uint8_t& val)
{
  uint8_t regAddr[2] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF)};
  // Write register address first
  _bus->write(_addr, reg >> 8, 1, regAddr);
  return _bus->read(_addr, reg & 0xFF, 1, &val);
}

int8_t RangefinderVL6180X::readReg16(uint16_t reg, uint16_t& val)
{
  uint8_t regAddr[2] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF)};
  // Write register address first
  _bus->write(_addr, reg >> 8, 1, regAddr);
  
  uint8_t data[2];
  int8_t ret = _bus->read(_addr, reg & 0xFF, 2, data);
  val = ((uint16_t)data[0] << 8) | data[1];
  return ret;
}

void RangefinderVL6180X::init()
{
  // Check if we need to reset (fresh out of reset)
  uint8_t fresh = 0;
  readReg(SYSTEM_FRESH_OUT_OF_RESET, fresh);
  
  if (fresh & 0x01) {
    // Perform initialization sequence
    writeReg(0x207, 0x01);
    writeReg(0x208, 0x01);
    writeReg(0x133, 0x01);
    
    // Set to continuous ranging mode
    writeReg(SYSRANGE_START, 0x01);
    
    // Clear reset bit
    writeReg(SYSTEM_FRESH_OUT_OF_RESET, 0x00);
  }
  
  // Enable continuous ranging at 10Hz
  writeReg(SYSRANGE_START, 0x01);
}

} // namespace Espfc::Device
