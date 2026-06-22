#include "Device/RangefinderVL53L0X.hpp"
#include "Debug_Espfc.h"

namespace Espfc::Device {

int RangefinderVL53L0X::begin(BusDevice* bus)
{
  return begin(bus, DEVICE_ADDRESS);
}

int RangefinderVL53L0X::begin(BusDevice* bus, uint8_t addr)
{
  if (!bus) return 0;
  setBus(bus, addr);

  // Verify device connection
  if (!testConnection()) return 0;

  init();
  return 1;
}

RangefinderDeviceType RangefinderVL53L0X::getType() const
{
  return RANGEFINDER_VL53L0X;
}

uint16_t RangefinderVL53L0X::readDistance()
{
  // Start measurement
  writeReg(SYSRANGE_START, 0x01);
  
  // Poll status
  uint8_t status = 0;
  uint16_t timeout = 1000;
  while ((status & 0x01) == 0 && timeout-- > 0) {
    delay(1);
    readReg(RESULT_RANGE_STATUS, status);
  }
  
  // Read distance result (16-bit value at offset 14-15)
  uint16_t distance = 0;
  readReg16(0x14 + 10, distance);
  
  // Clear interrupt
  writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);
  
  return distance;
}

bool RangefinderVL53L0X::testConnection()
{
  uint8_t whoAmI = 0;
  readReg(WHO_AM_I_REG, whoAmI);
  return whoAmI == WHO_AM_I_VALUE;
}

int8_t RangefinderVL53L0X::writeReg(uint8_t reg, uint8_t val)
{
  return _bus->write(_addr, reg, 1, &val);
}

int8_t RangefinderVL53L0X::writeReg16(uint8_t reg, uint16_t val)
{
  uint8_t data[2] = {(uint8_t)(val >> 8), (uint8_t)(val & 0xFF)};
  return _bus->write(_addr, reg, 2, data);
}

int8_t RangefinderVL53L0X::readReg(uint8_t reg, uint8_t& val)
{
  return _bus->read(_addr, reg, 1, &val);
}

int8_t RangefinderVL53L0X::readReg16(uint8_t reg, uint16_t& val)
{
  uint8_t data[2];
  int8_t ret = _bus->read(_addr, reg, 2, data);
  val = ((uint16_t)data[0] << 8) | data[1];
  return ret;
}

int8_t RangefinderVL53L0X::readReg32(uint8_t reg, uint32_t& val)
{
  uint8_t data[4];
  int8_t ret = _bus->read(_addr, reg, 4, data);
  val = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) | ((uint32_t)data[2] << 8) | data[3];
  return ret;
}

void RangefinderVL53L0X::init()
{
  // Set I2C address
  writeReg(I2C_SLAVE_DEVICE_ADDRESS, _addr << 1);
  
  // Load default settings for ranging
  uint8_t seq_config = 0x01;
  writeReg(SYSTEM_SEQUENCE_CONFIG, seq_config);
  
  // Set to 33ms intermeasurement period (30 Hz)
  writeReg16(SYSTEM_INTERMEASUREMENT_PERIOD, 33);
  
  // Set interrupt config
  writeReg(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
  writeReg(GPIO_HV_MUX_ACTIVE_HIGH, 0x01);
  
  // Start continuous ranging
  writeReg(SYSRANGE_START, 0x01);
}

} // namespace Espfc::Device
