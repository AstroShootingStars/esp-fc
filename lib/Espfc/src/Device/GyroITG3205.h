#ifndef _ESPFC_DEVICE_GYRO_ITG3205_H_
#define _ESPFC_DEVICE_GYRO_ITG3205_H_

#include "GyroDevice.h"

#define ITG3205_ADDRESS_FIRST 0x68
#define ITG3205_ADDRESS_SECOND 0x69

#define ITG3205_RA_WHO_AM_I 0x00
#define ITG3205_RA_SMPLRT_DIV 0x15
#define ITG3205_RA_DLPF_FS 0x16
#define ITG3205_RA_GYRO_XOUT_H 0x1D
#define ITG3205_RA_PWR_MGM 0x3E

#define ADXL345_ADDRESS 0x53
#define ADXL345_RA_DEVID 0x00
#define ADXL345_RA_BW_RATE 0x2C
#define ADXL345_RA_POWER_CTL 0x2D
#define ADXL345_RA_DATA_FORMAT 0x31
#define ADXL345_RA_DATAX0 0x32
#define ADXL345_DEVID 0xE5

namespace Espfc::Device {

class GyroITG3205: public GyroDevice
{
public:
  int begin(BusDevice* bus) override
  {
    return begin(bus, ITG3205_ADDRESS_FIRST) ? 1 : begin(bus, ITG3205_ADDRESS_SECOND);
  }

  int begin(BusDevice* bus, uint8_t addr) override
  {
    setBus(bus, addr);
    if (!testConnection()) return 0;

    // PLL with X gyro reference, clear sleep.
    _bus->writeByte(_addr, ITG3205_RA_PWR_MGM, 0x01);
    delay(15);

    setDLPFMode(GYRO_DLPF_256);
    setRate(8000);
    delay(5);

    _accelAvailable = initAdxl345();

    return 1;
  }

  GyroDeviceType getType() const override
  {
    return GYRO_ITG3205;
  }

  int FAST_CODE_ATTR readGyro(VectorInt16& v) override
  {
    uint8_t buffer[6];
    _bus->readFast(_addr, ITG3205_RA_GYRO_XOUT_H, 6, buffer);

    v.x = (((int16_t)buffer[0]) << 8) | buffer[1];
    v.y = (((int16_t)buffer[2]) << 8) | buffer[3];
    v.z = (((int16_t)buffer[4]) << 8) | buffer[5];

    return 1;
  }

  int readAccel(VectorInt16& v) override
  {
    if (!_accelAvailable)
    {
      v = VectorInt16(0, 0, 0);
      return 0;
    }

    uint8_t buffer[6];
    int8_t len = _bus->readFast(ADXL345_ADDRESS, ADXL345_RA_DATAX0, 6, buffer);
    if (len != 6)
    {
      v = VectorInt16(0, 0, 0);
      return 0;
    }

    // ADXL345 outputs little-endian values. Convert to MPU-like raw scale
    // (approximately x8) so existing accel scaling remains reasonable.
    int16_t ax = (int16_t)(((int16_t)buffer[1] << 8) | buffer[0]);
    int16_t ay = (int16_t)(((int16_t)buffer[3] << 8) | buffer[2]);
    int16_t az = (int16_t)(((int16_t)buffer[5] << 8) | buffer[4]);

    auto scaleToMpuRaw = [](int16_t in) -> int16_t {
      int32_t out = (int32_t)in * 8;
      if (out > 32767) out = 32767;
      if (out < -32768) out = -32768;
      return (int16_t)out;
    };

    v.x = scaleToMpuRaw(ax);
    v.y = scaleToMpuRaw(ay);
    v.z = scaleToMpuRaw(az);

    return 1;
  }

  bool accelAvailable() const
  {
    return _accelAvailable;
  }

  void setDLPFMode(uint8_t mode) override
  {
    _dlpf = mode;

    uint8_t dlpf = mode & 0x07;
    // FS_SEL=3 (2000dps) in bits 4:3, DLPF in bits 2:0.
    const uint8_t dlpfFs = (0x03 << 3) | dlpf;
    _bus->writeByte(_addr, ITG3205_RA_DLPF_FS, dlpfFs);
  }

  int getRate() const override
  {
    // DLPF setting 0 keeps internal sample at 8kHz, others are 1kHz.
    return (_dlpf == GYRO_DLPF_256 || _dlpf == GYRO_DLPF_EX) ? 8000 : 1000;
  }

  void setRate(int rate) override
  {
    const int base = getRate();
    const int safeRate = rate <= 0 ? base : rate;
    int dividerInt = (base / safeRate) - 1;
    if (dividerInt < 0) dividerInt = 0;
    if (dividerInt > 255) dividerInt = 255;
    uint8_t divider = (uint8_t)dividerInt;
    _bus->writeByte(_addr, ITG3205_RA_SMPLRT_DIV, divider);
  }

  bool testConnection() override
  {
    uint8_t whoami = 0;
    uint8_t len = _bus->readByte(_addr, ITG3205_RA_WHO_AM_I, &whoami);
    // ITG3205 WHO_AM_I usually reflects upper 7-bit address (0x68/0x69).
    return len == 1 && (whoami == 0x68 || whoami == 0x69);
  }

private:
  bool initAdxl345()
  {
    uint8_t devId = 0;
    uint8_t len = _bus->readByte(ADXL345_ADDRESS, ADXL345_RA_DEVID, &devId);
    if (len != 1 || devId != ADXL345_DEVID)
    {
      return false;
    }

    // 800 Hz output rate.
    if (!_bus->writeByte(ADXL345_ADDRESS, ADXL345_RA_BW_RATE, 0x0D)) return false;
    // Full-resolution, +/-16g.
    if (!_bus->writeByte(ADXL345_ADDRESS, ADXL345_RA_DATA_FORMAT, 0x0B)) return false;
    // Measurement mode.
    if (!_bus->writeByte(ADXL345_ADDRESS, ADXL345_RA_POWER_CTL, 0x08)) return false;

    return true;
  }

  uint8_t _dlpf = GYRO_DLPF_256;
  bool _accelAvailable = false;
};

} // namespace Espfc::Device

#endif