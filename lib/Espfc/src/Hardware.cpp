#include "Device/Baro/BaroBMP085.hpp"
#include "Device/Baro/BaroBMP280.hpp"
#include "Device/Baro/BaroSPL06.hpp"
#include "Device/BaroDevice.hpp"
#include "Device/GyroBMI160.h"
#include "Device/GyroDevice.h"
#include "Device/GyroICM20602.h"
#include "Device/GyroITG3205.h"
#include "Device/GyroLSM6DSO.h"
#include "Device/GyroMPU6050.h"
#include "Device/GyroMPU6500.h"
#include "Device/GyroMPU9250.h"
#include "Device/Mag/MagAK8963.hpp"
#include "Device/Mag/MagHMC5883L.hpp"
#include "Device/Mag/MagQMC5883L.hpp"
#include "Device/Mag/MagQMC5883P.hpp"
#include "Device/OpticalFlowMatek3901L0X.hpp"
#include "Device/OpticalFlowMicoAirMTF02P.hpp"
#include "Device/OledSSD1306.hpp"
#include "Device/RangefinderVL53L0X.hpp"
#include "Device/RangefinderVL53L1X.hpp"
#include "Device/RangefinderVL6180X.hpp"
#include "Hal/Gpio.h"
#include "Hardware.h"
#if defined(ARCH_RP2040)
#include <pico/bootrom.h>
#endif
#if defined(ESP32S3) || defined(ESP32S2) || defined(ESP32C3)
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_private/system_internal.h"
#endif
#if defined(ESPFC_WIFI_ALT)
#include <ESP8266WiFi.h>
#elif defined(ESPFC_WIFI)
#include <WiFi.h>
#endif

namespace {
#if defined(ESPFC_SPI_0)
#if defined(ESP32C3) || defined(ESP32S3) || defined(ESP32S2)
static SPIClass SPI1(HSPI);
#elif defined(ESP32)
static SPIClass SPI1(VSPI);
#endif
static Espfc::Device::BusSPI spiBus(ESPFC_SPI_0_DEV);
#endif
#if defined(ESPFC_I2C_0)
static Espfc::Device::BusI2C i2cBus(WireInstance);
#endif
static Espfc::Device::BusSlave gyroSlaveBus;
static Espfc::Device::GyroMPU6050 mpu6050;
static Espfc::Device::GyroMPU6500 mpu6500;
static Espfc::Device::GyroMPU9250 mpu9250;
static Espfc::Device::GyroLSM6DSO lsm6dso;
static Espfc::Device::GyroICM20602 icm20602;
static Espfc::Device::GyroBMI160 bmi160;
static Espfc::Device::GyroITG3205 itg3205;
static Espfc::Device::Mag::MagHMC5883L hmc5883l;
static Espfc::Device::Mag::MagQMC5883L qmc5883l;
static Espfc::Device::Mag::MagQMC5883P qmc5883p;
static Espfc::Device::Mag::MagAK8963 ak8963;
static Espfc::Device::Baro::BaroBMP085 bmp085;
static Espfc::Device::Baro::BaroBMP280 bmp280;
static Espfc::Device::Baro::BaroSPL06 spl06;
static Espfc::Device::RangefinderVL53L0X vl53l0x;
static Espfc::Device::RangefinderVL6180X vl6180x;
static Espfc::Device::RangefinderVL53L1X vl53l1x;
static Espfc::Device::OpticalFlowMatek3901L0X opFlowMatek;
static Espfc::Device::OpticalFlowMicoAirMTF02P opFlowMicoAir;
static Espfc::Device::OledSSD1306 oledSsd1306;
} // namespace

namespace Espfc {

Hardware::Hardware(Model& model): _model(model) {}

int Hardware::begin()
{
#if defined(ESPFC_GY87_I2C) && (ESPFC_GY87_I2C)
  // GY-87 wiring preset only: keep sensor type dynamic by I2C detection.
  _model.config.pin[PIN_I2C_0_SDA] = 9;
  _model.config.pin[PIN_I2C_0_SCL] = 10;
  _model.config.i2cSpeed = 100;
  // Disable SPI CS defaults to keep probing on I2C only for this module.
  _model.config.pin[PIN_SPI_CS0] = -1;
  _model.config.pin[PIN_SPI_CS1] = -1;
  _model.logger.info().logln(F("SENSORS I2C preset pins 9/10 @100k"));
#endif

  // Recover from accidental configurator writes that disable core sensors.
  // If gyro is disabled, force a safe auto/default core sensor profile.
  if(_model.config.gyro.dev == GYRO_NONE)
  {
    _model.config.gyro.dev = GYRO_AUTO;
    if(_model.config.accel.dev == GYRO_NONE) _model.config.accel.dev = GYRO_AUTO;
    if(_model.config.baro.dev == BARO_NONE) _model.config.baro.dev = BARO_DEFAULT;
    if(_model.config.mag.dev == MAG_NONE) _model.config.mag.dev = MAG_DEFAULT;
    _model.logger.info().log(F("SENSORS")).logln(F("recovered gyro/accel/baro/mag defaults"));
  }

  // Keep accel enabled whenever gyro is enabled unless user explicitly changed it later.
  if(_model.config.gyro.dev != GYRO_NONE && _model.config.accel.dev == GYRO_NONE)
  {
    _model.config.accel.dev = GYRO_AUTO;
    _model.logger.info().log(F("SENSORS")).logln(F("recovered accel default"));
  }

#if defined(ESPFC_I2C_0)
  // Recover broken persisted I2C configuration (e.g. pins set to -1 by UI writes).
  if(_model.config.pin[PIN_I2C_0_SDA] < 0 || _model.config.pin[PIN_I2C_0_SCL] < 0)
  {
    _model.config.pin[PIN_I2C_0_SDA] = ESPFC_I2C_0_SDA;
    _model.config.pin[PIN_I2C_0_SCL] = ESPFC_I2C_0_SCL;
    if(_model.config.i2cSpeed <= 0) _model.config.i2cSpeed = 400;
    _model.logger.info().log(F("SENSORS")).log(F("recovered i2c pins")).log(_model.config.pin[PIN_I2C_0_SDA]).logln(_model.config.pin[PIN_I2C_0_SCL]);
  }
#endif

  _model.logger.info()
      .log(F("SENSORS"))
      .log(F("begin i2c sda/scl/speed"))
      .log(_model.config.pin[PIN_I2C_0_SDA])
      .log(_model.config.pin[PIN_I2C_0_SCL])
      .logln(_model.config.i2cSpeed);

  initBus();

#if defined(ESPFC_I2C_0)
  // Dynamic sensor-type hints from detected I2C addresses/IDs.
  // This avoids hard-coding specific IMU/baro/mag while still preferring likely devices.
  if(_model.config.pin[PIN_I2C_0_SDA] != -1 && _model.config.pin[PIN_I2C_0_SCL] != -1)
  {
    auto probeI2cAddr = [&](uint8_t addr) -> bool {
      WireInstance.beginTransmission(addr);
      return WireInstance.endTransmission() == 0;
    };

    auto readI2cReg = [&](uint8_t addr, uint8_t reg, uint8_t& out) -> bool {
      return i2cBus.read(addr, reg, 1, &out) == 1;
    };

#if defined(ESPFC_I2C_SCAN_DEBUG) && (ESPFC_I2C_SCAN_DEBUG)
    int ackCount = 0;
    for(uint8_t addr = 0x03; addr < 0x78; addr++)
    {
      if(!probeI2cAddr(addr)) continue;
      ackCount++;
      _model.logger.info().log(F("I2C ACK")).logln((int)addr);
    }
    _model.logger.info().log(F("I2C ACK COUNT")).logln(ackCount);
#endif

    const bool has68 = probeI2cAddr(0x68);
    const bool has69 = probeI2cAddr(0x69);
    const bool has1E = probeI2cAddr(0x1E);
    const bool has0C = probeI2cAddr(0x0C);
    const bool has0D = probeI2cAddr(0x0D);

    uint8_t id76 = 0;
    uint8_t id77 = 0;
    const bool has76Addr = probeI2cAddr(0x76);
    const bool has77Addr = probeI2cAddr(0x77);
    const bool has76 = has76Addr && readI2cReg(0x76, 0xD0, id76);
    const bool has77 = has77Addr && readI2cReg(0x77, 0xD0, id77);

    bool hasSpl06 = false;
    if(has76Addr && !has76)
    {
      uint8_t spl = 0;
      hasSpl06 = readI2cReg(0x76, 0x0D, spl) && spl == 0x10;
    }
    if(has77Addr && !has77 && !hasSpl06)
    {
      uint8_t spl = 0;
      hasSpl06 = readI2cReg(0x77, 0x0D, spl) && spl == 0x10;
    }

    if((has68 || has69) && (_model.config.gyro.dev == GYRO_AUTO || _model.config.gyro.dev == GYRO_NONE))
    {
      _model.config.gyro.dev = GYRO_AUTO;
      if(_model.config.accel.dev == GYRO_NONE || _model.config.accel.dev == GYRO_AUTO) _model.config.accel.dev = GYRO_AUTO;
    }

    if(has1E && (_model.config.mag.dev == MAG_DEFAULT || _model.config.mag.dev == MAG_NONE))
    {
      _model.config.mag.dev = MAG_HMC5883L;
    }
    else if(has0C && (_model.config.mag.dev == MAG_DEFAULT || _model.config.mag.dev == MAG_NONE))
    {
      _model.config.mag.dev = MAG_AK8963;
    }
    else if(has0D && (_model.config.mag.dev == MAG_DEFAULT || _model.config.mag.dev == MAG_NONE))
    {
      _model.config.mag.dev = MAG_QMC5883L;
    }

    if((has76 || has77 || hasSpl06) && (_model.config.baro.dev == BARO_DEFAULT || _model.config.baro.dev == BARO_NONE))
    {
      const uint8_t baroId = has77 ? id77 : id76;
      if(baroId == 0x55) _model.config.baro.dev = BARO_BMP085;
      else if(baroId == 0x58 || baroId == 0x60) _model.config.baro.dev = BARO_BMP280;
      else if(hasSpl06) _model.config.baro.dev = BARO_SPL06;
      else _model.config.baro.dev = BARO_DEFAULT;
    }

#if defined(ESPFC_I2C_SCAN_DEBUG) && (ESPFC_I2C_SCAN_DEBUG)
    _model.logger.info().log(F("I2C addr 68/69/1E/0C/0D/76/77"))
      .log((int)has68).log((int)has69).log((int)has1E).log((int)has0C).log((int)has0D).log((int)has76Addr).logln((int)has77Addr);
#endif
  }
#endif

  detectGyro();
  detectMag();
  detectBaro();

#if defined(ESPFC_SPI_0) && !(defined(ESPFC_GY87_I2C) && (ESPFC_GY87_I2C))
  // If no core sensor is detected, retry once with target-default SPI pins.
  if(!_model.state.gyro.present && !_model.state.mag.present && !_model.state.baro.present)
  {
    bool spiPinsRecovered = false;

    if(_model.config.pin[PIN_SPI_0_SCK] != ESPFC_SPI_0_SCK)
    {
      _model.config.pin[PIN_SPI_0_SCK] = ESPFC_SPI_0_SCK;
      spiPinsRecovered = true;
    }
    if(_model.config.pin[PIN_SPI_0_MOSI] != ESPFC_SPI_0_MOSI)
    {
      _model.config.pin[PIN_SPI_0_MOSI] = ESPFC_SPI_0_MOSI;
      spiPinsRecovered = true;
    }
    if(_model.config.pin[PIN_SPI_0_MISO] != ESPFC_SPI_0_MISO)
    {
      _model.config.pin[PIN_SPI_0_MISO] = ESPFC_SPI_0_MISO;
      spiPinsRecovered = true;
    }
    if(_model.config.pin[PIN_SPI_CS0] != ESPFC_SPI_CS_GYRO)
    {
      _model.config.pin[PIN_SPI_CS0] = ESPFC_SPI_CS_GYRO;
      spiPinsRecovered = true;
    }
    if(_model.config.pin[PIN_SPI_CS1] != ESPFC_SPI_CS_BARO)
    {
      _model.config.pin[PIN_SPI_CS1] = ESPFC_SPI_CS_BARO;
      spiPinsRecovered = true;
    }

    if(spiPinsRecovered)
    {
      _model.logger.info().logln(F("SENSORS retry spi defaults"));
      initBus();
      detectGyro();
      detectMag();
      detectBaro();
    }
  }
#endif

#if defined(ESPFC_I2C_0)
  // If no core sensor is detected, retry with target-default I2C pins/speed once.
  if(!_model.state.gyro.present && !_model.state.mag.present && !_model.state.baro.present)
  {
    _model.config.pin[PIN_I2C_0_SDA] = ESPFC_I2C_0_SDA;
    _model.config.pin[PIN_I2C_0_SCL] = ESPFC_I2C_0_SCL;
    _model.config.i2cSpeed = 400;
    _model.logger.info().logln(F("SENSORS retry i2c defaults"));
    initBus();
    detectGyro();
    detectMag();
    detectBaro();

    if(!_model.state.gyro.present && !_model.state.mag.present && !_model.state.baro.present)
    {
      static const uint8_t probeAddr[] = {0x68, 0x69, 0x1E, 0x0D, 0x0C, 0x76, 0x77};
      for(size_t i = 0; i < sizeof(probeAddr); i++)
      {
        uint8_t v = 0;
        const int8_t c = i2cBus.read(probeAddr[i], 0x00, 1, &v);
        _model.logger.info().log(F("I2C probe")).log((int)probeAddr[i]).log(c).logln((int)v);
      }

      uint8_t who = 0;
      const int8_t whoCount = i2cBus.read(0x68, 0x75, 1, &who);
      _model.logger.info().log(F("I2C whoami 0x68")).log(whoCount).logln((int)who);
    }
  }
#endif

  detectRangefinder();
  detectTemperatureSensor();
  detectOpticalFlow();
  detectOled();

  _model.logger.info()
      .log(F("SENSORS"))
      .log(F("detect g/a/m/b"))
      .log((int)_model.state.gyro.present)
      .log((int)_model.state.accel.present)
      .log((int)_model.state.mag.present)
      .logln((int)_model.state.baro.present);
  return 1;
}

void Hardware::onI2CError()
{
  _model.state.i2cErrorCount++;
  _model.state.i2cErrorDelta++;
}

void Hardware::initBus()
{
#if defined(ESPFC_SPI_0)
  int spiResult = spiBus.begin(_model.config.pin[PIN_SPI_0_SCK], _model.config.pin[PIN_SPI_0_MOSI],
                               _model.config.pin[PIN_SPI_0_MISO]);
  _model.logger.info()
      .log(F("SPI"))
      .log(_model.config.pin[PIN_SPI_0_SCK])
      .log(_model.config.pin[PIN_SPI_0_MOSI])
      .log(_model.config.pin[PIN_SPI_0_MISO])
      .logln(spiResult);
#endif
#if defined(ESPFC_I2C_0)
  int i2cResult =
      i2cBus.begin(_model.config.pin[PIN_I2C_0_SDA], _model.config.pin[PIN_I2C_0_SCL], _model.config.i2cSpeed * 1000ul);
  i2cBus.onError = std::bind(&Hardware::onI2CError, this);
  _model.logger.info()
      .log(F("I2C"))
      .log(F("begin"))
      .log(_model.config.pin[PIN_I2C_0_SDA])
      .log(_model.config.pin[PIN_I2C_0_SCL])
      .log(_model.config.i2cSpeed)
      .logln(i2cResult);

#endif
}

void Hardware::detectGyro()
{
  _model.state.gyro.dev = nullptr;
  _model.state.gyro.present = false;
  _model.state.accel.present = false;

  if (_model.config.gyro.dev == GYRO_NONE) return;

  Device::GyroDevice* detectedGyro = nullptr;
  auto scanGyroI2c = [&]() -> Device::GyroDevice* {
    Device::GyroDevice* gyro = nullptr;
    _model.logger.info().log(F("I2C")).log(F("scanning for gyro...")).logln("");
    if (!gyro && detectDevice(mpu9250, i2cBus)) gyro = &mpu9250;
    if (!gyro && detectDevice(mpu6500, i2cBus)) gyro = &mpu6500;
    if (!gyro && detectDevice(icm20602, i2cBus)) gyro = &icm20602;
    if (!gyro && detectDevice(bmi160, i2cBus)) gyro = &bmi160;
    if (!gyro && detectDevice(mpu6050, i2cBus)) gyro = &mpu6050;
    if (!gyro && detectDevice(itg3205, i2cBus)) gyro = &itg3205;
    if (!gyro && detectDevice(lsm6dso, i2cBus)) gyro = &lsm6dso;
    return gyro;
  };
#if defined(ESPFC_SPI_0)
  if (_model.config.pin[PIN_SPI_CS0] != -1)
  {
    Hal::Gpio::digitalWrite(_model.config.pin[PIN_SPI_CS0], HIGH);
    Hal::Gpio::pinMode(_model.config.pin[PIN_SPI_CS0], OUTPUT);
    if (!detectedGyro && detectDevice(mpu9250, spiBus, _model.config.pin[PIN_SPI_CS0])) detectedGyro = &mpu9250;
    if (!detectedGyro && detectDevice(mpu6500, spiBus, _model.config.pin[PIN_SPI_CS0])) detectedGyro = &mpu6500;
    if (!detectedGyro && detectDevice(icm20602, spiBus, _model.config.pin[PIN_SPI_CS0])) detectedGyro = &icm20602;
    if (!detectedGyro && detectDevice(bmi160, spiBus, _model.config.pin[PIN_SPI_CS0])) detectedGyro = &bmi160;
    if (!detectedGyro && detectDevice(lsm6dso, spiBus, _model.config.pin[PIN_SPI_CS0])) detectedGyro = &lsm6dso;
    if (detectedGyro) gyroSlaveBus.begin(&spiBus, detectedGyro->getAddress());
  }
#endif
#if defined(ESPFC_I2C_0)
  if (!detectedGyro && _model.config.pin[PIN_I2C_0_SDA] != -1 && _model.config.pin[PIN_I2C_0_SCL] != -1)
  {
    auto scanGyroI2cOn = [&](int sda, int scl, int speedKhz) -> Device::GyroDevice* {
      const int i2cResult = i2cBus.begin(sda, scl, (uint32_t)speedKhz * 1000ul);
      if(!i2cResult) return nullptr;
      _model.logger.info().log(F("I2C")).log(F("scan gyro")).log(sda).log(scl).logln(speedKhz);
      return scanGyroI2c();
    };

    // Prefer configured pins/speed first.
    detectedGyro = scanGyroI2cOn(_model.config.pin[PIN_I2C_0_SDA], _model.config.pin[PIN_I2C_0_SCL], _model.config.i2cSpeed);

    // Older IMU modules (e.g. ITG3205 on GY-85) can fail at higher bus rates.
    if(!detectedGyro && _model.config.i2cSpeed > 400)
    {
      detectedGyro = scanGyroI2cOn(_model.config.pin[PIN_I2C_0_SDA], _model.config.pin[PIN_I2C_0_SCL], 400);
      if(detectedGyro) _model.config.i2cSpeed = 400;
    }
    if(!detectedGyro && _model.config.i2cSpeed > 100)
    {
      detectedGyro = scanGyroI2cOn(_model.config.pin[PIN_I2C_0_SDA], _model.config.pin[PIN_I2C_0_SCL], 100);
      if(detectedGyro) _model.config.i2cSpeed = 100;
    }

#if defined(ESP32S3)
    // Some ESP32-S3 boards use either GPIO9/10 or GPIO21/22 for user I2C.
    // If initial scan fails, probe both common pin pairs and lock to the one that works.
    // Keep this short to avoid long cold-boot delays on boards without I2C sensors attached.
    if (!detectedGyro)
    {
      const int pairs[][2] = {{9, 10}, {21, 22}};
      for(size_t i = 0; i < 2 && !detectedGyro; i++)
      {
        const int altSda = pairs[i][0];
        const int altScl = pairs[i][1];
        if(_model.config.pin[PIN_I2C_0_SDA] == altSda && _model.config.pin[PIN_I2C_0_SCL] == altScl) continue;

        detectedGyro = scanGyroI2cOn(altSda, altScl, _model.config.i2cSpeed);
        if(!detectedGyro && _model.config.i2cSpeed > 400)
        {
          detectedGyro = scanGyroI2cOn(altSda, altScl, 400);
          if(detectedGyro) _model.config.i2cSpeed = 400;
        }
        if(!detectedGyro && _model.config.i2cSpeed > 100)
        {
          detectedGyro = scanGyroI2cOn(altSda, altScl, 100);
          if(detectedGyro) _model.config.i2cSpeed = 100;
        }

        if (detectedGyro)
        {
          _model.config.pin[PIN_I2C_0_SDA] = altSda;
          _model.config.pin[PIN_I2C_0_SCL] = altScl;
        }
      }
    }
#endif

    if (detectedGyro) gyroSlaveBus.begin(&i2cBus, detectedGyro->getAddress());
  }
#endif
  if (!detectedGyro) return;

  detectedGyro->setDLPFMode(_model.config.gyro.dlpf);
  _model.state.gyro.dev = detectedGyro;
  _model.state.gyro.present = (bool)detectedGyro;
  _model.state.accel.present = _model.state.gyro.present && _model.config.accel.dev != GYRO_NONE;
  if (detectedGyro && detectedGyro->getType() == GYRO_ITG3205)
  {
    _model.state.accel.present = ((Device::GyroITG3205*)detectedGyro)->accelAvailable() && _model.config.accel.dev != GYRO_NONE;
  }
  _model.state.gyro.clock = detectedGyro->getRate();
}

void Hardware::detectMag()
{
  _model.state.mag.dev = nullptr;
  _model.state.mag.present = false;
  _model.state.mag.rate = 0;

  if (_model.config.mag.dev == MAG_NONE) return;

  Device::MagDevice* detectedMag = nullptr;
#if defined(ESPFC_I2C_0)
  if (_model.config.pin[PIN_I2C_0_SDA] != -1 && _model.config.pin[PIN_I2C_0_SCL] != -1)
  {
    if (!detectedMag && detectDevice(ak8963, i2cBus)) detectedMag = &ak8963;
    if (!detectedMag && detectDevice(hmc5883l, i2cBus)) detectedMag = &hmc5883l;
    if (!detectedMag && detectDevice(qmc5883l, i2cBus)) detectedMag = &qmc5883l;
    if (!detectedMag && detectDevice(qmc5883p, i2cBus)) detectedMag = &qmc5883p;
  }
#endif
  if (gyroSlaveBus.getBus())
  {
    if (!detectedMag && detectDevice(ak8963, gyroSlaveBus)) detectedMag = &ak8963;
    if (!detectedMag && detectDevice(hmc5883l, gyroSlaveBus)) detectedMag = &hmc5883l;
    if (!detectedMag && detectDevice(qmc5883l, gyroSlaveBus)) detectedMag = &qmc5883l;
    if (!detectedMag && detectDevice(qmc5883p, gyroSlaveBus)) detectedMag = &qmc5883p;
  }
  _model.state.mag.dev = detectedMag;
  _model.state.mag.present = (bool)detectedMag;
  _model.state.mag.rate = detectedMag ? detectedMag->getRate() : 0;
}

void Hardware::detectBaro()
{
  _model.state.baro.dev = nullptr;
  _model.state.baro.present = false;

  if (_model.config.baro.dev == BARO_NONE) return;

  // GY-85 class builds (ITG3205 + ADXL345 + HMC5883L) do not include barometer.
  // When baro is in auto/default mode, skip probing to avoid false positives.
  if (_model.config.baro.dev == BARO_DEFAULT && _model.state.gyro.dev && _model.state.gyro.dev->getType() == GYRO_ITG3205)
  {
    _model.state.baro.dev = nullptr;
    _model.state.baro.present = false;
    return;
  }

  Device::BaroDevice* detectedBaro = nullptr;
#if defined(ESPFC_SPI_0)
  if (_model.config.pin[PIN_SPI_CS1] != -1)
  {
    Hal::Gpio::digitalWrite(_model.config.pin[PIN_SPI_CS1], HIGH);
    Hal::Gpio::pinMode(_model.config.pin[PIN_SPI_CS1], OUTPUT);
    if (!detectedBaro && detectDevice(bmp280, spiBus, _model.config.pin[PIN_SPI_CS1])) detectedBaro = &bmp280;
    if (!detectedBaro && detectDevice(bmp085, spiBus, _model.config.pin[PIN_SPI_CS1])) detectedBaro = &bmp085;
    if (!detectedBaro && detectDevice(spl06, spiBus, _model.config.pin[PIN_SPI_CS1])) detectedBaro = &spl06;
  }
#endif
#if defined(ESPFC_I2C_0)
  if (_model.config.pin[PIN_I2C_0_SDA] != -1 && _model.config.pin[PIN_I2C_0_SCL] != -1)
  {
    if (!detectedBaro && detectDevice(bmp280, i2cBus)) detectedBaro = &bmp280;
    if (!detectedBaro && detectDevice(bmp085, i2cBus)) detectedBaro = &bmp085;
    if (!detectedBaro && detectDevice(spl06, i2cBus)) detectedBaro = &spl06;
  }
#endif
  if (gyroSlaveBus.getBus())
  {
    if (!detectedBaro && detectDevice(bmp280, gyroSlaveBus)) detectedBaro = &bmp280;
    if (!detectedBaro && detectDevice(bmp085, gyroSlaveBus)) detectedBaro = &bmp085;
    if (!detectedBaro && detectDevice(spl06, gyroSlaveBus)) detectedBaro = &spl06;
  }

  _model.state.baro.dev = detectedBaro;
  _model.state.baro.present = (bool)detectedBaro;
}

void Hardware::detectRangefinder()
{
  for(size_t i = 0; i < RANGEFINDER_COUNT; i++)
  {
    _model.state.rangefinder[i].dev = nullptr;
    _model.state.rangefinder[i].present = false;
    _model.state.rangefinder[i].rate = 0;
    _model.state.rangefinder[i].position = i;
  }

  auto detectByConfig = [&](const RangefinderConfig& cfg) -> Device::RangefinderDevice* {
    if(!cfg.enabled) return nullptr;
    if(cfg.dev == Device::RANGEFINDER_NONE) return nullptr;
    if(cfg.bus == BUS_NONE) return nullptr;
    if(cfg.bus != BUS_AUTO && cfg.bus != BUS_I2C) return nullptr;

    const int8_t wantedDev = cfg.dev;
    const uint8_t wantedAddr = cfg.address;

    Device::RangefinderDevice* detectedRangefinder = nullptr;
#if defined(ESPFC_I2C_0)
    if (_model.config.pin[PIN_I2C_0_SDA] != -1 && _model.config.pin[PIN_I2C_0_SCL] != -1)
    {
      if (!detectedRangefinder && (wantedDev == Device::RANGEFINDER_DEFAULT || wantedDev == Device::RANGEFINDER_VL53L0X))
      {
        if (wantedAddr > 0 && wantedAddr < 0x80) {
          if (detectDevice(vl53l0x, i2cBus, wantedAddr)) detectedRangefinder = &vl53l0x;
        } else {
          if (detectDevice(vl53l0x, i2cBus)) detectedRangefinder = &vl53l0x;
        }
      }
      if (!detectedRangefinder && (wantedDev == Device::RANGEFINDER_DEFAULT || wantedDev == Device::RANGEFINDER_VL6180X))
      {
        if (wantedAddr > 0 && wantedAddr < 0x80) {
          if (detectDevice(vl6180x, i2cBus, wantedAddr)) detectedRangefinder = &vl6180x;
        } else {
          if (detectDevice(vl6180x, i2cBus)) detectedRangefinder = &vl6180x;
        }
      }
      if (!detectedRangefinder && (wantedDev == Device::RANGEFINDER_DEFAULT || wantedDev == Device::RANGEFINDER_VL53L1X))
      {
        if (wantedAddr > 0 && wantedAddr < 0x80) {
          if (detectDevice(vl53l1x, i2cBus, wantedAddr)) detectedRangefinder = &vl53l1x;
        } else {
          if (detectDevice(vl53l1x, i2cBus)) detectedRangefinder = &vl53l1x;
        }
      }
    }
#endif

    return detectedRangefinder;
  };

  Device::RangefinderDevice* bottom = detectByConfig(_model.config.rangefinder[RANGEFINDER_BOTTOM]);
  Device::RangefinderDevice* front = detectByConfig(_model.config.rangefinder[RANGEFINDER_FRONT]);

  const auto& bottomCfg = _model.config.rangefinder[RANGEFINDER_BOTTOM];
  const auto& frontCfg = _model.config.rangefinder[RANGEFINDER_FRONT];
  const bool duplicateRangefinderConfig = bottomCfg.enabled && frontCfg.enabled &&
    bottomCfg.bus == frontCfg.bus &&
    bottomCfg.dev == frontCfg.dev &&
    bottomCfg.address == frontCfg.address &&
    bottomCfg.address > 0;

  if(duplicateRangefinderConfig)
  {
    front = nullptr;
  }

  _model.state.rangefinder[RANGEFINDER_BOTTOM].dev = bottom;
  _model.state.rangefinder[RANGEFINDER_BOTTOM].present = (bool)bottom;
  _model.state.rangefinder[RANGEFINDER_BOTTOM].rate = bottom ? bottom->getRate() : 0;

  _model.state.rangefinder[RANGEFINDER_FRONT].dev = front;
  _model.state.rangefinder[RANGEFINDER_FRONT].present = (bool)front;
  _model.state.rangefinder[RANGEFINDER_FRONT].rate = front ? front->getRate() : 0;
}

void Hardware::detectTemperatureSensor()
{
  // Temperature sensor detection is optional and graceful - if not found, thermal compensation is simply disabled.
  // Try to detect temperature sensors only if thermal compensation is enabled on at least one rangefinder.
  
  bool tempCompensationNeeded = false;
  for(size_t i = 0; i < RANGEFINDER_COUNT; i++)
  {
    if(_model.config.rangefinder[i].tempCompensationEnabled)
    {
      tempCompensationNeeded = true;
      break;
    }
  }
  
  if(!tempCompensationNeeded) return;  // No detection needed if thermal compensation is disabled
  
  // Try to detect I2C temperature sensors (TMP102, BME280, BMP280)
#if defined(ESPFC_I2C_0)
  if(_model.config.pin[PIN_I2C_0_SDA] != -1 && _model.config.pin[PIN_I2C_0_SCL] != -1)
  {
    // For now, log that I2C temperature sensor detection is available
    // Actual I2C temp sensor implementations (TMP102, BME280, BMP280) can be added later
    // This structure allows graceful fallback if sensor not found
    _model.logger.info().log(F("TempSensor")).logln(F("I2C capable"));
  }
#endif
  
  // Try to detect NTC thermistor on configured ADC pin
  // NTC detection: if ADC pin is configured and not -1, assume potential for thermistor
  // Actual ADC reading will happen at runtime when thermal compensation is enabled
#ifdef ESPFC_ADC_0
  if(_model.config.pin[PIN_INPUT_ADC_0] != -1 && _model.config.rangefinder[0].tempSensorType == 1) // 1 = NTC_ADC
  {
    // Configure ADC pin for thermistor reading
    // Actual initialization deferred to first use (lazy init)
    _model.logger.info()
        .log(F("TempSensor"))
        .log(F("NTC"))
        .logln(_model.config.pin[PIN_INPUT_ADC_0]);
  }
#endif
  
  // Log thermal compensation configuration status
  for(size_t i = 0; i < RANGEFINDER_COUNT; i++)
  {
    if(_model.config.rangefinder[i].tempCompensationEnabled)
    {
      const char* sensorNames[] = {"NONE", "NTC_ADC", "BMP280", "VL53L0X"};
      const char* sensorName = (_model.config.rangefinder[i].tempSensorType < 4) 
                                ? sensorNames[_model.config.rangefinder[i].tempSensorType]
                                : "UNKNOWN";
      
      _model.logger.info()
          .log(F("RF"))
          .log((int)i)
          .log(F("TempComp"))
          .log(sensorName)
          .log(F("coeff="))
          .logln(_model.config.rangefinder[i].tempCoefficient);
    }
  }
}

void Hardware::detectOpticalFlow()
{
  if (_model.config.opticalFlow.dev == Device::OPFLOW_NONE) return;

  Device::OpticalFlowDevice* detectedFlow = nullptr;

#if defined(ESPFC_I2C_0)
  if (_model.config.pin[PIN_I2C_0_SDA] != -1 && _model.config.pin[PIN_I2C_0_SCL] != -1)
  {
    if (!detectedFlow && (_model.config.opticalFlow.dev == Device::OPFLOW_DEFAULT || _model.config.opticalFlow.dev == Device::OPFLOW_MATEK_3901_L0X))
    {
      if (detectDevice(opFlowMatek, i2cBus)) detectedFlow = &opFlowMatek;
    }
    if (!detectedFlow && (_model.config.opticalFlow.dev == Device::OPFLOW_DEFAULT || _model.config.opticalFlow.dev == Device::OPFLOW_MICOAIR_MTF_02P))
    {
      if (detectDevice(opFlowMicoAir, i2cBus)) detectedFlow = &opFlowMicoAir;
    }
  }
#endif

  _model.state.opticalFlow.dev = detectedFlow;
  _model.state.opticalFlow.present = (bool)detectedFlow;
  _model.state.opticalFlow.rate = detectedFlow ? detectedFlow->getRate() : ((_model.config.opticalFlow.dev == Device::OPFLOW_DEFAULT || _model.config.opticalFlow.dev == Device::OPFLOW_MSP) ? 100 : 0);
}

void Hardware::detectOled()
{
  if (_model.config.oled.dev == Device::OLED_NONE) return;

  oledSsd1306.setHeight(_model.config.oled.height);
  oledSsd1306.setPageInterval(_model.config.oled.pageInterval);
  oledSsd1306.setStartupDuration(_model.config.oled.startupMs);

  Device::OledDevice* detectedOled = nullptr;

#if defined(ESPFC_I2C_0)
  if (_model.config.pin[PIN_I2C_0_SDA] != -1 && _model.config.pin[PIN_I2C_0_SCL] != -1)
  {
    if (!detectedOled && (_model.config.oled.dev == Device::OLED_DEFAULT || _model.config.oled.dev == Device::OLED_SSD1306))
    {
      if (detectDevice(oledSsd1306, i2cBus)) detectedOled = &oledSsd1306;
    }
  }
#endif

  _model.state.oled.dev = detectedOled;
  _model.state.oled.present = (bool)detectedOled;
}

void Hardware::updateOled()
{
  if(!_model.oledActive()) return;
  if(_model.state.oled.dev == &oledSsd1306)
  {
    oledSsd1306.update(_model);
  }
}

void Hardware::restart(const Model& model)
{
  if (model.state.mixer.escMotor) model.state.mixer.escMotor->end();
  if (model.state.mixer.escServo) model.state.mixer.escServo->end();
#ifdef ESPFC_SERIAL_SOFT_0_WIFI
  WiFi.disconnect();
  WiFi.softAPdisconnect();
#endif
  delay(100);
  targetReset();
}

void Hardware::restartToBootloader(const Model& model, BootloaderRequestType requestType)
{
  (void)requestType;
  if (model.state.mixer.escMotor) model.state.mixer.escMotor->end();
  if (model.state.mixer.escServo) model.state.mixer.escServo->end();
#ifdef ESPFC_SERIAL_SOFT_0_WIFI
  WiFi.disconnect();
  WiFi.softAPdisconnect();
#endif
  delay(100);

#if defined(ARCH_RP2040)
  if(requestType == BOOTLOADER_REQUEST_ROM)
  {
#ifdef ESPFC_SERIAL_USB_DEV
    ESPFC_SERIAL_USB_DEV.flush();
    ESPFC_SERIAL_USB_DEV.end();
    delay(80);
#endif
    reset_usb_boot(0, 0);
    while(1) {}
  }
  targetReset();
#elif defined(ESP32S3) || defined(ESP32S2) || defined(ESP32C3)
  // For bootloader requests, set the ROM download latch and then do a normal restart.
  // Using ESP.restart() here is more stable for USB re-enumeration than noos reset.
  if(requestType == BOOTLOADER_REQUEST_ROM || requestType == BOOTLOADER_REQUEST_FLASH)
  {
    REG_SET_BIT(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
  }
  targetReset();
#else
  // Best-effort fallback on non-RP targets where explicit bootloader entry is board/ROM-specific.
  targetReset();
#endif
}

} // namespace Espfc
