#include "Device/Baro/BaroBMP085.hpp"
#include "Device/Baro/BaroBMP280.hpp"
#include "Device/Baro/BaroSPL06.hpp"
#include "Device/BaroDevice.hpp"
#include "Device/GyroBMI160.h"
#include "Device/GyroDevice.h"
#include "Device/GyroICM20602.h"
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
  initBus();
  detectGyro();
  detectMag();
  detectBaro();
  detectRangefinder();
  detectOpticalFlow();
  detectOled();
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
      .log(_model.config.pin[PIN_I2C_0_SDA])
      .log(_model.config.pin[PIN_I2C_0_SCL])
      .log(_model.config.i2cSpeed)
      .logln(i2cResult);
#endif
}

void Hardware::detectGyro()
{
  if (_model.config.gyro.dev == GYRO_NONE) return;

  Device::GyroDevice* detectedGyro = nullptr;
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
    if (!detectedGyro && detectDevice(mpu9250, i2cBus)) detectedGyro = &mpu9250;
    if (!detectedGyro && detectDevice(mpu6500, i2cBus)) detectedGyro = &mpu6500;
    if (!detectedGyro && detectDevice(icm20602, i2cBus)) detectedGyro = &icm20602;
    if (!detectedGyro && detectDevice(bmi160, i2cBus)) detectedGyro = &bmi160;
    if (!detectedGyro && detectDevice(mpu6050, i2cBus)) detectedGyro = &mpu6050;
    if (!detectedGyro && detectDevice(lsm6dso, i2cBus)) detectedGyro = &lsm6dso;
    if (detectedGyro) gyroSlaveBus.begin(&i2cBus, detectedGyro->getAddress());
  }
#endif
  if (!detectedGyro) return;

  detectedGyro->setDLPFMode(_model.config.gyro.dlpf);
  _model.state.gyro.dev = detectedGyro;
  _model.state.gyro.present = (bool)detectedGyro;
  _model.state.accel.present = _model.state.gyro.present && _model.config.accel.dev != GYRO_NONE;
  _model.state.gyro.clock = detectedGyro->getRate();
}

void Hardware::detectMag()
{
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
  if (_model.config.baro.dev == BARO_NONE) return;

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

  _model.state.rangefinder[RANGEFINDER_BOTTOM].dev = bottom;
  _model.state.rangefinder[RANGEFINDER_BOTTOM].present = (bool)bottom;
  _model.state.rangefinder[RANGEFINDER_BOTTOM].rate = bottom ? bottom->getRate() : 0;

  _model.state.rangefinder[RANGEFINDER_FRONT].dev = front;
  _model.state.rangefinder[RANGEFINDER_FRONT].present = (bool)front;
  _model.state.rangefinder[RANGEFINDER_FRONT].rate = front ? front->getRate() : 0;
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

} // namespace Espfc
