#ifndef _ESPFC_SERIAL_DEVICE_ADAPTER_H_
#define _ESPFC_SERIAL_DEVICE_ADAPTER_H_

#include "Device/SerialDevice.h"
#if defined(ESP32C3) || defined(ESP32S3)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "hal/usb_serial_jtag_ll.h"
#pragma GCC diagnostic pop
#endif
#ifdef ESPFC_SERIAL_SOFT_0_WIFI
#include <WiFiClient.h>
#endif

namespace Espfc {

namespace Device {

template<typename T>
class SerialDeviceAdapter: public SerialDevice
{
  public:
    SerialDeviceAdapter(T& dev): _dev(dev) {}
    void begin(const SerialDeviceConfig& conf) override { targetSerialInit(_dev, conf); }
      void updateBaudRate(int baud) override {
  #if defined(STM32F7xx) || defined(STM32F7) || defined(STM32H7xx) || defined(STM32H7)
    _dev.begin(baud);
  #else
    _dev.updateBaudRate(baud);
  #endif
      };
    int available() override { return _dev.available(); }
    int read() override { return _dev.read(); }
    size_t readMany(uint8_t * c, size_t l) override {
#if defined(ARCH_RP2040) || defined(STM32F7xx) || defined(STM32F7) || defined(STM32H7xx) || defined(STM32H7)
      size_t count = std::min(l, (size_t)available());
      for(size_t i = 0; i < count; i++)
      {
        c[i] = read();
      }
      return count;
#else
      return _dev.read(c, l);
#endif
    }
    int peek() override { return _dev.peek(); }
    void flush() override { _dev.flush(); }
    size_t write(uint8_t c) override { return _dev.write(c); }
    size_t write(const uint8_t * c, size_t l) override { return _dev.write(c, l); }
    bool isSoft() const override { return false; };
    int availableForWrite() override { return _dev.availableForWrite(); }
    bool isTxFifoEmpty() override { return _dev.availableForWrite() >= SERIAL_TX_FIFO_SIZE; }
    operator bool() const override { return (bool)_dev; }
  private:
    T& _dev;
};

// WiFiClient specializations
#ifdef ESPFC_SERIAL_SOFT_0_WIFI
template<>
inline void SerialDeviceAdapter<WiFiClient>::begin(const SerialDeviceConfig& conf)
{
}

template<>
inline int SerialDeviceAdapter<WiFiClient>::availableForWrite()
{
  return SERIAL_TX_FIFO_SIZE;
}

template<>
inline bool SerialDeviceAdapter<WiFiClient>::isTxFifoEmpty()
{
  return true;
}

template<>
inline void SerialDeviceAdapter<WiFiClient>::updateBaudRate(int baud) {}

#endif

#if defined(ESP32C3) || defined(ESP32S3)
template<>
inline void SerialDeviceAdapter<HWCDC>::begin(const SerialDeviceConfig& conf)
{
  targetSerialInit(_dev, conf);
}

template<>
inline int SerialDeviceAdapter<HWCDC>::available()
{
  return _dev.available();
}

template<>
inline int SerialDeviceAdapter<HWCDC>::read()
{
  return _dev.read();
}

template<>
inline size_t SerialDeviceAdapter<HWCDC>::readMany(uint8_t * c, size_t l)
{
  return _dev.read(c, l);
}

template<>
inline int SerialDeviceAdapter<HWCDC>::peek()
{
  return _dev.peek();
}

template<>
inline void SerialDeviceAdapter<HWCDC>::flush()
{
  _dev.flush();
}

template<>
inline size_t SerialDeviceAdapter<HWCDC>::write(uint8_t c)
{
  return _dev.write(c);
}

template<>
inline size_t SerialDeviceAdapter<HWCDC>::write(const uint8_t * c, size_t l)
{
  return _dev.write(c, l);
}

template<>
inline int SerialDeviceAdapter<HWCDC>::availableForWrite()
{
  return _dev.availableForWrite();
}

template<>
inline bool SerialDeviceAdapter<HWCDC>::isTxFifoEmpty()
{
  return _dev.availableForWrite() >= SERIAL_TX_FIFO_SIZE;
}

template<>
inline void SerialDeviceAdapter<HWCDC>::updateBaudRate(int baud) {}

template<>
inline SerialDeviceAdapter<HWCDC>::operator bool() const
{
  return true;
}
#endif

#if defined(ESP32S2) || defined(ESP32S3)
template<>
inline void SerialDeviceAdapter<USBCDC>::updateBaudRate(int baud) {}
#endif

#if defined(ARCH_RP2040)
template<>
inline void SerialDeviceAdapter<SerialUART>::updateBaudRate(int baud)
{
  _dev.begin(baud);
}

template<>
inline void SerialDeviceAdapter<ESPFC_SERIAL_USB_DEV_T>::updateBaudRate(int baud) {}
#endif

}

}

#endif