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
    void updateBaudRate(int baud) override { _dev.updateBaudRate(baud); };
    int available() override { return _dev.available(); }
    int read() override { return _dev.read(); }
    size_t readMany(uint8_t * c, size_t l) override {
#if defined(ARCH_RP2040)
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
  (void)conf;
}

template<>
inline int SerialDeviceAdapter<HWCDC>::available()
{
  return usb_serial_jtag_ll_rxfifo_data_available() ? 1 : 0;
}

template<>
inline int SerialDeviceAdapter<HWCDC>::read()
{
  uint8_t ch = 0;
  usb_serial_jtag_ll_read_rxfifo(&ch, 1);
  return ch;
}

template<>
inline size_t SerialDeviceAdapter<HWCDC>::readMany(uint8_t * c, size_t l)
{
  size_t i = 0;
  while(i < l && usb_serial_jtag_ll_rxfifo_data_available())
  {
    usb_serial_jtag_ll_read_rxfifo(c + i, 1);
    i++;
  }
  return i;
}

template<>
inline int SerialDeviceAdapter<HWCDC>::peek()
{
  return -1;
}

template<>
inline void SerialDeviceAdapter<HWCDC>::flush()
{
  usb_serial_jtag_ll_txfifo_flush();
}

template<>
inline size_t SerialDeviceAdapter<HWCDC>::write(uint8_t c)
{
  const uint32_t written = usb_serial_jtag_ll_write_txfifo(&c, 1);
  usb_serial_jtag_ll_txfifo_flush();
  return (size_t)written;
}

template<>
inline size_t SerialDeviceAdapter<HWCDC>::write(const uint8_t * c, size_t l)
{
  static constexpr uint32_t USB_VCP_TX_TIMEOUT_MS = 20;
  const uint8_t * p = c;
  int remaining = (int)l;
  uint32_t lastProgressMs = millis();
  while(remaining > 0)
  {
    const uint32_t written = usb_serial_jtag_ll_write_txfifo(p, remaining);
    usb_serial_jtag_ll_txfifo_flush();
    if(written > 0)
    {
      p += written;
      remaining -= (int)written;
      lastProgressMs = millis();
    }
    else if(millis() - lastProgressMs >= USB_VCP_TX_TIMEOUT_MS)
    {
      break;
    }
  }
  return (size_t)(l - (size_t)remaining);
}

template<>
inline int SerialDeviceAdapter<HWCDC>::availableForWrite()
{
  return usb_serial_jtag_ll_txfifo_writable() ? SERIAL_TX_FIFO_SIZE : 0;
}

template<>
inline bool SerialDeviceAdapter<HWCDC>::isTxFifoEmpty()
{
  return usb_serial_jtag_ll_txfifo_writable();
}

template<>
inline void SerialDeviceAdapter<HWCDC>::updateBaudRate(int baud) {}

template<>
inline SerialDeviceAdapter<HWCDC>::operator bool() const
{
  return true;
}
#endif

#if defined(ESP32S2) || defined(ESP32S3) || defined(ESP32C3)
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
inline void SerialDeviceAdapter<SerialUSB>::updateBaudRate(int baud) {}
#endif

}

}

#endif