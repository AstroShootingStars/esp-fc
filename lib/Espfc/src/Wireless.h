#pragma once

#include "Model.h"
#include "Device/SerialDeviceAdapter.h"

#ifdef ESPFC_SERIAL_SOFT_0_WIFI
#if defined(ESPFC_WIFI_ALT)
#include <ESP8266WiFi.h>
#elif defined(ESPFC_WIFI)
#include <WiFi.h>
#endif

#if defined(ESPFC_BT_OTA) && defined(ESP32) && !defined(ESP32S2) && !defined(ESP32S3) && !defined(ESP32C3) && defined(CONFIG_BT_ENABLED) && defined(CONFIG_BLUEDROID_ENABLED)
#define ESPFC_BT_OTA_SUPPORTED
#include <BluetoothSerial.h>
#endif

namespace Espfc {

class Wireless
{
  enum Status {
    STOPPED,
    STARTED,
  };
  public:
    Wireless(Model& model);

    int begin();
    int update();

    void startAp();
    int connect();
    void wifiEventConnected(const String& ssid, int channel);
    void wifiEventApConnected(const uint8_t* mac);
    void wifiEventGotIp(const IPAddress& ip);
    void wifiEventDisconnected();

  private:
    void beginOta();
    void updateOta();
  #ifdef ESPFC_BT_OTA_SUPPORTED
    void beginBtOta();
    void updateBtOta();
  #endif

    Model& _model;
    Status _status;
    WiFiServer _server;
    WiFiClient _client;
    Device::SerialDeviceAdapter<WiFiClient> _adapter;
    bool _otaStarted;
#ifdef ESPFC_WIFI_ALT
    WiFiEventHandler _events[4];
#endif
  #ifdef ESPFC_BT_OTA_SUPPORTED
    BluetoothSerial _btSerial;
    bool _btOtaStarted;
    bool _btUpdateActive;
    size_t _btExpected;
    size_t _btReceived;
    char _btLine[24];
    uint8_t _btLinePos;
  #endif
};

}

#endif
