#include "Wireless.h"
#ifdef ESPFC_SERIAL_SOFT_0_WIFI
#include <ArduinoOTA.h>
#if defined(ESPFC_WIFI_ALT) || defined(ARCH_RP2040)
#include <Updater.h>
#else
#include <Update.h>
#endif
#endif
#include <algorithm>
#include <cstdio>

#ifdef ESPFC_SERIAL_SOFT_0_WIFI

namespace Espfc {

Wireless::Wireless(Model& model): _model(model), _status(STOPPED), _server(1111), _adapter(_client), _otaStarted(false)
#ifdef ESPFC_BT_OTA_SUPPORTED
  , _btOtaStarted(false), _btUpdateActive(false), _btExpected(0), _btReceived(0), _btLine{0}, _btLinePos(0)
#endif
{}

int Wireless::begin()
{
  WiFi.persistent(false);
#ifdef ESPFC_ESPNOW
  if(_model.isFeatureActive(FEATURE_RX_SPI))
  {
    startAp();
  }
#endif
  return 1;
}

void Wireless::startAp()
{
  bool status = WiFi.softAP("ESP-FC");
  _model.logger.info().log(F("WIFI AP START")).logln(status);
}

int Wireless::connect()
{
#ifdef ESPFC_WIFI_ALT
  // https://github.com/esp8266/Arduino/issues/2545#issuecomment-249222211
  _events[0] = WiFi.onStationModeConnected([this](const WiFiEventStationModeConnected& ev) { this->wifiEventConnected(ev.ssid, ev.channel); });
  _events[1] = WiFi.onStationModeGotIP([this](const WiFiEventStationModeGotIP& ev) { this->wifiEventGotIp(ev.ip); });
  _events[2] = WiFi.onStationModeDisconnected([this](const WiFiEventStationModeDisconnected& ev) { this->wifiEventDisconnected(); });
  _events[3] = WiFi.onSoftAPModeStationConnected([this](const WiFiEventSoftAPModeStationConnected& ev) { this->wifiEventApConnected(ev.mac); });
#elif defined(ESPFC_WIFI)
  WiFi.onEvent([this](WiFiEvent_t ev, WiFiEventInfo_t info) { 
    this->wifiEventConnected(String(info.wifi_sta_connected.ssid, info.wifi_sta_connected.ssid_len), info.wifi_sta_connected.channel);
  }, ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.onEvent([this](WiFiEvent_t ev, WiFiEventInfo_t info) { this->wifiEventGotIp(IPAddress(info.got_ip.ip_info.ip.addr)); }, ARDUINO_EVENT_WIFI_STA_GOT_IP);
  WiFi.onEvent([this](WiFiEvent_t ev, WiFiEventInfo_t info) { this->wifiEventDisconnected(); }, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
#endif
  if(_model.config.wireless.ssid[0] != 0)
  {
    WiFi.begin(_model.config.wireless.ssid, _model.config.wireless.pass);
    _model.logger.info().log(F("WIFI STA")).log(_model.config.wireless.ssid).log(_model.config.wireless.pass).log(WiFi.getMode()).logln(WiFi.status());
  }
  if(!(WiFi.getMode() & WIFI_AP))
  {
    startAp();
  }
  _server.begin(_model.config.wireless.port);
  _server.setNoDelay(true);
  _model.state.serial[SERIAL_SOFT_0].stream = &_adapter;
  _model.logger.info().log(F("WIFI SERVER PORT")).log(WiFi.status()).logln(_model.config.wireless.port);
  beginOta();
#ifdef ESPFC_BT_OTA_SUPPORTED
  beginBtOta();
#endif
  return 1;
}

void Wireless::wifiEventConnected(const String& ssid, int channel)
{
  _model.logger.info().log(F("WIFI STA CONN")).log(ssid).logln(channel);
}

void Wireless::wifiEventApConnected(const uint8_t* mac)
{
  char buf[20];
  snprintf(buf, sizeof(buf), "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  _model.logger.info().log(F("WIFI AP CONNECT")).logln(buf);
}

void Wireless::wifiEventGotIp(const IPAddress& ip)
{
  _model.logger.info().log(F("WIFI STA IP")).logln(ip.toString());
}

void Wireless::wifiEventDisconnected()
{
  _model.logger.info().logln(F("WIFI STA DISCONNECT"));
}

int Wireless::update()
{
  Utils::Stats::Measure measure(_model.state.stats, COUNTER_WIFI);

  switch(_status)
  {
    case STOPPED:
      if((_model.state.mode.rescueConfigMode == RESCUE_CONFIG_ACTIVE && _model.isFeatureActive(FEATURE_SOFTSERIAL))
         || _model.config.wireless.otaEnabled
#ifdef ESPFC_BT_OTA_SUPPORTED
         || _model.config.wireless.btOtaEnabled
#endif
      )
      {
        connect();
        _status = STARTED;
        return 1;
      }
      break;
    case STARTED:
      if(_server.hasClient())
      {
        _client = _server.accept();
      }
      updateOta();
#ifdef ESPFC_BT_OTA_SUPPORTED
      updateBtOta();
#endif
      break;
  }

  return 1;
}

void Wireless::beginOta()
{
  if(_otaStarted || !_model.config.wireless.otaEnabled) return;

  const char* modelName = _model.config.modelName[0] != 0 ? _model.config.modelName : "ESP-FC";
  ArduinoOTA.setHostname(modelName);
  ArduinoOTA.setPort((uint16_t)_model.config.wireless.otaPort);
  if(_model.config.wireless.otaPass[0] != 0)
  {
    ArduinoOTA.setPassword(_model.config.wireless.otaPass);
  }

  ArduinoOTA.onStart([this]() {
    _model.logger.info().logln(F("OTA START"));
  });
  ArduinoOTA.onProgress([this](unsigned int progress, unsigned int total) {
    const uint8_t pct = total ? (uint8_t)((progress * 100u) / total) : 0u;
    if((pct % 20u) == 0u)
    {
      _model.logger.info().log(F("OTA PROGRESS")).logln((int)pct);
    }
  });
  ArduinoOTA.onEnd([this]() {
    _model.logger.info().logln(F("OTA END"));
  });
  ArduinoOTA.onError([this](ota_error_t error) {
    _model.logger.info().log(F("OTA ERROR")).logln((int)error);
  });

  ArduinoOTA.begin();
  _otaStarted = true;
  _model.logger.info().log(F("OTA WIFI READY")).log(modelName).log(':').logln(_model.config.wireless.otaPort);
}

void Wireless::updateOta()
{
  if(!_otaStarted) return;
  ArduinoOTA.handle();
}

#ifdef ESPFC_BT_OTA_SUPPORTED
void Wireless::beginBtOta()
{
  if(_btOtaStarted || !_model.config.wireless.btOtaEnabled) return;

  const char* modelName = _model.config.modelName[0] != 0 ? _model.config.modelName : "ESP-FC";
  char btName[40] = {0};
  snprintf(btName, sizeof(btName), "%s-OTA", modelName);
  const bool ok = _btSerial.begin(btName);
  _btOtaStarted = ok;
  _model.logger.info().log(F("BT OTA")).log(btName).logln(ok ? F(" READY") : F(" FAIL"));
}

void Wireless::updateBtOta()
{
  if(!_btOtaStarted) return;

  if(!_btSerial.hasClient())
  {
    if(_btUpdateActive)
    {
      Update.abort();
      _btUpdateActive = false;
      _btExpected = 0;
      _btReceived = 0;
      _btLinePos = 0;
      _model.logger.info().logln(F("BT OTA ABORT"));
    }
    return;
  }

  while(_btSerial.available())
  {
    if(!_btUpdateActive)
    {
      const int c = _btSerial.read();
      if(c < 0) break;
      if(c == '\n' || c == '\r')
      {
        _btLine[_btLinePos] = 0;
        unsigned long size = 0;
        if(sscanf(_btLine, "OTA %lu", &size) == 1 && size > 0)
        {
          if(Update.begin((size_t)size))
          {
            _btExpected = (size_t)size;
            _btReceived = 0;
            _btUpdateActive = true;
            _btSerial.println("OK");
            _model.logger.info().log(F("BT OTA START")).logln((int)_btExpected);
          }
          else
          {
            _btSerial.println("ERR begin");
            _model.logger.info().logln(F("BT OTA BEGIN FAIL"));
          }
        }
        _btLinePos = 0;
      }
      else if(_btLinePos < (sizeof(_btLine) - 1))
      {
        _btLine[_btLinePos++] = (char)c;
      }
      continue;
    }

    uint8_t buff[256] = {0};
    const size_t available = (size_t)_btSerial.available();
    if(!available) break;
    const size_t toRead = std::min(sizeof(buff), available);
    const size_t read = _btSerial.readBytes(buff, toRead);
    if(!read) break;

    const size_t written = Update.write(buff, read);
    _btReceived += written;
    if(written != read)
    {
      Update.abort();
      _btSerial.println("ERR write");
      _model.logger.info().logln(F("BT OTA WRITE FAIL"));
      _btUpdateActive = false;
      _btExpected = 0;
      _btReceived = 0;
      _btLinePos = 0;
      continue;
    }

    if(_btReceived >= _btExpected)
    {
      if(Update.end(true))
      {
        _btSerial.println("OK reboot");
        _model.logger.info().logln(F("BT OTA END"));
        delay(50);
        ESP.restart();
      }
      else
      {
        _btSerial.println("ERR end");
        _model.logger.info().logln(F("BT OTA END FAIL"));
      }
      _btUpdateActive = false;
      _btExpected = 0;
      _btReceived = 0;
      _btLinePos = 0;
    }
  }
}
#endif

}

#endif
