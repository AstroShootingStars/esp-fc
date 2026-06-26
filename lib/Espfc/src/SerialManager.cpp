#include "SerialManager.h"
#include "Device/SerialDeviceAdapter.h"
#include "Debug_Espfc.h"
#include <algorithm>

extern "C" {
  #include "msp/msp_protocol.h"
}

// TODO: move to target
#ifdef ESPFC_SERIAL_0
  static Espfc::Device::SerialDeviceAdapter<ESPFC_SERIAL_0_DEV_T> _uart0(ESPFC_SERIAL_0_DEV);
#endif

#ifdef ESPFC_SERIAL_1
  static Espfc::Device::SerialDeviceAdapter<ESPFC_SERIAL_1_DEV_T> _uart1(ESPFC_SERIAL_1_DEV);
#endif

#ifdef ESPFC_SERIAL_2
  static Espfc::Device::SerialDeviceAdapter<ESPFC_SERIAL_2_DEV_T> _uart2(ESPFC_SERIAL_2_DEV);
#endif

#ifdef ESPFC_SERIAL_USB
  static Espfc::Device::SerialDeviceAdapter<ESPFC_SERIAL_USB_DEV_T> _usb(ESPFC_SERIAL_USB_DEV);
#endif

namespace Espfc {

namespace {
uint32_t g_mspInitMs = 0;
uint32_t g_mspNextReinitMs = 0;
bool g_mspSeen = false;
int8_t g_startupMspPort = -1;
constexpr uint32_t USB_REBIND_INITIAL_DELAY_MS = 300;
constexpr uint32_t USB_REBIND_INTERVAL_MS = 750;
constexpr uint32_t USB_STARTUP_PRIORITY_MS = 20000;
}

SerialManager::SerialManager(Model& model, TelemetryManager& telemetry): _model(model), _current(0), _msp(model), _osd(model), _cli(model), _vtx(model),
  _telemetry(telemetry), _gps(model)
#ifdef ESPFC_SERIAL_SOFT_0_WIFI
  , _wireless(model)
#endif
{
}

int SerialManager::begin()
{
  g_mspInitMs = millis();
  g_mspNextReinitMs = g_mspInitMs + USB_REBIND_INITIAL_DELAY_MS;
  g_mspSeen = false;
  g_startupMspPort = -1;

  _osd.begin();
#ifdef ESPFC_SERIAL_USB
  bool hasActiveMspStream = false;
#endif

  for(int i = 0; i < SERIAL_UART_COUNT; i++)
  {
    Device::SerialDevice * port = getSerialPortById((SerialPort)i);
    const SerialPortConfig& spc = _model.config.serial[i];

  #ifdef ESPFC_SERIAL_USB
    const bool hasUsbPort = true;
    const bool isUsbPort = i == SERIAL_USB;
  #else
    const bool hasUsbPort = false;
    const bool isUsbPort = false;
  #endif

    uint16_t functionMask = spc.functionMask;
    int32_t baud = spc.baud;

  #ifdef ESPFC_SERIAL_USB
    // Keep USB MSP always available even with stale persisted serial config.
    if(isUsbPort)
    {
      functionMask |= SERIAL_FUNCTION_MSP;
      if(baud == SERIAL_SPEED_NONE) baud = SERIAL_SPEED_115200;
    }
  #endif

    if(port == nullptr || !functionMask)
    {
      continue;
    }

    SerialDeviceConfig sdc;
    sdc.baud = baud;

#ifdef ESPFC_SERIAL_REMAP_PINS
    if(!isUsbPort)
    {
      const size_t pin_idx = 2 * (hasUsbPort ? i - 1 : i);
      sdc.tx_pin = _model.config.pin[pin_idx + PIN_SERIAL_0_TX];
      sdc.rx_pin = _model.config.pin[pin_idx + PIN_SERIAL_0_RX];
      if(sdc.tx_pin == -1 && sdc.rx_pin == -1)
      {
        continue;
      }
    }
#else
  (void)(isUsbPort && hasUsbPort);
#endif

    if(functionMask & SERIAL_FUNCTION_RX_SERIAL)
    {
      switch(_model.config.input.serialRxProvider)
      {
        case SERIALRX_SBUS:
          sdc.baud = 100000ul;
          sdc.parity = SDC_SERIAL_PARITY_EVEN;
          sdc.stop_bits = SDC_SERIAL_STOP_BITS_2;
          sdc.inverted = true;
          break;
        case SERIALRX_IBUS:
          sdc.baud = 115200ul;
          break;
        case SERIALRX_CRSF:
          sdc.baud = 420000ul;
          break;
        case SERIALRX_SPEKTRUM1024:
        case SERIALRX_SPEKTRUM2048:
          sdc.baud = 115200ul;
          break;
        case SERIALRX_SUMD:
        case SERIALRX_SUMH:
          sdc.baud = 115200ul;
          break;
        case SERIALRX_FPORT:
          sdc.baud = 115200ul;
          break;
        default:
          break;
      }
    }
    else if(functionMask & SERIAL_FUNCTION_BLACKBOX)
    {
      //sdc.baud = spc.blackboxBaud;
      if(sdc.baud == 230400 || sdc.baud == 460800)
      {
        sdc.stop_bits = SDC_SERIAL_STOP_BITS_2;
      }
    }
    else if(functionMask & SERIAL_FUNCTION_TELEMETRY_IBUS)
    {
      sdc.baud = 115200;
    }
    else if(functionMask & SERIAL_FUNCTION_VTX_SMARTAUDIO)
    {
      sdc.baud = 4800;
      sdc.parity = SDC_SERIAL_PARITY_NONE;
      sdc.stop_bits = SDC_SERIAL_STOP_BITS_2;
      sdc.data_bits = 8;
    }

    if(!sdc.baud)
    {
      continue;
    }

    port->begin(sdc);   
    _model.state.serial[i].stream = port;
    if(functionMask & SERIAL_FUNCTION_MSP)
    {
      if(g_startupMspPort < 0)
      {
        g_startupMspPort = i;
      }
#ifdef ESPFC_SERIAL_USB
      hasActiveMspStream = true;
#endif
    }

    if(i == ESPFC_SERIAL_DEBUG_PORT)
    {
#if !defined(ESPFC_SERIAL_USB) || (ESPFC_SERIAL_DEBUG_PORT != SERIAL_USB)
      initDebugStream(port);
#endif
    }
    if(functionMask & SERIAL_FUNCTION_TELEMETRY_IBUS)
    {
      _ibus.begin(port);
    }
    if(functionMask & SERIAL_FUNCTION_VTX_SMARTAUDIO)
    {
      _vtx.begin(port);
    }
    if(functionMask & SERIAL_FUNCTION_GPS)
    {
      _gps.begin(port, sdc.baud);
    }

    _model.logger.info().log(F("UART")).log(i).log(spc.id).log(functionMask).log(sdc.baud).log(i == ESPFC_SERIAL_DEBUG_PORT).log(sdc.tx_pin).logln(sdc.rx_pin);
  }

#ifdef ESPFC_SERIAL_USB
  // Recovery path: keep at least one MSP endpoint alive even if persisted serial config is invalid.
  if(!hasActiveMspStream)
  {
    Device::SerialDevice * port = getSerialPortById(SERIAL_USB);
    if(port != nullptr)
    {
      SerialDeviceConfig sdc;
      sdc.baud = SERIAL_SPEED_115200;
      port->begin(sdc);
      _model.state.serial[SERIAL_USB].stream = port;

      SerialPortConfig& usbCfg = _model.config.serial[SERIAL_USB];
      usbCfg.functionMask |= SERIAL_FUNCTION_MSP;
      if(usbCfg.baud == SERIAL_SPEED_NONE) usbCfg.baud = SERIAL_SPEED_115200;
      g_startupMspPort = SERIAL_USB;

      _model.logger.info().logln(F("UART USB fallback MSP"));
    }
  }
#endif

#ifdef ESPFC_SERIAL_SOFT_0_WIFI
  _wireless.begin();
#endif

  return 1;
}

int FAST_CODE_ATTR SerialManager::update()
{
  const uint32_t nowMs = millis();
  const bool startupMspPriority = !g_mspSeen && g_startupMspPort >= 0 && g_mspInitMs && (uint32_t)(nowMs - g_mspInitMs) <= USB_STARTUP_PRIORITY_MS;

  // During startup, process MSP port first to reduce first-connect races.
  if(startupMspPriority)
  {
    _current = static_cast<size_t>(g_startupMspPort);
  }

#ifdef ESPFC_SERIAL_USB
  // Cold power-up can leave USB endpoints racey on some hosts. Rebind USB MSP
  // periodically during startup, then stop once MSP traffic is seen.
  if(startupMspPriority && g_startupMspPort == SERIAL_USB && (int32_t)(nowMs - g_mspNextReinitMs) >= 0)
  {
    Device::SerialDevice * usbPort = getSerialPortById(SERIAL_USB);
    if(usbPort)
    {
      SerialDeviceConfig usbCfg;
      usbCfg.baud = _model.config.serial[SERIAL_USB].baud ? _model.config.serial[SERIAL_USB].baud : SERIAL_SPEED_115200;
      usbPort->begin(usbCfg);
      _model.state.serial[SERIAL_USB].stream = usbPort;
      _model.logger.info().logln(F("UART USB rebind"));
    }
    g_mspNextReinitMs = nowMs + USB_REBIND_INTERVAL_MS;
  }
#endif

  const SerialPortConfig& sc = _model.config.serial[_current];
  SerialPortState& ss = _model.state.serial[_current];

  if(ss.stream && !(sc.functionMask & SERIAL_FUNCTION_RX_SERIAL))
  {
    Utils::Stats::Measure measure(_model.state.stats, COUNTER_SERIAL);
    if(startupMspPriority)
    {
      if(_current == static_cast<size_t>(g_startupMspPort) && (sc.functionMask & SERIAL_FUNCTION_MSP))
      {
        processMsp(ss);
      }
      next();
      return 1;
    }
    if (sc.functionMask & SERIAL_FUNCTION_MSP)
    {
      processMsp(ss);
    }
    if(sc.functionMask & SERIAL_FUNCTION_TELEMETRY_FRSKY && _model.state.telemetryTimer.check())
    {
      _telemetry.process(*ss.stream, TELEMETRY_PROTOCOL_TEXT);
    }
    if(sc.functionMask & SERIAL_FUNCTION_TELEMETRY_IBUS)
    {
      _ibus.update();
    }
    if(sc.functionMask & SERIAL_FUNCTION_VTX_SMARTAUDIO)
    {
      _vtx.update();
    }
    if(sc.functionMask & SERIAL_FUNCTION_GPS)
    {
      _gps.update();
    }
    if(sc.functionMask & SERIAL_FUNCTION_FRSKY_OSD)
    {
      _osd.update(*ss.stream);
    }
  }

#ifdef ESPFC_SERIAL_SOFT_0_WIFI
  if(_current == SERIAL_SOFT_0)
  {
    _wireless.update();
  }
#endif

  next();

  return 1;
}

void SerialManager::processMsp(SerialPortState& ss)
{
  size_t len = ss.stream->available();
  if(!len) return;

  size_t consumedBytes = 0;
  size_t droppedBytes = 0;
  size_t handledCommands = 0;
  uint8_t droppedSample[8] = {0};
  size_t droppedSampleLen = 0;

  uint8_t buff[64] = {0};
  len = std::min(len, (size_t)sizeof(buff));
  const size_t readLen = ss.stream->readMany(buff, len);
  if(!readLen) return;
  char * c = (char*)&buff[0];
  len = readLen;
  while(len--)
  {
    const uint8_t byte = static_cast<uint8_t>(*c);

    bool consumed = _msp.parse(*c, ss.mspRequest);

    if(consumed)
    {
      consumedBytes++;
      if(ss.mspRequest.isReady() && ss.mspRequest.isCmd())
      {
        handledCommands++;
        if(_current == static_cast<size_t>(g_startupMspPort))
        {
          g_mspSeen = true;
        }
        _msp.processCommand(ss.mspRequest, ss.mspResponse, *ss.stream);
        _msp.sendResponse(ss.mspResponse, *ss.stream);
        _msp.postCommand();
        ss.mspRequest = Connect::MspMessage();
        ss.mspResponse = Connect::MspResponse();
      }
    }
    else
    {
      droppedBytes++;
      if(droppedSampleLen < sizeof(droppedSample))
      {
        droppedSample[droppedSampleLen++] = byte;
      }

      (void)byte;
#ifdef ESPFC_SERIAL_USB
      if(_current != SERIAL_USB)
      {
        _cli.process(*c, ss.cliCmd, *ss.stream);
      }
#else
      _cli.process(*c, ss.cliCmd, *ss.stream);
#endif
    }
    c++;
  }

#ifdef ESPFC_SERIAL_USB
  (void)consumedBytes;
  (void)droppedBytes;
  (void)droppedSample;
  (void)droppedSampleLen;
#endif
}

Device::SerialDevice * SerialManager::getSerialPortById(SerialPort portId)
{
  switch(portId)
  {
#ifdef ESPFC_SERIAL_0
    case SERIAL_UART_0: return &_uart0;
#endif
#ifdef ESPFC_SERIAL_1
    case SERIAL_UART_1: return &_uart1;
#endif
#ifdef ESPFC_SERIAL_2
    case SERIAL_UART_2: return &_uart2;
#endif
#ifdef ESPFC_SERIAL_USB
    case SERIAL_USB: return &_usb;
#endif
    default: return nullptr;
  }
}

}
