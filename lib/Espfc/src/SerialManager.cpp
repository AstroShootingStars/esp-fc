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

SerialManager::SerialManager(Model& model, TelemetryManager& telemetry): _model(model), _current(0), _msp(model), _osd(model), _cli(model), _vtx(model),
  _telemetry(telemetry), _gps(model)
#ifdef ESPFC_SERIAL_SOFT_0_WIFI
  , _wireless(model)
#endif
{
  std::fill_n(_bfApiHandshakeState, SERIAL_UART_COUNT, 0);
}

int SerialManager::begin()
{
  _osd.begin();

  for(int i = 0; i < SERIAL_UART_COUNT; i++)
  {
    Device::SerialDevice * port = getSerialPortById((SerialPort)i);
    const SerialPortConfig& spc = _model.config.serial[i];

    if(!port || !spc.functionMask)
    {
      continue;
    }

    SerialDeviceConfig sdc;
    sdc.baud = spc.baud;

#ifdef ESPFC_SERIAL_USB
    const bool hasUsbPort = true;
    const bool isUsbPort = i == SERIAL_USB;
#else
    const bool hasUsbPort = false;
    const bool isUsbPort = false;
#endif

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

    if(spc.functionMask & SERIAL_FUNCTION_RX_SERIAL)
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
    else if(spc.functionMask & SERIAL_FUNCTION_BLACKBOX)
    {
      //sdc.baud = spc.blackboxBaud;
      if(sdc.baud == 230400 || sdc.baud == 460800)
      {
        sdc.stop_bits = SDC_SERIAL_STOP_BITS_2;
      }
    }
    else if(spc.functionMask & SERIAL_FUNCTION_TELEMETRY_IBUS)
    {
      sdc.baud = 115200;
    }
    else if(spc.functionMask & SERIAL_FUNCTION_VTX_SMARTAUDIO)
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

    if(i == ESPFC_SERIAL_DEBUG_PORT)
    {
      initDebugStream(port);
    }
    if(spc.functionMask & SERIAL_FUNCTION_TELEMETRY_IBUS)
    {
      _ibus.begin(port);
    }
    if(spc.functionMask & SERIAL_FUNCTION_VTX_SMARTAUDIO)
    {
      _vtx.begin(port);
    }
    if(spc.functionMask & SERIAL_FUNCTION_GPS)
    {
      _gps.begin(port, sdc.baud);
    }

    _model.logger.info().log(F("UART")).log(i).log(spc.id).log(spc.functionMask).log(sdc.baud).log(i == ESPFC_SERIAL_DEBUG_PORT).log(sdc.tx_pin).logln(sdc.rx_pin);
  }

#ifdef ESPFC_SERIAL_SOFT_0_WIFI
  _wireless.begin();
#endif

  return 1;
}

int FAST_CODE_ATTR SerialManager::update()
{
  const SerialPortConfig& sc = _model.config.serial[_current];
  SerialPortState& ss = _model.state.serial[_current];

  if(ss.stream && !(sc.functionMask & SERIAL_FUNCTION_RX_SERIAL))
  {
    Utils::Stats::Measure measure(_model.state.stats, COUNTER_SERIAL);
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

  uint8_t buff[64] = {0};
  len = std::min(len, (size_t)sizeof(buff));
  ss.stream->readMany(buff, len);
  char * c = (char*)&buff[0];
  while(len--)
  {
    const uint8_t byte = static_cast<uint8_t>(*c);
    bool consumed = _msp.parse(*c, ss.mspRequest);

    if(detectBetaflightApiRequest(byte, _current) && !consumed)
    {
      sendBetaflightApiVersion(*ss.stream);
      ss.mspRequest = Connect::MspMessage();
      ss.mspResponse = Connect::MspResponse();
      c++;
      continue;
    }

    if(consumed)
    {
      if(ss.mspRequest.isReady() && ss.mspRequest.isCmd())
      {
        _msp.processCommand(ss.mspRequest, ss.mspResponse, *ss.stream);
        _msp.sendResponse(ss.mspResponse, *ss.stream);
        _msp.postCommand();
        ss.mspRequest = Connect::MspMessage();
        ss.mspResponse = Connect::MspResponse();
      }
    }
    else
    {
      _cli.process(*c, ss.cliCmd, *ss.stream);
    }
    c++;
  }
}

bool SerialManager::detectBetaflightApiRequest(uint8_t byte, size_t portIndex)
{
  uint8_t& state = _bfApiHandshakeState[portIndex];

  switch(state)
  {
    case 0: // '$'
      state = byte == '$' ? 1 : 0;
      break;
    case 1: // 'M'
      state = byte == 'M' ? 2 : (byte == '$' ? 1 : 0);
      break;
    case 2: // '<'
      state = byte == '<' ? 3 : (byte == '$' ? 1 : 0);
      break;
    case 3: // payload size = 0
      state = byte == 0x00 ? 4 : (byte == '$' ? 1 : 0);
      break;
    case 4: // command = MSP_API_VERSION (1)
      state = byte == MSP_API_VERSION ? 5 : (byte == '$' ? 1 : 0);
      break;
    case 5: // checksum = 0x01
      state = 0;
      return byte == 0x01;
    default:
      state = 0;
      break;
  }

  return false;
}

void SerialManager::sendBetaflightApiVersion(Device::SerialDevice& stream) const
{
  const uint8_t payloadSize = 3;
  const uint8_t cmd = MSP_API_VERSION;
  const uint8_t protocolVersion = MSP_PROTOCOL_VERSION;
  const uint8_t apiMajor = API_VERSION_MAJOR;
  const uint8_t apiMinor = API_VERSION_MINOR;
  const uint8_t checksum = payloadSize ^ cmd ^ protocolVersion ^ apiMajor ^ apiMinor;

  const uint8_t response[9] = {
    '$', 'M', '>',
    payloadSize,
    cmd,
    protocolVersion,
    apiMajor,
    apiMinor,
    checksum
  };

  stream.write(response, sizeof(response));
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
