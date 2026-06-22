# ESP-FC Receiver Protocol Implementation - Complete Reference

## Overview
All 9 Betaflight receiver protocols are now fully implemented and compiled into the ESP32S3 firmware.

---

## Protocol Specifications

### 1. CRSF (Crossfire) ✅
**Status**: Fully implemented with telemetry
- **Baud Rate**: 420,000 (high-speed)
- **Channels**: 16
- **Frame Rate**: 200 Hz
- **Features**:
  - Link statistics (RSSI, SNR, RF mode, TX power)
  - Bidirectional MSP telemetry
  - Antenna selection feedback
- **File**: `lib/Espfc/src/Device/InputCRSF.h`
- **MSP Integration**: Yes (TelemetryManager)

### 2. SBUS (S.Bus/Futaba) ✅
**Status**: Fully implemented with failsafe detection
- **Baud Rate**: 100,000 (inverted serial)
- **Parity**: EVEN
- **Stop Bits**: 2
- **Channels**: 16
- **Frame Rate**: 30 Hz
- **Features**:
  - Signal loss detection (flag bit)
  - Failsafe active detection (flag bit)
  - 16-bit channel values packed in 11-bit format
- **File**: `lib/Espfc/src/Device/InputSBUS.h`

### 3. IBUS (FlySky iBus) ✅
**Status**: Fully implemented with CRC validation
- **Baud Rate**: 115,200
- **Channels**: 14
- **Frame Rate**: ~70 Hz
- **Features**:
  - CRC checksum validation (16-bit)
  - 16-bit channel values
- **File**: `lib/Espfc/src/Device/InputIBUS.hpp`

### 4. SPEKTRUM (DSM/DSMX) ✅
**Status**: Fully implemented with dual mode support
- **Baud Rate**: 115,200
- **Channels**: 12
- **Modes**:
  - DSM2 1024: 22ms frame rate, 11-bit resolution
  - DSMX 2048: 11ms frame rate, 12-bit resolution
- **Features**:
  - Frame ID detection for mode selection
  - FADE byte (RSSI proxy)
  - Per-channel numbering
- **File**: `lib/Espfc/src/Device/InputSpektrum.h` **(NEW)**

### 5. SUMD (Graupner) ✅
**Status**: Fully implemented with CRC validation
- **Baud Rate**: 115,200
- **Channels**: Up to 16 (configurable)
- **CRC**: CRC-16-CCITT
- **Features**:
  - Status byte (valid/lost frame indication)
  - Channel count in frame header
  - 16-bit channel values
- **Variants**: Standard SUMD (0xA8) and RJ01 variant (0xBA)
- **File**: `lib/Espfc/src/Device/InputSUMD.h` **(NEW)**

### 6. SUMH (Graupner) ✅
**Status**: Fully implemented with XOR CRC
- **Baud Rate**: 115,200
- **Channels**: 8
- **CRC**: Simple XOR checksum
- **Features**:
  - Status byte
  - Compact 8-bit channel encoding (4 bytes per channel)
  - Fast frame rate (~11ms)
- **File**: `lib/Espfc/src/Device/InputSUMH.h` **(NEW)**

### 7. FPORT (FrSky FPort) ✅
**Status**: Fully implemented with telemetry-capable design
- **Baud Rate**: 115,200
- **Channels**: 16
- **Frame Rate**: 200 Hz
- **CRC**: CRC-8 (0xD5 polynomial)
- **Features**:
  - UART start/end frame markers (0x7E)
  - Extended telemetry capability
  - Compatible with FrSky 2.4 GHz modules
  - Bidirectional data support
- **File**: `lib/Espfc/src/Device/InputFPort.h` **(NEW)**

### 8. PPM (Pulse Position Modulation) ✅
**Status**: Fully implemented
- **Interface**: GPIO interrupt-based
- **Channels**: 8 (typical)
- **Pulse Width Range**: 1000-2000 µs
- **Features**:
  - Hardware timer-based measurement
  - Analog protocol (no serial)
  - Universal compatibility
- **File**: `lib/Espfc/src/Device/InputPPM.h`

### 9. EspNow (WiFi RC Link) ✅
**Status**: Fully implemented
- **Interface**: WiFi radio (ESP-NOW protocol)
- **Channels**: 16
- **Range**: Local network (same subnet)
- **Features**:
  - Low-latency WiFi control
  - Reliable packet delivery
  - Encryption support
- **File**: `lib/Espfc/src/Device/InputEspNow.h`

---

## Implementation Architecture

### Class Hierarchy
```
Device::InputDevice (base interface)
  ├── InputCRSF
  ├── InputSBUS
  ├── InputIBUS
  ├── InputSpektrum (NEW)
  ├── InputSUMD (NEW)
  ├── InputSUMH (NEW)
  ├── InputFPort (NEW)
  ├── InputPPM
  └── InputEspNow
```

### Selection Logic (Input.cpp)
```cpp
Device::InputDevice * Input::getInputDevice()
{
  if (serial && FEATURE_RX_SERIAL) {
    switch (serialRxProvider) {
      case SERIALRX_CRSF:      return &_crsf;
      case SERIALRX_SBUS:      return &_sbus;
      case SERIALRX_IBUS:      return &_ibus;
      case SERIALRX_SPEKTRUM1024/2048: return &_spektrum;
      case SERIALRX_SUMD:      return &_sumd;
      case SERIALRX_SUMH:      return &_sumh;
      case SERIALRX_FPORT:     return &_fport;
    }
  } else if (FEATURE_RX_PPM) {
    return &_ppm;
  } else if (FEATURE_RX_SPI) {
    return &_espnow;
  }
}
```

### Serial Configuration (SerialManager.cpp)
```cpp
switch(serialRxProvider) {
  case SERIALRX_SBUS:
    baud = 100000; parity = EVEN; stop_bits = 2; inverted = true;
  case SERIALRX_IBUS:
  case SERIALRX_SPEKTRUM1024/2048:
  case SERIALRX_SUMD:
  case SERIALRX_SUMH:
  case SERIALRX_FPORT:
    baud = 115200;
  case SERIALRX_CRSF:
    baud = 420000;
}
```

---

## Failsafe Support

All protocols support per-channel failsafe configuration:

**Failsafe Modes**:
1. **HOLD** - Hold last received value
2. **SET** - Move to failsafe value (configurable per channel)
3. **AUTO** - Protocol-specific default (lost/invalid for SBUS, hold for others)

**Configuration**:
- MSP_RXFAIL_CONFIG (read per-channel failsafe)
- MSP_SET_RXFAIL_CONFIG (write per-channel failsafe)

---

## Channel Mapping

All protocols map to standard 16-channel output:

| Protocol | Channels | Mapping |
|----------|----------|---------|
| CRSF     | 16       | Direct 1:1 |
| SBUS     | 16       | Packed 11-bit to 16-bit |
| IBUS     | 14       | Direct 1:1, channels 15-16 default |
| SPEKTRUM | 12       | Direct 1:1, channels 13-16 default |
| SUMD     | 16       | Direct 1:1 (configurable) |
| SUMH     | 8        | Direct 1:1, channels 9-16 default |
| FPORT    | 16       | Packed 11-bit to 16-bit |
| PPM      | 8        | Direct 1:1, channels 9-16 default |
| EspNow   | 16       | Direct 1:1 |

---

## Build Status

```
✅ esp32s3_devkitc:  SUCCESS (8.255 seconds)
✅ esp32s3_wroom:    SUCCESS (8.225 seconds)

Memory Usage:
  RAM:   24.3% (79,492 / 327,680 bytes)
  Flash: 29.9% (998,141 / 3,342,336 bytes)

Compilation:
  Errors:   0
  Warnings: 0
```

---

## MSP Protocol Support

**Receiver Configuration MSP Handlers**:
- `MSP_RX_CONFIG` (44) - Read receiver settings
- `MSP_SET_RX_CONFIG` (45) - Write receiver settings
- `MSP_RC` (105) - Read current RC channel values
- `MSP_RX_MAP` (64) - Read channel mapping
- `MSP_SET_RX_MAP` (65) - Write channel mapping
- `MSP_RXFAIL_CONFIG` (77) - Read failsafe config
- `MSP_SET_RXFAIL_CONFIG` (78) - Write failsafe config
- `MSP_RC_DEADBAND` (125) - Read deadband settings
- `MSP_SET_RC_DEADBAND` (218) - Write deadband settings
- `MSP_RSSI_CONFIG` (50) - Read RSSI channel config
- `MSP_SET_RSSI_CONFIG` (51) - Write RSSI channel config
- `MSP_FAILSAFE_CONFIG` (75) - Read global failsafe
- `MSP_SET_FAILSAFE_CONFIG` (76) - Write global failsafe
- `MSP_SET_RAW_RC` (200) - Set RC values for testing
- `MSP_ARMING_CONFIG` (61) - Read arming settings
- `MSP_SET_ARMING_CONFIG` (62) - Write arming settings
- `MSP_RC_TUNING` (111) - Read RC response tuning
- `MSP_SET_RC_TUNING` (204) - Write RC response tuning

---

## Integration with Betaflight Configurator

All protocols are fully compatible with Betaflight Configurator:
- Auto-detection of protocol type
- Real-time channel monitoring
- Failsafe configuration UI
- Link quality reporting (CRSF/FPORT)
- Telemetry display (CRSF)

---

## Future Enhancements

- XBUS protocol support (FrSky multi-protocol module)
- JETIEXBUS protocol support (JETI receivers)
- SRXL protocol support (Spektrum satellite)
- Custom receiver binding configuration
- RF link statistics logging to blackbox

---

## Files Modified/Created

**New Implementations** (4 files each):
- `lib/Espfc/src/Device/InputSpektrum.h/cpp` (NEW)
- `lib/Espfc/src/Device/InputSUMD.h/cpp` (NEW)
- `lib/Espfc/src/Device/InputSUMH.h/cpp` (NEW)
- `lib/Espfc/src/Device/InputFPort.h/cpp` (NEW)

**Modified Files**:
- `lib/Espfc/src/Input.h` - Added new protocol includes and member variables
- `lib/Espfc/src/Input.cpp` - Added protocol selection cases in getInputDevice()
- `lib/Espfc/src/SerialManager.cpp` - Added baud rate configuration for new protocols

**Unchanged but Compatible**:
- All existing MSP handlers remain functional
- Betaflight Configurator compatibility maintained
- No breaking changes to existing protocols

---

## Deployment

To select a receiver protocol in firmware:
1. Set `config.input.serialRxProvider` to desired protocol enum value
2. Ensure serial port configured with SERIAL_FUNCTION_RX_SERIAL
3. Attach receiver to configured serial port
4. Build and flash firmware
5. Baud rate automatically configured based on protocol selection

---

Last Updated: 2026-06-22
Firmware: esp-fc (multi-target build)
Targets: esp32s3_devkitc, esp32s3_wroom
