# Board Reference

This page provides detailed hardware specifications for all supported targets, including serial ports, communication buses, sensors, and available features.

## Default Motor Control Port Map

Source of truth: target headers in `lib/Espfc/src/Target/Target*.h`, applied to runtime defaults via `ModelConfig::pin[PIN_OUTPUT_n]`.

| Board | Motor 1 | Motor 2 | Motor 3 | Motor 4 | Extra Outputs | PWM Controller |
|---|---:|---:|---:|---:|---|---|
| **ESP32** | GPIO27 | GPIO25 | GPIO4 | GPIO12 | Outputs 4–7 configurable (default `-1`) | Hardware PWM |
| **ESP32-S2** | GPIO39 | GPIO40 | GPIO41 | GPIO42 | None (4-output fixed) | Hardware PWM |
| **ESP32-S3** (all variants) | GPIO39 | GPIO40 | GPIO41 | GPIO42 | None (4-output fixed) | Hardware PWM |
| — ESP32-S3 (lolin_s3_mini) | GPIO39 | GPIO40 | GPIO41 | GPIO42 | — | — |
| — ESP32-S3-DevKitC | GPIO39 | GPIO40 | GPIO41 | GPIO42 | — | — |
| — ESP32-S3-WROOM | GPIO39 | GPIO40 | GPIO41 | GPIO42 | — | — |
| — ESP32-S3-N8R8 | GPIO39 | GPIO40 | GPIO41 | GPIO42 | — | — |
| — ESP32-S3-N8R16 | GPIO39 | GPIO40 | GPIO41 | GPIO42 | — | — |
| **ESP32-C3** | GPIO2 | GPIO3 | GPIO4 | GPIO5 | None (4-output fixed) | Hardware PWM |
| **ESP8266** | GPIO16 (D0) | GPIO14 (D5) | GPIO12 (D6) | GPIO15 (D8) | None (4-output fixed) | Software PWM (≤400 Hz) |
| **RP2040** (Pico) | GPIO2 | GPIO3 | GPIO4 | GPIO5 | Outputs 4–7 configurable (default `-1`) | Hardware PWM |
| **RP2350** (Pico 2) | GPIO2 | GPIO3 | GPIO4 | GPIO5 | Outputs 4–7 configurable (default `-1`) | Hardware PWM |

**Configuration:**
- Use `set pin_output_n <gpio>` to remap a motor output to a different GPIO
- Use `set pin_output_n -1` to disable/unmap a motor output
- **ESP32-S3 variants** (devkitc, wroom, n8r8, n8r16) share identical pin configuration
- **ESP8266** uses software PWM with limited frequency (≤400 Hz); hardware PWM not available
- **RP2040/RP2350** support up to 8 motor outputs (configure extras via `set pin_output_4..7`)

## Default VTX Control Port Map

SmartAudio is the default VTX control protocol. Wire the VTX control line to the listed TX pin.

| Board | Default serial function | TX pin | RX pin | Notes |
|---|---|---:|---:|---|
| ESP32 | UART1 / `SERIAL_FUNCTION_VTX_SMARTAUDIO` | GPIO33 | GPIO32 | Dedicated hardware UART by default |
| ESP32-S2 | UART0 / `SERIAL_FUNCTION_VTX_SMARTAUDIO` | GPIO43 | GPIO44 | USB CDC remains default MSP link |
| ESP32-S3 | UART0 / `SERIAL_FUNCTION_VTX_SMARTAUDIO` | GPIO43 | GPIO44 | USB CDC remains default MSP link |
| ESP32-C3 | UART0 / `SERIAL_FUNCTION_VTX_SMARTAUDIO` | GPIO21 | GPIO20 | USB CDC remains default MSP link |
| ESP8266 | UART1 / `SERIAL_FUNCTION_VTX_SMARTAUDIO` | GPIO2 (D4) | -1 | TX-only, suitable for SmartAudio |
| RP2040 / RP2350 | UART0 / `SERIAL_FUNCTION_VTX_SMARTAUDIO` | GPIO0 | GPIO1 | USB CDC remains default MSP link |

## ESP32 (Classic)

**Overview**: Full-featured classic ESP32 with dual-core, WiFi, Bluetooth, and comprehensive I/O.

### Serial Ports
- **UART0**: TX=GPIO1, RX=GPIO3 (default: MSP/Betaflight)
- **UART1**: TX=GPIO33, RX=GPIO32 (default: SmartAudio VTX control)
- **UART2**: TX=GPIO17, RX=GPIO16 (default: RX_SERIAL for receiver)
- **Soft-Serial 0**: Virtual UART via GPIO pins (WiFi bridge enabled, MSP capable)
- **USB CDC**: Not available on classic ESP32

### Communication Buses
- **SPI**: Bus 1 — SCK=GPIO18, MOSI=GPIO23, MISO=GPIO19 (gyro, barometer)
- **I2C**: SCL=GPIO22, SDA=GPIO21 (software-based, sensors)

### Sensors
- **Gyroscope**: SPI (CS=GPIO5) or I2C (software)
- **Barometer**: SPI (CS=GPIO13) or I2C (software)
- **ADC Channels**: 2 available (GPIO36, GPIO39) for analog input

### Motor Outputs
- **Motor/ESC pins**: GPIO27, GPIO25, GPIO4, GPIO12 (4 outputs supported, 8 slots configurable)

### Audio/LED/Buttons
- **Buzzer**: GPIO26
- **LED**: GPIO2
- **Button**: Not configured

### Features
- ✅ Receiver input: Serial RX_SERIAL via UART2
- ✅ PPM receiver: GPIO35 (analog input pin)
- ✅ Default VTX control: SmartAudio on UART1 TX=GPIO33
- ✅ Dynamic filter: Supported
- ✅ WiFi OTA update: Yes (soft-serial bridge)
- ✅ Bluetooth OTA: Yes (if `ESPFC_BT_OTA` build flag enabled and BT stack present)
- ✅ ESP-NOW wireless: Yes
- ⚠️ DShot telemetry: Supported

### Memory & Performance
- **RAM**: ~512 KB total (typical usage: 26–28% in flight controller mode)
- **Gyro sampling**: I2C max 2 kHz, SPI max 4 kHz

---

## ESP32-S2

**Overview**: Single-core variant with reduced RAM (320 KB), no Bluetooth, optimized for power efficiency.

### Serial Ports
- **UART0**: TX=GPIO43, RX=GPIO44 (default: SmartAudio VTX control)
- **UART1**: TX=GPIO17, RX=GPIO16 (default: RX_SERIAL for receiver)
- **USB CDC**: Native USB support for MSP and debugging
- **Soft-Serial 0**: WiFi bridge **disabled** on this target (RAM optimization)

### Communication Buses
- **SPI**: Bus 0 — SCK=GPIO12, MOSI=GPIO11, MISO=GPIO13 (gyro, barometer)
- **I2C**: SCL=GPIO9, SDA=GPIO8 (software-based, sensors)

### Sensors
- **Gyroscope**: SPI (CS=GPIO10) or I2C
- **Barometer**: SPI (CS=GPIO7) or I2C
- **ADC Channels**: 2 available (GPIO1, GPIO4)

### Motor Outputs
- **Motor/ESC pins**: GPIO39, GPIO40, GPIO41, GPIO42 (4 outputs)

### Audio/LED/Buttons
- **Buzzer**: GPIO5
- **LED**: GPIO15
- **Button**: Not configured

### Features
- ✅ Receiver input: Serial RX_SERIAL via UART1
- ✅ PPM receiver: Not configured
- ✅ Default VTX control: SmartAudio on UART0 TX=GPIO43
- ✅ Dynamic filter: Supported
- ❌ WiFi OTA: No soft-serial bridge (use USB for configuration)
- ❌ Bluetooth OTA: Not available (no BT on S2)
- ❌ Soft-serial WiFi: Disabled (RAM constraints)
- ⚠️ DShot telemetry: Supported

### Memory & Performance
- **RAM**: ~320 KB total (tight budget; soft-serial WiFi disabled to preserve headroom)
- **Gyro sampling**: I2C max 2 kHz, SPI max 2 kHz
- ⚠️ **Note**: Use USB or UART ports for Betaflight configuration; no WiFi alternative

---

## ESP32-S3

**Overview**: Dual-core variant with 512 KB RAM, WiFi, USB-C, and most ESP32-like capabilities.

### Serial Ports
- **UART0**: TX=GPIO43, RX=GPIO44 (default: SmartAudio VTX control)
- **UART1**: TX=GPIO16, RX=GPIO15 (default: GPS)
- **UART2**: TX=GPIO18, RX=GPIO17 (default: RX_SERIAL for receiver)
- **USB CDC**: Native USB support for MSP (optional, depends on Arduino build config)
- **Soft-Serial 0**: Virtual UART with WiFi bridge enabled (MSP capable)

### Communication Buses
- **SPI**: Bus 1 — SCK=GPIO12, MOSI=GPIO11, MISO=GPIO13 (gyro, barometer)
- **I2C**: SCL=GPIO10, SDA=GPIO9 (software-based, sensors)

### Sensors
- **Gyroscope**: SPI (CS=GPIO8) or I2C
- **Barometer**: SPI (CS=GPIO7) or I2C
- **ADC Channels**: 2 available

### Motor Outputs
- **Motor/ESC pins**: GPIO39, GPIO40, GPIO41, GPIO42 (4 outputs)

### Audio/LED/Buttons
- **Buzzer**: GPIO5
- **LED**: GPIO15
- **Button**: Not configured

### Features
- ✅ Receiver input: Serial RX_SERIAL via UART2 or PPM via GPIO6
- ✅ Default VTX control: SmartAudio on UART0 TX=GPIO43
- ✅ Dynamic filter: Supported
- ✅ WiFi OTA update: Yes (soft-serial bridge)
- ❌ Bluetooth OTA: Not available on S3
- ✅ ESP-NOW wireless: Yes
- ⚠️ DShot telemetry: Supported

### Memory & Performance
- **RAM**: ~512 KB (26–30% typical usage)
- **Gyro sampling**: I2C max 2 kHz, SPI max 4 kHz

---

## ESP32-S3-N8R8 & ESP32-S3-N8R16 (BOX / Smart Home Boards)

**Overview**: ESP32-S3 variants optimized for compact form factors with built-in sensors. N8R8/N8R16 refer to 8MB-Flash with 8MB/16MB PSRAM variants. These use the same target configuration as lolin_s3_mini (1.3MB flash partition).

### Serial Ports
- **UART0**: TX=GPIO43, RX=GPIO44 (default: SmartAudio VTX control)
- **UART1**: TX=GPIO16, RX=GPIO15 (default: GPS)
- **UART2**: TX=GPIO18, RX=GPIO17 (default: RX_SERIAL for receiver)
- **USB CDC**: Not available (compact form factor)
- **Soft-Serial 0**: Virtual UART with WiFi bridge enabled

### Communication Buses
- **SPI**: Bus 1 — SCK=GPIO12, MOSI=GPIO11, MISO=GPIO13 (gyro, barometer)
- **I2C**: SCL=GPIO10, SDA=GPIO9 (software-based, sensors)

### Sensors
- **Gyroscope**: SPI (CS=GPIO8) or I2C
- **Barometer**: SPI (CS=GPIO7) or I2C
- **ADC Channels**: 2 available

### Motor Outputs
- **Motor/ESC pins**: GPIO39, GPIO40, GPIO41, GPIO42 (4 outputs)

### Audio/LED/Buttons
- **Buzzer**: GPIO5
- **LED**: GPIO15
- **Button**: GPIO6

### Features
- ✅ Receiver input: Serial RX_SERIAL via UART2
- ✅ Default VTX control: SmartAudio on UART0 TX=GPIO43
- ✅ Motor Stop: Enabled
- ✅ Dynamic filter: Supported
- ✅ WiFi OTA update: Yes (soft-serial bridge)
- ✅ ESP-NOW wireless: Yes
- ⚠️ DShot telemetry: Supported

### Memory & Performance
- **RAM**: ~512 KB (26–30% typical usage)
- **PSRAM**: 8 MB (N8R8) or 16 MB (N8R16) for additional buffering
- **Flash**: 1.3 MB for firmware (same as S3 mini)
- **Gyro sampling**: I2C max 2 kHz, SPI max 4 kHz

**Notes**: N8R8/N8R16 boards are particularly well-suited for autonomous missions or heavy telemetry logging due to PSRAM availability.

---

## ESP32-C3

**Overview**: Ultra-compact RISC-V single-core with 320 KB RAM, WiFi, USB-C.

### Serial Ports
- **UART0**: TX=GPIO21, RX=GPIO20 (default: SmartAudio VTX control)
- **UART1**: TX, RX disabled (pins reserved for flash/USB; not usable)
- **USB CDC**: Native USB for MSP and debugging
- **Soft-Serial 0**: Virtual UART with WiFi bridge (MSP capable)

### Communication Buses
- **SPI**: Not available on standard C3 (pins reserved or not routed)
- **I2C**: SCL=GPIO6, SDA=GPIO8 (software-based, sensors only)

### Sensors
- **Gyroscope**: I2C only (no SPI on C3)
- **Barometer**: I2C only
- **ADC Channels**: 2 available (GPIO0, GPIO1)

### Motor Outputs
- **Motor/ESC pins**: GPIO2, GPIO3, GPIO4, GPIO5 (4 outputs)

### Audio/LED/Buttons
- **Buzzer**: Not configured
- **LED**: Not configured
- **Button**: Not configured

### Features
- ✅ Receiver input: Limited (no RX_SERIAL port available; consider soft-serial or PPM)
- ❌ PPM receiver: Not configured
- ✅ Default VTX control: SmartAudio on UART0 TX=GPIO21
- ✅ Dynamic filter: Supported
- ✅ WiFi OTA update: Yes (soft-serial bridge)
- ❌ Bluetooth OTA: Not available
- ✅ ESP-NOW wireless: Yes
- ⚠️ Constraint: I2C-only sensors; no SPI gyro available

### Memory & Performance
- **RAM**: ~320 KB (tight; sufficient for I2C-only setup)
- **Gyro sampling**: I2C max 1 kHz

---

## ESP8266

**Overview**: Legacy single-core WiFi board with minimal resources (80 KB RAM), popular in legacy drone projects.

### Serial Ports
- **UART0**: TX=GPIO1, RX=GPIO3 (default: MSP/Betaflight)
- **UART1**: TX=GPIO2 (TX-only, no RX; default: SmartAudio VTX control)
- **Soft-Serial 0**: Virtual UART with WiFi bridge (MSP capable)

### Communication Buses
- **I2C**: SCL=GPIO5 (D1), SDA=GPIO4 (D2) (software-based, sensors only)
- **SPI**: Not configured/available on standard layout

### Sensors
- **Gyroscope**: I2C only
- **Barometer**: I2C only
- **ADC**: Single ADC channel (GPIO17/A0) for analog input

### Motor Outputs
- **Motor/ESC pins**: GPIO16 (D0), GPIO14 (D5), GPIO12 (D6), GPIO15 (D8) (4 outputs)

### Audio/LED/Buttons
- **Buzzer**: Not configured
- **LED**: Not configured
- **Button**: Not configured

### Features
- ✅ Receiver input: PPM via GPIO13 (D7) or soft-serial
- ✅ PPM receiver: Supported (GPIO13)
- ✅ Default VTX control: SmartAudio on UART1 TX=GPIO2 (D4)
- ✅ Dynamic filter: Supported
- ✅ WiFi OTA update: Yes (soft-serial bridge)
- ❌ Bluetooth: Not available
- ✅ ESP-NOW wireless: Yes
- ⚠️ **Constraint**: Very limited RAM (80 KB); I2C-only sensors; UART1 is TX-only

### Memory & Performance
- **RAM**: ~80 KB (tight; careful with feature selection)
- **Gyro sampling**: I2C max 1 kHz

---

## RP2040 (Raspberry Pi Pico)

**Overview**: Dual-core ARM Cortex-M0+ with 264 KB RAM, USB, and excellent SPI support.

### Serial Ports
- **UART0** (Serial1): TX=GPIO0, RX=GPIO1 (default: SmartAudio VTX control)
- **UART1** (Serial2): TX=GPIO8, RX=GPIO9 (default: RX_SERIAL for receiver)
- **USB CDC**: Native USB for MSP and debugging
- **Soft-Serial**: Not available on RP2040

### Communication Buses
- **SPI**: Bus 1 — SCK=GPIO14, MOSI=GPIO15, MISO=GPIO12 (gyro, barometer)
- **I2C**: SDA=GPIO16, SCL=GPIO17 (sensors)

### Sensors
- **Gyroscope**: SPI (CS=GPIO13) or I2C
- **Barometer**: SPI (CS=GPIO11) or I2C
- **ADC Channels**: 2 available (GPIO26, GPIO27)

### Motor Outputs
- **Motor/ESC pins**: GPIO2, GPIO3, GPIO4, GPIO5 (4 outputs, 8 slots configurable)

### Audio/LED/Buttons
- **Buzzer**: Not configured
- **LED**: Not configured
- **Button**: Not configured

### Features
- ✅ Receiver input: Serial RX_SERIAL via UART1
- ❌ PPM receiver: Not configured
- ✅ Default VTX control: SmartAudio on UART0 TX=GPIO0
- ✅ Dynamic filter: Supported
- ❌ WiFi OTA: Not available
- ❌ Bluetooth OTA: Not available
- ❌ Soft-serial WiFi: Not available
- ✅ Multi-core support: Yes (dual-core ARM M0+)
- ⚠️ DShot telemetry: Supported

### Memory & Performance
- **RAM**: 264 KB total (efficient; use for standalone or USB-connected setups)
- **Gyro sampling**: I2C max 1 kHz, SPI max 1 kHz (RP2040) or 4 kHz (RP2350 variant)

---

## RP2350 (Raspberry Pi Pico 2)

**Overview**: Dual-core ARM Cortex-M33 microcontroller with 520 KB RAM, 4 MB Flash, and excellent performance for USB-standalone flight control. Faster variant of RP2040.

### Serial Ports
- **UART0** (Serial1): TX=GPIO0, RX=GPIO1 (default: SmartAudio VTX control)
- **UART1** (Serial2): TX=GPIO8, RX=GPIO9 (default: RX_SERIAL for receiver)
- **USB CDC**: Native USB for MSP and debugging
- **Soft-Serial**: Not available on RP2350

### Communication Buses
- **SPI**: Bus 1 — SCK=GPIO14, MOSI=GPIO15, MISO=GPIO12 (gyro, barometer)
- **I2C**: SDA=GPIO16, SCL=GPIO17 (sensors)

### Sensors
- **Gyroscope**: SPI (CS=GPIO13) or I2C
- **Barometer**: SPI (CS=GPIO11) or I2C
- **ADC Channels**: 2 available (GPIO26, GPIO27)

### Motor Outputs
- **Motor/ESC pins**: GPIO2, GPIO3, GPIO4, GPIO5 (4 outputs, 8 slots configurable)

### Audio/LED/Buttons
- **Buzzer**: Not configured
- **LED**: Not configured
- **Button**: Not configured

### Features
- ✅ Receiver input: Serial RX_SERIAL via UART1
- ❌ PPM receiver: Not configured (USB-standalone focus)
- ✅ Default VTX control: SmartAudio on UART0 TX=GPIO0
- ✅ Motor Stop: Enabled
- ✅ GPS Support: Enabled (excellent headroom with 4MB flash and 520KB RAM)
- ✅ Telemetry (CRSF): Enabled for long-range setups
- ✅ Dynamic filter: Supported
- ❌ WiFi OTA: Not available (no wireless modules)
- ❌ Bluetooth OTA: Not available
- ❌ Soft-serial: Not available
- ✅ DShot telemetry: Supported
- ✅ Multi-core support: Yes (dual-core ARM M33)

### Memory & Performance
- **RAM**: 520 KB total (excellent; only 6.7% used by firmware)
- **Flash**: 4 MB total (93.3% headroom available for user data)
- **CPU**: Dual-core ARM Cortex-M33 @ 150 MHz (excellent performance)
- **Gyro sampling**: I2C max 1 kHz, SPI max 4 kHz (faster than RP2040)

### Key Advantages
- **Excellent flash and RAM headroom**: Most feature-rich board with maximum available features enabled by default
- **High-performance ARM M33**: Dual-core execution enables background tasks without affecting flight stability
- **USB-standalone**: Perfect for bench testing or tethered flight applications
- **GPS & Telemetry ready**: Supports autonomous missions and real-time telemetry out-of-the-box

---

## Feature Comparison Table

| Feature | ESP32 | ESP32-S2 | ESP32-S3 | ESP32-C3 | ESP8266 | RP2040 | RP2350 |
|---|---|---|---|---|---|---|---|
| **UART Ports** | 3 full | 2 full | 3 full | 1 full | 1 full + 1 TX-only | 2 full | 2 full |
| **I2C** | Yes | Yes | Yes | Yes (I2C only) | Yes | Yes | Yes |
| **SPI** | Yes | Yes | Yes | No | No | Yes | Yes |
| **USB CDC** | No | Yes | Optional | Yes | No | Yes | Yes |
| **WiFi OTA** | Yes | No | Yes | Yes | Yes | No | No |
| **Bluetooth OTA** | Optional* | No | No | No | No | No | No |
| **Soft-Serial WiFi** | Yes | No** | Yes | Yes | Yes | No | No |
| **DShot Telemetry** | Yes | Yes | Yes | Yes | No | Yes | Yes |
| **Flash Storage** | 1.3 MB | 1.3 MB | 1.3 MB (mini) / 3.3 MB (devkitc/wroom) | 1.3 MB | 1 MB | 2 MB | 4 MB |
| **Receiver Input** | ✅ Serial | ✅ Serial | ✅ Serial | ✅ Serial | ✅ PPM | ✅ Serial | ✅ Serial |
| **Motor Stop** | ✅ Enabled | ✅ Enabled | ✅ Enabled | ✅ Enabled | ✅ Enabled | ✅ Enabled | ✅ Enabled |
| **GPS Support** | Available | Available | Available | Available | ✅ Enabled | ✅ Enabled | ✅ Enabled |
| **Telemetry (CRSF)** | Available | Available | Available | Available | ✅ Enabled | ✅ Enabled | ✅ Enabled |
| **Dynamic Filter** | ✅ Enabled | ✅ Enabled | ✅ Enabled | ✅ Enabled | ✅ Enabled | ✅ Enabled | ✅ Enabled |
| **Dual-Core** | Yes | No | Yes | No | No | Yes (ARM M0+) | Yes (ARM M33) |
| **Total RAM** | 512 KB | 320 KB | 512 KB | 320 KB | 80 KB | 264 KB | 520 KB |
| **Flash Usage** | 84.7% | 76.5% | 81.5% | 87.3% | 52.7% | 13.6% | 6.7% |
| **Typical Usage** | General purpose | RAM-constrained | Balanced | Compact I2C | Legacy | USB-standalone | High-performance USB |

**Notes:**
- *ESP32 Bluetooth OTA requires `ESPFC_BT_OTA` build flag and Bluetooth stack support
- **ESP32-S2 soft-serial WiFi disabled to preserve RAM
- GPS Support: "Available" = feature can be enabled but not default; "✅ Enabled" = feature enabled by default
- Telemetry (CRSF): Feature flag determines if CRSF telemetry output is available when using CRSF receiver
- Flash Usage: Percentage of total flash used by compiled firmware (lower = more room for user data)

---

## Feature Enablement Strategy

### How Feature Flags Work

ESP-FC uses compile-time feature masks to control which features are available on each board. The `ESPFC_FEATURE_MASK` in each target header (e.g., `TargetESP32.h`) defines the default set of enabled features.

Features are categorized by complexity and resource requirements:

| Feature | Purpose | Flash Impact | RAM Impact | Hardware Requirements |
|---|---|---|---|---|
| **FEATURE_MOTOR_STOP** | Safety: Stop motors when throttle is below 5% | None (logic already compiled) | None | None; always recommended |
| **FEATURE_GPS** | Altitude hold, position hold, return-to-home | ~5–10 KB | Minimal | UART port for GPS module (typically UART1) |
| **FEATURE_TELEMETRY** | CRSF telemetry output for long-range transmitters | ~2–3 KB | None | Uses same UART as GPS receiver protocol |
| **FEATURE_DYNAMIC_FILTER** | Runtime-adaptive gyro filtering (enabled for loop rates ≥ 1 kHz) | ~2–3 KB | None | Enabled only if gyro sampling ≥ 1 kHz |

### Board-Specific Feature Recommendations

**Ultra-Compact Boards (Limited RAM/Flash)**
- **ESP32-C3** (87.3% flash, 320 KB RAM): Minimal feature set
  - ✅ Enabled: Motor Stop, Dynamic Filter (only at 1 kHz+)
  - ❌ Recommended: GPS, Telemetry (insufficient headroom; I2C-only sensors also limit receiver options)
  - **Action**: Keep defaults; use CLI to manually enable GPS if needed for testing

**Tight Budget Boards (Flash > 75%)**
- **ESP32** (84.7% flash, 512 KB RAM), **ESP32-S3** (81.5% flash, 512 KB RAM), **ESP32-S3-N8R8/N8R16** (81.4% flash with standard partition)
  - ✅ Enabled: Motor Stop, Dynamic Filter
  - ⚠️ Available: GPS and Telemetry (can be manually enabled via CLI if needed for specific missions)
  - **Action**: Sufficient for most flights; GPS/Telemetry can be enabled for extended flights if desirable

**RAM-Constrained (< 350 KB)**
- **ESP32-S2** (320 KB RAM, 76.5% flash)
  - ✅ Enabled: Motor Stop, Dynamic Filter
  - ⚠️ Available: GPS, Telemetry (soft-serial WiFi bridge disabled to preserve RAM)
  - **Action**: Use USB or standard serial ports; no WiFi OTA available

**Moderate Headroom (Flash 50–70%)**
- **ESP8266** (52.7% flash, 80 KB RAM)
  - ✅ Enabled: Motor Stop, GPS, Telemetry, Dynamic Filter (excellent flash headroom)
  - ⚠️ Constraint: Only 80 KB RAM (very tight); single UART limits simultaneous RX + GPS
  - **Action**: GPS/Telemetry enabled; recommend PPM receiver + soft-serial GPS

**Excellent Headroom (Flash < 20%)**
- **RP2040** (13.6% flash, 264 KB RAM), **RP2350** (6.7% flash, 520 KB RAM)
  - ✅ Enabled: Motor Stop, GPS, Telemetry, Dynamic Filter
  - ✅ Bonus: Can add additional custom features if needed (>80% headroom available)
  - **Action**: Maximum feature set; ideal for autonomous missions, logging, and advanced telemetry

### How to Check/Modify Features

**CLI Commands** (in Betaflight Configurator):
```
# View all features
get | grep feature

# Enable a feature (example: GPS)
set feature_gps 1
save

# Disable a feature
set feature_gps 0
save
```

**To Recompile with Different Feature Defaults**:
Edit the target header file (e.g., `lib/Espfc/src/Target/TargetESP32.h`):
```
// Current: Motor Stop + Dynamic Filter only
#define ESPFC_FEATURE_MASK (FEATURE_RX_SERIAL | FEATURE_MOTOR_STOP | FEATURE_DYNAMIC_FILTER)

// Modified: Add GPS + Telemetry (if flash headroom available)
#define ESPFC_FEATURE_MASK (FEATURE_RX_SERIAL | FEATURE_MOTOR_STOP | FEATURE_GPS | FEATURE_TELEMETRY | FEATURE_DYNAMIC_FILTER)
```

---

## Sensor Configuration Guidelines

### Best Sensor Choice by Board

| Board | Recommended Setup |
|---|---|
| **ESP32** | SPI gyro + SPI barometer (fastest) |
| **ESP32-S2** | SPI gyro + SPI barometer |
| **ESP32-S3** | SPI gyro + SPI barometer or I2C-only for compact builds |
| **ESP32-C3** | I2C gyro + I2C barometer (no SPI) |
| **ESP8266** | I2C gyro + I2C barometer (limited to slow I2C) |
| **RP2040** | SPI gyro + SPI barometer (excellent SPI support) |

---

## Receiver Protocol Support

All boards support the following receiver protocols when configured via serial RX_SERIAL port or PPM input:
- **SBUS**: Serial protocol, all boards with RX_SERIAL port
- **IBUS**: Serial protocol, all boards with RX_SERIAL port
- **CRSF**: Serial protocol, all boards with RX_SERIAL port (excellent for long-range)
- **PPM**: Pulse-position modulation, boards with PPM input configured
- **ESP-NOW**: Wireless protocol (ESP8266, ESP32 variants)

See [Receiver Protocols](RECEIVER_PROTOCOLS.md) for detailed configuration.

---

## Configuration Methods: GUI vs CLI

ESP-FC features can be configured via **Betaflight Configurator (GUI/MSP)** or **CLI (command-line)**. Not all features are available through both interfaces.

### GUI (Betaflight Configurator) — Available on All Boards

Use the **Betaflight Configurator** for these features:

| Feature Area | Tabs in BFC | Example Config |
|---|---|---|
| **Serial Ports** | Ports | MSP, RX_SERIAL, Telemetry |
| **Receiver** | Receiver | SBUS, IBUS, CRSF protocols, failsafe |
| **Sensor Config** | Sensors | Gyro/Baro bus (I2C/SPI), alignment |
| **Sensor Calibration** | Calibration | Gyro, accelerometer, magnetometer calibration |
| **Gyro/D-Term Filters** | Filters | LPF, dynamic notch, RPM filter |
| **Motor/ESC Setup** | Motors | Protocol (PWM, DShot), rate, idle throttle |
| **PID Tuning** | PID tuning | P, I, D, F gains per axis |
| **Board Alignment** | Configuration | Yaw/Pitch/Roll rotation |
| **Mixer** | Motors | Quad X, Hex, etc. |
| **Receiver/RC Tuning** | Receiver | Rates, expo, throttle limit |
| **Arming** | Configuration | Small angle, arm/disarm behavior |
| **Battery Monitor** | Power & Battery | Voltage/current sensor scaling |
| **Feature Flags** | Configuration | GPS, DynNotch, Motor Stop, RX protocol flags |

### CLI-Only Configuration — Not in GUI

These features can **only be configured via CLI** (type `get` / `set` in Betaflight CLI tab):

| Feature | CLI Command | Board-Specific Limits |
|---|---|---|
| **Pin Mapping** | `set pin_*_tx/-rx <GPIO>` | Per-target default pins; remappable if `ESPFC_SERIAL_REMAP_PINS` enabled |
| **Soft-Serial Config** | `set serial_soft_0 <function> <baud>` | Only on boards with `ESPFC_SERIAL_SOFT_0` enabled |
| **WiFi / Bluetooth** | `wifi` command | ESP32 variants only; not available RP2040, ESP8266 (limited), ESP32-S2 (no soft-serial) |
| **Sensor Offset/Bias** | `set gyro_offset_* <value>` | Runtime trim; persisted to EEPROM |
| **Sensor Calibration Advanced** | `cal gyro`, `cal mag`, `cal reset_*` | Programmatic control; CLI-only in BFC |
| **Receiver Filter Presets** | `preset` commands | Input smoothing, scaler configuration |
| **Landing Assist Presets** | `preset landing_indoor/outdoor` | Indoor/outdoor/freestyle mode presets |
| **Altitude Fusion Presets** | `preset alt_fusion_*` | GPS/barometer fusion modes |
| **GPS Home Management** | `gps clear_home` | CLI-only GPS command |
| **Motor Testing** | `motors <test_value>` | Direct motor output test |
| **DShot Motor Poles** | `set output_motor_poles <n>` | Only if `ESPFC_DSHOT_TELEMETRY` enabled |
| **Beeper Presets** | `preset` beeper masks | Audible warning configuration |
| **Rangefinder Config** | `range_bottom/range_front [params]` | Bottom/front sensor setup |
| **Optical Flow Config** | `set opflow_* <value>` | Advanced sensor setup |
| **Real-Time Diagnostics** | `status`, `stats`, `devinfo` | Runtime CPU load, memory, FreeRTOS tasks |
| **Blackbox / Flash** | `logs`, `flash [erase/test/print]` | Dataflash logging control |
| **Dump/Export Config** | `dump` | Export all settings as CLI commands |
| **Presets / Tuning Profiles** | `load <preset_name>` | Pre-tuned setups for different frame types |

---

## Feature Availability by Board

### Always Available (All Boards)
- ✅ Serial MSP/CLI configuration (Ports tab in BFC)
- ✅ Receiver setup (Receiver tab)
- ✅ Sensor config + calibration (Sensors + Calibration tabs)
- ✅ Gyro/D-Term filters (Filters tab)
- ✅ Motor/ESC setup (Motors tab)
- ✅ PID tuning (PID Tuning tab)
- ✅ Mixer/board alignment (Motors/Configuration tabs)

### Board-Specific Limitations

| Board | CLI-Only Features | MSP Limitations |
|---|---|---|
| **ESP32** | Pin remapping, WiFi, BT, motor poles, landing/alt fusion presets | All standard MSP available |
| **ESP32-S2** | Pin remapping, motor poles (RAM constrained) | No WiFi soft-serial; use USB for config |
| **ESP32-S3** | Pin remapping, WiFi, motor poles, landing/alt fusion presets | All standard MSP available |
| **ESP32-C3** | Pin remapping, WiFi, motor poles (I2C-only sensors, no SPI) | Sensor limited to I2C; no SPI gyro available |
| **ESP8266** | Pin remapping, motor poles, landing/alt fusion (limited RAM) | Very limited RAM; minimize feature flags |
| **RP2040** | Pin remapping, motor poles, rangefinder/opflow config | No WiFi/BT; no soft-serial; USB-only wireless config |

---

## Example Configuration Workflow

### For Core Flight Controller Setup (GUI + BFC)
1. Connect via USB
2. **Ports tab**: Assign serial functions (MSP, RX_SERIAL, Telemetry)
3. **Receiver tab**: Choose protocol (SBUS, IBUS, CRSF)
4. **Sensors tab**: Select gyro/baro bus (I2C or SPI)
5. **Filters tab**: Set LPF and notch filters
6. **Motors tab**: Assign ESC protocol, test motors
7. **PID Tuning tab**: Load defaults, tune to your frame
8. Save & reboot

### For Advanced/Board-Specific Setup (CLI-Only)
1. Open **CLI tab** in BFC (Ctrl+L)
2. **Check capabilities**: `status` (shows CPU/RAM/task info)
3. **Custom tuning**: `preset landing_outdoor` (load preset)
4. **Pin remapping** (if needed): `set pin_serial_0_tx 1` (for alternative wiring)
5. **Sensor calibration**: `cal gyro`, then `save`
6. **Diagnostics**: `stats` (see loop timing)
7. **Export config**: `dump` (backup as CLI commands)

---

## Next Steps

- For quick serial setup examples, see [Setup — Quick serial setup by target](setup.md#quick-serial-setup-by-target)
- For sensor pinouts and wiring, see [Connections & Wiring](connections.md)
- For CLI commands to configure sensors and ports, see [CLI Reference](cli.md)
- For wireless receiver setup, see [Wireless Functions](wireless.md)
