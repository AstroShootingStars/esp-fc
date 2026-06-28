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
| **STM32F7** (Nucleo F767ZI, experimental) | D3 | D5 | D6 | D9 | None (4-output fixed) | Hardware PWM |
| **STM32H7** (Nucleo H743ZI/H723ZG, H743VGT6 custom, experimental) | D3 | D5 | D6 | D9 | None (4-output fixed) | Hardware PWM |

**Configuration:**
- Use `set pin_output_n <gpio>` to remap a motor output to a different GPIO
- Use `set pin_output_n -1` to disable/unmap a motor output
- **ESP32-S3 variants** (devkitc, wroom, n8r8, n8r16) share identical pin configuration
- **ESP8266** uses software PWM with limited frequency (≤400 Hz); hardware PWM not available
- **RP2040/RP2350** support up to 8 motor outputs (configure extras via `set pin_output_4..7`)
- **STM32F7/STM32H7** support is currently experimental scaffold-level with board defaults tuned for Nucleo headers
- **STM32H7** scaffold is build-validated on `nucleo_h743zi`, `nucleo_h723zg` (STM32H723ZGT6), and `stm32h743vgt6` (custom board definition)

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
| STM32F7 | Not assigned by default | -1 | -1 | Single-UART scaffold; assign SmartAudio manually if needed |
| STM32H7 | Not assigned by default | -1 | -1 | Single-UART scaffold; assign SmartAudio manually if needed |

## Default Buzzer / Beeper Pin Map

Default buzzer pins are assigned only where the target header defines `ESPFC_BUZZER_PIN`. The listed pins do not overlap with the other default feature pins for that board.

| Board | Default buzzer pin | Conflict check | Status |
|---|---:|---|---|
| ESP32 | GPIO26 | No overlap with default motor, serial, SPI, I2C, LED, or ADC pins | Configured |
| ESP32-S2 | GPIO5 | No overlap with default motor, serial, SPI, I2C, LED, or ADC pins | Configured |
| ESP32-S3 | GPIO5 | No overlap with default motor, serial, SPI, I2C, LED, or ADC pins | Configured |
| ESP32-C3 | -1 | Not configured by default | Unassigned |
| ESP8266 | -1 | Not configured by default | Unassigned |
| RP2040 | -1 | Not configured by default | Unassigned |
| RP2350 | -1 | Not configured by default | Unassigned |
| STM32F7 | -1 | Not configured by default | Unassigned |
| STM32H7 | -1 | Not configured by default | Unassigned |

## STM32F7 (Nucleo F767ZI, Experimental)

**Overview**: Experimental STM32 scaffold target using Arduino STM32 core with HAL/LL integration.

### Serial Ports
- **UART0**: board default serial object (`Serial`) for MSP
- **UART1/UART2**: not assigned by default in scaffold config

### Communication Buses
- **SPI**: SCK=D13, MOSI=D11, MISO=D12, CS_GYRO=D10, CS_BARO=D4
- **I2C**: SCL=D15, SDA=D14

### Sensors
- **Gyroscope**: SPI or I2C (board wiring dependent)
- **Barometer**: SPI or I2C (board wiring dependent)
- **ADC**: available via remap; default ADC pins are unassigned (`-1`)

### Motor Outputs
- **Motor/ESC pins**: D3, D5, D6, D9

### Features
- ✅ MSP over serial
- ✅ SPI/I2C sensor bus support
- ⚠️ RX/VTX/OSD require manual serial function assignment
- ❌ WiFi OTA / Bluetooth OTA / ESP-NOW

---

## STM32H7 (Nucleo H743ZI/H723ZG + H743VGT6 Custom, Experimental)

**Overview**: Experimental STM32 scaffold target using Arduino STM32 core with HAL/LL integration. Build-validated on Nucleo H743ZI, Nucleo H723ZG (STM32H723ZGT6), and STM32H743VGT6 via custom PlatformIO board (`boards/stm32h743vgt6.json`).

### Serial Ports
- **UART0**: board default serial object (`Serial`) for MSP
- **UART1/UART2**: not assigned by default in scaffold config

### Communication Buses
- **SPI**: SCK=D13, MOSI=D11, MISO=D12, CS_GYRO=D10, CS_BARO=D4
- **I2C**: SCL=D15, SDA=D14

### Sensors
- **Gyroscope**: SPI or I2C (board wiring dependent)
- **Barometer**: SPI or I2C (board wiring dependent)
- **ADC**: available via remap; default ADC pins are unassigned (`-1`)

### Motor Outputs
- **Motor/ESC pins**: D3, D5, D6, D9

### Features
- ✅ MSP over serial
- ✅ SPI/I2C sensor bus support
- ⚠️ RX/VTX/OSD require manual serial function assignment
- ❌ WiFi OTA / Bluetooth OTA / ESP-NOW

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

> [!NOTE]
> STM32F7/STM32H7 are experimental scaffold targets and are intentionally excluded from this condensed matrix; see the dedicated STM32 sections above for current defaults and limitations.

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
@@
@@### Complete Feature Support Matrix by Board
@@
@@| Feature | Purpose | ESP32 | ESP32-S2 | ESP32-S3 | ESP32-C3 | ESP8266 | RP2040 | RP2350 | Configure Via |
@@|---|---|---|---|---|---|---|---|---|---|
@@| **Motor PWM Output** | ESC control | ✅ 8 configurable | ✅ 4 fixed | ✅ 4 fixed | ✅ 4 fixed | ✅ 4 fixed | ✅ 8 configurable | ✅ 8 configurable | GUI + CLI |
@@| **SBUS Receiver** | Serial protocol | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | GUI |
@@| **IBUS Receiver** | Serial protocol | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | GUI |
@@| **CRSF Receiver** | Long-range protocol | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | GUI |
@@| **PPM Receiver** | Pulse-position modulation | ✅ GPIO35 | ❌ No | ✅ GPIO6 | ❌ No | ✅ GPIO13 | ❌ No | ❌ No | CLI |
@@| **ESP-NOW** | Wireless link | ✅ Yes | ❌ No | ✅ Yes | ✅ Yes | ✅ Yes | ❌ No | ❌ No | CLI |
@@| **Gyro I2C** | Primary sensor | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | GUI |
@@| **Gyro SPI** | Alternative sensor | ✅ Yes (CS=GPIO5) | ✅ Yes (CS=GPIO10) | ✅ Yes (CS=GPIO8) | ❌ No SPI | ❌ No SPI | ✅ Yes (CS=GPIO13) | ✅ Yes (CS=GPIO13) | GUI |
@@| **Barometer I2C** | Altitude measurement | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | GUI |
@@| **Barometer SPI** | Alternative sensor | ✅ Yes (CS=GPIO13) | ✅ Yes (CS=GPIO7) | ✅ Yes (CS=GPIO7) | ❌ No SPI | ❌ No SPI | ✅ Yes (CS=GPIO11) | ✅ Yes (CS=GPIO11) | GUI |
@@| **Magnetometer I2C** | Compass | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | GUI |
@@| **GPS (UART)** | Position/altitude | ✅ Available | ✅ Available | ✅ Available | ✅ Available | ✅ Enabled | ✅ Enabled | ✅ Enabled | GUI |
@@| **GPS (UART1)** | Default GPS port | GPIO16/17 | GPIO16/17 | GPIO15/16 | ❌ N/A | ❌ N/A | GPIO9/8 | GPIO9/8 | CLI |
@@| **CRSF Telemetry Out** | TX telemetry | ✅ Available | ✅ Available | ✅ Available | ✅ Available | ✅ Enabled | ✅ Enabled | ✅ Enabled | GUI |
@@| **SmartAudio VTX** | VTX control | ✅ Yes (UART1) | ✅ Yes (UART0) | ✅ Yes (UART0) | ✅ Yes (UART0) | ✅ Yes (UART1) | ✅ Yes (UART0) | ✅ Yes (UART0) | GUI |
@@| **DShot ESC Protocol** | Modern PWM | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | ❌ Partial | ✅ Yes | ✅ Yes | GUI |
@@| **DShot Telemetry** | RPM feedback | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | ❌ No | ✅ Yes | ✅ Yes | CLI |
@@| **Rangefinder (sonar)** | Altitude below 5m | ✅ Available | ✅ Available | ✅ Available | ⚠️ Limited GPIO | ⚠️ No GPIO left | ✅ Available | ✅ Available | CLI |
@@| **Optical Flow** | Ground speed | ✅ Available | ✅ Available | ✅ Available | ⚠️ Limited GPIO | ⚠️ No GPIO left | ✅ Available | ✅ Available | CLI |
@@| **Blackbox Logging** | Flight data recording | ✅ Available | ✅ Available | ✅ Available | ✅ Available | ✅ Enabled | ✅ Enabled | ✅ Enabled | GUI + CLI |
@@| **OSD (On-Screen Display)** | AIO board integration | ✅ Yes (MSP) | ✅ Yes (MSP) | ✅ Yes (MSP) | ✅ Yes (MSP) | ✅ Yes (MSP) | ✅ Yes (MSP) | ✅ Yes (MSP) | GUI |
@@| **Motor Stop** | Failsafe disarm | ✅ Enabled | ✅ Enabled | ✅ Enabled | ✅ Enabled | ✅ Enabled | ✅ Enabled | ✅ Enabled | GUI |
@@| **Low Battery Alarm** | Power monitoring | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | GUI |
@@| **Buzzer / Beeper** | Audio alerts | ✅ GPIO26 | ✅ GPIO5 | ✅ GPIO5 | ❌ Not assigned | ❌ Not assigned | ❌ Not assigned | ❌ Not assigned | CLI |
@@| **LED Indicator** | Status light | ✅ GPIO2 | ✅ GPIO15 | ✅ GPIO15 | ❌ Not assigned | ❌ Not assigned | ❌ Not assigned | ❌ Not assigned | CLI |
@@| **Board Button** | ARM/disarm shortcut | ❌ Not assigned | ❌ Not assigned | ✅ GPIO6 (WROOM) | ❌ Not assigned | ❌ Not assigned | ❌ Not assigned | ❌ Not assigned | CLI |
@@| **WiFi OTA Firmware** | Wireless update | ✅ Yes (soft-serial) | ❌ No (RAM constrained) | ✅ Yes (soft-serial) | ✅ Yes (soft-serial) | ✅ Yes (soft-serial) | ❌ No WiFi | ❌ No WiFi | CLI |
@@| **Bluetooth OTA** | BT wireless update | ✅ Optional** | ❌ No | ❌ No | ❌ No | ❌ No | ❌ No | ❌ No | CLI (if enabled) |
@@| **USB OTA Firmware** | USB update method | ❌ No USB | ✅ Yes (USB CDC) | ⚠️ Optional | ✅ Yes (USB CDC) | ❌ No USB | ✅ Yes (USB CDC) | ✅ Yes (USB CDC) | Build system |
@@| **CLI Configuration** | Command-line config | ✅ Full | ✅ Full | ✅ Full | ✅ Full | ✅ Full | ✅ Full | ✅ Full | CLI tab in BFC |
@@| **Sensor Calibration** | Gyro/Mag calibration | ✅ Automatic | ✅ Automatic | ✅ Automatic | ✅ Automatic | ✅ Automatic | ✅ Automatic | ✅ Automatic | GUI |
@@| **Dynamic Filter** | Adaptive gyro LPF | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | GUI |
@@| **Gyro LPF Presets** | Filter tuning | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | GUI |
@@| **PID Tuning** | Flight control gains | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | GUI |
@@| **RC Rates & Expo** | Input response | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | GUI |
@@| **Failsafe Config** | Loss-of-signal behavior | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | GUI |
@@| **Arming Conditions** | Safety interlocks | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes | GUI |
@@

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
@@
@@---
@@
@@## Sensor Wiring Quick-Reference Guide
@@
@@### Standard I2C Sensor Wiring (All Boards)
@@
@@**MPU6050/MPU6500 + HMC5883L Gyro/Accel/Mag I2C Setup** (most common):
@@```
@@Gyroscope (MPU6050/6500)  Magnetometer (HMC5883L)   Flight Controller
@@├─ VCC ─────────────────┬──────────── VCC ────────────→ 3.3V
@@├─ GND ─────────────────┴──────────── GND ────────────→ GND
@@├─ SCL ─────────────────────────────→ SCL ────────────→ Board I2C SCL pin
@@├─ SDA ─────────────────────────────→ SDA ────────────→ Board I2C SDA pin
@@├─ INT ──────────────────────────────────────────────→ (Optional: GPIO interrupt)
@@└─ AD0 ───────────────────────────────────────────→ GND (selects 0x68 address)
@@                                                       (leave floating or high for 0x69)
@@```
@@
@@**Barometer (BMP280) Addition**:
@@```
@@BMP280        Flight Controller
@@├─ VCC ────→ 3.3V
@@├─ GND ────→ GND
@@├─ SCL ────→ Board I2C SCL pin (same as gyro)
@@├─ SDA ────→ Board I2C SDA pin (same as gyro)
@@└─ SDO ────→ GND (selects 0x76) or VCC (selects 0x77)
@@```
@@
@@### SPI Sensor Wiring (ESP32, ESP32-S3, RP2040, RP2350)
@@
@@**MPU6500 (SPI) + BMP280 (SPI) on ESP32**:
@@```
@@Gyroscope (MPU6500 SPI)  Barometer (BMP280 SPI)   Flight Controller
@@├─ VCC ───────────────┬──────────── VCC ────────────→ 3.3V
@@├─ GND ───────────────┴──────────── GND ────────────→ GND
@@├─ SCL ───────────────────────────→ SCK ────────────→ GPIO18 (shared)
@@├─ SDA ───────────────────────────→ MOSI ──────────→ GPIO23 (shared)
@@├─ AD0 ─────────────────────────────────────────→ GND
@@├─ CS ────────────────────────────────────────────→ GPIO5 (Gyro select)
@@│
@@└─ MISO ────────────────────────────────────────→ GPIO19 (shared)
@@                                     │
@@                                     ├─ CS ─────→ GPIO13 (Baro select)
@@                                     └─ SDO ────→ GND (default 0x76)
@@```
@@
@@**Notes on SPI**:
@@- All sensors on same bus share SCK, MOSI, MISO lines
@@- Each sensor needs its own **CS (Chip Select)** pin
@@- CS goes LOW to activate sensor, HIGH to deselect
@@- Max frequency: 4 MHz on ESP32, 1 MHz on RP2040, 4 MHz on RP2350
@@
@@### Board-Specific Pin Reference for I2C
@@
@@```
@@ESP32:   SCL=GPIO22  SDA=GPIO21
@@ESP32-S2: SCL=GPIO9  SDA=GPIO8
@@ESP32-S3: SCL=GPIO10 SDA=GPIO9
@@ESP32-C3: SCL=GPIO6  SDA=GPIO8
@@ESP8266: SCL=GPIO5 (D1)  SDA=GPIO4 (D2)
@@RP2040:  SCL=GPIO17 SDA=GPIO16
@@RP2350:  SCL=GPIO17 SDA=GPIO16
@@```
@@
@@### GPS Module Wiring
@@
@@**GPS (UART) Example on ESP32**:
@@```
@@GPS Module          Flight Controller
@@├─ VCC ──────────→ 5V (or 3.3V; check module)
@@├─ GND ──────────→ GND
@@├─ TX  ──────────→ GPIO16 (FC RX for GPS data)
@@└─ RX  ──────────→ GPIO17 (FC TX for GPS config)
@@```
@@
@@**Default GPS UART per board**:
@@- **ESP32**: UART2 (TX=GPIO17, RX=GPIO16) - configured as UART2_RX_SERIAL
@@- **ESP32-S2**: UART1 (TX=GPIO17, RX=GPIO16)
@@- **ESP32-S3**: UART1 (TX=GPIO16, RX=GPIO15)
@@- **ESP32-C3**: ❌ No dedicated GPS port; use soft-serial or remapping
@@- **ESP8266**: Soft-serial only (WiFi bridge); GPIO13 as RX
@@- **RP2040**: UART1 (TX=GPIO8, RX=GPIO9)
@@- **RP2350**: UART1 (TX=GPIO8, RX=GPIO9)
@@
@@### Receiver (RC Input) Wiring
@@
@@**SBUS/IBUS/CRSF Serial Receiver Example on ESP32**:
@@```
@@Receiver (TX only, e.g. FrSky X4R-SB)   Flight Controller
@@├─ VCC (5V or 3.3V) ──────────────────→ 5V or 3.3V
@@├─ GND ────────────────────────────────→ GND
@@└─ DATA/TX ────────────────────────────→ GPIO16 (RX_SERIAL on UART2)
@@```
@@
@@**Default RX_SERIAL port per board**:
@@- **ESP32**: UART2 RX=GPIO16 (shares GPS UART TX=GPIO17)
@@- **ESP32-S2**: UART1 RX=GPIO16
@@- **ESP32-S3**: UART2 RX=GPIO17
@@- **ESP32-C3**: ❌ No dedicated RX port; use soft-serial or remapping (limited GPIO)
@@- **ESP8266**: UART0 RX=GPIO3 (shares MSP)
@@- **RP2040**: UART1 RX=GPIO9
@@- **RP2350**: UART1 RX=GPIO9
@@
@@### VTX SmartAudio Wiring
@@
@@**SmartAudio Module (e.g., TBS Unify Pro Nano) on ESP32**:
@@```
@@VTX SmartAudio       Flight Controller
@@├─ +5V ─────────────→ 5V
@@├─ GND ─────────────→ GND
@@└─ SA (control) ────→ GPIO33 (UART1 TX on ESP32)
@@```
@@
@@**Default VTX SmartAudio UART by board**:
@@- **ESP32**: UART1 TX=GPIO33
@@- **ESP32-S2**: UART0 TX=GPIO43
@@- **ESP32-S3**: UART0 TX=GPIO43
@@- **ESP32-C3**: UART0 TX=GPIO21
@@- **ESP8266**: UART1 TX=GPIO2 (D4) — TX-only
@@- **RP2040**: UART0 TX=GPIO0
@@- **RP2350**: UART0 TX=GPIO0
@@
@@### Quick Decision Table: Which Sensor Bus?
@@
@@| Your Setup | Recommendation | Board Compatibility | Notes |
@@|---|---|---|---|
@@| **Basic drone, I2C gyro** | Use I2C | All boards | Simplest wiring; max 2 kHz gyro rate |
@@| **Acro drone, fast gyro needed** | Use SPI | ESP32, ESP32-S2/S3, RP2040/RP2350 | 4 MHz = 4 kHz gyro; better stability |
@@| **Ultra-compact, minimal GPIO** | I2C only | ESP32-C3, ESP8266 | No SPI; must use I2C sensors |
@@| **High-performance FPV** | SPI + DShot + CRSF | Any board with SPI | SPI gyro + DShot ESCs + CRSF RX = optimal responsiveness |
@@| **Budget build** | I2C + PWM ESCs | ESP8266 | Works well for casual flying; lower update rates acceptable |
@@
@@### Troubleshooting I2C Sensor Not Found
@@
@@If Betaflight Configurator shows "No Gyro Detected":
@@
@@1. **Check wiring**: Verify SCL/SDA pins match your board (see pin reference above)
@@2. **Check power**: Sensor should be **3.3V only** (not 5V); verify with multimeter
@@3. **Check I2C addresses**: Use CLI command `i2cdetect` (if available) or connect to BFC Sensors tab
@@4. **Check pull-ups**: I2C lines need 4.7kΩ pull-up resistors to 3.3V (usually on gyro breakboard)
@@5. **Check AD0 pin**: If using MPU6050 variant, AD0 should be grounded for address 0x68 (default)
@@6. **Check sensor orientation**: Gyro must be physically oriented correctly; see board alignment in BFC
@@
@@### Troubleshooting SPI Sensor Not Found
@@
@@1. **Check CS pin**: Each sensor needs its own CS pin; verify mapping in firmware
@@2. **Check SPI frequency**: Reduce to 1 MHz if using long wires or low-quality breadboards
@@3. **Check SPI mode**: Firmware uses **Mode 3** (CPOL=1, CPHA=1) — double-check sensor datasheet
@@4. **Check power and GND**: Both power and GND must be connected; use **short wires < 5cm**
@@
@@## Sensor Pin Reference and I2C Address Map
@@
@@### Default I2C Addresses (7-bit format)
@@
@@| Sensor Type | Typical Address | Variants | Notes |
@@|---|---|---|---|
@@| **Gyroscope + Accel** | — | — | — |
@@| MPU6050 / MPU6500 / ICM20602 | `0x68` | `0x69` (if AD0 pin high) | Most common gyro; dual-address support |
@@| MPU9250 | `0x68` | `0x69` | Includes onboard magnetometer |
@@| BMI160 | `0x68` | `0x69` | Bosch 6-axis IMU |
@@| LSM6DSO | `0x6A` | `0x6B` | STMicroelectronics 6-axis |
@@| ITG3205 | `0x68` | — | Legacy gyro-only |
@@| **Magnetometer** | — | — | — |
@@| HMC5883L | `0x1E` | — | Popular external mag; I2C |
@@| AK8963 (inside MPU9250) | `0x0C` | — | Integrated in MPU9250 only |
@@| QMC5883L/QMC5883P | `0x0D` | — | Budget alternative |
@@| **Barometer** | — | — | — |
@@| BMP085 | `0x77` | — | Legacy barometer |
@@| BMP280 | `0x76` | `0x77` (if SDO pin high) | Very common; dual-address |
@@| SPL06 | `0x76` | `0x77` | Compact barometer |
@@| MS5611 | `0x76` | `0x77` | High-precision barometer |
@@
@@### I2C/SPI Bus Pin Map by Board
@@
@@| Board | I2C (SCL / SDA) | SPI (SCK / MOSI / MISO) | Gyro CS | Baro CS | Notes |
@@|---|---|---|---|---|---|
@@| **ESP32** | GPIO22 / GPIO21 | GPIO18 / GPIO23 / GPIO19 | GPIO5 | GPIO13 | 4 MHz SPI; I2C 400 kHz |
@@| **ESP32-S2** | GPIO9 / GPIO8 | GPIO12 / GPIO11 / GPIO13 | GPIO10 | GPIO7 | 2 MHz SPI; I2C 400 kHz |
@@| **ESP32-S3** | GPIO10 / GPIO9 | GPIO12 / GPIO11 / GPIO13 | GPIO8 | GPIO7 | 4 MHz SPI; I2C 400 kHz |
@@| **ESP32-C3** | GPIO6 / GPIO8 | ❌ Not available | N/A | N/A | I2C only; no SPI |
@@| **ESP8266** | GPIO5 (D1) / GPIO4 (D2) | ❌ Not configured | N/A | N/A | I2C only; 400 kHz |
@@| **RP2040** | GPIO17 / GPIO16 | GPIO14 / GPIO15 / GPIO12 | GPIO13 | GPIO11 | 1 MHz SPI; I2C 400 kHz |
@@| **RP2350** | GPIO17 / GPIO16 | GPIO14 / GPIO15 / GPIO12 | GPIO13 | GPIO11 | 4 MHz SPI; I2C 400 kHz (faster) |
@@
@@### I2C Sensor Auto-Discovery Sequence
@@
@@When I2C is used, the firmware scans for sensors in this order (first match wins):
@@
@@1. **Gyro/Accel I2C Address 0x68** (MPU6050, MPU6500, ICM20602, etc.)
@@2. **Gyro/Accel I2C Address 0x69** (AD0 pin high variant)
@@3. **Magnetometer 0x1E** (HMC5883L)
@@4. **Magnetometer 0x0D** (QMC5883L)
@@5. **Magnetometer 0x0C** (AK8963; requires MPU9250 gyro)
@@6. **Barometer 0x76** (BMP280, SPL06, MS5611 default)
@@7. **Barometer 0x77** (BMP280, SPL06 alternative; BMP085)
@@
@@### SPI Sensor Configuration
@@
@@**SPI Protocol**: Mode 3 (CPOL=1, CPHA=1), 8-bit frames.  
@@**Typical setup** (gyro + baro on same SPI bus):
@@- Gyro CS pin → Board-specific (see table)
@@- Baro CS pin → Board-specific (see table)
@@- SCK, MOSI, MISO → All sensors share same lines
@@
@@**Example wiring (ESP32)**:
@@```
@@  Gyro (MPU6500 SPI)       Barometer (BMP280 SPI)
@@  ├─ CS:   GPIO5            ├─ CS:   GPIO13
@@  ├─ SCK:  GPIO18           ├─ SCK:  GPIO18 (shared)
@@  ├─ MOSI: GPIO23           ├─ MOSI: GPIO23 (shared)
@@  └─ MISO: GPIO19           └─ MISO: GPIO19 (shared)
@@```
@@
@@### GPIO Availability After Default Config
@@
@@| Board | Total GPIO | Default Assigned | Available | Expansion Notes |
@@|---|---:|---:|---|---|
@@| **ESP32** | 34 | 24 | ~10 | Good; can add rangefinder, optical flow, extra UART |
@@| **ESP32-S2** | 43 | 22 | ~21 | Good; some strapping pins reserved |
@@| **ESP32-S3** | 48 | 24 | ~24 | Excellent; very flexible for custom sensors |
@@| **ESP32-C3** | 22 | 18 | ~4 | Very tight; minimal expansion possible |
@@| **ESP8266** | 11 | 11 | ~0 | **Fully saturated**; no spare pins (use pin remapping) |
@@| **RP2040** | 30 | 18 | ~12 | Good; USB alternative for config |
@@| **RP2350** | 30 | 18 | ~12 | Good; extra RAM/flash helps with custom code |
@@
@@

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
