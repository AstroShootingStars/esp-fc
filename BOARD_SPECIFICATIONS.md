# ESP-FC Board Specifications & Configuration Reference

**Source:** Extracted from `platformio.ini`, `Target/*.h` headers, `Hardware.cpp`, and `docs/boards.md`

**Last Updated:** 2026-06-23

---

## Table of Contents
1. [Supported Boards Overview](#supported-boards-overview)
2. [Board-by-Board Specifications](#board-by-board-specifications)
3. [Sensor Defaults & I2C Addresses](#sensor-defaults--i2c-addresses)
4. [Communication Interfaces](#communication-interfaces)
5. [Motor Output Pins](#motor-output-pins)
6. [Serial Ports](#serial-ports)

---

## Supported Boards Overview

| Board | MCU/Chip | Flash | RAM | PSRAM | USB | WiFi | BLE | Core(s) |
|---|---|---:|---:|---:|:---:|:---:|:---:|---:|
| **ESP32** | ESP32 | 4 MB | 520 KB | ✅ | No | ✅ | ✅ | 2 |
| **ESP32-S2** | ESP32-S2 | 4 MB | 320 KB | ✗ | CDC | ✅ | ✗ | 1 |
| **ESP32-S3** (all) | ESP32-S3 | 4 MB | 512 KB | Variant | ✅ USB-C | ✅ | ✅ | 2 |
| **ESP32-C3** | ESP32-C3 | 4 MB | 400 KB | ✗ | CDC | ✅ | ✅ | 1 |
| **ESP8266** | ESP8266 | 4 MB | 160 KB | ✗ | No | ✅ | ✗ | 1 |
| **RP2040** | ARM Cortex-M0+ | 2 MB | 264 KB | ✗ | ✅ | ✗ | ✗ | 2 |
| **RP2350** | ARM Cortex-M33 | 4 MB | 512 KB | ✗ | ✅ | ✗ | ✗ | 2 |

---

## Board-by-Board Specifications

### ESP32 (Classic)

**PlatformIO Environment:** `esp32`  
**Board:** `lolin32`  
**MCU:** ESP32 (dual-core Xtensa @ 240 MHz)

#### Memory & Storage
- **Flash:** 4 MB (configured QIO @ 80 MHz)
- **SRAM:** 520 KB
- **PSRAM:** ✅ Available
- **Partition:** Custom 4MB partition table (`partitions_4M_nota.csv`)

#### Serial Ports (UART)
| Port | Function | TX | RX | Baud | Notes |
|---:|---|---:|---:|---|---|
| **UART0** | MSP (default) | GPIO1 | GPIO3 | 115200 | Main CLI/MSP port |
| **UART1** | SmartAudio (VTX) | GPIO33 | GPIO32 | 115200 | Optional VTX control |
| **UART2** | RX_SERIAL | GPIO17 | GPIO16 | 115200 | Receiver input (SBUS/CRSF) |
| **Soft-Serial** | WiFi Bridge MSP | Virtual | Virtual | — | WiFi-to-UART bridge enabled |
| **USB CDC** | Not available | — | — | — | — |

#### Communication Buses

**SPI Bus 1 (Gyro/Baro)**
```
SCK:  GPIO18
MOSI: GPIO23
MISO: GPIO19
CS0:  GPIO5   (Gyroscope)
CS1:  GPIO13  (Barometer)
```

**I2C Bus 0 (Software-based)**
```
SCL: GPIO22
SDA: GPIO21
Speed: 400 kHz (default)
```

#### Motor Outputs (4 base + 4 configurable extra)
```
Motor 1: GPIO27
Motor 2: GPIO25
Motor 3: GPIO4
Motor 4: GPIO12
Outputs 5-8: Configurable (default -1)
```

#### Sensor Support

**Gyroscope / Accelerometer** (dual bus: SPI/I2C)
- **SPI:** MPU9250, MPU6500, ICM20602, BMI160, LSM6DSO
- **I2C:** All above + MPU6050, ITG3205
- **Default I2C Address:** 0x68 (AD0 low) or 0x69 (AD0 high)
- **Max I2C Rate:** 2000 Hz
- **Max SPI Rate:** 4000 Hz

**Magnetometer** (I2C only)
- **HMC5883L:** 0x1E (default if detected)
- **AK8963:** 0x0C or 0x0D
- **QMC5883L/QMC5883P:** 0x0D

**Barometer** (SPI/I2C)
- **BMP085:** 0x77
- **BMP280:** 0x76 or 0x77
- **SPL06:** 0x76 or 0x77 (chip ID 0x10)
- **MS5611:** 0x76 or 0x77

#### Analog Inputs (ADC)
```
ADC Channel 0: GPIO36 (VP) — Voltage monitoring
ADC Channel 1: GPIO39 (VN) — Current monitoring
ADC Scale: 3.3V / 4096 counts
```

#### Receiver Input
- **PPM:** GPIO35 (analog input)
- **Serial RX:** GPIO16 (UART2 RX)

#### Audio/LED/Status
```
Buzzer:  GPIO26
LED:     GPIO2
Button:  Not configured (-1)
```

#### Features Enabled
```
✅ RX_SERIAL | Motor Stop | Dynamic Filter
✅ WiFi OTA (via soft-serial bridge)
✅ Bluetooth OTA (if ESPFC_BT_OTA flag set)
✅ ESP-NOW wireless
✅ DShot telemetry
```

#### Build Flags
```
-DESPFC_LED_PIN=2
-DESPFC_LED_TYPE=1
-DESPFC_LED_FORCE_DEFAULT=1
```

---

### ESP32-S2

**PlatformIO Environment:** `esp32s2`  
**Board:** `esp32-s2-saola-1`  
**MCU:** ESP32-S2 (single-core Xtensa @ 240 MHz)

#### Memory & Storage
- **Flash:** 4 MB (QIO @ 80 MHz)
- **SRAM:** 320 KB (tight budget, no PSRAM)
- **PSRAM:** ✗ Not available
- **RAM Constraints:** Soft-serial WiFi disabled to preserve headroom

#### Serial Ports (UART)
| Port | Function | TX | RX | Baud | Notes |
|---:|---|---:|---:|---|---|
| **UART0** | SmartAudio (VTX) | GPIO43 | GPIO44 | 115200 | Default VTX control |
| **UART1** | RX_SERIAL | GPIO17 | GPIO16 | 115200 | Receiver input |
| **USB CDC** | MSP (default) | — | — | — | Primary config port |
| **Soft-Serial** | ✗ Disabled | — | — | — | Disabled due to RAM constraints |

#### Communication Buses

**SPI Bus 0**
```
SCK:  GPIO12
MOSI: GPIO11
MISO: GPIO13
CS0:  GPIO10  (Gyroscope)
CS1:  GPIO7   (Barometer)
```

**I2C Bus 0 (Software-based)**
```
SCL: GPIO9
SDA: GPIO8
Speed: 400 kHz (default)
```

#### Motor Outputs (4 fixed)
```
Motor 1: GPIO39
Motor 2: GPIO40
Motor 3: GPIO41
Motor 4: GPIO42
```

#### Sensor Support

**Gyroscope / Accelerometer**
- **Max I2C Rate:** 2000 Hz
- **Max SPI Rate:** 2000 Hz (lower than ESP32)
- Same sensor support as ESP32

**Barometer, Magnetometer, ADC**
- Same as ESP32

#### Analog Inputs (ADC)
```
ADC Channel 0: GPIO1
ADC Channel 1: GPIO4
ADC Scale: 3.3V / 4096 counts
```

#### Audio/LED/Status
```
Buzzer:  GPIO5
LED:     GPIO15
Button:  Not configured (-1)
```

#### Features Enabled
```
✅ RX_SERIAL | Motor Stop | Dynamic Filter
❌ WiFi OTA (no soft-serial bridge)
❌ Bluetooth (no BLE on S2)
✅ DShot telemetry
```

#### Special Notes
- **⚠️ RAM Tight:** Use USB for configuration; WiFi alternative not available
- Single core: No multi-core optimizations
- Optimal for weight/power-sensitive applications

---

### ESP32-S3

**PlatformIO Environments:** `esp32s3`, `esp32s3_devkitc`, `esp32s3_wroom`, `esp32s3_n8r8`, `esp32s3_n8r16`  
**Boards:** `lolin_s3_mini`, `esp32-s3-devkitc-1`, `freenove_esp32_s3_wroom`, `esp32s3box`  
**MCU:** ESP32-S3 (dual-core Xtensa @ 240 MHz)

#### Memory & Storage
- **Flash:** 4 MB (QIO @ 80 MHz)
- **SRAM:** 512 KB
- **PSRAM:** Variant-dependent
  - **lolin_s3_mini:** 0 MB (no external PSRAM)
  - **esp32-s3-devkitc-1:** 8 MB
  - **freenove_esp32_s3_wroom:** 8 MB
  - **esp32s3box (N8R8):** 8 MB
  - **esp32s3box (N8R16):** 16 MB

#### Serial Ports (UART)
| Port | Function | TX | RX | Baud | Notes |
|---:|---|---:|---:|---|---|
| **UART0** | SmartAudio (VTX) | GPIO43 | GPIO44 | 115200 | Default VTX via hardserial |
| **UART1** | GPS | GPIO16 | GPIO15 | 115200 | Optional GPS UART |
| **UART2** | RX_SERIAL | GPIO18 | GPIO17 | 115200 | Receiver input |
| **USB CDC** | MSP (default) | — | — | — | Native USB, primary config |
| **Soft-Serial** | WiFi Bridge | Virtual | Virtual | — | Enabled, WiFi-to-MSP |

#### Communication Buses

**SPI Bus 1**
```
SCK:  GPIO12
MOSI: GPIO11
MISO: GPIO13
CS0:  GPIO8   (Gyroscope)
CS1:  GPIO7   (Barometer)
```

**I2C Bus 0 (Software-based, flexible pins)**
```
SCL: GPIO10 (or GPIO21-22 alternate on some boards)
SDA: GPIO9  (or GPIO21-22 alternate on some boards)
Speed: 400 kHz (default)
Note: S3 supports probing multiple pin pairs if initial scan fails
```

#### Motor Outputs (4 fixed)
```
Motor 1: GPIO39
Motor 2: GPIO40
Motor 3: GPIO41
Motor 4: GPIO42
```

#### Sensor Support

**Gyroscope / Accelerometer**
- **Max I2C Rate:** 2000 Hz
- **Max SPI Rate:** 4000 Hz
- Same device support as ESP32
- **Auto-detect alternative I2C pin pairs:** GPIO9/10, GPIO21/22 on probe failure

**Barometer, Magnetometer**
- Identical to ESP32

#### Analog Inputs (ADC)
```
ADC Channel 0: GPIO1  (Voltage)
ADC Channel 1: GPIO4  (Current)
ADC Scale: 3.3V / 4096 counts
```

#### Audio/LED/Status
```
Buzzer:  GPIO5
LED:     GPIO47 (RGB WS2812 recommended)
Button:  Not configured (-1)
```

#### Features Enabled
```
✅ RX_SERIAL | Motor Stop | Dynamic Filter
✅ WiFi OTA (via soft-serial bridge)
✅ Bluetooth OTA (if ESPFC_BT_OTA flag set)
✅ USB CDC Native
✅ DShot telemetry
```

#### Build Flags
```
-DESP32S3
-DARDUINO_USB_MODE=1
-DARDUINO_USB_CDC_ON_BOOT=1
-DESPFC_LED_PIN=47
-DESPFC_LED_TYPE=1  (WS2812 addressable)
-DESPFC_LED_FORCE_DEFAULT=1
```

#### Variant Pin Changes
- **Reserved pins to avoid:** 0, 3, 45, 46 (strapping)
- **Flash/PSRAM reserved:** 26-37
- **USB/JTAG:** 19, 20

---

### ESP32-C3

**PlatformIO Environment:** `esp32c3`  
**Board:** `esp32-c3-devkitm-1`  
**MCU:** ESP32-C3 (single-core RISC-V @ 160 MHz)

#### Memory & Storage
- **Flash:** 4 MB (QIO @ 80 MHz)
- **SRAM:** 400 KB
- **PSRAM:** ✗ Not available
- **CPU Freq:** 160 MHz (lower than ESP32 variants)

#### Serial Ports (UART)
| Port | Function | TX | RX | Baud | Notes |
|---:|---|---:|---:|---|---|
| **UART0** | SmartAudio (VTX) | GPIO21 | GPIO20 | 115200 | Default SmartAudio |
| **UART1** | Not configured | — | — | — | TX-only or disabled |
| **USB CDC** | MSP (default) | — | — | — | Native USB, primary config |
| **Soft-Serial** | WiFi Bridge | Virtual | Virtual | — | Enabled |

#### Communication Buses

**SPI Bus 1**
```
SCK:  GPIO12
MOSI: GPIO11
MISO: GPIO13
CS0:  GPIO8   (Gyroscope)
CS1:  GPIO7   (Barometer)
```

**I2C Bus 0 (Software-based)**
```
SCL: GPIO6
SDA: GPIO8
Speed: 400 kHz (default)
```

#### Motor Outputs (4 fixed)
```
Motor 1: GPIO2
Motor 2: GPIO3
Motor 3: GPIO4
Motor 4: GPIO5
```

#### Sensor Support

**Gyroscope / Accelerometer**
- **Max I2C Rate:** 1000 Hz (lower than ESP32/S3)
- **Max SPI Rate:** 2000 Hz
- Same device support as ESP32

**Barometer, Magnetometer**
- Same support

#### Analog Inputs (ADC)
```
ADC Channel 0: GPIO0
ADC Channel 1: GPIO1
ADC Scale: 3.3V / 4096 counts
```

#### Audio/LED/Status
```
Buzzer:  Not configured (-1)
LED:     Not configured (-1)
Button:  Not configured (-1)
```

#### Features Enabled
```
✅ RX_SERIAL | Motor Stop | Dynamic Filter
✅ WiFi OTA (via soft-serial bridge)
✅ Bluetooth OTA support (if flag set)
✅ USB CDC Native
✅ DShot telemetry
```

#### Build Flags
```
-DESP32C3
-DARDUINO_USB_MODE=1
-DARDUINO_USB_CDC_ON_BOOT=1
```

#### Special Notes
- Single-core RISC-V processor (not Xtensa like other ESP32)
- Lower clock speed (160 MHz) — good for power, less performance
- Minimal GPIO count (22 pins total)

---

### ESP8266

**PlatformIO Environment:** `esp8266`  
**Board:** `d1_mini`  
**MCU:** ESP8266 (single-core @ 160 MHz)

#### Memory & Storage
- **Flash:** 4 MB (QIO @ 80 MHz)
- **SRAM:** 160 KB (very tight)
- **PSRAM:** ✗ Not available

#### Serial Ports (UART)
| Port | Function | TX | RX | Baud | Notes |
|---:|---|---:|---:|---|---|
| **UART0** | MSP (default) | GPIO1 | GPIO3 | 115200 | Main CLI/MSP |
| **UART1** | SmartAudio (VTX) | GPIO2 | -1 | 115200 | TX-only suitable for VTX |
| **Soft-Serial** | WiFi Bridge | Virtual | Virtual | — | WiFi-to-MSP |
| **USB CDC** | ✗ Not available | — | — | — | — |

#### Communication Buses

**No SPI0 configured** (pins reserved for flash)

**I2C Bus 0 (Software-based)**
```
SCL: GPIO5 (D1)
SDA: GPIO4 (D2)
Speed: 400 kHz (default)
```

#### Motor Outputs (4 fixed, software PWM)
```
Motor 1: GPIO16 (D0)
Motor 2: GPIO14 (D5)
Motor 3: GPIO12 (D6)
Motor 4: GPIO15 (D8)
PWM Type: Software-based
Max Frequency: ~400 Hz (limited)
```

#### Sensor Support

**Gyroscope / Accelerometer** (I2C only)
- **Max I2C Rate:** 1000 Hz
- **Max SPI Rate:** 1000 Hz (soft SPI only)
- Supported: MPU6050, MPU6500, MPU9250, ICM20602, BMI160, ITG3205, LSM6DSO

**Barometer, Magnetometer**
- Same I2C addresses as ESP32

#### Analog Inputs (ADC)
```
ADC Channel 0: GPIO17 (A0)
Single 10-bit ADC input
ADC Scale: 1.0 / 1024 counts
Note: Wemos D1 Mini has 220k:100k divider for 0–3.3V
```

#### Audio/LED/Status
```
Buzzer:  Not configured (-1)
LED:     Not configured (-1)
Button:  Not configured (-1)
```

#### Features Enabled
```
✅ RX_PPM | Motor Stop | Dynamic Filter | GPS | Telemetry
✅ WiFi OTA (alt mode)
✅ ESP-NOW wireless
❌ DShot telemetry (limited PWM)
⚠️  Software PWM only (limited frequency)
```

#### Special Notes
- **⚠️ RAM Extreme Constraint:** 160 KB SRAM only
- **⚠️ Limited PWM:** Software-based, max ~400 Hz (not suitable for fast ESCs)
- Soft-serial WiFi available
- No integrated SPI (flash uses SPI0)
- Best for light builds or testing only

#### Build Flags
```
-DESP8266
-DESP_WIFI_ALT
```

---

### RP2040

**PlatformIO Environment:** `rp2040`  
**Board:** `pico`  
**MCU:** ARM Cortex-M0+ dual-core @ 125 MHz

#### Memory & Storage
- **Flash:** 2 MB
- **SRAM:** 264 KB
- **PSRAM:** ✗ Not available

#### Serial Ports (UART)
| Port | Function | TX | RX | Baud | Notes |
|---:|---|---:|---:|---|---|
| **UART1** | SmartAudio (VTX) | GPIO0 | GPIO1 | 115200 | First hardware UART |
| **UART2** | RX_SERIAL | GPIO8 | GPIO9 | 115200 | Second hardware UART |
| **USB** | MSP (default) | — | — | — | Native USB, primary config |

#### Communication Buses

**SPI Bus 1**
```
SCK:  GPIO14
MOSI: GPIO15
MISO: GPIO12
CS0:  GPIO13  (Gyroscope)
CS1:  GPIO11  (Barometer)
```

**I2C Bus 0**
```
SCL: GPIO17
SDA: GPIO16
Speed: 400 kHz (default)
```

#### Motor Outputs (4 base + 4 configurable)
```
Motor 1: GPIO2
Motor 2: GPIO3
Motor 3: GPIO4
Motor 4: GPIO5
Outputs 5-8: Configurable (default -1)
PWM: Hardware-based
```

#### Sensor Support

**Gyroscope / Accelerometer**
- **Max I2C Rate:** 1000 Hz
- **Max SPI Rate:** 1000 Hz (standard RP2040)
- Same device support as ESP32

**Barometer, Magnetometer**
- Same as other boards

#### Analog Inputs (ADC)
```
ADC Channel 0: GPIO26
ADC Channel 1: GPIO27
ADC Scale: 3.3V / 4096 counts
```

#### Audio/LED/Status
```
Buzzer:  Not configured (-1)
LED:     Not configured (-1)
Button:  Not configured (-1)
```

#### Features Enabled
```
✅ RX_SERIAL | Motor Stop | GPS | Telemetry | Dynamic Filter
✅ Multi-core enabled
✅ USB CDC
```

#### Build Flags
```
-DARCH_RP2040
-DIRAM_ATTR=""
```

#### Special Notes
- Dual M0+ cores (slower than ARM M4 but adequate)
- Good for hobbyist/DIY builds
- Lower power consumption than ESP32

---

### RP2350

**PlatformIO Environment:** `rp2350`  
**Board:** `rpipico2`  
**MCU:** ARM Cortex-M33 dual-core @ 150 MHz

#### Memory & Storage
- **Flash:** 4 MB (doubled from RP2040)
- **SRAM:** 512 KB (doubled from RP2040)
- **PSRAM:** ✗ Not available

#### Serial Ports (UART)
- Same as RP2040

#### Communication Buses
- Same as RP2040

#### Motor Outputs (4 base + 4 configurable)
- Same as RP2040

#### Sensor Support

**Gyroscope / Accelerometer**
- **Max I2C Rate:** 1000 Hz
- **Max SPI Rate:** 4000 Hz (**doubled from RP2040!**)
- Same device support as RP2040

#### Features & Performance
- **M33 core** vs M0+ (much better performance)
- **4x the flash** and **2x the RAM** of RP2040
- Better suited for complex flight control tasks
- Hardware PWM same as RP2040

#### Build Flags
```
-DARCH_RP2040
-DARCH_RP2350
-DIRAM_ATTR=""
```

#### Key Advantage
- **RP2350 is the recommended RP2 variant** for ESP-FC due to:
  - 4000 Hz SPI gyro rate (vs 1000 Hz on RP2040)
  - M33 ARM core (better performance per cycle)
  - More flash/RAM for future features
  - Better value proposition

---

## Sensor Defaults & I2C Addresses

### I2C Auto-Detection Sequence

The firmware probes the following I2C addresses in order at startup:

```c
// From Hardware.cpp auto-detection
0x68, 0x69  → Gyroscope/Accel (MPU-family default addresses)
0x1E        → HMC5883L Magnetometer
0x0C, 0x0D  → AK8963, QMC5883L Magnetometer
0x76, 0x77  → Barometer (BMP085, BMP280, SPL06)
```

### Gyroscope/Accelerometer

| Device | Type | I2C Addr | SPI Ready | Notes |
|---|---|---:|:---:|---|
| **MPU9250** | Gyro+Accel+Mag | 0x68/0x69 | ✅ | Most common, integrated 3-axis mag |
| **MPU6500** | Gyro+Accel | 0x68/0x69 | ✅ | Excellent performance |
| **MPU6050** | Gyro+Accel | 0x68/0x69 | ❌ | Classic module, I2C-only |
| **ICM20602** | Gyro+Accel | 0x68/0x69 | ✅ | High performance |
| **LSM6DSO** | Gyro+Accel | 0x6B/0x6C | ✅ | ST Micro, good alternative |
| **BMI160** | Gyro+Accel | 0x68/0x69 | ✅ | High resolution |
| **ITG3205** | Gyro only | 0x68/0x69 | ❌ | Older, I2C-only (from GY-85) |

### Magnetometer

| Device | I2C Address | Notes |
|---|---:|---|
| **HMC5883L** | 0x1E | Most common compass |
| **AK8963** | 0x0C, 0x0D (configurable) | Integrated in MPU9250 |
| **QMC5883L** | 0x0D | Budget alternative |
| **QMC5883P** | 0x0D | Variant of QMC5883L |

### Barometer

| Device | I2C Address | SPI Ready | Notes |
|---|---:|:---:|---|
| **BMP085** | 0x77 | ✅ | Classic, 5-point calibration |
| **BMP280** | 0x76, 0x77 | ✅ | Modern standard |
| **SPL06** | 0x76, 0x77 (ID: 0x10) | ✅ | Compact, good accuracy |
| **MS5611** | 0x76, 0x77 | ❌ | I2C-only, high precision |

---

## Communication Interfaces

### SPI Characteristics

| Board | Bus | SCK | MOSI | MISO | Gyro CS | Baro CS | Gyro Max | Baro Max |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| **ESP32** | SPI1 (VSPI) | 18 | 23 | 19 | 5 | 13 | 4000 Hz | 4000 Hz |
| **ESP32-S2** | SPI0 (HSPI) | 12 | 11 | 13 | 10 | 7 | 2000 Hz | 2000 Hz |
| **ESP32-S3** | SPI1 (HSPI) | 12 | 11 | 13 | 8 | 7 | 4000 Hz | 4000 Hz |
| **ESP32-C3** | SPI1 (HSPI) | 12 | 11 | 13 | 8 | 7 | 2000 Hz | 2000 Hz |
| **ESP8266** | None | — | — | — | — | — | 1000 Hz | 1000 Hz |
| **RP2040** | SPI1 | 14 | 15 | 12 | 13 | 11 | 1000 Hz | 1000 Hz |
| **RP2350** | SPI1 | 14 | 15 | 12 | 13 | 11 | 4000 Hz | 4000 Hz |

### I2C Characteristics

| Board | SCL | SDA | Type | Default Speed |
|---|---:|---:|---|---:|
| **ESP32** | 22 | 21 | Software | 400 kHz |
| **ESP32-S2** | 9 | 8 | Software | 400 kHz |
| **ESP32-S3** | 10 | 9 | Software | 400 kHz |
| **ESP32-C3** | 6 | 8 | Software | 400 kHz |
| **ESP8266** | 5 | 4 | Software | 400 kHz |
| **RP2040** | 17 | 16 | Hardware | 400 kHz |
| **RP2350** | 17 | 16 | Hardware | 400 kHz |

**Note:** Software I2C on ESP boards allows arbitrary pin assignment. Hardware I2C on RP boards is fixed to specific pin pairs.

---

## Motor Output Pins

### PWM Output Mapping

| Motor | ESP32 | ESP32-S2 | ESP32-S3 | ESP32-C3 | ESP8266 | RP2040 | RP2350 |
|---:|---:|---:|---:|---:|---:|---:|---:|
| **1** | 27 | 39 | 39 | 2 | 16 (D0) | 2 | 2 |
| **2** | 25 | 40 | 40 | 3 | 14 (D5) | 3 | 3 |
| **3** | 4 | 41 | 41 | 4 | 12 (D6) | 4 | 4 |
| **4** | 12 | 42 | 42 | 5 | 15 (D8) | 5 | 5 |
| **5** | Conf. | N/A | N/A | N/A | N/A | Conf. | Conf. |
| **6** | Conf. | N/A | N/A | N/A | N/A | Conf. | Conf. |
| **7** | Conf. | N/A | N/A | N/A | N/A | Conf. | Conf. |
| **8** | Conf. | N/A | N/A | N/A | N/A | Conf. | Conf. |

**Legend:**
- `Conf.` = Configurable (default -1, can map to any available GPIO)
- `N/A` = Not available (board limited to 4 outputs)

### Motor Output Configuration

```bash
# Change motor 1 output pin
set pin_output_0 27

# Disable motor output
set pin_output_7 -1

# Enable additional motor on RP2040
set pin_output_4 10  # Custom GPIO10 for motor 5
```

---

## Serial Ports

### Port Availability by Board

| Board | UART0 | UART1 | UART2 | USB CDC | Soft-Serial |
|---|:---:|:---:|:---:|:---:|:---:|
| **ESP32** | ✅ | ✅ | ✅ | ❌ | ✅ (WiFi) |
| **ESP32-S2** | ✅ | ✅ | ❌ | ✅ | ❌ (RAM) |
| **ESP32-S3** | ✅ | ✅ | ✅ | ✅ | ✅ (WiFi) |
| **ESP32-C3** | ✅ | ❌ | ❌ | ✅ | ✅ (WiFi) |
| **ESP8266** | ✅ | ✅ (TX-only) | ❌ | ❌ | ✅ (WiFi) |
| **RP2040** | ✅ | ✅ | ❌ | ✅ | ❌ |
| **RP2350** | ✅ | ✅ | ❌ | ✅ | ❌ |

### Default Serial Functions

| Board | MSP Port | VTX Port | RX Port | Notes |
|---|---|---|---|---|
| **ESP32** | UART0 | UART1 | UART2 | Full duplex all ports |
| **ESP32-S2** | USB CDC | UART0 | UART1 | Soft-serial disabled (RAM) |
| **ESP32-S3** | USB CDC | UART0 | UART2 | WiFi bridge available |
| **ESP32-C3** | USB CDC | UART0 | Soft-serial | Minimal UARTs |
| **ESP8266** | UART0 | UART1 (TX only) | Soft-serial | Very limited |
| **RP2040** | USB CDC | UART1 | UART2 | Clean hardware UART setup |
| **RP2350** | USB CDC | UART1 | UART2 | Same as RP2040 |

---

## Board Selection Guide

### For Production/Racing
- **Recommended:** ESP32-S3 with PSRAM (best balance: WiFi, dual-core, good perf)
- **Alternative:** RP2350 (stable, proven, good perf, smaller size)

### For Development
- **Recommended:** ESP32 or ESP32-S3 (extensive debugging, WiFi OTA)
- **Alternative:** RP2040 (simpler, fewer surprises)

### For Minimal Builds
- **Recommended:** RP2040 (small, cheap, adequate performance)
- **Avoid:** ESP8266 (too slow PWM, RAM constraints)

### For Experiments
- **Recommended:** ESP32 (full features, good for learning)
- **ESP32-C3:** If RISC-V interest

### For Production Micro Drones
- **Last Resort:** ESP8266 (OK for very light builds, not recommended)

---

## Advanced Configuration Examples

### Enable Extra Motor Outputs (ESP32)

```bash
# Enable outputs 5-8 on custom pins
set pin_output_4 14
set pin_output_5 26
set pin_output_6 33
set pin_output_7 32
```

### Remap Receiver Input

```bash
# Use GPIO32 as new PPM input (ESP32)
set pin_input_rx 32
```

### Remap I2C (ESP32-S3)

```bash
# Use alternate I2C pin pair (GPIO21/22 instead of GPIO9/10)
set pin_i2c_0_sda 21
set pin_i2c_0_scl 22
set i2c_speed 400
```

### SPI Gyro at Different Chip Select

```bash
# Move gyro CS to GPIO10 (ESP32)
set pin_spi_cs_0 10
```

---

## Limitations & Special Cases

### ESP32-S2
- ⚠️ **RAM Tight:** Soft-serial WiFi disabled; use USB for configuration
- ⚠️ **Single Core:** Less capable than dual-core variants
- ✅ **Good For:** Power-efficient setups, weight-sensitive builds

### ESP8266
- ⚠️ **Software PWM:** Limited to ~400 Hz (not suitable for high-speed ESCs)
- ⚠️ **RAM Extreme:** 160 KB only; no room for advanced features
- ✅ **Good For:** Minimal test builds, legacy projects

### RP2040 vs RP2350
- **RP2040:** 1000 Hz max SPI gyro rate, 2 MB flash
- **RP2350:** 4000 Hz max SPI gyro rate, 4 MB flash (recommended)

### ESP32 vs ESP32-S3
- **ESP32:** Classic, battle-tested, good for all uses
- **ESP32-S3:** Newer, USB-native, better single-core perf, PSRAM options

---

## Appendix: Chip ID Functions

All boards implement board ID retrieval for unique identification:

```c
uint32_t getBoardId0()   // First 4 bytes of unique ID
uint32_t getBoardId1()   // Next 4 bytes of unique ID
uint32_t getBoardId2()   // Additional ID (usually 0 on ESP, varies on RP)
```

Used for MAC address generation, unit identification, and telemetry.

---

**End of Board Specifications**

Generated from esp-fc codebase analysis.  
For latest updates, refer to: `lib/Espfc/src/Target/Target*.h` and `platformio.ini`
