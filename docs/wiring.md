# ESP-FC wiring examples and PIN mapping

Most ESP-FC targets allow pin remapping, so wiring is not final and you can remap inputs/outputs to your needs. To change pin function go to the CLI and use `get pin` command to check current assignment. For example, to set first output to pin 1 use command

`set pin_output_0 1`

Tu unmap pin function use -1 as pin number

`set pin_output_3 -1`

> [!NOTE]
> There are still some limitations in remapping. Not all pins can be used with all functions. Some pins are input only, others are strapping pins and may prevent from booting if used incorrectly. please consult MCU documentation for available options. Default layout is proven to work.

## Default I2C pin mapping for gyro modules

| Module Pin | CLI Name         | ESP32 | ESP32-S3 | STM32F7/H7* |
|------------|------------------|------:|---------:|------------:|
| SCK/SCL    | `pin_i2c_scl`    | 22    | 10       | 15          |
| SDA/SDI    | `pin_i2c_sda`    | 21    | 9        | 14          |

`*` STM32 values follow current experimental Nucleo scaffold defaults.

> [!NOTE]
> I2C driver accepts only pins from 1 to 31

## Default SPI pin mapping gyro modules

| Module Pin  | CLI Name         | ESP32 | ESP32-S3 | STM32F7/H7* |
|-------------|------------------|------:|---------:|------------:|
| SCK/SCL     | `pin_spi_0_sck`  | 18    | 12       | 13          |
| SDA/SDI     | `pin_spi_0_mosi` | 23    | 11       | 11          |
| SAO/SDO/ADO | `pin_spi_0_miso` | 19    | 13       | 12          |
| NCS         | `pin_spi_cs_0`   |  5    |  8       | 10          |
| CSB*        | `pin_spi_cs_1`   | 13    |  7       |  4          |

**Note:** `CSB` is required for barometer on 10-DOF MPU-9250 modules

> [!TIP]
> For better performance preffer SPI

## Default Servo/Motor output mapping

| Motor | CLI name | ESP32 | ESP32-S2 | ESP32-S3 | ESP32-C3 | ESP8266 | RP2040/RP2350 | STM32F7/H7* |
|------:|----------|------:|---------:|---------:|---------:|--------:|--------------:|------------:|
| 1 | `pin_output_0` | 27 | 39 | 39 | 2 | 16 (D0) | 2 | 3 |
| 2 | `pin_output_1` | 25 | 40 | 40 | 3 | 14 (D5) | 3 | 5 |
| 3 | `pin_output_2` | 4 | 41 | 41 | 4 | 12 (D6) | 4 | 6 |
| 4 | `pin_output_3` | 12 | 42 | 42 | 5 | 15 (D8) | 5 | 9 |

> [!NOTE]
> ESP32 and RP2040 targets expose `pin_output_4` to `pin_output_7`, but those extra slots default to `-1` (unmapped).

## Motor ESC Wiring

### **Brushless Motor + Electronic Speed Controller (ESC)**

**Motor Type:** Brushless motors (BLDC) with 3-phase commutation  
**Protocols:** DShot (recommended), OneShot, PWM  
**Best For:** Standard multirotors, FPV drones, racing

#### **Basic Wiring Diagram**

```
                    ┌─── Battery ───┐
                    |               |
                   (+)             (-)
                    |               |
    ┌────────────────┴───────────────┴─────────────────┐
    |                                                   |
    V                                                   V
   GND ────────────────────┬─────────────────┬─────────┐
                           |                 |         |
  ┌─────────────┐   ┌──────┴─────────┐  ┌────┴──────┐  |
  │   FC GPIO   │   │    ESC FET     │  │ Gyro/Baro │  |
  │  (Motor 1)  │───│ (Power Stage)  │  │   (SPI)   │  |
  │  GPIO27     │   │                │  │           │  |
  │  PWM Signal │   │ MOT_A ─────────────┤ Motor A   │  |
  └─────────────┘   │ MOT_B ─────────────┤ Motor B   │  |
                    │ MOT_C ─────────────┤ Motor C   │  |
                    │                    │           │  |
                    │ BEC (5V out) ──────┤ FC Power  │  |
                    │                    └───────────┘  |
                    │ GND ───────────────────────┐      |
                    └────────────────────────────┴──────┘
```

#### **Signal Connection (PWM/DShot)**

| Pin | Connection | Notes |
|---|---|---|
| **FC GPIO** | ESC Signal | PWM or DShot digital signal from FC output pin |
| **ESC Signal** | FC GND | Signal reference (must share common ground with FC) |
| **ESC GND** | FC GND | Critical: Common ground return path |

#### **Power Connection**

| Wire | Connection | Notes |
|---|---|---|
| **Battery +** | ESC Power Input | Direct from battery (liters, connector, or soldered) |
| **Battery −** | ESC GND | Direct from battery negative |
| **ESC BEC Output** | FC Power (5V) | ESC provides regulated 5V for FC and sensors |
| **ESC GND** | FC GND | **Critical:** Ensure solid connection for signal integrity |

#### **Motor Connections**

| Wire | Connection | Notes |
|---|---|---|
| **MOT_A, MOT_B, MOT_C** | Motor Phase A/B/C | Three-phase motor wires (any order; if reversed, motor spins backward) |
| **Motor GND** | Not used | BLDC motor has no ground wire |

#### **Key Wiring Rules for Brushless**

1. ✅ **Use thick gauge wire** for battery connections (at least 14AWG for 4S; thicker for higher current)
2. ✅ **Keep signal cable short** and away from high-current motor leads
3. ✅ **Common ground is critical** — Signal reference must be same potential as FC ground
4. ✅ **Solid connections** — Use solder or reliable connectors (XT60, XT30, etc.)
5. ✅ **Strain relief** — Protect wires at connector and motor solder joints
6. ❌ **Never reverse motor phases** more than needed (swap only 2 wires if direction is wrong)
7. ❌ **Never run without ESC** directly connected (no loose signal wires)

#### **DShot Wiring Considerations**

- **Signal voltage:** 3.3V from FC GPIO (ESC accepts 3.3V–5V logic)
- **Cable length:** Keep PWM/DShot signal wire <30 cm to reduce noise
- **Twisted pair:** Optional but recommended for long signal runs
- **Capacitor:** 100µF/16V across ESC power input (optional, helps with voltage sag)

---

### **Brushed Motor + FET Driver**

**Motor Type:** Brushed DC motors (older or micro builds)  
**Protocols:** Direct PWM (Brushed mode in CLI)  
**Best For:** Micro quadcopters, legacy setups, direct drive

#### **Basic Wiring Diagram**

```
                    ┌─── Battery ───┐
                    |               |
                   (+)             (-)
                    |               |
         ┌──────────┘               └─────────────┐
         |                                         |
         V                                         V
    ┌────┴────┐                            ┌──────┴────┐
    │ MOSFET  │                            │   Motor   │
    │ D-Pin   │                            │    (+)    │
    │  (Gate) │                            └──────┬────┘
    │         │                                   |
    │  Gate   │◄─── FC GPIO (PWM)  (e.g., GPIO27)
    │  (G)    │
    │         │◄─── GND (Signal Reference)
    └────┬────┘
         │
         │  (Drain/Source Path)
         │
    ┌────┴────────────────────────────────┬────┐
    |         FET Current Path             |    |
    |  When Gate HIGH: Source→Motor→Drain  |    |
    └────┴────────────────────────────────┴────┘
         |                                 |
         └─────────────┬──────────────────┘
                       |
                  Motor (−) / GND Return
```

#### **Signal Connection**

| Pin | Connection | Notes |
|---|---|---|
| **FC GPIO** | MOSFET Gate | PWM signal (3.3V logic to 4.7kΩ resistor) |
| **4.7kΩ Resistor** | MOSFET Gate & GND | Gate pull-down (limits voltage spikes) |
| **FC GND** | Gate resistor, FET Source | **Critical:** Common ground for PWM logic |

#### **Power Connection**

| Wire | Connection | Notes |
|---|---|---|
| **Battery +** | MOSFET Drain | Direct from battery positive |
| **Battery −** | FET Source / Motor (−) | Return path for motor current |
| **Capacitor (+)** | Battery + (at MOSFET) | 470µF/16V or 1000µF for EMI suppression |
| **Capacitor (−)** | FET Source / GND | Across MOSFET to reduce noise |

#### **Motor Connections**

| Wire | Connection | Notes |
|---|---|---|
| **Motor (+)** | Battery + (before FET) | Direct positive supply |
| **Motor (−)** | FET Source output | Motor returns through FET to battery −  |

#### **MOSFET Selection**

| Parameter | Requirement | Example |
|---|---|---|
| **Type** | N-channel MOSFET | IRF540N, AO3400A, or similar |
| **Drain Current** | Must exceed max motor current | 30A+ for typical micro frames |
| **Gate Voltage** | Logic-level compatible (≤5V) | Rated for 3.3V–5V signals |
| **On-Resistance (Rds_on)** | < 0.1Ω (lower = less heating) | Critical for efficiency |
| **Package** | TO-220 or SO-8 | Thermal management important |

#### **Key Wiring Rules for Brushed**

1. ✅ **Use thick wire** for battery and motor connections (≥18 AWG)
2. ✅ **Add capacitor across motor leads** — 470µF/16V minimum to suppress EMI
3. ✅ **Short FET-to-motor wires** — Minimize parasitic inductance
4. ✅ **Gate pull-down resistor** (4.7kΩ) — Keeps FET OFF when FC GPIO floating
5. ✅ **Common ground** — Battery GND, Motor GND, and FC GND must be connected
6. ✅ **Heatsink MOSFET** if current > 20A (especially at 100% duty cycle)
7. ❌ **Never reverse motor polarity** without discharging the capacitor first
8. ❌ **Never use logic-level incompatible FET** (must accept 3.3V gate signal)

#### **Typical Micro Quad Brushed Setup (ESP32 + 1S Battery)**

```
Battery 3.7V LiPo
    |
    +─ 470µF cap ─┬─ MOSFET Drain (IRF540N)
    |             |
    |             ├─ MOSFET Source ─┬─ Motor (−)
    |             |                  |
    └─────────────┴─ Motor (+) ◄─────┴─ GND Return
                  
               Gate Circuit:
                  |
              4.7kΩ ← (Pull-down)
              Resistor
                  |
                ┌─+─┐
         GPIO27 │   │ (MOSFET Gate)
         (FC)   └─┘
                  |
                FC GND (Common)
```

---

### **Motor Power and Signal Isolation**

#### **Power Distribution Best Practice**

```
                   Main Battery Pack
                        |
                 ┌───────┴───────┐
                 |               |
            Power Main        Power BEC
            (To all ESCs)     (Optional regulators)
                 |               |
            ┌────┴────┐      ┌────┴────┐
            | ESC 1–4 |      | Separate|
            | (Motors)│      |Voltage  │
            └────┬────┘      │Converter│
                 |           │(if needed
                 |           │5V/12V) │
            Motor Power      └────┬───┘
                                  |
                          ┌───────┴────────┐
                          | FC Power       |
                          | Input (5V)     |
                          └────────────────┘
```

#### **Signal Integrity Rules**

| Rule | Why | How |
|---|---|---|
| Common ground essential | Signal reference must match | Solder FC GND to ESC/FET GND; use star point distribution |
| Keep signal wires short | Long wires pick up EMI | Bundle PWM signals separately from power |
| Twisted pair optional | Reduces electromagnetic coupling | Especially for DShot over >30cm |
| Ferrite choke recommended | Suppresses high-frequency noise | Add ferrite ring on motor power leads (especially for brushed) |

---

### **Common Wiring Mistakes**

| Mistake | Symptom | Fix |
|---|---|---|
| **Signal and power wires mixed** | Motors twitch or won't arm | Keep signal wires separate from battery/motor wires |
| **Ground not connected** | Motors don't respond or behave erratically | Solder FC GND to ESC GND (solid connection) |
| **Reversed motor phase** | Motor spins backward | Swap any 2 of 3 motor wires (brushless) or motor leads (brushed) |
| **Missing capacitor (brushed)** | Radio interference or choppy throttle | Add 470µF across motor leads |
| **Gate resistor missing (brushed)** | MOSFET stays partially ON | Add 4.7kΩ from gate to source GND |
| **Weak solder joints** | Intermittent ESC dropouts | Re-solder all connections; use good flux |
| **Thin signal wire** | DShot telemetry failures | Use at least 24AWG; keep runs <30cm |
| **Battery voltage sag** | Brown-out resets under load | Use lower ESR battery; add power capacitor |

---

## Default VTX control wiring

SmartAudio is the default VTX control protocol. Connect the VTX control wire to the TX pin below.

| Board | CLI serial function | TX pin | RX pin | Notes |
|---|---|---:|---:|---|
| ESP32 | `SERIAL_FUNCTION_VTX_SMARTAUDIO` on UART1 | 33 | 32 | RX is unused by SmartAudio but remains mapped |
| ESP32-S2 | `SERIAL_FUNCTION_VTX_SMARTAUDIO` on UART0 | 43 | 44 | USB remains the default Betaflight/MSP port |
| ESP32-S3 | `SERIAL_FUNCTION_VTX_SMARTAUDIO` on UART0 | 43 | 44 | USB remains the default Betaflight/MSP port |
| ESP32-C3 | `SERIAL_FUNCTION_VTX_SMARTAUDIO` on UART0 | 21 | 20 | USB remains the default Betaflight/MSP port |
| ESP8266 | `SERIAL_FUNCTION_VTX_SMARTAUDIO` on UART1 | 2 (D4) | -1 | TX-only default, suitable for SmartAudio |
| RP2040/RP2350 | `SERIAL_FUNCTION_VTX_SMARTAUDIO` on UART0 | 0 | 1 | USB remains the default Betaflight/MSP port |
| STM32F7/H7 | Not assigned by default | -1 | -1 | Experimental single-UART scaffold; assign manually if needed |

## Default Uart/Serial pin mapping

| Uart | CLI name          | ESP32 | ESP32-S3 |
|-----:|-------------------|------:|---------:|
| RX 1 | `pin_serial_0_rx` |  3    | 44       |
| TX 1 | `pin_serial_0_tx` |  1    | 43       |
| RX 2 | `pin_serial_1_rx` | 32    | 15       |
| TX 2 | `pin_serial_1_tx` | 33    | 16       |
| RX 3 | `pin_serial_2_rx` | 16    | 17       |
| TX 3 | `pin_serial_2_tx` | 17    | 18       |

## Power and Battery Sensors (Voltage and Current)

When `vbat_source` or `ibat_source` is set to `ADC`, firmware now auto-selects a safe default ADC pin if the configured pin is missing or conflicts with another mapped feature.

### Supported Sensor Types

| Measurement | Supported sensor type | CLI source |
|-------------|-----------------------|-----------:|
| Battery voltage | Analog voltage divider to ADC | `vbat_source = 1` |
| Battery current | Analog current sensor with voltage output (ACS/INA-style analog OUT) | `ibat_source = 1` |

### Board Default ADC Wiring Ports

| Board | Voltage ADC (`pin_input_adc_0`) | Current ADC (`pin_input_adc_1`) | Notes |
|------:|-------------------------------:|--------------------------------:|-------|
| ESP32 | GPIO36 | GPIO39 | Uses ADC1-safe pins by default |
| ESP32-S2 | GPIO1 | GPIO4 | Default mapping for both sensors |
| ESP32-S3 (all variants) | GPIO1 | GPIO4 | Same mapping for `esp32s3`, `esp32s3_devkitc`, `esp32s3_wroom` |
| ESP32-C3 | GPIO0 | GPIO1 | Both are available ADC defaults |
| ESP8266 | GPIO17 (A0) | Not available by default | Single ADC channel target |
| RP2040 / RP2350 | GPIO26 | GPIO27 | Optional fallback can use GPIO28 |
| STM32F7 / STM32H7 | Not assigned by default | Not assigned by default | ADC pins are available but unset in scaffold defaults |

### Auto-fallback Pin Selection by Target

If a configured ADC pin is invalid/conflicting, firmware picks a non-conflicting fallback from this list:

| Board | Fallback list |
|------:|---------------|
| ESP32 | 36, 39, 34, 35, 32, 33 |
| ESP32-S2 | 1, 4 |
| ESP32-S3 | 1, 4 |
| ESP32-C3 | 0, 1 |
| ESP8266 | 17 |
| RP2040 / RP2350 | 26, 27, 28 |
| STM32F7 / STM32H7 | none (configure manually) |

> [!NOTE]
> On ESP32, fallback selection intentionally stays on ADC1-capable pins to avoid ADC2 conflicts with WiFi.

### Wiring Guidance

1. **Voltage sensor (divider):** Battery `+` -> divider -> ADC voltage pin, battery `-` -> GND.
2. **Current sensor (analog output):** Sensor `OUT` -> ADC current pin, sensor `GND` -> FC GND, sensor power per module spec.
3. Keep ADC signal wires short and use a common ground with the FC.
4. Set and verify in CLI:
	- `set vbat_source 1`
	- `set ibat_source 1` (if board has a usable current ADC pin)
	- `save`

If no safe ADC pin exists for a selected source, firmware automatically disables that source to prevent pin conflicts.

## Default PPM receiver pin mapping

| Uart    | CLI name       | ESP32 | ESP32-S3 |
|--------:|----------------|------:|---------:|
| PPM     | `pin_input_rx` |  35   | 6        |

## Other pin functions

| CLI name            | ESP32 | ESP32-S3 | Comment       |
|---------------------|------:|---------:|---------------|
| `pin_buzzer`        |  0    | 5        | Status buzzer |
| `pin_led`           |  26   | -        | Status led    |

## Example ESP32 SPI MPU-6500/MPU-9250 gyro

![ESP-FC ESP32 SPI Wiring](./images/esp-fc-esp32_spi_wiring.png)

## Example ESP32 I2C MPU-6050 gyro

![ESP-FC ESP32 I2C Wiring](./images/esp-fc-esp32_i2c_wiring.png)

## Example ESP8266 I2C MPU-6050 gyro

ESP8266 has limited ability to remap pins, use `get pin` command to list available options.

![ESP-FC ESP8266 I2C Wiring](./images/espfc_wemos_d1_mini_wiring.png)

---

## Advanced Features: Altitude Hold, Landing Assist & Obstacle Avoidance

### **Lidar Thermal Drift Compensation**

Modern lidar/ToF sensors (VL53L0X, VL53L1X) exhibit temperature-dependent drift: distance measurements shift as sensor temperature changes. Thermal compensation corrects this drift using a temperature coefficient.

#### **Thermal Compensation Methods**

| Method | Sensor | Accuracy | Setup Complexity | Best For |
|---|---|---|---|---|
| **NTC Thermistor (ADC)** | NTC 10kΩ on ADC pin | ±1°C | Simple (1 resistor) | Budget-friendly, accurate |
| **Integrated Barometer Temp** | BMP280/BME280 | ±1°C | Existing I2C | Already deployed |
| **Lidar Internal Sensor** | VL53L0X/VL53L1X register | ±2°C | Built-in | Easiest, lower accuracy |

#### **Board-Specific Temperature Sensor Pins (NTC Thermistor)**

| Board | ADC Pin for NTC | Pin ID | Notes |
|---|---|---|---|
| **ESP32** | GPIO34 (ADC1_6) | 34 | Use with 10k pull-up to 3.3V |
| **ESP32-S2** | GPIO2 (ADC2_1) | 2 | Avoid ADC2 if WiFi active |
| **ESP32-S3** | GPIO2 (ADC1_1) | 2 | Recommended; safe with WiFi |
| **ESP32-C3** | GPIO3 (ADC1_3) | 3 | Only ADC option besides GPIO0/1 |
| **ESP8266** | GPIO17 (A0) | 17 | Single ADC channel |
| **RP2040** | GPIO28 (ADC2) | 28 | Dedicated ADC input |
| **RP2350** | GPIO28 (ADC2) | 28 | Same as RP2040 |

#### **NTC Thermistor Wiring (All Boards)**

```
                    NTC Thermistor (10kΩ)
                    ┌─────────────────┐
        3.3V ───────├─ R (Pull-up)    │
                    │                 │ 
        GPIO(ADC) ◄─├─ Thermistor     │
                    │                 │
        GND ────────├─ GND            │
                    └─────────────────┘
```

**Component values:**
- **Thermistor**: 10kΩ NTC thermistor (B3950, common 10k@25°C)
- **Pull-up resistor**: 10kΩ (matches thermistor for optimal linearity)
- **Capacitor**: 100nF across ADC pin to GND (optional, reduces noise)

#### **CLI Configuration for Thermal Compensation**

**Enable thermal compensation with NTC thermistor:**
```bash
# Set rangefinder index (0=bottom, 1=front)
set rangefinder_temp_compensation_enabled 1    # Enable correction
set rangefinder_temp_sensor_type 1             # 1=NTC_ADC, 2=BMP280_INTERNAL, 3=VL53L0X_INTERNAL

# For NTC: configure NTC thermistor parameters
set rangefinder_temp_coefficient -30           # mm/°C drift rate (negative=shrinks with heat)
set rangefinder_reference_temp 20              # Calibration reference (°C)
set rangefinder_ntc_adc_pin 34                 # GPIO pin for thermistor (board-specific)
set rangefinder_ntc_r0 10000                   # Nominal resistance @ 25°C (Ohms)
set rangefinder_ntc_b_coeff 3950               # B coefficient (Kelvin) - check sensor datasheet
set rangefinder_ntc_vref 3300                  # ADC reference voltage (mV)
set rangefinder_ntc_pullup_r 10000             # Pull-up resistor (Ohms)

save
```

**Enable thermal compensation with barometer temperature:**
```bash
set rangefinder_temp_compensation_enabled 1
set rangefinder_temp_sensor_type 2             # 2=BMP280_INTERNAL (if baro on I2C)
set rangefinder_temp_coefficient -30           # Adjust based on your lidar
set rangefinder_reference_temp 20

save
```

**Enable thermal compensation with lidar internal sensor (simplest):**
```bash
set rangefinder_temp_compensation_enabled 1
set rangefinder_temp_sensor_type 3             # 3=VL53L0X_INTERNAL
set rangefinder_temp_coefficient -30           # Check VL53L0X datasheet (±0.1%/°C typical)
set rangefinder_reference_temp 20

save
```

**Calibrate thermal coefficient (procedure):**
```bash
# 1. Record distance at warm state (e.g., 25°C): D_hot = X mm
# 2. Let sensor cool to cold state (e.g., 10°C): D_cold = Y mm
# 3. Calculate: coeff = (D_cold - D_hot) / (10 - 25) = (D_cold - D_hot) / -15
# 4. Example: D_hot=1000, D_cold=1045 → coeff = (1045-1000)/-15 = -3 mm/°C

set rangefinder_temp_coefficient <calculated_value>
set rangefinder_reference_temp 25
save
```

#### **Thermal Compensation Algorithm**

```
corrected_distance = raw_distance + temp_coefficient × (current_temp - reference_temp)

Example:
  - Raw reading: 1000 mm
  - Current temp: 40°C
  - Reference temp: 20°C
  - Coeff: -30 mm/°C (sensor shrinks 30mm per °C increase)
  - Correction: -30 × (40 - 20) = -600 mm
  - Result: 1000 - 600 = 400 mm adjusted reading
```

---

### **Altitude Hold (Barometer-based)**

Altitude hold uses a barometer to maintain constant altitude automatically.

#### **Mandatory Sensor**
- ✅ **Barometer** (I2C or SPI) — Required for altitude hold; if not detected, the mode is blocked

#### **Optional Sensors**
- ⚠️ **GPS** — Improves altitude fusion and long-range stability
- ⚠️ **Accelerometer** — Helps vertical response, but altitude hold will still run without it

#### **Supported Barometer Modules**

| Module | Interface | I2C Address | Board Support | Notes |
|---|---|---|---|---|
| BMP180 | I2C | 0x77 | ESP32, S2, S3, C3, ESP8266, STM32F7/H7 | Older; 0.5 m accuracy |
| BMP280 | I2C/SPI | 0x76, 0x77 | All boards | Common; 1 m accuracy |
| BME280 | I2C/SPI | 0x76, 0x77 | All boards | BMP280 + humidity; recommended |
| MS5611 | I2C/SPI | 0x77 | All boards | High-precision; 0.1 m accuracy |
| BMP390 | I2C/SPI | 0x77 | All boards | Latest; low power; ±0.5 m |

#### **Barometer Wiring (I2C Example — ESP32)**

```
                    Barometer (BMP280)
                    ┌─────────────────┐
        GPIO22 (SCL)├─ SCL            │
        GPIO21 (SDA)├─ SDA            │
        3.3V ───────├─ VCC            │
        GND ────────├─ GND            │
                    └─────────────────┘

CLI Configuration:
  set gyro_bus I2C       # (usually auto-detected)
  set baro_bus I2C       # Enable barometer I2C
  save
```

#### **Barometer Wiring (SPI Example — ESP32)**

```
                    Barometer (BMP280-SPI)
                    ┌─────────────────┐
        GPIO18 (SCK)├─ SCK            │
        GPIO23 (MOSI)├─ MOSI          │
        GPIO19 (MISO)├─ MISO          │
        GPIO13 (CS) ├─ CS  (via jumper from pin_spi_cs_1)
        3.3V ───────├─ VCC            │
        GND ────────├─ GND            │
                    └─────────────────┘

CLI Configuration:
  set baro_bus SPI       # Enable barometer SPI
  save
```

#### **CLI Commands for Altitude Hold**

**Enable altitude hold:**
```bash
set feature_gps 1       # Enable GPS (optional but recommended)
set baro_bus I2C        # or SPI, depending on wiring
save
```

**Verify barometer detected:**
```bash
status                  # Should show "baro rate: X Hz" (not 0 Hz)
```

**Altitude hold tuning (in Betaflight GUI):**
- Go to **PID Tuning** tab
- Adjust `vario_p`, `vario_i`, `vario_d` (vertical velocity PIDs)
- Use `Receiver` tab to assign RC channel to altitude hold mode

---

### **Landing Assist (Soft Landing)**

Landing assist automatically reduces throttle to zero when drone detects landing (low throttle + high vertical acceleration).

#### **Mandatory Sensor**
- ✅ **Accelerometer** — Required for landing assist; if it is missing the feature stays disabled

#### **Optional Sensors**
- ⚠️ **Barometer** — Adds descent limiting and altitude awareness during touchdown
- ⚠️ **Rangefinder (bottom / landing slot)** — Improves low-altitude landing detection and flare timing
- ⚠️ **GPS** — Helps with fail-safe landing behavior
- ⚠️ **Optical Flow** — Improves landing detection in GPS-denied environments

#### **Supported Rangefinder Modules**

| Module | Type | Range | Interface | Board Support | Notes |
|---|---|---|---|---|---|
| HC-SR04 | Ultrasonic | 2–400 cm | PWM (GPIO) | All | Budget-friendly; 1 Hz update |
| VL53L0X | ToF Laser | 5–200 cm | I2C | ESP32, S2, S3, C3, ESP8266, STM32F7/H7 | Accurate; 50 Hz update |
| TFMINI | ToF Laser | 0.3–12 m | UART/I2C | All | Industrial-grade; 100 Hz |
| JSN-SR04T | Waterproof Sonar | 2–600 cm | PWM/UART | All | Weather-resistant |

#### **HC-SR04 Ultrasonic Rangefinder Wiring**

```
            HC-SR04 Ultrasonic Module
            ┌─────────────────┐
GPIO14 ─────├─ TRIG           │  (Trigger signal to module)
GPIO27 ◄─────├─ ECHO          │  (Echo return from module)
5V ─────────├─ VCC            │  (Power supply 5V)
GND ────────├─ GND            │  (Common ground)
            └─────────────────┘

CLI Configuration:
  set pin_input_rf_bottom 14     # Trigger pin
  set pin_input_rf_bottom_echo 27 # Echo pin
  save
```

#### **VL53L0X Laser ToF Wiring (I2C)**

```
            VL53L0X ToF Sensor
            ┌─────────────────┐
GPIO22 (SCL)├─ SCL            │
GPIO21 (SDA)├─ SDA            │
3.3V ───────├─ VCC            │
GND ────────├─ GND            │
GPIO5 ◄─────├─ GPIO (optional)│  (Interrupt/shutdown)
            └─────────────────┘

CLI Configuration:
  set rangefinder_type VL53L0X
  set rangefinder_address 0x29    # Default I2C address
  save
```

#### **Landing Assist CLI Commands**

**Enable landing assist mode:**
```bash
# Prerequisite: Enable landing assist in Betaflight Configuration tab
# Then configure landing speed and behavior

# Enable landing assist feature (if available)
set landing_assist_mode SOFT    # or AUTO

# Set landing sensitivity (lower = more aggressive)
set landing_throttle 1000       # Minimum throttle during landing

# Set altitude to trigger landing assist (if rangefinder present)
set landing_assist_altitude 50  # cm above ground (optional)

save
```

**Test landing assist (CLI):**
```bash
# In CLI:
status              # Verify rangefinder/baro detected
range_bottom        # Check rangefinder readings (if available)
```

**Landing assist presets (recommended):**
```bash
# Indoor environment (smaller areas, slow descent)
preset landing_indoor

# Outdoor environment (larger areas, moderate descent)
preset landing_outdoor

# Freeestyle mode (disable auto-landing for tricks)
preset landing_disabled
```

---

### **Obstacle Avoidance (Rangefinder + Lidar)**

Obstacle avoidance uses rangefinders to detect and warn of nearby obstacles.

#### **Mandatory Sensor**
- ✅ **Front rangefinder** — Required for obstacle avoidance; if the front sensor is missing, the feature remains inactive

#### **Optional Sensors**
- ⚠️ **Bottom rangefinder** — Useful for altitude context and landing/hold fusion, but not required for obstacle avoidance

#### **Supported Sensor Combinations**

| Setup | Sensors | Range | Best For | Notes |
|---|---|---|---|---|
| **Bottom only** | 1x Rangefinder (bottom) | 2–400 cm | Altitude / landing support | Not enough for obstacle avoidance |
| **Front only** | 1x Rangefinder (front) | 2–400 cm | Forward object detection | Minimum obstacle avoidance setup |
| **Front + Bottom** | 2x Rangefinders | 2–400 cm each | Full avoidance + altitude context | Use separate slots, pins, and addresses |

#### **Bottom Rangefinder Wiring (HC-SR04 via GPIO)**

```
            HC-SR04 (Bottom Range)
            ┌─────────────────┐
GPIO14 ─────├─ TRIG           │
GPIO27 ◄─────├─ ECHO          │
5V ─────────├─ VCC            │
GND ────────├─ GND            │
            └─────────────────┘

CLI Configuration:
  set pin_input_rf_bottom 14        # Trigger GPIO
  set pin_input_rf_bottom_echo 27   # Echo GPIO
  set rangefinder_type HCSR04       # Specify sensor type
  save
```

#### **Front Rangefinder Wiring (VL53L0X via I2C)**

```
            VL53L0X (Front Range)
            ┌─────────────────┐
GPIO22 (SCL)├─ SCL            │
GPIO21 (SDA)├─ SDA            │
3.3V ───────├─ VCC            │
GND ────────├─ GND            │
            └─────────────────┘

CLI Configuration:
  set rangefinder_type VL53L0X
  set rangefinder_position FRONT
  set rangefinder_address 0x29
  save
```

#### **Dual Rangefinder Setup (Bottom + Front)**

```
Bottom (HC-SR04)        |        Front (VL53L0X)
────────────────        |        ─────────────────
GPIO14 (TRIG)           |        GPIO22 (SCL)
GPIO27 (ECHO)           |        GPIO21 (SDA)
5V, GND                 |        3.3V, GND

CLI Configuration:
  # Bottom rangefinder (GPIO-based)
  set pin_input_rf_bottom 14
  set pin_input_rf_bottom_echo 27
  set rangefinder_type HCSR04
  
  # Front rangefinder (I2C-based)
  set rangefinder_type_front VL53L0X
  set rangefinder_address_front 0x29
  
  save
```

> [!NOTE]
> Keep the bottom and front rangefinder assignments separate. Do not reuse the same physical sensor, GPIO pair, or I2C address for both slots.

#### **Obstacle Avoidance CLI Commands**

**Enable obstacle detection:**
```bash
# Enable bottom rangefinder (landing / altitude support)
set rangefinder_enabled 1
set rangefinder_type HCSR04         # or VL53L0X, TFMINI, etc.
set pin_input_rf_bottom 14
set pin_input_rf_bottom_echo 27
save

# Verify detection
status                              # Should show rangefinder data
range_bottom                        # Read bottom rangefinder values
```

**Enable front obstacle detection:**
```bash
set rangefinder_front_enabled 1
set rangefinder_type_front VL53L0X
set rangefinder_address_front 0x29
save

range_front                         # Read front rangefinder values
```

**Set obstacle warning thresholds:**
```bash
# Buzzer alarm if obstacle closer than X cm
set rangefinder_alert_distance 50   # 50 cm warning threshold
set rangefinder_max_range 400       # Max measurement range

# Landing assist triggers at this altitude
set landing_trigger_altitude 30     # 30 cm above ground

save
```

**Obstacle avoidance tuning:**
```bash
# Increase responsiveness to obstacles
set rangefinder_filter_samples 1    # Raw, unfiltered (fast)
set rangefinder_filter_samples 3    # Averaged (smoother)

# Adjust altitude fusion blending (barometer vs rangefinder)
set altitude_fusion_mode GPS_BARO   # GPS + Barometer only
set altitude_fusion_mode GPS_BARO_RF # GPS + Barometer + Rangefinder

save
```

---

### **Sensor Integration Summary**

#### **Altitude Hold Setup**
- **Minimum**: Barometer (I2C)
- **Recommended**: Barometer + GPS
- **Optional**: Accelerometer (already on-board)
- **CLI Enable**: `set baro_bus I2C` + `set feature_gps 1`

#### **Landing Assist Setup**
- **Minimum**: Accelerometer (on-board)
- **Recommended**: Accelerometer + Barometer
- **Optional**: Rangefinder (bottom) for precision landing
- **CLI Enable**: `preset landing_outdoor` (or choose based on environment)

#### **Obstacle Avoidance Setup**
- **Minimum**: 1x Rangefinder (bottom or front)
- **Recommended**: Dual rangefinders (bottom + front)
- **Optional**: Lidar for extended range (30+ meters)
- **CLI Enable**: `set rangefinder_enabled 1` + sensor type/pins

---

### **Common Sensor Wiring Issues & Troubleshooting**

| Issue | Symptom | Fix |
|---|---|---|
| **Barometer not detected** | `status` shows "baro rate: 0 Hz" | Verify I2C/SPI bus, check address (0x76/0x77), reboot |
| **Altitude jumps erratically** | Altitude oscillates by ±10 cm | Increase `vario_i`; check sensor noise; verify ground plane |
| **Landing assist too aggressive** | Throttle cuts suddenly | Increase `landing_throttle` threshold; reduce sensitivity |
| **Rangefinder returns 0 cm** | Obstacle avoidance inactive | Check GPIO connections; verify sensor power; test with `range_bottom` |
| **Front rangefinder not working** | Dual setup, front data missing | Verify second sensor I2C address; check for address conflicts |
| **Ultrasonic noise interference** | HC-SR04 readings jump 10+ cm | Add 100nF capacitor on echo pin; move sensor away from EMI |
| **Lidar thermal drift** | VL53L0X accuracy decreases over time | Enable `rangefinder_temp_compensation`; warm up sensor first |


