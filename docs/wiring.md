# ESP-FC wiring examples and PIN mapping

ESP32 MCUs allows to remap pins, so the wiring is not final and you can remap intputs and outputs to your needs. To change pin function go to the CLI and use `get pin` command to check current assignment. For example, to set first output to pin 1 use command

`set pin_output_0 1`

Tu unmap pin function use -1 as pin number

`set pin_output_3 -1`

> [!NOTE]
> There are still some limitations in remapping. Not all pins can be used with all functions. Some pins are input only, others are strapping pins and may prevent from booting if used incorrectly. please consult MCU documentation for available options. Default layout is proven to work.

## Default I2C pin mapping for gyro modules

| Module Pin | CLI Name         | ESP32 | ESP32-S3 |
|------------|------------------|------:|---------:|
| SCK/SCL    | `pin_i2c_scl`    | 22    | 10       |
| SDA/SDI    | `pin_i2c_sda`    | 21    | 9        |

> [!NOTE]
> I2C driver accepts only pins from 1 to 31

## Default SPI pin mapping gyro modules

| Module Pin  | CLI Name         | ESP32 | ESP32-S3 |
|-------------|------------------|------:|---------:|
| SCK/SCL     | `pin_spi_0_sck`  | 18    | 12       |
| SDA/SDI     | `pin_spi_0_mosi` | 23    | 11       |
| SAO/SDO/ADO | `pin_spi_0_miso` | 19    | 13       |
| NCS         | `pin_spi_cs_0`   |  5    |  8       |
| CSB*        | `pin_spi_cs_1`   | 13    |  7       |

**Note:** `CSB` is required for barometer on 10-DOF MPU-9250 modules

> [!TIP]
> For better performance preffer SPI

## Default Servo/Motor output mapping

| Motor | CLI name | ESP32 | ESP32-S2 | ESP32-S3 | ESP32-C3 | ESP8266 | RP2040/RP2350 |
|------:|----------|------:|---------:|---------:|---------:|--------:|--------------:|
| 1 | `pin_output_0` | 27 | 39 | 39 | 2 | 16 (D0) | 2 |
| 2 | `pin_output_1` | 25 | 40 | 40 | 3 | 14 (D5) | 3 |
| 3 | `pin_output_2` | 4 | 41 | 41 | 4 | 12 (D6) | 4 |
| 4 | `pin_output_3` | 12 | 42 | 42 | 5 | 15 (D8) | 5 |

> [!NOTE]
> ESP32 and RP2040 targets expose `pin_output_4` to `pin_output_7`, but those extra slots default to `-1` (unmapped).

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
