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

## Default Analog pin mapping

| Uart    | CLI name          | ESP32 | ESP32-S3 |
|--------:|-------------------|------:|---------:|
| Voltage | `pin_input_adc_0` |  36   | 1        |
| Current | `pin_input_adc_1` |  39   | 4        |

> [!NOTE]
> On ESP32 choose only pins assigned to ADC1 channels

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
