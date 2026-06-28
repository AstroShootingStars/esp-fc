# WEMOS D1 MINI pinout

**Before wiring any board:** Check [Board Reference](boards.md) for your target's serial ports, I2C/SPI pins, sensor support, and available features. This page shows ESP8266 pinout as an example; see the Board Reference for other targets (ESP32 variants, RP2040/RP2350, STM32F7, STM32H7).

For quick per-target feature and serial-port availability, see [Board capability matrix](setup.md#board-capability-matrix).

## All Boards Quick Connection Matrix

Use this as a fast wiring index before opening the detailed pinout sections below.

| Board Family | Motor Outputs (default) | RX Serial (default) | VTX SmartAudio (default) | I2C (default) | SPI (default) |
|---|---|---|---|---|---|
| ESP32 | GPIO27, GPIO25, GPIO4, GPIO12 | UART2 RX=GPIO16 TX=GPIO17 | UART1 TX=GPIO33 RX=GPIO32 | SDA=GPIO21 SCL=GPIO22 | SCK=GPIO18 MOSI=GPIO23 MISO=GPIO19 CS_GYRO=GPIO5 CS_BARO=GPIO13 |
| ESP32-S2 | GPIO39, GPIO40, GPIO41, GPIO42 | UART1 RX=GPIO16 TX=GPIO17 | UART0 TX=GPIO43 RX=GPIO44 | SDA=GPIO8 SCL=GPIO9 | SCK=GPIO12 MOSI=GPIO11 MISO=GPIO13 CS_GYRO=GPIO10 CS_BARO=GPIO7 |
| ESP32-S3 (all variants) | GPIO39, GPIO40, GPIO41, GPIO42 | UART2 RX=GPIO17 TX=GPIO18 | UART0 TX=GPIO43 RX=GPIO44 | SDA=GPIO9 SCL=GPIO10 | SCK=GPIO12 MOSI=GPIO11 MISO=GPIO13 CS_GYRO=GPIO8 CS_BARO=GPIO7 |
| ESP32-C3 | GPIO2, GPIO3, GPIO4, GPIO5 | Board default UART RX_SERIAL | UART0 TX=GPIO21 RX=GPIO20 | I2C supported (see board defaults) | Not used by default target profile |
| ESP8266 (D1 mini) | GPIO16, GPIO14, GPIO12, GPIO15 | UART0 RX=GPIO3 TX=GPIO1 (shared MSP) | UART1 TX=GPIO2 (TX-only) | SDA=GPIO4 SCL=GPIO5 | Not used by default target profile |
| RP2040 (Pico) | GPIO2, GPIO3, GPIO4, GPIO5 | UART1 RX=GPIO9 TX=GPIO8 | UART0 TX=GPIO0 RX=GPIO1 | SDA=GPIO6 SCL=GPIO7 | SCK=GPIO10 MOSI=GPIO11 MISO=GPIO12 CS_GYRO=GPIO13 CS_BARO=GPIO14 |
| RP2350 / RP2350B | GPIO2, GPIO3, GPIO4, GPIO5 | UART1 RX=GPIO9 TX=GPIO8 | UART0 TX=GPIO0 RX=GPIO1 | SDA=GPIO6 SCL=GPIO7 | SCK=GPIO10 MOSI=GPIO11 MISO=GPIO12 CS_GYRO=GPIO13 CS_BARO=GPIO14 |
| STM32F7 / STM32H7 | D3, D5, D6, D9 | UART0 (`Serial`) | Not assigned by default | SDA=D14 SCL=D15 | SCK=D13 MOSI=D11 MISO=D12 CS_GYRO=D10 CS_BARO=D4 |

Notes:
- RP2350B ARM and Hazard3 RISC-V environments share the same pin defaults.
- ESP32-S3 variants (`esp32s3`, `esp32s3_devkitc`, `esp32s3_wroom`, `esp32s3_n8r8`, `esp32s3_n8r16`) share the same FC wiring defaults.
- For any remap, use `get pin` and `set pin_*` commands in the CLI, then `save` and `reboot`.

```
Pin | Function         | ESP-8266 Pin | ESPFC external device
----+------------------+--------------+----------------------------------------------
TX  | TXD0             | TXD, GPIO1   | (UART0 TX) | CLI | MSP | OSD
RX  | RXD0             | RXD, GPIO3   | (UART0 RX) | CLI | MSP | OSD
A0  | Analog In,  3.3V | A0           | Batery voltage or current sensor, analog only
D0  | IO               | GPIO16       | LED | BUZZER (output recommended, no interrupts)
D1  | IO, SCL          | GPIO5        | I2C Gyro, Compas, Pressure and other sensors
D2  | IO, SDA          | GPIO4        | I2C Gyro, Compas, Pressure and other sensors
D3  | IO, PU           | GPIO0        | PWM 1 (servo/motor)
D4  | IO, PU, LD, TXD1 | GPIO2        | Blackbox (UART1 TX)
D5  | IO, SCK          | GPIO14       | PWM 2 (servo/motor)
D6  | IO, MISO         | GPIO12       | PWM 3 (servo/motor)
D7  | IO, MOSI         | GPIO13       | PPM RX | PWM
D8  | IO, SS, PD       | GPIO15       | PWM 4 (servo/motor)
G   | Ground           | GND          | Ground
5V  | 5V               | N/A          | 5V power
3V3 | 3.3V             | 3.3V         | 3.3V power - to I2C devices
RST | Reset            | RST          | Reset switch (optional)
```

## Legend:
```
   PU - 10k pull up
   PD - 10k pull down
 RXD0 - UART0 hardware RX
 TXD0 - UART0 hardware TX
 TXD1 - UART1 hardware TX
   LD - builtin LED
```

## Notice:
```
 A0 - In ESP-8266 is 1.0V range, Wemos has 220k:100k divider for 3.3V range, 22k:4k7 additional divider required for 4S Lipo
 D0 - No iterrupts (recommended output)
 D3, GPIO0 - if low at boot, then board enters to flashing mode (avoid using as input)
 D4, GPIO2 - must be high during flashing (avoid using as input)
 D8, GPIO15 - must be low during flashing or boot (avoid using as input)
```
https://github.com/esp8266/esp8266-wiki/wiki/Boot-Process#esp-boot-modes

## Wemos D1 mini board layout:
```
      /-----------\
   TX |           | RST
   RX |           | A0
   D1 |           | D0
   D2 |           | D5 ~>
<~ D3 |           | D6 ~>
<~ D4 |           | D7 ~<
  GND |           | D8 ~>
   5v |           | 3.3V
      +---|---|---+
           USB
```

# ESP32 pinout
```
Pin | Name | Function                                   | ESPFC external device
----+------+--------------------------------------------+----------------------------------------------
 0  |      | PU,ADC2_CH1,CLK_OUT1                       | >  BUTTON
 1  | TXD  | U0TXD,CLK_OUT3                             |    TX0, PROG, MSP
 2  |      | ADC2_CH2,HSPIWP,HS2_DATA0,SD_DATA0         | >  LED
 3  | RXD  | U0RXD,CLK_OUT2                             |    RX0, PROG, MSP
 4  |      | ADC2_CH0,HSPIHD,HS2_DATA1,SD_DATA1         |    M2
 5  |      | VSPICS0,HS1_DATA6                          | >  SPI_CS0_GYRO, SPI0_SS
12  | TD1  | PD,ADC2_CH5,MTDI,HSPIQ,HS2_DATA2,SD_DATA2  | >  M3, SPI1_MISO
13  | TCK  | ADC2_CH4,MTCK,HSPID,HS2_DATA3,SD_DATA3     |    SPI_CS1_BARO, SPI1_MOSI
14  | TMS  | ADC2_CH6,MTMS,HSPICLK,HS2_CLK,SD_CLK       | $  FREE, SPI1_SCK
15  | TD0  | PU,ADC2_CH3,MTDO,HSPICS0,HS2_CMD,SD_CMD    | >  FREE, SPI_CS2_SDCARD, SPI1_SS
16  |      | HS1_DATA4,U2RXD                            |    RX2
17  |      | HS1_DATA5,U2TXD                            |    TX2
18  |      | VSPICLK,HS1_DATA7                          |    SPI0_SCK
19  |      | VSPIQ,U0CTS                                |    SPI0_MISO
21  |      | VSPIHD                                     |    I2C0_SDA
22  |      | VSPIWP,U0RTS                               |    I2C0_SCL
23  |      | VSPID,HS1_STROBE                           |    SPI0_MOSI
25  |      | ADC2_CH8,DAC1                              |    M1
26  |      | ADC2_CH9,DAC2                              |    BUZZER
27  |      | ADC2_CH7                                   |    M0
32  |      | ADC1_CH4,XTAL                              |    RX1
33  |      | ADC1_CH5,XTAL                              |    TX1
34  |      | ADC1_CH6                                   | <$ ADC, INTR
35  |      | ADC1_CH7                                   | <  PPM, ADC, INTR
36  | SVP  | ADC1_CH0                                   | <  ADC_VOLTAGE
37  | -    | ADC1_CH1                                   | -  N/A
38  | -    | ADC1_CH2                                   | -  N/A
39  | SVN  | ADC1_CH3                                   | <  ADC_CURRENT
```

## Legend
```
 '>' recommended output only
 '<' input only
 '$' free
 '-' not available
```
GPIO `0, 2, 5, 12, 15` recommended as output because of boot process impact
GPIO `34-39` are input-only by chip
GPIO `6-11` are reserved for integrated flash
GPIO `20,24,28,29,30,31` does not exist

- GPIO0 - Low/GND	ROM serial bootloader for esptool.py, High/VCC	Normal execution mode
- GPIO2 - GPIO2 must also be either left unconnected/floating, or driven Low, in order to enter the serial bootloader. In normal boot mode (GPIO0 high), GPIO2 is ignored.
- GPIO12 - If driven High, flash voltage (VDD_SDIO) is 1.8V not default 3.3V. Has internal pull-down, so unconnected = Low = 3.3V. May prevent flashing and/or booting if 3.3V flash is used and this pin is pulled high, causing the flash to brownout. See the ESP32 datasheet for more details.
- GPIO15 - If driven Low, silences boot messages printed by the ROM bootloader. Has an internal pull-up, so unconnected = High = normal output.
- GPIO5 - configures SDIO Slave in "Download Boot"

https://github.com/espressif/esptool/wiki/ESP32-Boot-Mode-Selection

## ESP32 mini board layout
```
                  /-------------\
 GND      RST     |             |  TXD (1)    GND
  NC      SVP(36) |             |  RXD (3)    IO(27)
 SVN(39)   IO(26) |             |   IO(22)    IO(25)
  IO(35)   IO(18) |             |   IO(21)    IO(32)
  IO(33)   IO(19) |             |   IO(17)   TD1(12)
  IO(34)   IO(23) |             |   IO(16)    IO(04)
 TMS(14)   IO(05) |             |  GND        IO(00)
  NC      3V3     |             |  VCC(5v)    IO(02)
 SD2(09)  TCK(13) |             |  TD0(15)   SD1(08)
 CMD(11)  SD3(10) |             |  SD0(07)   CLK(06)
                  +----|---|----+
                        USB
```

# ESP32-S3

https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/gpio.html

```
Pin | Function             | ESPFC external device
----+------+---------------+----------------------------------------------
 0  | Strapping            |
 1  | ADC1_CH0             | ADC_VOLTAGE
 2  | ADC1_CH1             | DEBUG
 3  | ADC1_CH2, Strapping  |
 4  | ADC1_CH3             | ADC_CURRENT
 5  | ADC1_CH4             | BUZZER
 6  | ADC1_CH5             | PPM
 7  | ADC1_CH6             | SPI_CS_BARO
 8  | ADC1_CH7             | SPI_CS_GYRO
 9  | ADC1_CH8             | I2C_0_SDA
10  | ADC1_CH9             | I2C_0_SCK
11  | ADC2_CH0             | SPI_0_MOSI
12  | ADC2_CH1             | SPI_0_SCK
13  | ADC2_CH2             | SPI_0_MISO
14  | ADC2_CH3             | ?
15  | ADC2_CH4             | RX1
16  | ADC2_CH5             | TX1
17  | ADC2_CH6             | RX2
18  | ADC2_CH7             | TX2
19  | ADC2_CH8, USB        | USB
20  | ADC2_CH9, USB        | USB
21  | RTC_21               | ?
38  | RGB                  | LED_RGB
39  |                      | M0
40  |                      | M1
41  |                      | M2
42  |                      | M3
43  | U1TX                 | TX0
44  | U1RX                 | RX0
45  |                      | ?
46  | Strapping            | SPI_CS_SD
47  | Strapping            | ?
48  |                      | ?

RESERVED
26  | FLASH                |
27  | FLASH                |
28  | FLASH                |
29  | FLASH                |
30  | FLASH                |
31  | FLASH                |
32  | FLASH                |
33  | FLASH                |
34  | FLASH                |
35  | FLASH                |
36  | FLASH                |
37  | FLASH                |

UNAVAILABLE
22, 23, 24, 25
```