# Setup

This page describe steps absolutly required to proceed before first flight.

## Connecting to Betaflight Configurator

Once you download and install [Betaflight configurator](https://github.com/betaflight/betaflight-configurator) you need to change few options first. Open configurator and go to **Options** tab, and then
1. Disable `Advanced CLI AutoComplete`
2. Enable `Show all serial devices`
3. Disable `Auto-Connect`

![Options](/docs/images/bfc/bfc_options.png)

Then select your device from list and click connect.

## Bootloader / DFU Mode (Betaflight Configurator)

`MSP_REBOOT` now supports Betaflight-style reboot modes used by Configurator firmware flashing flows.

- Mode `0` (`MSP_REBOOT_FIRMWARE`): normal reboot back to firmware
- Mode `1` (`MSP_REBOOT_BOOTLOADER_ROM`): reboot to bootloader/flash mode request

Board behavior:

- `RP2040/RP2350`: bootloader mode enters USB UF2 bootloader (`RPI-RP2` mass storage), matching Configurator flash expectations for RP targets.
- `ESP32/ESP32-S2/ESP32-S3/ESP32-C3/ESP8266`: bootloader request is accepted and the board reboots; enter ROM flashing mode with normal board-specific boot procedure (BOOT/IO0 straps, USB/JTAG ROM downloader, etc.).

Configurator workflow:

1. Connect in Betaflight Configurator.
2. Open Firmware Flasher and start flash operation.
3. Configurator issues `MSP_REBOOT` bootloader request.
4. Reconnect to the enumerated bootloader/flash port if prompted by host OS.

Notes:

- Reboot commands are ignored while armed for safety.
- A successful reboot request returns the selected reboot mode in MSP response, consistent with Betaflight behavior.

> [!NOTE]
> Not all functions displayed in configurator are avalable in firmware. The rule of thumb is if you cannot change specific option in Betaflight Configurator, that means it is not supported. It usually rolls back to previous value after save.

> [!NOTE]
> Leaving some tabs, especially CLI requires you to click "Disconnect" button twice, and then click "Connect" again.

## Configuration via GUI vs CLI

Most flight controller features are available through the **Betaflight Configurator GUI** (Ports, Receiver, Sensors, Filters, Motors, PID, etc.). However, some advanced features are **CLI-only** (pin remapping, WiFi config, landing assists, diagnostics).

| Your Board | Primary Config Tool | Additional CLI Features |
|---|---|---|
| **ESP32** | BFC GUI (Ports, Sensors, Motors, PID tabs) | WiFi/Bluetooth config, pin remapping, landing assist presets |
| **ESP32-S2** | BFC GUI + USB (soft-serial WiFi disabled) | Pin remapping, motor poles, minimal CLI-only features |
| **ESP32-S3** | BFC GUI (Ports, Sensors, Motors, PID tabs) | WiFi config, pin remapping, landing assist presets |
| **ESP32-C3** | BFC GUI (Ports, I2C Sensors only, Motors, PID tabs) | WiFi config (soft-serial), pin remapping, I2C-only sensors |
| **ESP8266** | BFC GUI (Ports, I2C Sensors, Motors, PID tabs) | I2C-only sensors, pin remapping (tight RAM) |
| **RP2040** | BFC GUI + USB (best via USB CDC) | Pin remapping, rangefinder/opflow config, no WiFi |

**See [Configuration Methods in Board Reference](boards.md#configuration-methods-gui-vs-cli) for detailed feature tables.**

## Configure wiring

Go to the `CLI` tab and type `get pin`. This command will show what pins are associated to specified functions. If you wiring is diferrent, you can make some adjustments here. For more details, see [Pin Functions CLI Reference](/docs/cli.md#pin-functions)

![CLI Pins](/docs/images/bfc/bfc_cli_pins.png)

## Power and Battery CLI quick-start

Use this when wiring analog voltage/current sensors and calibrating values for the Power & Battery tab.

### 1) Board default ADC pins

| Board | Voltage (`pin_input_adc_0`) | Current (`pin_input_adc_1`) |
|---|---:|---:|
| ESP32 | 36 | 39 |
| ESP32-S2 | 1 | 4 |
| ESP32-S3 (all variants) | 1 | 4 |
| ESP32-C3 | 0 | 1 |
| ESP8266 | 17 (A0) | not available by default |
| RP2040 / RP2350 | 26 | 27 |

> [!NOTE]
> Firmware now auto-selects a safe ADC fallback if your configured ADC pin is missing/conflicting. If no safe pin exists, that source is disabled.

### 2) Enable sources

Open CLI and apply:

```bash
set vbat_source 1
set ibat_source 1
save
```

If your target has no usable second ADC (for example many ESP8266 builds), set:

```bash
set ibat_source 0
save
```

### 3) Verify assigned pins

```bash
get pin_input_adc_0
get pin_input_adc_1
```

If needed, remap manually and save:

```bash
set pin_input_adc_0 <gpio>
set pin_input_adc_1 <gpio>
save
```

### 4) Calibrate voltage and current

Start with defaults, then tune against a multimeter:

```bash
set vbat_scale 100
set vbat_mul 1
set vbat_div 10
set ibat_scale 100
set ibat_offset 0
save
```

Calibration loop:

1. Connect battery and compare FC voltage reading vs multimeter.
2. Increase `vbat_scale` if FC voltage is low; decrease if high.
3. With known current load, tune `ibat_scale` to match measured current.
4. If current is shifted at zero/idle, adjust `ibat_offset`.
5. Save after each adjustment and re-check.

### 5) Set battery warning thresholds

```bash
set vbat_cell_warn 350
save
```

`350` means 3.50 V per cell warning threshold.

For full wiring and supported sensor details, see [Wiring Guide](wiring.md#power-and-battery-sensors-voltage-and-current).

### Battery sensor troubleshooting

If readings look wrong after wiring and calibration, check these common cases.

1. **Voltage reads 0.0 V**
	- Verify `vbat_source` is `1`.
	- Verify ADC pin mapping with `get pin_input_adc_0`.
	- Confirm divider output is wired to the configured ADC pin.
	- Confirm battery ground and FC ground are shared.

2. **Current reads 0.00 A always**
	- Verify `ibat_source` is `1`.
	- Verify ADC pin mapping with `get pin_input_adc_1`.
	- Check board capability: some targets may not have a usable second ADC by default.
	- Confirm sensor OUT pin is analog voltage output (not digital protocol).

3. **Voltage is consistently too high/too low**
	- Tune `vbat_scale` against a trusted multimeter.
	- Re-check `vbat_mul` and `vbat_div` values.
	- Ensure the divider resistor ratio matches your wiring.

4. **Current offset at idle (non-zero with motors off)**
	- Tune `ibat_offset` until idle current is near zero.
	- Re-check sensor supply voltage and reference grounding.

5. **Noisy or jumping readings**
	- Keep ADC signal wiring short and away from ESC/motor power lines.
	- Use twisted pair or shielded cable for sensor signal+ground where practical.
	- Ensure all grounds are common and low-impedance.

6. **Cell count unstable at plug-in**
	- Wait a few seconds after connect; initial samples are used for cell detection.
	- Verify battery connector and divider wiring quality.

7. **Settings revert after reboot**
	- Ensure `save` was executed in CLI.
	- Re-open CLI and verify values with `get vbat_` and `get ibat_`.

Useful quick checks:

```bash
get vbat_source
get ibat_source
get pin_input_adc_0
get pin_input_adc_1
get vbat_scale
get vbat_mul
get vbat_div
get ibat_scale
get ibat_offset
```

## Board capability auto-sanitization

ESP-FC now automatically sanitizes configuration based on active board IO and runtime capabilities so Betaflight changes do not leave the FC in an invalid state.

What is auto-adjusted:

1. Unsupported serial functions are removed for the active target.
2. Invalid function mixes on one port are reduced to one primary function (priority favors MSP).
3. Functions needing RX are removed from TX-only ports.
4. Functions needing TX are removed from RX-only ports.
5. Duplicate critical functions are collapsed to one port (`RX_SERIAL`, `GPS`, external OSD).
6. If no MSP port remains, one valid MSP port is restored automatically.
7. Wireless OTA and BT OTA are disabled automatically on unsupported targets/builds.

This behavior helps keep all boards configurable from Betaflight without serial-port deadlocks after save/reboot.

> [!NOTE]
> On ESP32-S2, soft-serial WiFi bridge is disabled to keep RAM usage within safe limits. Use USB/UART MSP ports for Betaflight configuration on this target.

### Board capability matrix

| Target | Betaflight MSP ports (default) | SoftSerial WiFi bridge | WiFi OTA | BT OTA | External OSD (MSP DisplayPort) |
|---|---|---|---|---|---|
| ESP32 | UART0, UART1 | Yes | Yes | Optional (build-flag + runtime enable) | Yes, use a free UART with TX and RX |
| ESP32-S2 | UART0, USB CDC | No | No | No | Yes, use free UART with TX and RX |
| ESP32-S3 | UART0, USB CDC (build-dependent) | Yes | Yes | No | Yes, use UART1 or UART2 when free |
| ESP32-C3 | UART0, USB CDC | Yes | Yes | No | Yes, use a free UART with TX and RX |
| ESP8266 | UART0 | Yes | Yes | No | Limited: prefer UART1 TX-only OSD targets or remapped pins |
| RP2040 | USB CDC, UART0 | No | No | No | Yes, use free UART pins |
| RP2350 | USB CDC, UART0 | No | No | No | Yes, use free UART pins |

Notes:

1. Auto-sanitization keeps at least one MSP port active, and removes invalid serial assignments at boot.
2. OSD is enabled only when `osd_enabled=1`, `osd_msp_displayport=1`, and a serial port has function `65536` (`frsky_osd`).
3. ESP32 Bluetooth OTA is only available on classic ESP32 builds when `ESPFC_BT_OTA` is enabled and Bluetooth stack support is present.

### Quick serial setup by target

For detailed per-board port assignments, sensor connections, and feature availability, see [Board Reference](boards.md).

Copy and paste the CLI commands below for your board, then run `save` and `reboot`. Adjust port indices and functions as needed (e.g., if not using RX serial or OSD).

**ESP32** (Betaflight + RX serial + GPS + external OSD):
```
set serial_0 1 115200 0
set serial_1 64 115200 0
set serial_2 2 115200 0
set serial_soft_0 65536 115200 0
```

**ESP32-S2** (UART0 MSP + RX serial):
```
set serial_0 1 115200 0
set serial_1 64 115200 0
```

**ESP32-S3** (UART0 MSP + RX serial + GPS):
```
set serial_0 1 115200 0
set serial_1 2 115200 0
set serial_2 64 115200 0
```

**ESP32-C3** (UART0 MSP only, limited ports):
```
set serial_0 1 115200 0
```

**ESP8266** (UART0 MSP + OSD, no separate RX port):
```
set serial_0 65537 115200 0
```

**RP2040** (UART0 MSP + RX serial):
```
set serial_0 1 115200 0
set serial_1 64 115200 0
```

Serial function codes: `1=MSP`, `2=GPS`, `64=RX_SERIAL`, `65536=frsky_osd (external OSD)`, `65537=MSP+OSD`.

After applying these, go to Betaflight **Ports** tab to verify the allocations and adjust if needed.

## Gyro calibration

Go to `Setup tab`, make sure that your model is in horizontal steady state position. Then click **"Calibrate Accelerometer"** button and wait two seconds.

> [!CAUTION]
> Ensure that model in preview moves exactly in the same way as yours. If it is not the case, go to the `Configuration` tab, and configure `Board orientation` or `Sensor Alignment`

![Sensor Alignment](/docs/images/bfc/bfc_configuration.png)

## System configuration

In `Configuration` tab set `Pid loop frequency`. Recomended values are 1kHz to 2kHz. Keep eye on CPU Load displayed in bottom, it shoudn't exceed 50%.

## Receiver setup

If you want to use Serial based receiver (SBUS,IBUS,CRSF), you need to allocate UART port for it. You can do it in `Ports` tab, by ticking switch in `Serial Rx` column.

Then go to the `Receiver` tab, and select `Receiver mode` and `Serial Receiver Provider`.

To use ESP-NOW receiver, choose "SPI Rx (e.g. built-in Rx)" receiver mode in Receiver tab. You need compatible transmitter module. Read more about [ESP-FC Wireless Functions](/docs/wireless.md)

## Motor setup

In `Motors` tab you must configure

### Mixer

You can select mixer type here, this configuration depends on type of aircraft you are configuring. 

**Important!** If you build multirotor, you must ensure that

1. Specified motor number is connected to specified output and placed in specified position in your aircraft according to gyro orientation, presented on a picture.
2. Specified motor is rotating in correct direction according to image.

> [!CAUTION] 
> If these conditions aren't met, your quad will go crazy on the first start and may cause damage or even injury.

To verify that you can enable **test mode** and spin each motor selectively. To do that,
1. remove all propellers, 
2. connect batery,
3. click "I understand the risk...", 
4. move specified slider to spin motor.

If you are using any analog protocol (PWM, OneShot, Multishot), you also need to calibrate your ESCs here. To do that
1. click "I understand the risk...", 
2. move master sliders to highest value
3. connect battery
4. when escs plays calibration beeps, move master slider down to lowest value
6. escs should play confirmation beeps

> [!NOTE]
> Only Quad X supported at the moment. Motor direction wizard is not supported, you must configure your ESC separately. The easiest way to change direction is swapping motor wires.

### Motor direction is reversed

This option informs FC that your motors rotating in reverse order. This option desn't reverse motor direction. It has to be configured with ESC configurator or by swapping motor wires.

## Motors and ESC Configuration

### Brushed vs Brushless Motors

ESP-FC supports both brushed and brushless motor types. Understanding the differences is critical for proper configuration.

#### **Brushless Motors** (Recommended for multirotors)

**Characteristics:**
- **Commutation**: Electronic commutation via ESC (ESC controls timing)
- **Efficiency**: 85–90% efficient
- **Power**: High power-to-weight ratio
- **Speed Range**: Wide RPM range (3000–40,000+ RPM typical)
- **Cost**: Higher ($20–$100+ per motor)
- **Maintenance**: No brushes; virtually maintenance-free
- **Noise**: Quieter operation

**Suitable ESCs/Protocols:**
- ✅ DShot (150/300/600/ProShot) — **Recommended for high-performance multirotors** (digital, bidirectional, telemetry capable)
- ✅ OneShot (125/42) — Traditional analog protocol, older ESCs
- ✅ Multishot — Faster analog variant
- ❌ PWM — Not recommended for multirotors (poor throttle resolution)
- ❌ Brushed — Not suitable (requires brushed motor)

**Wiring:**
- Three motor wires (A, B, C) to ESC
- ESC power: BEC (Battery Eliminator Circuit) supplies power to FC
- ESC signal: PWM/DShot signal from FC GPIO
- **Phase order matters**: If motor spins backward, swap any two of the three wires

**Configuration in Betaflight:**
```
1. Motors tab: Select motor type (usually "Brushless")
2. ESC protocol: Select appropriate protocol (DSHOT300 recommended)
3. Motor direction: Configure in ESC software, not in FC
4. Throttle calibration: Usually automatic with DShot; manual for analog protocols
```

#### **Brushed Motors** (Legacy/Micro quadcopters)

**Characteristics:**
- **Commutation**: Mechanical brushes on rotating commutator
- **Efficiency**: 75–80% efficient
- **Power**: Lower power-to-weight (typically <20g frames)
- **Speed Range**: Limited RPM range (few thousand RPM)
- **Cost**: Lower ($1–$10 per motor)
- **Maintenance**: Brushes wear; periodic replacement needed
- **Noise**: Audible buzzing noise

**Suitable ESCs/Protocols:**
- ✅ Brushed — **Direct PWM control** to FET driver (not a traditional ESC)
- ❌ DShot/OneShot — Not suitable (designed for brushless)

**Wiring:**
- Two motor wires (±) to FET driver output
- FET driver connected to FC GPIO (PWM signal)
- FET driver power: Directly from battery
- **Polarity matters**: Reversed wires = reversed direction (easily fixable)
- **Capacitor**: 470µF/16V capacitor recommended across motor leads to reduce EMI

**Configuration in Betaflight:**
```
1. Motors tab: Select motor type ("Brushed" if available)
2. ESC protocol: Set to "Brushed"
3. Motor direction: Swap motor leads if spinning backward
4. Throttle calibration: Not applicable (direct PWM)
```

**Typical Brushed FET Driver Wiring (ESP32 example):**
```
                Battery (+)
                    |
            ┌───────┴────────┐
            |                |
            D                |
            | (MOSFET)       |
            S────────────M────┴──────Motor (−)
            |
            G (GPIO27) ← FC PWM output
            |
           GND ──────────────Battery (−)
                      |
                   Motor (+)
```

---

### ESC Protocols Comparison

| Protocol | Type | Use Case | DShot Telemetry | Frame Rate | Notes |
|---|---|---|---|---|---|
| **PWM** | Analog | Legacy/testing only | No | 50 Hz | Poor throttle resolution; not recommended for multirotors |
| **OneShot125** | Analog | Older ESCs | No | 125 Hz | Better resolution than PWM; standard for 2010s era |
| **OneShot42** | Analog | High-speed analog ESCs | No | 420 Hz | Faster refresh; improves responsiveness |
| **Multishot** | Analog | Experimental/optimized | No | 2 kHz+ | Between OneShot and DShot; rare |
| **Brushed** | Direct PWM | Brushed motors / FET drivers | No | Variable | Direct PWM to FET/driver; async-only |
| **DShot150** | Digital | Standard brushless (4S–6S) | Yes (6S+) | 150 Hz | Most common; good for all modern ESCs |
| **DShot300** | Digital | High-performance | Yes | 300 Hz | Better latency; recommended for racing |
| **DShot600** | Digital | Extreme performance | Yes | 600 Hz | Highest bandwidth; 32k ESC required |
| **ProShot** | Digital | Future-proofing | Yes | 32 kHz+ | Next-gen protocol; requires compatible ESCs |

---

### ESC Protocol Selection

**For Brushless Motors:**

1. **Racing / High-Performance** → `DSHOT300` or `DSHOT600`
   - Best low-latency response
   - Bidirectional DShot for telemetry and active braking
   - Requires modern high-speed ESC (≥24k firmware or 32k)

2. **Standard Multirotors** → `DSHOT150` ⭐ **Recommended**
   - Works with virtually all modern ESCs
   - Reliable and proven
   - DShot telemetry available
   - Good balance of performance and compatibility

3. **Older ESCs** → `OneShot125` or `OneShot42`
   - For pre-DShot era ESCs
   - Still functional; no telemetry
   - Slight responsiveness penalty vs DShot

4. **DO NOT USE** → `PWM`
   - Obsolete for multirotors
   - Poor throttle resolution
   - High latency

**For Brushed Motors:**

→ Always use `Brushed` protocol
   - Direct PWM output to FET driver
   - Fixed async operation (no sync option)
   - Typical PWM frequency: 16 kHz (can adjust via `set output_motor_rate`)

---

### Motor Configuration CLI Commands

**Check current configuration:**
```
get output_
```

**Set motor protocol (brushless example):**
```
set output_motor_protocol DSHOT300
save
```

**Set motor protocol (brushed example):**
```
set output_motor_protocol BRUSHED
save
```

**Set motor PWM frequency (if needed for analog protocols):**
```
set output_motor_rate 480    # PWM frequency in Hz
save
```

**Adjust motor min throttle (minimum command):**
```
set output_min_throttle 1050
save
```

**Available Protocol Options:**
- `PWM`
- `ONESHOT125`
- `ONESHOT42`
- `MULTISHOT`
- `BRUSHED`
- `DSHOT150` ⭐
- `DSHOT300`
- `DSHOT600`
- `PROSHOT`

---

### Motor Calibration

#### **Analog Protocols (OneShot, Brushed)**

Before first flight, ESCs must be calibrated to recognize min/max throttle signals:

1. **Remove all propellers** ⚠️
2. **Enable test mode** in Betaflight Motors tab
3. **Move throttle slider to 100%** and connect battery
4. ESCs emit calibration beep sequence
5. **Move throttle slider to 0%**
6. ESCs emit confirmation beeps
7. Calibration complete

**If using brushed motors with PWM driver:**
- Calibration typically not required (direct PWM)
- FET driver threshold is fixed by hardware

#### **DShot Protocols**

- **No calibration required** ✅
- ESC detects throttle range automatically
- Motors spin immediately after arming (if armed)

---

### Motor Direction Configuration

#### **Brushless Motors**

Motor direction is controlled by the **ESC firmware**, not the FC. To reverse direction:
1. **Swap any two of the three motor wires** (e.g., swap A↔B)
2. Or use **ESC configurator software** (BLHeli, ESC Pro, etc.) to reverse electronically

The `Motor direction is reversed` option in Betaflight simply informs the FC that your motor order is opposite to expected. It does **not** physically reverse motors.

#### **Brushed Motors**

Motor direction is controlled by **motor polarity**:
1. **Reverse the two motor wires** (swap + and −)
2. No software configuration needed

---

### Motor Speed Separated from PID Loop

Modern ESCs can operate at different frequencies than your PID loop:

```
set gyro_sync_denom 1        # PID loop = 8 kHz / 1 = 8 kHz
set output_motor_rate 480    # Motor PWM = 480 Hz (independent)
```

This allows:
- **High PID frequency** (1–8 kHz) for responsive control
- **Lower motor rate** if your ESC has limitations
- Better performance with legacy analog ESCs

**Recommended settings:**
- Modern DShot ESCs: Leave motor rate on auto (protocol determines it)
- Analog ESCs: Set `output_motor_rate` to ESC max (typically 480 Hz)

---

### ESC Troubleshooting

| Symptom | Likely Cause | Fix |
|---|---|---|
| Motors don't spin | Protocol mismatch or throttle inverted | Verify protocol in setup; check throttle direction in receiver tab |
| Motor spins wrong direction | Motor wires or ESC phase reversed | Swap two motor wires on that ESC |
| Jerky/unstable throttle | PWM frequency too low or ESC overheating | Increase motor_rate (if analog); reduce PID loop rate; check ESC temps |
| DShot telemetry missing | ESC/protocol incompatible | Verify DShot300+; ESC must support telemetry firmware (≥24k) |
| Brushed motor twitching | FET driver oscillating or low battery | Add 470µF capacitor across motor leads; ensure good power connections |



## Flight modes

Flight modes can be configured in `Modes` tab

### Arm

It is required to enable this mode to be able to fly. Armed quad cause that motors are spinning (unless Motor stop option is enabled).

### Angle

By default ACRO mode is engaged if none of flying mode is active. This mode enables self-leveling. In this case you control titlt angle of multirotor instead of rotationg rate. It affects only Roll and Pitch axes. Yaw axis behave the same way as in Acro mode.

### Air mode

Increases the range of control at extreme throttle positions. Recommended to enable, can be controlled by the same RC channel as Arm to get permanent behaviour. Activates, when throttle achive about 40% remains active until disarmed.

### Buzzer

Acitvates buzzer, for example to find lost model in high grass.

### Fail safe

Triggers Failsafe procedure.

## Blackbox

It is possible to collect flight data in two ways. Via `serial port` or with `onboard flash`

Onboard flash allows to store about 2.5MB of data. This is equivalent of 2-3 minutes of flight. It should be enough for tuning. 

If you need more, choose `Serial Port` and serial device like [D-ronin OpenLager](https://github.com/d-ronin/openlager) or [OpenLog](https://github.com/sparkfun/OpenLog). To configure it
1. In `Ports` select uart to generate stream and in Peripherals column select `Blackbox logging` on free port
2. Then in `Blackbox` tab select `Serial Port` or `Onboard flash` as `logging device`

> [!NOTE]
> Port speed from column `Configuration/MSP` is used, and the same speed must be used in logging device, _(this might be subject of change in a future versions)_.

Recommended settings

- To log with 1k rate, you need device that is capable to receive data with 500kbps.
- To log with 2k rate, you need 1Mbps

OpenLager can handle it easily. If you plan to use OpenLog, you might need to flash [blackbox-firmware](https://github.com/cleanflight/blackbox-firmware) to be able to handle more than 115.2kbps. Either 250kbps and 500kbps works well with modern SD cards up to 32GB size.

## Limitations

### Configuration tab

In Other features you can enable only `Dynamic Filter`, `GPS`, and `SoftSerial`

`AirMode` is only available in modes tab. If you want to enable it permanently, use the same control channel as ARM.

### Failsafe tab

Only "Drop" Stage 2 procedule is possible

### GPS tab

GPS configuration is currently CLI-only. Use the commands documented in the GPS Setup section above. The GPS tab in Betaflight Configurator shows GPS status but cannot modify GNSS constellation settings.

### Presets tab

Presets aren't supported, do not try to apply any of them.

### PID Tuning tab

1. There is only one pid profile and one rate profile
2. Feed-forward transition has no effect
3. Iterm-rotation has no effect
4. Dynamic damping has no effect
5. Throttle boost has no effect
6. Miscellaneous settings has no effect
7. Dynamic Notch works from 1k pid loop, but is not displayed in configurator if this speed is configured, to reconfigure switch pid loop to 2k.

Besides that most of Betaflight principles can be applied here according to PID and Filter tuning. But keep in mind, very aggresive tunnig tips aren't recommended to apply and might lead to diferrent results.

### Receiver

1. Not all protocols are implemented, currently only PPM, CRSF, SBUS, IBUS
2. Only CRSF telemetry and rssi_adc
3. RC deadband applies to RPY, no separate Yaw deadband

### Modes

Add range works only, no add link

### Adjustments

Implemented. The Adjustments tab maps to `MSP_ADJUSTMENT_RANGES` / `MSP_SET_ADJUSTMENT_RANGE` and persists the full adjustment record: state index, AUX range channel, range start/end, function, switch channel, and optional center/scale fields.

### Servos

Not Implemented, for replacemnt you can use [output cli](/docs/cli.md#output-channel-config) to change output protocol and rate.

### Motors

No 3D features, only Quad-X Mixer

### Video transmitter

Not yet implemented, work in progess

### OSD

External OSD over MSP DisplayPort is supported.

#### Recommended setup

1. Use one UART for Betaflight Configurator MSP and a different UART for external OSD.
2. Keep MSP enabled on the Configurator port.
3. Assign external OSD function to the OSD UART using CLI:

```
# Example: set UART1 to external OSD function only
set serial_1 65536 115200 0

# Enable OSD and MSP displayport behavior
set osd_enabled 1
set osd_msp_displayport 1
set osd_video_system 2
set osd_profiles 3
set osd_profile 1

save
reboot
```

#### Betaflight Configurator validation checklist

1. Connect FC to Betaflight Configurator and open OSD tab.
2. Verify OSD tab loads without command errors.
3. Move at least one OSD element, click Save.
4. Disconnect and reconnect Configurator.
5. Confirm moved element is still in the saved position.
6. Change active OSD profile in OSD tab and Save.
7. Reconnect and verify selected profile value is preserved.
8. With external OSD device connected to OSD UART, verify screen updates (heartbeat + redraw).

#### Expected behavior

- OSD config read/write works through MSP (`MSP_OSD_CONFIG`, `MSP_SET_OSD_CONFIG`, `MSP_OSD_VIDEO_CONFIG`, `MSP_SET_OSD_VIDEO_CONFIG`).
- Runtime MSP DisplayPort frames are emitted on serial ports configured with function `65536` (frsky_osd).
- Profile selection is MSP/CLI compatible; current firmware uses compact profile mapping for stored layout data.

#### Troubleshooting

- OSD tab opens but no external display output:
	- Verify OSD UART is assigned `65536` and wired to the external OSD device.
	- Verify baud rate and TX/RX direction for the target OSD module.
- OSD settings do not persist:
	- Ensure Save was clicked in Configurator and `save` was executed in CLI when using CLI changes.
- No reconnect persistence in Configurator:
	- Recheck that the active port still has MSP enabled for Configurator access.

## GPS Setup

ESP-FC supports u-blox M8, M9, F9, and M10 GPS modules via UART. M10 modules provide enhanced accuracy with dual-band L1+L5 GNSS support.

### Hardware Connection

1. Connect GPS TX to ESP32 RX pin (e.g., UART2 RX)
2. Connect GPS RX to ESP32 TX pin (e.g., UART2 TX)
3. Connect VCC (3.3V or 5V depending on module) and GND

### Enable GPS Feature

In the `Configuration` tab, under `Other Features`, enable **GPS** or use CLI:

```
set feature_gps 1
save
```

### Configure Serial Port

In the `Ports` tab, Disable MSP function and enable GPS on the UART connected to your GPS module. Set baud rate to `115200` (recommended for M10) or `230400` for 25Hz update rate.

Alternatively, via CLI:

```
# For UART2 (common GPS port)
set serial_1 2 115200 115200
save
```

### Basic GPS Configuration

Go to CLI tab and configure basic GPS settings:

```
# Minimum satellites required for valid fix
set gps_min_sats 8

# Set home only once (0 = update home on each arm)
set gps_set_home_once 1

save
```

### M10 GNSS Configuration (Advanced)

M10 modules support multiple GNSS constellations and dual-band (L1+L5) for better accuracy. Configure via CLI:

#### Quick Preset Modes

```
# Mode 0: Auto (use individual constellation flags) - DEFAULT
set gps_gnss_mode 0

# Mode 1: GPS only (maximum compatibility, lowest power)
set gps_gnss_mode 1

# Mode 2: GPS + GLONASS (good for high latitudes)
set gps_gnss_mode 2

# Mode 3: GPS + Galileo (best accuracy in Europe)
set gps_gnss_mode 3

# Mode 4: GPS + BeiDou (optimized for Asia-Pacific)
set gps_gnss_mode 4

# Mode 5: All constellations (maximum satellites, best accuracy)
set gps_gnss_mode 5

save
reboot
```

> [!NOTE]
> Some modules are not capable to track all constellations at the same time. In this case module initialization may fail.

#### Dual-Band Configuration

```
# Enable L1+L5 dual-band on M9 (better multipath rejection)
set gps_enable_dual_band 1

# Disable dual-band (force L1 only for compatibility)
set gps_enable_dual_band 0

save
reboot
```

> [!NOTE]
> M8/M10 modules always use L1 single-band. The `gps_enable_dual_band` setting only affects M9 modules.
> If module do not support it, this option has no effect.

#### Individual Constellation Control

When `gps_gnss_mode 0`, you can enable/disable each constellation individually:

```
set gps_enable_gps 1         # GPS (USA)
set gps_enable_glonass 1     # GLONASS (Russia)
set gps_enable_galileo 1     # Galileo (EU)
set gps_enable_beidou 1      # BeiDou (China)
set gps_enable_qzss 1        # QZSS (Japan/Asia-Pacific)
set gps_enable_sbas 1        # SBAS/WAAS/EGNOS augmentation

save
reboot
```

### Configuration Examples

#### Maximum Accuracy (M10 Recommended)
```
set gps_gnss_mode 5              # All constellations
set gps_enable_dual_band 1       # L1+L5 dual-band
save
reboot
```
**Result:** GPS+GLONASS+Galileo+BeiDou+QZSS+SBAS with L1+L5  
**Power:** High

#### Balanced Performance (M10)
```
set gps_gnss_mode 3              # GPS + Galileo
set gps_enable_dual_band 1       # L1+L5 for multipath rejection
save
reboot
```
**Result:** GPS+Galileo with L1+L5  
**Power:** Medium

#### Battery Saver (Any Module)
```
set gps_gnss_mode 1              # GPS only
set gps_enable_dual_band 0       # L1 only
save
reboot
```
**Result:** GPS only, L1 band  
**Power:** Low

#### Urban/City Flying (M10)
```
set gps_gnss_mode 3              # GPS + Galileo
set gps_enable_dual_band 1       # L5 rejects building reflections
save
reboot
```
**Result:** GPS+Galileo with L1+L5  
**Power:** Medium

#### Asia-Pacific Optimized (M10)
```
set gps_gnss_mode 0              # Custom mode
set gps_enable_gps 1
set gps_enable_galileo 1
set gps_enable_beidou 1          # Strong in Asia
set gps_enable_qzss 1            # Regional augmentation
set gps_enable_glonass 0         # Disable to save power
set gps_enable_dual_band 1
save
reboot
```
**Result:** GPS+Galileo+BeiDou+QZSS with L1+L5  
**Power:** Medium

### Verification

After rebooting, check GPS status in CLI:

```
# View current GPS configuration
get gps

# Check GPS status (in another CLI session or via status command)
status
```

Look for initialization messages in the boot log:

```
GPS DET 115200               # Baud rate detected
GPS VER: 000A0000            # M10 module detected
GPS GNSS L1+L5 [GPS GLO GAL BDS QZSS SBAS]  # Configuration applied
GPS RATE 40/1                # 25Hz update rate (M10 at 230400 baud)
GPS UBX                      # UBX protocol enabled
GPS NAV5                     # Navigation mode: Airborne
```

### Troubleshooting

#### No GPS Fix
1. Check antenna has clear sky view (away from carbon fiber, metal)
2. Enable more constellations: `set gps_gnss_mode 5`
3. Wait 2-3 minutes for initial fix (TTFF - Time To First Fix)
4. Verify baud rate matches GPS module (115200 or 230400)

#### GPS Not Detected
1. Check wiring (TX/RX crossed between GPS and FC)
2. Verify serial port configuration in `Ports` tab
3. Try different baud rates (9600, 38400, 57600, 115200, 230400)
4. Check boot logs for `GPS ERROR` or `GPS DET` messages

#### Low Satellite Count
1. Move to open area with clear sky view
2. Enable all constellations: `set gps_gnss_mode 5`
3. Check antenna connection
4. Wait longer for satellites to be acquired

#### M10 Not Using Dual-Band
1. Verify module is actually M10 (check boot log: `GPS VER: 000A0000`)
2. Ensure `gps_enable_dual_band 1` is set
3. Check log for `GPS GNSS L1+L5` (not just `L1`)
4. Some M10 modules need firmware update for L5 support

#### Slow Fix Time

1. Enable more constellations: `set gps_gnss_mode 5`
2. Ensure SBAS is enabled: `set gps_enable_sbas 1`
3. Check for clear sky view (no buildings, trees blocking)
4. First fix always takes longer (cold start), subsequent fixes are faster
