# ESP-FC CLI Reference

## CLI vs GUI (Betaflight Configurator)

**Most features are configurable via Betaflight Configurator GUI** (see [Board Reference — Configuration Methods](boards.md#configuration-methods-gui-vs-cli)). The CLI is used for:
- **Advanced configuration**: Pin remapping, sensor offset, motor poles (DShot telemetry)
- **Preset tuning**: Landing assist, altitude fusion, receiver smoothing
- **Diagnostics**: Real-time CPU load, memory usage, FreeRTOS task monitoring
- **Backup/Export**: `dump` exports all settings as CLI commands
- **Board-specific setup**: WiFi config, rangefinder, optical flow

**To access CLI**: Open Betaflight Configurator → **CLI tab** (or press Ctrl+L) → Type commands below.

---

## Help
```
help
available commands:
 help
 dump
 get param
 set param value ...
 cal [gyro]
 defaults
 save
 reboot
 scaler
 mixer
 stats
 status
 mode_debug
 version
```

## Command Function Index

This index documents what each top-level CLI function does.

| Command | Function | Typical Use |
|---|---|---|
| `help` | List supported top-level commands | Discover CLI capabilities on current firmware |
| `version` | Print build version, API, compiler metadata | Confirm target/build before setup or debugging |
| `status` | Show runtime rates, active sensors, arming flags, uptime | Live health/status checks |
| `stats` | Show timing and CPU load per subsystem | Performance tuning and loop load validation |
| `mode_debug` | Show AUX condition evaluation and active mode masks | Verify ARM/ANGLE/AIRMODE switch logic |
| `dump` | Export full current configuration as CLI commands | Backup and migration |
| `get` | Read one setting or list matching settings | Inspect current values before changing |
| `set` | Write one or more configuration values | Board setup, tuning, remapping |
| `cal` | Run calibration (for example gyro calibration) | Bench calibration after wiring changes |
| `scaler` | Configure input/output scaling profiles | Advanced receiver/output shaping |
| `mixer` | Configure mixer/output model | Custom output and mixer behavior |
| `defaults` | Reset configuration to board defaults | Recover from invalid configuration |
| `save` | Persist current config to storage | Commit changes before reboot |
| `reboot` | Reboot board and reload saved config | Apply saved settings |

All commands are available across supported targets, but effective behavior depends on board capabilities (serial ports, buses, wireless support). For board-specific limits, see [Board capability matrix](setup.md#board-capability-matrix).

## Informational

### Version

```
version
ESPF ESP32S3 v0.0.0 0000000 Mar 13 2025 23:16:36 api=1.43 gcc=8.4.0 std=201703
```

### Status
```
status
ESPF ESP32S3 v0.0.0 0000000 Mar 13 2025 23:16:36 api=1.43 gcc=8.4.0 std=201703
STATUS:
    cpu freq: 240 MHz
  gyro clock: 3200 Hz
   gyro rate: 1600 Hz
   loop rate: 1600 Hz
  mixer rate: 1600 Hz
  accel rate: 400 Hz
   baro rate: 0 Hz
    mag rate: 0 Hz

     devices: BMI160/I2C
       input: 43 Hz, 0.00 Hz, 0.14
   arm flags: FAILSAFE RX_FAILSAFE CLI MSP
 rescue mode: 0
      uptime: 6.4
```

    ### Mode Debug
    Use this to verify how AUX ranges and linked mode conditions are resolved at runtime.

    ```
    mode_debug
    MODE DEBUG:
     frameCount=12345
     channelsValid=1
     rxLoss=0 rxFailSafe=0
     configuredMask=0x0000008F
     switchMask(state)=0x00000005
     switchMask(resolved)=0x00000005
     activeMask=0x00000001
     armingDisabled=0x00000088
     idx id name ch val min max logic link range linked
     0 0 ARM 4 1998 1700 2100 0 0 1 1
     1 2 ANGLE 5 1000 1700 2100 0 0 0 0
    ```

     - `range`: direct AUX range match for the condition
     - `linked`: final condition result after applying `logic_mode` and `link_id`
     - `switchMask(resolved)`: mode bits requested by switches after condition logic
     - `activeMask`: final runtime modes after arming/sensor safety gates

### Statisticss

Ensure that total CPU usage doesn't exceed 60-70%
```
stats
ESPF ESP32S3 v0.0.0 0000000 Mar 13 2025 23:16:36 api=1.43 gcc=8.4.0 std=201703
    cpu freq: 240 MHz
  gyro clock: 3200 Hz
   gyro rate: 1600 Hz
   loop rate: 1600 Hz
  mixer rate: 1600 Hz
  accel rate: 400 Hz
   baro rate: 0 Hz
    mag rate: 0 Hz

gyro_r: 187us,  117us/i,  18.7%,  1600 Hz
gyro_f:  14us,    9us/i,   1.4%,  1600 Hz
 acc_r:  45us,  114us/i,   4.5%,   399 Hz
 acc_f:   2us,    5us/i,   0.2%,   399 Hz
 imu_p:   4us,   10us/i,   0.4%,   399 Hz
 pid_o:   2us,    1us/i,   0.2%,  1600 Hz
 pid_i:   2us,    1us/i,   0.2%,  1600 Hz
 mix_p:  11us,    7us/i,   1.1%,  1600 Hz
 mix_w:  39us,   24us/i,   3.9%,  1600 Hz
 mix_r:   3us,    2us/i,   0.3%,  1600 Hz
  rx_r:   7us,    9us/i,   0.7%,   800 Hz
  rx_s:   2us,    3us/i,   0.2%,   800 Hz
  rx_a:   0us,    1us/i,   0.0%,    51 Hz
serial:   6us,    6us/i,   0.6%,   962 Hz
  wifi:   1us,    2us/i,   0.1%,   318 Hz
   bat:   7us,   65us/i,   0.7%,   102 Hz
 cpu_0: 278us,  174us/i,  27.8%,  1600 Hz
 cpu_1: 108us,   54us/i,  10.8%,  1999 Hz
  TOTAL: 386us, 23.1%
```

## Configuration

 - **defaults** - restore defaults
 - **save** - persist configuration after changes
 - **reboot** - reboot board

> [!IMPORTANT]
> if you change any parameter, remember to apply `save` and `reboot`. Without that some changes might not take effect.

## Configuration parameters

 - **dump** - dump all parameters
 - **get parameter_name** - get paramter value, list matching patameters with current values
 - **set paramter value1 [value2] [value3]** - set parameter to new value (some parameters store multiple values, ex. input)

Many parameters have similar name and meaning like Betaflight.

### Configuration examples

Get single value
```
# get gyro_lpf_type
set gyro_lpf_type PT1
```

Get multiple values
```
# get output_
set mixer_output_limit 100
set output_motor_protocol DSHOT150
set output_motor_async 0
set output_motor_rate 480
set output_servo_rate 0
set output_min_command 1000
set output_min_throttle 1050
set output_max_throttle 2000
set output_dshot_idle 450
set output_0 M N 1000 1500 2000
set output_1 M N 1000 1500 2000
set output_2 M N 1000 1500 2000
set output_3 M N 1000 1500 2000
set pin_output_0 0
set pin_output_1 14
set pin_output_2 12
set pin_output_3 15
```

Setting single value
```
set pin_output_0 0
```
> [!NOTE]
> You don't need to use `=` character for assignment

Setting milti-argument values
```
set output_0 S R
```
Note that you don't have to enter all values, you can ommit last values if they are not changed.

## Specific parameters reference

### Pin functions

For complete pin assignments for all boards (ESP32, ESP32-S2, ESP32-S3, ESP32-C3, ESP8266, RP2040, RP2350, STM32F7, STM32H7), see [Board Reference — Pin Assignments](boards.md).

Default pin assignments for ESP32-S3 is listed below
```
set pin_input_rx 6
set pin_output_0 39
set pin_output_1 40
set pin_output_2 41
set pin_output_3 42
set pin_button -1
set pin_buzzer 5
set pin_led -1
set pin_serial_0_tx 43
set pin_serial_0_rx 44
set pin_serial_1_tx 16
set pin_serial_1_rx 15
set pin_serial_2_tx 18
set pin_serial_2_rx 17
set pin_i2c_scl 10
set pin_i2c_sda 9
set pin_input_adc_0 1
set pin_input_adc_1 4
set pin_spi_0_sck 12
set pin_spi_0_mosi 11
set pin_spi_0_miso 13
set pin_spi_cs_0 8
set pin_spi_cs_1 7
set pin_spi_cs_2 -1
set pin_buzzer_invert 1
set pin_led_invert 0
```

You can swap two motor outputs (example 0 and 3) by running commands

```
set pin_output_0 42
set pin_output_3 39
save
reboot
```

### Input channel config
```
set input_0 0 1000 1500 2000 A 1500
set input_{channel} {map} {min} {neutral} {max} {fs_mode} {fs_value}
```
 - `{channel}`: raw rc channel (0-index)
 - `{map}`: map raw channel to input channel (0-index)
 - `{min/neutral/max}`: same as `rxrange` to adjust radio endpoints, scale rx input range to 1000/1500/2000, enter your min/mid/max values transmitted by radio.
 - `{fs_mode}`: fail safe mode, can be A: auto, H: hold, S: set
 - `{fs_value}`: value to use when fsMode is S(set)

### Output channel config
```
set output_0 M N 1000 1500 2000
set output_{channel} {type} {inverse} {min} {neutral} {max}
```
 - `{channel}`: output channel (0-index)
 - `{type}`: output type, can be M: motor, S: servo
 - `{inverse}`: inverse output, can be set to N: normal, R: reversed
 - `{min/neutral/max}`: limit/adjust output

### Mode conditions
```
set mode_{index} {id} {channel} {min} {max} {logic_mode} {link_id}
set mode_0 0 4 1300 2100 0 0
```
 - `{index}`: mode activator index
 - `{id}`: mode Id
 - `{channel}`: observed input channel (0-index, must be greather than 3)
 - `{min/max}`: activation range, if both are set to 900, it means inactive
 - `{logic_mode}`: condition logic mode (`0` = direct range, `1` = range AND linked condition active)
 - `{link_id}`: referenced condition index used when `logic_mode` is `1`

Mode IDs:
 - 0: ARM
 - 1: AIRMODE
 - 2: ANGLE
 - 3: HORIZON
 - 4: ALTHOLD
 - 5: BEEPER
 - 6: FAILSAFE
 - 7: BLACKBOX
 - 8: BLACKBOX_ERASE
 - 9: MAG
 - 10: HEADFREE
 - 11: HEADADJ
 - 12: CALIB
 - 13: GPS_RESCUE
 - 14: PREARM
 - 15: FLIP_OVER_AFTER_CRASH
 - 16: USER1
 - 17: USER2
 - 18: USER3
 - 19: USER4
 - 20: ACRO_TRAINER
 - 21: LAUNCH_CONTROL
 - 22: MSP_OVERRIDE
 - 23: STICK_COMMANDS_DISABLE
 - 24: BEEPER_MUTE
 - 25: PARALYZE

### Advanced modes and CLI-only tuning

This section documents advanced behavior that is not fully configurable from standard Betaflight tabs.

| Mode / Feature | Activation | Betaflight GUI | CLI / protocol parameters |
|---|---|---|---|
| Altitude Hold (`MODE_ALTHOLD`, id `4`) | Assign with `mode_{index}` to an AUX channel range | Partially visible depending configurator mapping; CLI is the reliable path | `mode_*`, `pid_althold_vel_p`, `pid_althold_vel_i`, `pid_althold_vel_d`, `pid_althold_vel_f`, `pid_althold_iterm_center`, `pid_althold_iterm_range`, `alt_fuse_*`, `range_bottom_*`, `opflow_*` |
| Landing Assist (automatic touchdown logic) | Automatic while armed when landing intent is detected (no dedicated mode ID) | Not directly configurable in standard BF tabs | `landing_assist_enabled`, `landing_assist_thr_margin`, `landing_assist_desc_rate`, `landing_assist_desc_gain`, `landing_assist_desc_max`, `landing_assist_baro_h`, `landing_assist_baro_v`, `landing_assist_gps_down`, `landing_assist_gps_ground`, `landing_assist_flow_q`, `landing_assist_flow_hand_q`, `landing_assist_flow_rate`, `landing_assist_flow_hand_rate`, `landing_assist_hand_vario`, `landing_assist_hand_h_min`, `landing_assist_hand_h_max`, `landing_assist_hold_ms`, `landing_assist_ramp` |
| Obstacle Avoidance (front rangefinder assisted) | Automatic when enabled and front rangefinder is present (no dedicated mode ID) | Not exposed by standard BF tabs | No direct CLI key in current firmware. Configure via MSP2 custom messages: `MSP2_ESPFC_OBSTACLE_AVOIDANCE_CONFIG (0x1F0D)` / `MSP2_ESPFC_SET_OBSTACLE_AVOIDANCE_CONFIG (0x1F0E)`. Related sensor setup in CLI: `range_front_*`, `opflow_*`. |

Example: assign Altitude Hold to AUX2 high range:

```bash
set mode_3 4 5 1700 2100 0 0
save
reboot
```

Landing assist and altitude-fusion presets are available from CLI:

```bash
preset landing_indoor
# or
preset landing_outdoor
# or
preset landing_freestyle
save
reboot
```

### Copy-paste profiles (advanced modes)

Use these quick profiles as starting points.

#### Indoor hand-catch landing profile

```bash
preset landing_indoor
set landing_assist_enabled 1
set mode_3 4 5 1700 2100 0 0
save
reboot
```

#### Outdoor landing profile

```bash
preset landing_outdoor
set landing_assist_enabled 1
set mode_3 4 5 1700 2100 0 0
save
reboot
```

#### Freestyle quick touchdown profile

```bash
preset landing_freestyle
set landing_assist_enabled 1
save
reboot
```

#### Front obstacle sensor setup profile (CLI + MSP2)

```bash
# Front rangefinder device setup via CLI
set range_front_bus I2C
set range_front_dev VL53L0X
set range_front_addr 41
set range_front_enabled 1

# Optional bottom rangefinder for altitude context
set range_bottom_bus I2C
set range_bottom_dev VL53L0X
set range_bottom_addr 41
set range_bottom_enabled 1

save
reboot
```

After this, apply obstacle behavior settings via MSP2 custom config messages:
- `MSP2_ESPFC_OBSTACLE_AVOIDANCE_CONFIG (0x1F0D)`
- `MSP2_ESPFC_SET_OBSTACLE_AVOIDANCE_CONFIG (0x1F0E)`

#### Altitude-hold tuning starter profile

```bash
set pid_althold_vel_p 80
set pid_althold_vel_i 60
set pid_althold_vel_d 40
set pid_althold_vel_f 20
set pid_althold_iterm_center 50
set pid_althold_iterm_range 50
set alt_fuse_baro_h_w 65
set alt_fuse_baro_v_w 70
set alt_fuse_range_h_w 95
set alt_fuse_flow_v_w 20
save
reboot
```

### Serial port configuration
```
set serial_0 1 115200 0
set serial_{index} {function} {msp_baud} {blackbox_baud}
```
 - `{index}`: port number (0-index)
 - `{function}`: function id
 - `{msp_baud}`: msp baud rate
 - `{blackbox_baud}`: blackbox baud rate (unused)

Serial Functions:
 - 0: none
 - 1: msp
 - 128: blackbox
 - 65536: frsky_osd (external OSD)

For per-board port and capability limits, see [Board capability matrix](setup.md#board-capability-matrix).

### OLED configuration
```
set oled_bus AUTO
set oled_dev SSD1306
set oled_height 32
set oled_page_ms 3000
set oled_startup_ms 1500
```

 - `oled_height`: display height, use `0` for auto or fixed `32`/`64`
 - `oled_page_ms`: telemetry page auto-scroll interval in milliseconds (`500` to `30000`)
 - `oled_startup_ms`: startup info page duration in milliseconds (`0` disables startup page, max `10000`)

### External OSD configuration
```
set osd_enabled 1
set osd_msp_displayport 1
set osd_video_system 2
set osd_profiles 3
set osd_profile 1
set osd_units 0
set osd_rssi_alarm 20
```

 - `osd_enabled`: global OSD feature bit for MSP OSD config (`0`/`1`)
 - `osd_msp_displayport`: report MSP displayport-style external OSD support (`0`/`1`)
 - `osd_video_system`: `0=AUTO`, `1=PAL`, `2=NTSC`, `3=HD`
 - `osd_profiles`: number of available OSD profiles (`1` to `3`)
 - `osd_profile`: active OSD profile (`1` to `osd_profiles`)
 - current firmware uses compact profile mapping: profile selection is fully MSP/CLI compatible while sharing one stored element layout bank to keep EEPROM usage within target limits
 - `osd_units`: `0=metric`, `1=imperial`
 - `osd_rssi_alarm`: RSSI warning threshold

For wiring, Betaflight Configurator validation, and troubleshooting steps, see [External OSD checklist](setup.md#osd).

### Wireless OTA configuration
```
set wifi_ssid
set wifi_pass
set wifi_tcp_port 1111
set wifi_ota 1
set wifi_ota_port 3232
set wifi_ota_pass
set bt_ota 0
```

 - `wifi_ota`: enable WiFi OTA flashing (`0`/`1`)
 - `wifi_ota_port`: OTA service port (`1` to `65535`, default `3232`)
 - `wifi_ota_pass`: optional OTA password
 - `bt_ota`: enable Bluetooth OTA stream mode (`0`/`1`, classic ESP32 builds with `ESPFC_BT_OTA` only)

Use [Board capability matrix](setup.md#board-capability-matrix) to verify whether WiFi OTA / BT OTA is available on your target.

### Buzzer and beeper modes

ESP-FC now follows Betaflight-style beeper mode sequencing and priorities.

- GPIO polarity is controlled by `pin_buzzer_invert`
- Beeper mode mask follows the Betaflight rule: `bit = 1 << (mode_id - 1)`
- Mode `0` is silence and has no bit
- `BUZZER_ALL` and `BUZZER_PREFERENCE` are control IDs and are not part of the allowed beeper mask

Mode IDs:
- 1: `GYRO_CALIBRATED`
- 2: `RX_LOST`
- 3: `RX_LOST_LANDING`
- 4: `DISARMING`
- 5: `ARMING`
- 6: `ARMING_GPS_FIX`
- 7: `BAT_CRIT_LOW`
- 8: `BAT_LOW`
- 9: `GPS_STATUS`
- 10: `RX_SET`
- 11: `ACC_CALIBRATION`
- 12: `ACC_CALIBRATION_FAIL`
- 13: `READY_BEEP`
- 14: `MULTI_BEEPS`
- 15: `DISARM_REPEAT`
- 16: `ARMED`
- 17: `SYSTEM_INIT`
- 18: `USB`
- 19: `BLACKBOX_ERASE`
- 20: `CRASH_FLIP_MODE`
- 21: `CAM_CONNECTION_OPEN`
- 22: `CAM_CONNECTION_CLOSE`
- 23: `ARMING_GPS_NO_FIX`
- 24: `ALL` (control ID)
- 25: `PREFERENCE` (control ID)

Common examples:
- `RX_SET` bit: `1 << (10 - 1)` = `0x00000200`
- `ARMING_GPS_NO_FIX` bit: `1 << (23 - 1)` = `0x00400000`

### Scaler configuration

It's diferrent version of inflight adjustments, allow to control few parameters with rc channel, ex. by potentiomenters.

```
set scaler_0 769 5 20 400
set scaler_{index} {dimensions} {channel} {min_scale} {max_scale}
```

 - `{index}`: scaler instance (0-3)
 - `{dimensions}`: combination of dimentions
 - `{channel}`: observed iput channel (0-index, must be greather than 3)
 - `{min_scale/max_scale}`: min and max scale in percent

Scaler Dimensions:
 - 1: PID P component of inner loop
 - 2: PID I component of inner loop
 - 4: PID D component of inner loop
 - 8: PID F component of inner loop
 - 16: PID P component of outer loop
 - 32: PID I component of outer loop
 - 64: PID D component of outer loop
 - 128: PID F component of outer loop
 - 256: Roll Axis
 - 512: Pitch Axis
 - 1024: Yaw Axis

**Note 1**: To apply scaling you need to combine at least one PID component and Axis. You can also control multiple parameters by single channels. For example to control inner P and D on Pitch and Roll axes you need to use value 771 as dimentions (1 + 2 + 256 + 521),

**Note 2**: the scale is not uniform, in neutral position it is always 100 percent to represent original config value, if minimum is negative, then neutral position represents 0.

**Note 3**: inner loop is main loop for acro mode, outer loop refers to level mode.

### Mixer

There are few predefined mixers that can be used out-of-the box. You can choose it by setting paramter `mixer_type`

```
set mixer_type QUADX
```

Currently allowed are only:
 - QUADX (default)
 - QUAD1234 (untested)
 - TRI (untested)

You can debug mixer configuration with `mixer` command

```
# mixer
set mix_outputs 4
set mix_0 1 0 -100
set mix_1 1 1 -100
set mix_2 1 2 100
set mix_3 1 3 100
set mix_4 2 0 100
set mix_5 2 1 -100
set mix_6 2 2 100
set mix_7 2 3 -100
set mix_8 3 0 -100
set mix_9 3 1 100
set mix_10 3 2 100
set mix_11 3 3 -100
set mix_12 4 0 100
set mix_13 4 1 100
set mix_14 4 2 100
set mix_15 4 3 100
set mix_16 0 0 0
```

### Custom Mixer

To activate custom mixer you need to set mixer type to custom and declare number of outputs.

```
set mixer_type CUSTOM
set mix_outputs 4
```

Then apply your custom rules
```
set mix_0 0 0 0
set mix_{index} {src} {dst} {rate}
```
 - `{index}`: rule index
 - `{src}`: mixer source channel
 - `{dst}`: output channel (0-index)
 - `{rate}`: mix rate in percent

> [!IMPORTANT]
> You need to add a termination rule at the end, to inform mixer to stop processing. In this case set `src` argument to 0. When mixer encounters such a rule, it stops processing and ignore next rules. You can add maximum 64 rules.

Mixer Sources:
 - 0: null for termination rule
 - 1: stabilized roll
 - 2: stabilized pitch
 - 3: stabilized yaw
 - 4: thrust
 - 5: rc roll
 - 6: rc pitch
 - 7: rc yaw
 - 8: rc thrust
 - 9: rc aux 1
 - 10: rc aux 2
 - 11: rc aux 3

## Parameter Family Appendix (All Functions)

This appendix groups all configuration parameters by prefix so you can quickly find what to `get` / `set`.

### Core / Feature Flags
- Prefixes: `feature_`, `debug_`, `model_name`
- Includes: feature toggles (GPS, dynamic notch, motor stop, receiver paths), debug stream selection.
- Quick query: `get feature_`

### Sensors and Fusion
- Prefixes: `gyro_`, `accel_`, `mag_`, `baro_`, `fusion_`, `board_align_`
- Includes: sensor bus/device selection, filter chain, offsets, dynamic notch/RPM filter, AHRS fusion gains.
- Quick query: `get gyro_`

### RC Input and Failsafe
- Prefixes: `input_`, `failsafe_`, `mode_`, `scaler_`
- Includes: channel mapping, interpolation/filtering, per-channel failover behavior, AUX mode conditions, inflight scaling.
- Quick query: `get input_`

### PID and Flight Control
- Prefixes: `pid_`
- Includes: roll/pitch/yaw gains, level mode gains, d-term filters, I-term behavior, TPA.
- Quick query: `get pid_`

### Mixer and Outputs
- Prefixes: `mixer_`, `output_`, `mix_`, `mix_outputs`
- Includes: mixer type, custom mix rules, motor protocol/rate, throttle limits, per-output min/neutral/max.
- Quick query: `get output_`

### Pins and Hardware Mapping
- Prefixes: `pin_`, `i2c_speed`
- Includes: motor pins, serial pins, SPI/I2C pins, ADC pins, buzzer/LED polarity.
- Quick query: `get pin_`

### Serial, Telemetry, VTX, GPS
- Prefixes: `serial_`, `telemetry_`, `gps_`, `vtx_`
- Includes: serial role assignment, telemetry timing, GPS behavior, VTX band/channel/power.
- Quick query: `get serial_`

### Power and Battery
- Prefixes: `vbat_`, `ibat_`
- Includes: voltage/current source selection, scaling, offsets, warning thresholds.
- Quick query: `get vbat_`

### OSD, Display, and UI Devices
- Prefixes: `osd_`, `oled_`, `buzzer`/beeper-related config
- Includes: OSD profile/system/units/alarm, OLED bus/device/page timing, beeper mode behavior.
- Quick query: `get osd_`

### Blackbox and Logging
- Prefixes: `blackbox_`
- Includes: logging backend, rate, mode, per-signal logging switches.
- Quick query: `get blackbox_`

### Wireless / OTA
- Prefixes: `wifi_`, `bt_ota`
- Includes: WiFi SSID/password, TCP bridge port, OTA enable/port/password, BT OTA switch.
- Quick query: `get wifi_`

### Rescue / Misc Runtime
- Prefixes: `rescue_`
- Includes: rescue activation delay and related safety timing.
- Quick query: `get rescue_`

### Practical pattern-based lookup
Use these commands to list complete parameter families directly from firmware:

```bash
get feature_
get gyro_
get accel_
get mag_
get baro_
get fusion_
get input_
get failsafe_
get mode_
get scaler_
get pid_
get mixer_
get output_
get mix_
get pin_
get serial_
get telemetry_
get gps_
get vtx_
get vbat_
get ibat_
get osd_
get oled_
get blackbox_
get wifi_
get bt_ota
get rescue_
```

Tip: after changing values, always run `save` and `reboot`.

## Safe Defaults by Board Family

These presets are intentionally conservative and focus on reliable first-flight behavior. Apply the profile that matches your target, then tune from there.

### ESP32 Family (ESP32, ESP32-S2, ESP32-S3, ESP32-C3)

Recommended for a stable initial setup with DShot and moderate filtering:

```bash
set feature_motor_stop 1
set feature_rx_serial 1
set output_motor_protocol DSHOT300
set output_motor_poles 14
set output_dshot_idle 550
set gyro_lpf_type PT1
set gyro_lpf_freq 100
set gyro_lpf2_type PT1
set gyro_lpf2_freq 213
set pid_dterm_lpf_type PT1
set pid_dterm_lpf_freq 128
set input_rate_type ACTUAL
set input_roll_rate 20
set input_pitch_rate 20
set input_yaw_rate 30
set failsafe_delay 4
save
reboot
```

Optional on ESP32/ESP32-S3/ESP32-C3:

```bash
set wifi_ota 1
save
reboot
```

ESP32-S2 note: keep soft-serial WiFi disabled due to RAM limits.

### RP2040 / RP2350 / RP2350B Family

Recommended for USB-first bring-up with balanced loop load:

```bash
set feature_motor_stop 1
set feature_rx_serial 1
set output_motor_protocol DSHOT300
set output_motor_poles 14
set output_dshot_idle 550
set gyro_lpf_type PT1
set gyro_lpf_freq 100
set pid_dterm_lpf_type PT1
set pid_dterm_lpf_freq 128
set input_interpolation AUTO
set input_filter_type FILTER
set failsafe_delay 4
set telemetry_interval 1000
save
reboot
```

RP2350B note: ARM and Hazard3 RISC-V targets share the same runtime defaults and CLI profile.

### STM32 Family (STM32F7, STM32H7, STM32H723, STM32H743VG)

Recommended for scaffold targets where serial resources are limited:

```bash
set feature_motor_stop 1
set feature_rx_serial 1
set serial_0 1 115200 0
set output_motor_protocol DSHOT300
set output_motor_poles 14
set gyro_lpf_type PT1
set gyro_lpf_freq 100
set pid_dterm_lpf_type PT1
set pid_dterm_lpf_freq 128
set failsafe_delay 4
save
reboot
```

STM32 note: assign RX/GPS/VTX serial roles manually after confirming available UART mappings on your board.

### Battery and Arming Safety Baseline (All Boards)

Apply these if you use analog battery monitoring:

```bash
set vbat_source 1
set vbat_cell_warn 350
set ibat_source 1
save
reboot
```

If your target does not have current-sense ADC wired, set `ibat_source 0`.

## All supported parameters

Use `dump` command to backup all your settings. You can then load you configuration in CLI tab. Below you can find full list of parameters.

```
set feature_gps 0
set feature_dyn_notch 0
set feature_motor_stop 0
set feature_rx_ppm 0
set feature_rx_serial 1
set feature_rx_spi 0
set feature_soft_serial 0
set feature_telemetry 0
set debug_mode NONE
set debug_axis 1
set gyro_bus AUTO
set gyro_dev AUTO
set gyro_dlpf 256Hz
set gyro_align DEFAULT
set gyro_lpf_type PT1
set gyro_lpf_freq 100
set gyro_lpf2_type PT1
set gyro_lpf2_freq 213
set gyro_lpf3_type FO
set gyro_lpf3_freq 150
set gyro_notch1_freq 0
set gyro_notch1_cutoff 0
set gyro_notch2_freq 0
set gyro_notch2_cutoff 0
set gyro_dyn_lpf_min 170
set gyro_dyn_lpf_max 425
set gyro_dyn_notch_q 300
set gyro_dyn_notch_count 4
set gyro_dyn_notch_min 80
set gyro_dyn_notch_max 400
set gyro_rpm_harmonics 3
set gyro_rpm_q 500
set gyro_rpm_min_freq 100
set gyro_rpm_fade 30
set gyro_rpm_weight_1 100
set gyro_rpm_weight_2 100
set gyro_rpm_weight_3 100
set gyro_rpm_tlm_lpf_freq 150
set gyro_offset_x -6
set gyro_offset_y -1
set gyro_offset_z 8
set accel_bus AUTO
set accel_dev AUTO
set accel_lpf_type BIQUAD
set accel_lpf_freq 15
set accel_offset_x 0
set accel_offset_y 0
set accel_offset_z 0
set mag_bus AUTO
set mag_dev NONE
set mag_align DEFAULT
set mag_filter_type BIQUAD
set mag_filter_lpf 10
set mag_offset_x 0
set mag_offset_y 0
set mag_offset_z 0
set mag_scale_x 1000
set mag_scale_y 1000
set mag_scale_z 1000
set baro_bus AUTO
set baro_dev NONE
set baro_lpf_type BIQUAD
set baro_lpf_freq 3
set gps_min_sats 8
set gps_set_home_once 1
set board_align_roll 0
set board_align_pitch 0
set board_align_yaw 0
set vbat_source NONE
set vbat_scale 100
set vbat_mul 1
set vbat_div 10
set vbat_cell_warn 350
set ibat_source NONE
set ibat_scale 100
set ibat_offset 0
set fusion_mode MAHONY
set fusion_gain_p 50
set fusion_gain_i 5
set input_rate_type ACTUAL
set input_roll_rate 20
set input_roll_srate 40
set input_roll_expo 0
set input_roll_limit 1998
set input_pitch_rate 20
set input_pitch_srate 40
set input_pitch_expo 0
set input_pitch_limit 1998
set input_yaw_rate 30
set input_yaw_srate 36
set input_yaw_expo 0
set input_yaw_limit 1998
set input_deadband 3
set input_min 885
set input_mid 1500
set input_max 2115
set input_interpolation AUTO
set input_interpolation_interval 26
set input_filter_type FILTER
set input_lpf_type PT3
set input_lpf_freq 0
set input_lpf_factor 50
set input_ff_lpf_type PT3
set input_ff_lpf_freq 0
set input_rssi_channel 0
set input_0 0 1000 1500 2000 A 1500
set input_1 1 1000 1500 2000 A 1500
set input_2 3 1000 1500 2000 A 1500
set input_3 2 1000 1500 2000 A 1000
set input_4 4 1000 1500 2000 H 1500
set input_5 5 1000 1500 2000 H 1500
set input_6 6 1000 1500 2000 H 1500
set input_7 7 1000 1500 2000 H 1500
set input_8 8 1000 1500 2000 H 1500
set input_9 9 1000 1500 2000 H 1500
set input_10 10 1000 1500 2000 H 1500
set input_11 11 1000 1500 2000 H 1500
set input_12 12 1000 1500 2000 H 1500
set input_13 13 1000 1500 2000 H 1500
set input_14 14 1000 1500 2000 H 1500
set input_15 15 1000 1500 2000 H 1500
set failsafe_delay 4
set failsafe_kill_switch 0
set vtx_power 0
set vtx_channel 8
set vtx_band 1
set vtx_low_power_disarm 0
set serial_0 1 115200 0
set serial_1 1 115200 0
set serial_2 64 115200 0
set serial_soft_0 1 115200 0
set serial_usb 1 115200 0
set scaler_0 0 0 20 400
set scaler_1 0 0 20 400
set scaler_2 0 0 20 400
set mode_0 0 4 900 900 0 0
set mode_1 0 4 900 900 0 0
set mode_2 0 4 900 900 0 0
set mode_3 0 4 900 900 0 0
set mode_4 0 4 900 900 0 0
set mode_5 0 4 900 900 0 0
set mode_6 0 4 900 900 0 0
set mode_7 0 4 900 900 0 0
set pid_sync 1
set pid_roll_p 42
set pid_roll_i 85
set pid_roll_d 24
set pid_roll_f 72
set pid_pitch_p 46
set pid_pitch_i 90
set pid_pitch_d 26
set pid_pitch_f 76
set pid_yaw_p 45
set pid_yaw_i 90
set pid_yaw_d 0
set pid_yaw_f 72
set pid_level_p 45
set pid_level_i 0
set pid_level_d 0
set pid_level_angle_limit 55
set pid_level_rate_limit 300
set pid_level_lpf_type PT1
set pid_level_lpf_freq 90
set pid_yaw_lpf_type PT1
set pid_yaw_lpf_freq 90
set pid_dterm_lpf_type PT1
set pid_dterm_lpf_freq 128
set pid_dterm_lpf2_type PT1
set pid_dterm_lpf2_freq 128
set pid_dterm_notch_freq 0
set pid_dterm_notch_cutoff 0
set pid_dterm_dyn_lpf_min 60
set pid_dterm_dyn_lpf_max 145
set pid_dterm_weight 30
set pid_iterm_limit 30
set pid_iterm_zero 1
set pid_iterm_relax RP
set pid_iterm_relax_cutoff 15
set pid_tpa_scale 10
set pid_tpa_breakpoint 1650
set mixer_sync 1
set mixer_type QUADX
set mixer_yaw_reverse 0
set mixer_throttle_limit_type NONE
set mixer_throttle_limit_percent 100
set mixer_output_limit 100
set output_motor_protocol DSHOT300
set output_motor_async 0
set output_motor_rate 480
set output_motor_poles 14
set output_servo_rate 0
set output_min_command 1000
set output_min_throttle 1070
set output_max_throttle 2000
set output_dshot_idle 550
set output_dshot_telemetry 0
set output_0 M N 1000 1500 2000
set output_1 M N 1000 1500 2000
set output_2 M N 1000 1500 2000
set output_3 M N 1000 1500 2000
set pin_input_rx 6
set pin_output_0 39
set pin_output_1 40
set pin_output_2 41
set pin_output_3 42
set pin_button -1
set pin_buzzer 5
set pin_led -1
set pin_serial_0_tx 43
set pin_serial_0_rx 44
set pin_serial_1_tx 16
set pin_serial_1_rx 15
set pin_serial_2_tx 18
set pin_serial_2_rx 17
set pin_i2c_scl 13
set pin_i2c_sda 12
set pin_input_adc_0 1
set pin_input_adc_1 4
set pin_spi_0_sck -1
set pin_spi_0_mosi -1
set pin_spi_0_miso -1
set pin_spi_cs_0 8
set pin_spi_cs_1 7
set pin_spi_cs_2 -1
set pin_buzzer_invert 1
set pin_led_invert 0
set i2c_speed 800
set rescue_config_delay 30
set telemetry_interval 1000
set blackbox_dev NONE
set blackbox_mode NORMAL
set blackbox_rate 32
set blackbox_log_acc 1
set blackbox_log_alt 1
set blackbox_log_bat 1
set blackbox_log_debug 1
set blackbox_log_gps 1
set blackbox_log_gyro 1
set blackbox_log_gyro_raw 1
set blackbox_log_mag 1
set blackbox_log_motor 1
set blackbox_log_pid 1
set blackbox_log_rc 1
set blackbox_log_rpm 1
set blackbox_log_rssi 1
set blackbox_log_sp 1
set model_name
set wifi_ssid
set wifi_pass
set wifi_tcp_port 1111
set wifi_ota 1
set wifi_ota_port 3232
set wifi_ota_pass
set bt_ota 0
set mix_outputs 0
set mix_0 0 0 0
set mix_1 0 0 0
set mix_2 0 0 0
set mix_3 0 0 0
set mix_4 0 0 0
set mix_5 0 0 0
set mix_6 0 0 0
set mix_7 0 0 0
set mix_8 0 0 0
set mix_9 0 0 0
set mix_10 0 0 0
set mix_11 0 0 0
set mix_12 0 0 0
set mix_13 0 0 0
set mix_14 0 0 0
set mix_15 0 0 0
set mix_16 0 0 0
set mix_17 0 0 0
set mix_18 0 0 0
set mix_19 0 0 0
set mix_20 0 0 0
set mix_21 0 0 0
set mix_22 0 0 0
set mix_23 0 0 0
set mix_24 0 0 0
set mix_25 0 0 0
set mix_26 0 0 0
set mix_27 0 0 0
set mix_28 0 0 0
set mix_29 0 0 0
set mix_30 0 0 0
set mix_31 0 0 0
set mix_32 0 0 0
set mix_33 0 0 0
set mix_34 0 0 0
set mix_35 0 0 0
set mix_36 0 0 0
set mix_37 0 0 0
set mix_38 0 0 0
set mix_39 0 0 0
set mix_40 0 0 0
set mix_41 0 0 0
set mix_42 0 0 0
set mix_43 0 0 0
set mix_44 0 0 0
set mix_45 0 0 0
set mix_46 0 0 0
set mix_47 0 0 0
set mix_48 0 0 0
set mix_49 0 0 0
set mix_50 0 0 0
set mix_51 0 0 0
set mix_52 0 0 0
set mix_53 0 0 0
set mix_54 0 0 0
set mix_55 0 0 0
set mix_56 0 0 0
set mix_57 0 0 0
set mix_58 0 0 0
set mix_59 0 0 0
set mix_60 0 0 0
set mix_61 0 0 0
set mix_62 0 0 0
set mix_63 0 0 0
```