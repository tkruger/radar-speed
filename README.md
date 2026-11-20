# radar-speed

An Arduino radar speed sign: it reads vehicle speeds from an **HLK-LD2451** 24 GHz radar module and shows them on a **TM1637** 4-digit 7-segment display, like a roadside "YOUR SPEED" sign. A knob sets the speed limit, and the display blinks when a target goes over it.

## Features

- Shows the speed of the fastest detected target in **MPH**
- Speed limit from **5 to 120 MPH**, set with a potentiometer
- Blinks **slowly** at 1–4 MPH over the limit and **quickly** at 5+ MPH over
- Holds the last reading for 3 seconds after the target leaves
- Idle animation so you can tell the unit is powered and waiting
- Non-blocking main loop, plus debug output over USB serial

## Hardware

| Component | Notes |
|---|---|
| Arduino Mega 2560 | Needs a spare hardware UART (uses `Serial3`) |
| HLK-LD2451 radar module | 115200 baud UART output |
| TM1637 4-digit 7-segment display | |
| Potentiometer (e.g. 10 kΩ) | Or any 0–5 V analog control signal |

### Wiring

| Signal | Mega pin |
|---|---|
| Radar TX → | **15** (RX3) |
| Display CLK | **50** |
| Display DIO | **52** |
| Knob wiper | **A1** |

Connect power and ground for all modules (5 V and GND), and share a common ground between them.

> The Mega's analog inputs accept 0–5 V only. If your control signal can go higher, scale it down first (for example with a voltage divider).

## Software setup

1. Install the [Arduino IDE](https://www.arduino.cc/en/software) (or `arduino-cli`).
2. Install the **TM1637** library by Avishay Orpaz from the Library Manager. It provides `TM1637Display.h`.
3. Open `Radar_gun.ino`, select **Arduino Mega or Mega 2560** as the board, and upload.
4. Optional: open the Serial Monitor at **115200 baud** to see knob and radar debug messages.

## How it works

The display shows one of three things, in priority order:

1. **Set speed.** While you turn the knob, the selected limit is shown with leading zeros (e.g. `0035`). It disappears 2 seconds after the knob stops moving. It is also shown briefly at power-up.
2. **Radar speed.** When the radar reports a target, the fastest target's speed is shown (e.g. `  42`). It blinks if the speed is over the limit.
3. **Idle.** With no target, a single bottom segment steps across the four digits.

### Knob mapping

The knob voltage maps linearly onto the speed-limit range:

| Knob voltage | Speed limit |
|---|---|
| ≤ 0.1 V | 5 MPH |
| 0.1 – 4.2 V | linearly interpolated |
| ≥ 4.2 V | 120 MPH |

The knob reading is smoothed with a low-pass filter and a small deadband so the limit doesn't flicker.

### Radar protocol

The sketch parses LD2451 data frames byte by byte with a small state machine:

```
F4 F3 F2 F1 | len (2 bytes, little-endian) | payload | F8 F7 F6 F5
```

The payload starts with a target count and an alarm byte. After that, each target takes 5 bytes: angle, distance, direction, speed (km/h) and SNR. The sketch converts every target's speed to MPH and keeps the highest one.

## Configuration

All tunable values are constants at the top of [`Radar_gun.ino`](Radar_gun.ino):

| Constant | Default | Purpose |
|---|---|---|
| `DISPLAY_BRIGHTNESS` | `7` | Display brightness (0–7) |
| `SPEED_LIMIT_MIN_MPH` / `SPEED_LIMIT_MAX_MPH` | `5` / `120` | Range of the speed-limit knob |
| `KNOBW_LOW_VOLTAGE_MPH_MIN` / `KNOBW_HIGH_VOLTAGE_MPH_MAX` | `0.1` / `4.2` V | Knob voltage calibration |
| `KNOB_SETTLE_HIDE_MS` | `2000` | How long the set speed stays visible after adjusting |
| `KNOB_FILTER_ALPHA` | `0.05` | Knob smoothing (higher = more responsive) |
| `KNOB_NOISE_DEADBAND_RAW` | `7` | Knob jitter deadband (ADC counts) |
| `RADAR_HOLD_AFTER_LOST_MS` | `3000` | How long to hold the last reading |
| `OVER_LIMIT_*` | 1–4 / 5+ | Over-limit thresholds for slow and fast blinking |
| `SLOW_BLINK_*`, `FAST_BLINK_*` | 500/250, 200/100 ms | Blink on/off timing |
| `RUN_INDICATOR_INTERVAL_MS` | `1500` | Idle animation step time |

Pin assignments are also at the top of the file. The radar uses the Mega's `Serial3`; to use a different UART, change the `#define RadarSerial` line.

## License

Released under the [GNU General Public License v3.0](LICENSE).
