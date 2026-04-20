# nirlabs Scope Level

An ESP32-based anti-cant level for rifle scopes. Uses an ADXL345 accelerometer to detect tilt and provides real-time LED feedback so you know when your rifle is level before taking a shot.

## Features

- **Three-zone indication** — Level (green), Near-level (green fade + red flash), and Cant (red heartbeat double-pulse)
- **Directional feedback** — Left or right red LED indicates which way you're canted
- **Hysteresis** — Prevents flickering at zone boundaries
- **EMA filtering** — Exponential moving average smooths sensor noise
- **Auto-calibration** — Calibrates on boot assuming the device is held level
- **Battery monitoring** — Low-battery warning via red LED double-blink every 5 seconds
- **Power management** — Enters light sleep after 1 minute at level to conserve battery

## Hardware

| Component | Pin |
|---|---|
| Green LED | GPIO 25 |
| Red LED (Left) | GPIO 26 |
| Red LED (Right) | GPIO 27 |
| Battery voltage divider | GPIO 34 (ADC) |
| ADXL345 SDA | GPIO 21 |
| ADXL345 SCL | GPIO 22 |

**Board:** NodeMCU-32S (ESP32)

## Zone Thresholds

| Zone | Range |
|---|---|
| Level (green solid) | < 0.5° |
| Near level (green fade + red pulse) | 0.5° – 1.5° |
| Cant (red heartbeat) | > 1.5° |

## Build

This project uses [PlatformIO](https://platformio.org/).

```bash
# Build
pio run

# Upload
pio run --target upload

# Serial monitor
pio device monitor
```

### Dependencies

- `Adafruit Unified Sensor` ^1.1.15
- `Adafruit ADXL345` ^1.3.4

## Configuration

Key constants in `src/main.cpp`:

| Constant | Default | Description |
|---|---|---|
| `THRESHOLD` | 0.5° | Dead-level zone boundary |
| `NEAR_ZONE` | 1.5° | Near-level to cant boundary |
| `MAX_CANT` | 5.0° | Maximum urgency angle |
| `HYSTERESIS` | 0.15° | Zone boundary flicker prevention |
| `EMA_ALPHA` | 0.4 | Filter smoothing (lower = smoother) |
| `SLEEP_TIMEOUT` | 60000 ms | Idle time before light sleep |
| `BATT_LOW` | 3.3 V | Low battery warning threshold |
| `DEBUG_MODE` | true | Serial debug output (set false for production) |

## License

NIRINIUM|LABS
