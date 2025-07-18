
# 🔋 Multi-Channel Smart Battery Management System

This project implements a **4-channel battery charging and monitoring system** using an **ESP32**, **BQ2589x charger ICs**, **ACS781 current sensors**, **TCA9548A I2C multiplexer**, and **74HC595 shift registers**.  
Each battery channel supports real-time telemetry, RGB status indication, load-based capacity estimation, and user-triggered commands via a shared button.

---

## 📘 Class Overview

| Class            | Responsibility                                                                 |
|------------------|---------------------------------------------------------------------------------|
| `DeviceManager`  | Initializes and connects all hardware subsystems                                |
| `ChannelManager` | Manages one battery channel: charger, LED, sensor, capacity, telemetry          |
| `SwitchManager`  | Handles tap/hold button input to trigger channel-specific actions               |
| `GpioManager`    | Manages GPIO pins and shift register for RGB and load control                   |
| `bq2589x`        | External driver class for interfacing with the BQ2589x charger IC               |

---

## ⚙️ System Architecture

- **Microcontroller**: ESP32 (any variant with ≥4 ADCs, I2C, and enough GPIOs)
- **Charging ICs**: 4× BQ2589x (1 per channel)
- **Voltage/Current Sensors**: 4× ACS781 (connected to ADC)
- **Shift Registers**: 3× 74HC595 (controls RGB LEDs + load switches)
- **Multiplexer**: 1× TCA9548A I2C Mux (selects each charger’s I2C path)
- **Button**: Single shared GPIO input for tap/hold control

---

## 🧠 Class Details

### 🔧 `DeviceManager`

- Manages 4 `ChannelManager` instances.
- Calls `.begin()` to initialize I2C, GPIO, and all channels.
- Starts the `SwitchManager` to monitor button inputs.
- Periodically calls `.sendCapacityReport()` for each channel.

---

### 🔌 `ChannelManager`

Handles:
- BQ2589x configuration: voltage, current, OTG, watchdog.
- RGB LED status using shift register pins.
- Load resistor control for capacity testing.
- ADC-based voltage and current reading.
- Digital status inputs (STAT, INT, PG).
- MUX channel selection (via TCA9548A).

#### Key Methods:
- `startCharging()`, `stopCharging()`
- `measureCapacity()` — estimates mWh via load pulse.
- `sendCapacityReport()` — prints real-time status in JSON.
- `getStatus()` — returns `ChargingStatus` struct.
- `updateLedFromStatus()` — sets RGB:  
  - 🔴 Not charging  
  - 🟢 Charging  
  - 🔵 Fully charged

---

### 🧲 `GpioManager`

- Initializes ADC, GPIO, and shift register pins.
- Maintains a 24-bit `shiftState`.
- Provides:
  - `setShiftPin()`
  - `applyShiftState()`
  - `resetShiftRegisters()`

---

### 🖲️ `SwitchManager`

- Detects tap or hold interactions on one button.
- Launches a FreeRTOS task (`SwitchTask()`).
- Maps 1 to 4 tap counts to trigger capacity tests on the corresponding channel:

| Tap Count | Action                         |
|-----------|--------------------------------|
| 1 tap     | Measure capacity on channel 1  |
| 2 taps    | Measure capacity on channel 2  |
| 3 taps    | Measure capacity on channel 3  |
| 4 taps    | Measure capacity on channel 4  |

---

### 📦 `bq2589x`

External driver class (not detailed here) that supports:
- `begin()`
- `set_charge_voltage()`, `set_charge_current()`
- `enable_charger()`, `disable_charger()`
- `adc_read_battery_volt()`, `adc_read_sys_volt()`, etc.

---

## 📡 Real-Time JSON Output

Example output from `sendCapacityReport()`:

```json
{
  "channel": 1,
  "voltage": 4.19,
  "capacity_mWh": 27.34,
  "charging": true
}
````

---

## 📁 File Structure

```
/src
├── DeviceManager.h/.cpp      → Top-level manager and initializer
├── ChannelManager.h/.cpp     → Controls a single battery channel
├── SwitchManager.h/.cpp      → Tap/hold detection
├── GpioManager.h/.cpp        → GPIO and shift register abstraction
├── bq2589x.h/.cpp            → BQ2589x chip driver
├── main.cpp                  → Arduino-style entry point
```

---

## 🔋 Capacity Estimation Logic

1. Activate load resistor briefly via shift register.
2. Measure voltage before and after using ACS781.
3. Calculate current and estimate energy:

   ```
   ΔI = I_after - I_before
   Power = ΔI² × R
   Energy = Power × time (e.g. 100ms)
   Capacity ≈ mWh = energy (mWs) / 3600
   ```

---

## 🛠️ Setup Instructions

### 📦 Hardware Required

* 1× ESP32
* 4× BQ2589x
* 4× ACS781LLRTR-100B-T
* 3× 74HC595
* 1× TCA9548A
* 4× 2Ω, 12W power resistors
* 1× Push Button

### 🧪 Enable Debugging

To see logs over serial:

```cpp
Serial.begin(SERIAL_BAUD_RATE);
```

---

## 🔁 Runtime Behavior

* System initializes I2C, GPIO, and channels.
* Each channel is assigned a MUX slot (0–3).
* LEDs reflect charger state (red/green/blue).
* User taps the button:

  * Tap 1–4 times to measure capacity of that channel.
* `sendCapacityReport()` is called periodically in the loop.

---

## 🧠 Future Extensions

* BLE or Wi-Fi telemetry
* Web dashboard for monitoring
* SD card data logging
* OTA firmware updates

---

## 👤 Author

**Tshibangu Samuel**
🎯 Freelance Embedded Systems Engineer
📧 [tshibsamuel47@gmail.com](mailto:tshibsamuel47@gmail.com)
📱 +216 54 429 793
🌍 [Freelancer Profile](https://www.freelancer.com/u/tshibsamuel477)

---

```