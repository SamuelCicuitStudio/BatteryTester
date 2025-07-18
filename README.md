# 🔋 Multi-Channel Smart Battery Management System

This project implements a **4-channel battery charging and monitoring system** using an **ESP32**, **BQ2589x charger ICs**, **ACS781 current sensors**, **TCA9548A I2C multiplexer**, and **74HC595 shift registers**. Each battery channel supports real-time telemetry, RGB status indication, load-based capacity estimation, and user-triggered commands via a shared button.

---

## 📘 Top-Level Class Overview

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
- **Charging IC**: 4x BQ2589x (1 per channel)
- **Voltage/Current Sensor**: 4x ACS781 (connected to ADC)
- **Shift Register**: 3x 74HC595 (controls RGBs + load switches)
- **Multiplexer**: TCA9548A I2C Mux (selects each charger's I2C path)
- **Single Tap Button**: Shared GPIO input for all control modes

---

## 🧠 Class Details

### 🔧 `DeviceManager`

- Holds and manages 4 `ChannelManager` instances
- Calls `.begin()` for all subsystems
- Starts the `SwitchManager`
- Periodically calls `.sendCapacityReport()` for all channels

### 🔌 `ChannelManager`

Manages:
- BQ2589x configuration (voltage, current, OTG, watchdog, etc.)
- RGB LED color based on charge state
- Load resistor control (via shift register)
- ADC-based capacity estimation
- Digital inputs (STAT, INT, PG)
- I2C mux slot selection

Key Methods:
- `startCharging()`, `stopCharging()`
- `measureCapacity()`
- `sendCapacityReport()` → JSON via Serial
- `getStatus()` → returns `ChargingStatus` struct
- `updateLedFromStatus()` → RGB = 🔴 not charging, 🟢 charging, 🔵 done

### 🧲 `GpioManager`

- Configures ADC and GPIO pins
- Manages `shiftState` (24-bit register state)
- Controls OE, MR, SER, SCK, RCK
- Provides `setShiftPin()`, `applyShiftState()`, `resetShiftRegisters()`

### 🖲️ `SwitchManager`

- Detects tap or hold interactions on one button
- Supports up to **4 tap types**:
  | Tap Count | Action                         |
  |-----------|--------------------------------|
  | 1 tap     | Measure capacity on channel 1  |
  | 2 taps    | Measure capacity on channel 2  |
  | 3 taps    | Measure capacity on channel 3  |
  | 4 taps    | Measure capacity on channel 4  |
- Launches `SwitchTask()` on a dedicated FreeRTOS core

### 📦 `bq2589x`

- External driver
- Provides functions like:
  - `begin()`
  - `set_charge_voltage()`, `set_charge_current()`
  - `enable_charger()`, `disable_charger()`
  - `adc_read_*()` functions for battery, system, VBUS voltage, temperature, current

---

## 📡 Real-Time JSON Output

`ChannelManager::sendCapacityReport()` sends this over `Serial`:

```json
{
  "channel": 1,
  "voltage": 4.19,
  "capacity_mWh": 27.34,
  "charging": true
}
📁 File Tree
bash
Copy
Edit
/src
├── DeviceManager.h/.cpp      → Top-level initializer
├── ChannelManager.h/.cpp     → Controls one channel
├── SwitchManager.h/.cpp      → Button handler (tap detection)
├── GpioManager.h/.cpp        → Shift register + GPIO abstraction
├── bq2589x.h/.cpp            → BQ2589x chip driver
├── main.cpp                  → Arduino-style entry point
🔋 Capacity Estimation
Activates the load resistor for a short pulse

Measures current delta using ACS781

Calculates energy from ΔI² * R * Δt

Reports result in mWh

🛠️ Setup
📦 Hardware Requirements
ESP32

4x BQ2589x

4x ACS781

3x 74HC595

1x TCA9548A

4x 2Ω, 12W resistive loads

1x Push button

🧪 Optional Diagnostic Output
Enable DEBUG_PRINTLN() via Serial.begin(SERIAL_BAUD_RATE) in main.cpp.

🔁 Runtime Behavior
System boots and initializes I2C, GPIOs, channels

Each channel is assigned a MUX slot (0–3)

LED color reflects charge state

User taps the button:

1–4 taps → triggers measureCapacity() on a selected channel

sendCapacityReport() is called periodically in the main loop

🧠 Future Extensions
BLE/Wi-Fi telemetry

Web-based UI

SD card logging

Over-the-air updates

👤 Author
Tshibangu Samuel
🎯 Embedded Systems Engineer
📧 tshibsamuel47@gmail.com
📱 +216 54 429 793
🌍 Freelancer Profile