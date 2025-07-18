#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#define DEBUGMODE true // Enable or disable debug output
#define ENABLE_SERIAL_DEBUG // Enable serial debugging

#ifdef ENABLE_SERIAL_DEBUG
  #define DEBUG_PRINT(x) if (DEBUGMODE) Serial.print(x) // Print debug info
  #define DEBUG_PRINTF Serial.printf // Formatted print
  #define DEBUG_PRINTLN(x) if (DEBUGMODE) Serial.println(x) // Print line with newline
#else
  #define DEBUG_PRINT(x) // No-op
  #define DEBUG_PRINTF(...) // No-op
  #define DEBUG_PRINTLN(x) // No-op
#endif

#define SERIAL_BAUD_RATE        921600
//
// ────────────────────────────────────────────────
//  ANALOG INPUT PINS (ADC)
// ────────────────────────────────────────────────
#define VOUTCRR1_PIN   1
#define VOUTCRR2_PIN   2
#define VOUTCRR3_PIN   6
#define VOUTCRR4_PIN   7

// ─── Discharge Load Parameters ──────────────────
#define LOAD_RESISTOR_OHMS      2.0f    // 2Ω resistive load
#define LOAD_RESISTOR_POWER_W   12.0f   // 12W power rating

// ─── Current Sensor (ACS781LLRTR-100B-T) ────────
#define CURRENT_SENSOR_SENSITIVITY   0.0132f  // 13.2 mV/A
#define CURRENT_SENSOR_ZERO_VOLTAGE 1.65f     // Assuming 3.3V supply (midpoint)
#define ADC_REF_VOLTAGE              3.3f
#define ADC_RESOLUTION               4096.0f   // 12-bit ADC

//
// ────────────────────────────────────────────────
//  SHIFT REGISTER CONTROL PINS (74HC595)
// ────────────────────────────────────────────────
#define SHIFT_MR_PIN   15
#define SHIFT_SCK_PIN  16
#define SHIFT_RCK_PIN  17
#define SHIFT_SER_PIN  18
#define SHIFT_OE_PIN   38

// ─── Shift Register Output Mapping ──────────────
enum ShiftPin {
    // SHIFT1 (Q0–Q7)
    SHIFT_R4   = 0,
    SHIFT_R1   = 1,
    SHIFT_G1   = 2,
    SHIFT_G2   = 3,
    SHIFT_R3   = 4,
    SHIFT_R2   = 5,
    SHIFT_B4   = 6,
    SHIFT_G4   = 7,

    // SHIFT2 (Q8–Q15)
    SHIFT_B2   = 8,
    SHIFT_B1   = 9,
    SHIFT_B3   = 10,
    SHIFT_G3   = 11,
    SHIFT_LD04 = 12,
    SHIFT_LD03 = 13,
    SHIFT_LD02 = 14,
    SHIFT_LD01 = 15,

    // SHIFT3 (Q16–Q23)
    SHIFT_OTG4 = 16,
    SHIFT_CE1  = 17,
    SHIFT_OTG1 = 18,
    SHIFT_CE2  = 19,
    SHIFT_OTG2 = 20,
    SHIFT_OTG3 = 21,
    SHIFT_CE3  = 22,
    SHIFT_CE4  = 23
};

//
// ────────────────────────────────────────────────
//  REAL-TIME MONITORING STRUCTURE
// ────────────────────────────────────────────────
struct ChargingStatus {
    uint8_t channel;               // Mux channel (0–7)
    float batteryVoltage;          // V
    float systemVoltage;           // V
    float vbusVoltage;             // V
    float chargeCurrent;           // mA
    float temperature;             // °C
    uint16_t chargeVoltage;        // mV (configured)
    uint16_t chargeCurrentSet;     // mA (configured)
    bool chargerEnabled;           // Charger ON
    bool chargingDone;             // Fully charged
    String chargingStatusText;     // Human-readable
    bool pg;                       // Power Good
    bool stat;                     // STAT pin
    bool intLow;                   // INT pin (active LOW)
};

//
// ────────────────────────────────────────────────
//  BQ2589X STATUS / INTERRUPT PINS (per channel)
// ────────────────────────────────────────────────
#define INT4_PIN    8
#define PG4_PIN     3
#define STAT4_PIN   9

#define INT3_PIN    10
#define PG3_PIN     11
#define STAT3_PIN   12

#define INT2_PIN    13
#define STAT2_PIN   14
#define PG2_PIN     21

#define INT1_PIN    47
#define STAT1_PIN   48
#define PG1_PIN     45

//
// ────────────────────────────────────────────────
//  DEFAULT CHARGING PARAMETERS
// ────────────────────────────────────────────────
#define BQ2589X_I2C_ADDR           0x6A
#define DEFAULT_CHARGE_VOLTAGE_MV 4200   // 4.20V
#define DEFAULT_CHARGE_CURRENT_MA 1500   // 1.5A
#define DEFAULT_BOOST_VOLTAGE_MV  5000   // 5.0V OTG
#define DEFAULT_BOOST_CURRENT_MA  1000   // 1.0A OTG

//
// ────────────────────────────────────────────────
//  I2C INTERFACE
// ────────────────────────────────────────────────
#define I2C_SDA_PIN  4
#define I2C_SCL_PIN  5

//
// ────────────────────────────────────────────────
//  TCA9548A I2C MULTIPLEXER
// ────────────────────────────────────────────────
#define TCA_A0_PIN     41
#define TCA_A1_PIN     40  // Adjusted to avoid conflict with A0
#define TCA_A2_PIN     39
#define TCA_RESET_PIN  42
#define TCA9548A_ADDR  0x70  // A2:A1:A0 = 000

//
// ────────────────────────────────────────────────
//  USER INPUT
// ────────────────────────────────────────────────
#define USR_SWITCH_PIN 0

#endif // CONFIG_H
