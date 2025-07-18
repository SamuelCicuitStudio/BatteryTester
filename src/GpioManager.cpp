#include "GpioManager.h"
#include "Arduino.h"

/**
 * @brief Initialize all GPIOs related to the shift register and ADC input pins.
 * 
 * Sets up the ADC input pins as INPUT and the 74HC595 shift register control pins
 * (SER, SCK, RCK, OE, MR) as OUTPUT. Also clears the shift register and applies the
 * initial state (all outputs low).
 */
void GpioManager::begin() {
    DEBUG_PRINTLN("###########################################################");
    DEBUG_PRINTLN("#                  Starting GPIO Manager                  #");
    DEBUG_PRINTLN("###########################################################");

    DEBUG_PRINTLN("→ Configuring ADC input pins:");
    pinMode(VOUTCRR1_PIN, INPUT);
    DEBUG_PRINTF("   - VOUTCRR1_PIN (GPIO %d) set as INPUT\n", VOUTCRR1_PIN);
    pinMode(VOUTCRR2_PIN, INPUT);
    DEBUG_PRINTF("   - VOUTCRR2_PIN (GPIO %d) set as INPUT\n", VOUTCRR2_PIN);
    pinMode(VOUTCRR3_PIN, INPUT);
    DEBUG_PRINTF("   - VOUTCRR3_PIN (GPIO %d) set as INPUT\n", VOUTCRR3_PIN);
    pinMode(VOUTCRR4_PIN, INPUT);
    DEBUG_PRINTF("   - VOUTCRR4_PIN (GPIO %d) set as INPUT\n", VOUTCRR4_PIN);

    DEBUG_PRINTLN("→ Configuring shift register control pins:");
    pinMode(SHIFT_MR_PIN, OUTPUT);
    DEBUG_PRINTF("   - SHIFT_MR_PIN (GPIO %d) set as OUTPUT\n", SHIFT_MR_PIN);
    pinMode(SHIFT_SCK_PIN, OUTPUT);
    DEBUG_PRINTF("   - SHIFT_SCK_PIN (GPIO %d) set as OUTPUT\n", SHIFT_SCK_PIN);
    pinMode(SHIFT_RCK_PIN, OUTPUT);
    DEBUG_PRINTF("   - SHIFT_RCK_PIN (GPIO %d) set as OUTPUT\n", SHIFT_RCK_PIN);
    pinMode(SHIFT_SER_PIN, OUTPUT);
    DEBUG_PRINTF("   - SHIFT_SER_PIN (GPIO %d) set as OUTPUT\n", SHIFT_SER_PIN);
    pinMode(SHIFT_OE_PIN, OUTPUT);
    DEBUG_PRINTF("   - SHIFT_OE_PIN (GPIO %d) set as OUTPUT\n", SHIFT_OE_PIN);

    // Enable shift register output and disable reset
    digitalWrite(SHIFT_OE_PIN, LOW);   // Output enabled
    digitalWrite(SHIFT_MR_PIN, HIGH);  // Not in reset
    DEBUG_PRINTLN("✓ Shift register output ENABLED (OE=LOW)");
    DEBUG_PRINTLN("✓ Shift register RESET disabled (MR=HIGH)");

    // Optional: clear and apply zero state
    resetShiftRegisters();
    shiftOutState();
    DEBUG_PRINTLN("✓ Shift register state cleared and applied (all LOW)");
}

/**
 * @brief Set the state of an individual shift register output pin in memory.
 * 
 * This does NOT immediately update the hardware shift register.
 * Call applyShiftState() to push the updated state to the output pins.
 * 
 * @param pin   The ShiftPin to modify (0–23).
 * @param level HIGH to set, LOW to clear.
 */
void GpioManager::setShiftPin(ShiftPin pin, bool level) {
    if (level)
        shiftState |= (1UL << pin);
    else
        shiftState &= ~(1UL << pin);
}

/**
 * @brief Apply the currently stored shift state to the shift register.
 * 
 * This pushes all 24 bits from memory (`shiftState`) to the physical shift
 * register chain, updating all outputs at once.
 */
void GpioManager::applyShiftState() {
    shiftOutState();
}

/**
 * @brief Reset the shift register contents to all 0s.
 * 
 * This temporarily pulls the master reset (MR) pin LOW and then re-enables it.
 * After reset, all shift register outputs are cleared.
 */
void GpioManager::resetShiftRegisters() {
    digitalWrite(SHIFT_MR_PIN, LOW);
    delayMicroseconds(1);  // Hold low briefly
    digitalWrite(SHIFT_MR_PIN, HIGH);
}

/**
 * @brief Enable or disable the output of the shift register.
 * 
 * @param enable If true, enables output (OE = LOW). If false, disables output (OE = HIGH).
 */
void GpioManager::enableShiftRegisterOutput(bool enable) {
    digitalWrite(SHIFT_OE_PIN, enable ? LOW : HIGH);
}

/**
 * @brief Shift out the current 24-bit state to the shift register.
 * 
 * Outputs all bits in `shiftState`, MSB first (bit 23 to 0).
 * Called internally by applyShiftState().
 */
void GpioManager::shiftOutState() {
    for (int i = 23; i >= 0; --i) {
        digitalWrite(SHIFT_SCK_PIN, LOW);
        digitalWrite(SHIFT_SER_PIN, (shiftState >> i) & 0x01);
        digitalWrite(SHIFT_SCK_PIN, HIGH);
    }
    // Latch output
    digitalWrite(SHIFT_RCK_PIN, LOW);
    digitalWrite(SHIFT_RCK_PIN, HIGH);
}
