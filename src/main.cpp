#include <Arduino.h>
#include "DeviceManager.h"

// Global instance of DeviceManager with I2C bus
DeviceManager device(&Wire);  // Pass Wire pointer to constructor

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);  // Start serial debugging
  // Initialize I2C with custom SDA/SCL pins
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);  // Pins from config.h
  device.begin();  // Initialize all components
}

void loop() {
  // device.updateAllChannels();  // Optional update loop
  vTaskDelay(pdMS_TO_TICKS(5000)); // Delay 5s using FreeRTOS
}
