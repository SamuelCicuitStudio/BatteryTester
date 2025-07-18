#include <Arduino.h>
#include "DeviceManager.h"

// Global instance of DeviceManager with I2C bus
DeviceManager device(&Wire);  // Pass Wire pointer to constructor

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);  // Start serial debugging
  device.begin();                  // Initialize all components
}

void loop() {
//  device.updateAllChannels();      // Periodically update all channel statuses
  vTaskDelay(pdMS_TO_TICKS(5000)); // Delay 5s using FreeRTOS
}
