#include "SwitchManager.h"

SwitchManager* SwitchManager::instance = nullptr;

SwitchManager::SwitchManager(ChannelManager* ch1, ChannelManager* ch2,
                             ChannelManager* ch3, ChannelManager* ch4)
    : channel1(ch1), channel2(ch2), channel3(ch3), channel4(ch4) {
    DEBUG_PRINTLN("###########################################################");
    DEBUG_PRINTLN("#                  Starting Switch Manager                #");
    DEBUG_PRINTLN("###########################################################");
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    instance = this;
}

void SwitchManager::detectTapOrHold() {
    uint8_t tapCount = 0;
    unsigned long pressStart = 0;
    unsigned long lastTapTime = 0;

    while (true) {
        if (digitalRead(BUTTON_PIN) == LOW) {
            pressStart = millis();
            while (digitalRead(BUTTON_PIN) == LOW) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            unsigned long pressDuration = millis() - pressStart;
            if (pressDuration >= HOLD_THRESHOLD_MS) {
                DEBUG_PRINTLN("Long press detected 🕒 - Restarting device");
                // Put the ESP32 into deep sleep for 1 second (simulate power-down)
                esp_sleep_enable_timer_wakeup(1000000); // 1 second (in microseconds)
                esp_deep_sleep_start();  // Enter deep sleep
                tapCount = 0;
                continue;
            }

            tapCount++;
            lastTapTime = millis();
        }

        if (tapCount > 0 && (millis() - lastTapTime) > TAP_WINDOW_MS) {
            switch (tapCount) {
                case 1:
                    DEBUG_PRINTLN("🔋 Tap x1 - Testing Channel 1");
                    if (channel1) channel1->sendCapacityReport();
                    break;
                case 2:
                    DEBUG_PRINTLN("🔋 Tap x2 - Testing Channel 2");
                    if (channel2) channel2->sendCapacityReport();
                    break;
                case 3:
                    DEBUG_PRINTLN("🔋 Tap x3 - Testing Channel 3");
                    if (channel3) channel3->sendCapacityReport();
                    break;
                case 4:
                    DEBUG_PRINTLN("🔋 Tap x4 - Testing Channel 4");
                    if (channel4) channel4->sendCapacityReport();
                    break;
                default:
                    DEBUG_PRINTLN("❓ Unknown tap count. Ignoring.");
                    break;
            }

            tapCount = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void SwitchManager::SwitchTask(void* pvParameters) {
    for (;;) {
        if (SwitchManager::instance) {
            SwitchManager::instance->detectTapOrHold();
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void SwitchManager::TapDetect() {
    xTaskCreatePinnedToCore(
        SwitchTask,
        "SwitchTask",
        4096,
        nullptr,
        1,
        nullptr,
        1
    );
}