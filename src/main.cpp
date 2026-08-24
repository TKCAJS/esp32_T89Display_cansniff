// ASMG-MD servo CAN sniffer — temporary diagnostic firmware for the T89
// display node (ESP32-S3 8048S043C). Task split, display init order and touch
// calibration are copied from esp32_T89Display/src/main.cpp — see the comments
// there for why the display must be initialised from core 1.
#include <Arduino.h>
#include <esp32_smartdisplay.h>

#include "ServoBus.h"
#include "SnifferUI.h"

#define SOFTWARE_VERSION "0.1.0"

// Core 0: CAN poll engine — 2 ms tick keeps reply-latency quantisation small
static void protocolTask(void*) {
    for (;;) {
        servoBusUpdate();
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// Core 1: LVGL rendering
static void displayTask(void*) {
    auto lv_last_tick = millis();
    for (;;) {
        auto const now = millis();
        lv_tick_inc(now - lv_last_tick);
        lv_last_tick = now;
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n========================================");
    Serial.printf("ASMG servo CAN sniffer v%s\n", SOFTWARE_VERSION);
    Serial.println("========================================\n");

    smartdisplay_init();
    snifferUiInit();
    servoBusBegin();

    lv_point_t screen_points[3] = {{0, 0}, {799, 0}, {799, 479}};
    lv_point_t touch_points[3]  = {{20, 20}, {460, 20}, {460, 261}};
    touch_calibration_data = smartdisplay_compute_touch_calibration(screen_points, touch_points);
    touch_calibration_data.valid = true;

    xTaskCreatePinnedToCore(protocolTask, "proto", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(displayTask,  "disp",  8192, NULL, 2, NULL, 1);
}

void loop() {
    vTaskDelete(NULL);
}
