#include "Arduino.h"

#define FASTLED_INTERNAL
#include <FastLED.h>

#include "esp32-hal-cpu.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PIN_DATA_PIN 1
#define PIN_LED_POWER 2
#define NUM_LEDS 2
#define NR_OF_ALL_RMT_LED_BITS 24 * NUM_LEDS

#include "RMTController.h"

#define INIT_LED_BRIGHTNESS 192
#define COLOR_CORRECTION 0xFFC0F0  // LEDColorCorrection::TypicalLEDStrip
#define COLOR_TEMP ColorTemperature::UncorrectedTemperature

#define START_FREQ 160
CRGB pixels[NUM_LEDS] = {0};
uint32_t _test_loop_counter = 0;

CRGB inline get_test_color() {
    _test_loop_counter++;
    const uint8_t val = (_test_loop_counter >> 1) % 256;
    const uint8_t colorSwitch = ((_test_loop_counter >> 1) % (256 * 12)) / 256;

    CRGB color = CRGB::Black;
    if (colorSwitch == 0)
        color.r = val;
    else if (colorSwitch == 1)
        color.g = val;
    else if (colorSwitch == 2)
        color.b = val;
    else if (colorSwitch == 3 || colorSwitch == 7)
        color = CRGB(val, val, val);
    else if (colorSwitch == 4) {
        CRGB c = CRGB::Cyan;
        color.r = scale8(c.r, val);
        color.g = scale8(c.g, val);
        color.b = scale8(c.b, val);
    } else if (colorSwitch == 5) {
        CRGB c = CRGB::Yellow;
        color.r = scale8(c.r, val);
        color.g = scale8(c.g, val);
        color.b = scale8(c.b, val);
    } else if (colorSwitch == 6) {
        CRGB c = CRGB::Magenta;
        color.r = scale8(c.r, val);
        color.g = scale8(c.g, val);
        color.b = scale8(c.b, val);
    } else if (colorSwitch >= 8) {
        color = CHSV(val, 200, 255);
    }

    if (colorSwitch > 12) _test_loop_counter = 0;

    return color;
}

void IRAM_ATTR task_test_led_task(void* params) {
    bool odd;
    CRGB color;
    while (1) {
        odd = millis() % 2;
        color = odd ? get_test_color() : CRGB::Black;
        pixels[0] = color;
        pixels[1] = color;

        RMTController.show();  // NOTE: This is Blocking and has built in yields

        // taskYIELD();  // Loop as fast as the driver will allow, but yield for other threads
    }

    vTaskSuspend(NULL);
};

void IRAM_ATTR task_test_change_clk_frq(void* params) {
    while (1) {
        // Simulate Wifi Loops for a bit
        for (int i = 0; i < 100; i++) {
            setCpuFrequencyMhz(80);
            delay(1);

            setCpuFrequencyMhz(80);
            delay(3);

            setCpuFrequencyMhz(10);
            delay(70);
        }

        // Simulate USB Plugged IN
        setCpuFrequencyMhz(160);
        delay(2000);
    }

    vTaskSuspend(NULL);
};

// Back to Arduino Land
void setup() {
    setCpuFrequencyMhz(START_FREQ);
    // Serial.begin(115200);

    pinMode(PIN_LED_POWER, OUTPUT);
    digitalWrite(PIN_LED_POWER, LOW);
    delay(2);

    RMTController.setup(PIN_DATA_PIN, pixels, NUM_LEDS);
    RMTController.setBrightness(INIT_LED_BRIGHTNESS);
    RMTController.setCorrection(COLOR_CORRECTION);
    RMTController.setTemperature(COLOR_TEMP);
    delay(2);

    digitalWrite(PIN_LED_POWER, HIGH);  // Finally turn on LEDs

    // Basic LED Test Thread to run animations
    xTaskCreate(
        task_test_led_task, /* Task function. */
        "LEDTask",          /* String with name of task. */
        2048,               /* Stack size in bytes. */
        NULL,               /* Parameter passed as input of the task */
        6,                  /* Priority of the task. */
        NULL);              /* Task handle. */

    // delay(2000);  // Wait ten seconds as a buffer to enable re-flashing because I'm dumb sometimes

    // // Make a thead to chagne the clock async from the display loop
    // xTaskCreate(
    //     task_test_change_clk_frq, /* Task function. */
    //     "Clk_changer",            /* String with name of task. */
    //     1024,                     /* Stack size in bytes. */
    //     NULL,                     /* Parameter passed as input of the task */
    //     2,                        /* Priority of the task. */
    //     NULL);                    /* Task handle. */
}

void loop() {
    // Nothing here
}