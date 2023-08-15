#pragma once

#ifndef __LEDG_RMT_CONTROLLER_H
#define __LEDG_RMT_CONTROLLER_H

#include "Arduino.h"

#define FASTLED_INTERNAL
#include <FastLED.h>

#include "esp32-hal-cpu.h"
#include "esp32-hal-rmt.h"
#include "esp32-hal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#ifndef LEDG_LED_CONTROLLER_USE_LL_RMT
#define LEDG_LED_CONTROLLER_USE_LL_RMT true
#endif

// Stuff in here will be put into a class eventually
#define POST_TX_END_DELAY_US 490

#ifndef NUM_LEDS
#warning "NUM_LEDS not defined! Please define NUM_LEDS to the number of LEDs you are using."
#define NUM_LEDS 2
#endif

#ifndef NR_OF_ALL_RMT_LED_BITS
#define NR_OF_ALL_RMT_LED_BITS NUM_LEDS * 24
#else
#warning "Overriding NR_OF_ALL_RMT_LED_BITS to match NUM_LEDS * 24"
#endif

#ifndef NR_OF_ALL_BITS
#define NR_OF_ALL_BITS NR_OF_ALL_RMT_LED_BITS
#endif

#if defined(ESP32) && defined(LEDG_LED_CONTROLLER_USE_LL_RMT) && LEDG_LED_CONTROLLER_USE_LL_RMT == 1


class LEDGlovesRMTController {
public:
  static xSemaphoreHandle __g_rmt_clkstoppoer_locks;

  /**
     * @brief RMT Controller for LED Gloves
     */
  LEDGlovesRMTController();

  /**
     * @brief RMT Controller for LED Gloves
     */
  ~LEDGlovesRMTController();

  /**
     * @brief Sets up and initializes the RMT Controller
     *
     * @param pin GPIO pin to use for RMT TX
     *
     * @param ledArray Pointer to the starting pixel LED Array to use
     *
     * @param led_num Number of LEDs in the LED Array
     *
     * @return true if setup was successful, false otherwise
     */
  bool IRAM_ATTR setup(int pin, CRGB* ledArray, uint16_t led_num);

  /**
     * @brief Stops the RMT Controller
     *
     * @note This is a static function because it needs to be called from an ISR
     *
     * @return void
     */
  void IRAM_ATTR end(void);

  /**
     * @brief Displays the current data in the pointed to pixel strip
     *
     * @note this blocks/yields the calling thread until the RMT has finished writing the data
     *
     * @return void
     */
  void IRAM_ATTR show();

  /**
     * @brief Blocks the caller until the RMT is done TX-ing and is available for clock changes, etc
     */
  static void __always_inline blockCallerWhileWritingBuffer(void);

  /**
     * @brief Sets the Brightness of the LEDs and stores/recalculates the total color adjustment
     *
     * @param brightness brightness scaler
     *
     * @return void
     */
  void IRAM_ATTR setBrightness(uint8_t brightness);

  /**
     * @brief Sets the Color Correction of the LEDs and stores/recalculates the total color adjustment
     *
     * @param correction color correction to apply to output
     *
     * @return void
     */
  void IRAM_ATTR setCorrection(const struct CRGB& correction);

  /**
     * @brief Sets the Color Temp of the LEDs and stores/recalculates the total color adjustment
     *
     * @param temp color Temp to apply to output
     *
     * @return void
     */
  void IRAM_ATTR setTemperature(const struct CRGB& temp);

  /**
     * @brief Enables/Disables the RMT TX Dithering - Not Implemented
     *
     * @param ditherMode 0 = Disabled, 1 = Enabled
     */
  void IRAM_ATTR setDither(uint8_t ditherMode);  // Mock FastLED interface

private:
  xSemaphoreHandle g_rmt_colorBuf_locks = NULL;
  rmt_data_t rmt_led_data[NR_OF_ALL_BITS];
  rmt_obj_t* rmt_send = NULL;
  uint8_t led_brightness_scaler = 255;
  CRGB* pixel_array_start_ptr = NULL;
  CRGB colorCorrection = CRGB::White;
  CRGB colorTemperature = CRGB::White;
  CRGB totalColorAdjustment = CRGB::White;
  uint16_t _led_number = 0;
  TickType_t xLastWakeTime;
  bool _cb_setup = false;
  uint32_t _delayTimeMs;

  /**
     * @brief Locks the writing buffer Mutex to prevent clocking changes or config changeswhile TX-ing
     */
  static void __always_inline takeWritingBufferLock(void);

  /**
     * @brief Releases the writing buffer lock
     */
  static void __always_inline releaseWritingBufferLock(void);

  /**
     * @brief Compute overall Color Adjustment for Output LEDs based on brightness, color correction, and color temperature
     *
     * @param scale brightness scaler
     *
     * @param colorCorrection color correction to apply to output
     *
     * @param colorTemperature color temperature adjustment to apply to output
     *
     * @return CRGB correction color to apply to output
     */
  CRGB __always_inline computeAdjustment(uint8_t scale, const CRGB& colorCorrection, const CRGB& colorTemperature);

  /**
     * @brief Manipulates Pixels in-place to correct for per-pixel byte order
     *
     * @param pixel Pixel to manipulate in-place
     *
     * @param pixelIndex Index of the pixel in the strip
     */
  void __always_inline translatePixelColorOrder(CRGB& pixel, uint16_t pixelIndex);

  /**
     * @brief Adjusts the RMT's Clock Divider to match the current CPU/APB clock
     *
     * @note This is a static function because it needs to be called from an ISR
     *
     * @return void
     */
  void IRAM_ATTR updateRMTClkDiv();  // don't make static for now

  /**
     * @brief Copies and translates the Pixel Data from the pointed toCRGB Array to the RMT Buffer
     *
     * @note This will likely need an additional Mutex like in the LEDController itself to lock
     *      access to the pixel array when it's being read from, etc
     *
     * @return void
     */
  void __always_inline copy_pixel_data();

  /**
     * @brief Actually Send the RMT Buffer Data out in a Blocking way
     *
     * @return void
     */
  void __always_inline tx_pixel_data();

  /***
     * @brief delays the currnt Calling thread by [ms] milliseconds
     *        and maintains that delay time until changed
     *
     * @param ms milliseconds to delay by
     *
     */
  void __always_inline delayMs(uint32_t ms);

  /***
     * @brief delays the currnt Calling thread by the current delay time
     */
  void __always_inline delayMs();

  // TODO: Add support for Multiple Controllers/Pointers to CRGB lists like FastLED to support Multiple strips

protected:
  static bool tx_should_reset_clk_div;

  /**
     * @brief APC Change Callback for updating the RMT's Clock Divider to match the current CPU/APB clock
     *        -- apb_change_cb_t apb_change_CB_ptr = apb_change_CB;
     *
     * @note This is a static function because it needs to be called from an ISR
     *
     * @param arg unused as per Arduino Docs.
     *
     * @param ev_type type of APB-Change event that triggered the callback
     *
     * @param old_apb old APB clock frequency/config
     *
     * @param new_apb new APB clock frequency/config
     *
     * @return void
     *
     * @note Don't call anything like Serial.println() too much from this function, it will crash the ESP32,
     *       since it's called from an ISR, etc
     */
  static void IRAM_ATTR apb_change_CB(void* arg, apb_change_ev_t ev_type, uint32_t old_apb, uint32_t new_apb);
};

/**
 * @brief SGlobal LEDController Object
 */
extern LEDGlovesRMTController RMTController;

#endif
#endif