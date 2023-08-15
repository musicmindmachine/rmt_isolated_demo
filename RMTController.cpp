#include "esp32-hal-cpu.h"
#include "RMTController.h"

#include "Arduino.h"

#if defined(ESP32) && defined(LEDG_LED_CONTROLLER_USE_LL_RMT) && LEDG_LED_CONTROLLER_USE_LL_RMT == 1


/**
 * @brief RMT Controller for LED Gloves
 */
LEDGlovesRMTController::LEDGlovesRMTController() {
  // Nothing Special
  tx_should_reset_clk_div = false;
}

/**
 * @brief RMT Controller for LED Gloves
 */
LEDGlovesRMTController::~LEDGlovesRMTController() {
}

// Statics
xSemaphoreHandle LEDGlovesRMTController::__g_rmt_clkstoppoer_locks = NULL;
bool LEDGlovesRMTController::tx_should_reset_clk_div = false;  // Flag for eventual RMT clock freq update if we changed APB clock before next TX.

/**
 * @brief Locks the writing buffer Mutex to prevent clocking changes or config changeswhile TX-ing
 */
void __always_inline LEDGlovesRMTController::takeWritingBufferLock(void) {
  do {
  } while (xSemaphoreTake(LEDGlovesRMTController::__g_rmt_clkstoppoer_locks, portMAX_DELAY) != pdPASS);
}

/**
 * @brief Releases the writing buffer lock
 */
void __always_inline LEDGlovesRMTController::releaseWritingBufferLock(void) {
  xSemaphoreGive(LEDGlovesRMTController::__g_rmt_clkstoppoer_locks);
}

/**
 * @brief Blocks the caller until the RMT is done TX-ing and is available for clock changes, etc
 */
void __always_inline LEDGlovesRMTController::blockCallerWhileWritingBuffer(void) {
  LEDGlovesRMTController::takeWritingBufferLock();
  LEDGlovesRMTController::releaseWritingBufferLock();
}

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
CRGB __always_inline LEDGlovesRMTController::computeAdjustment(uint8_t scale, const CRGB& colorCorrection, const CRGB& colorTemperature) {
#if defined(NO_CORRECTION) && (NO_CORRECTION == 1)
  return CRGB(scale, scale, scale);
#else
  CRGB adj(0, 0, 0);

  if (scale > 0) {
    for (uint8_t i = 0; i < 3; ++i) {
      uint8_t cc = colorCorrection.raw[i];
      uint8_t ct = colorTemperature.raw[i];
      if (cc > 0 && ct > 0) {
        uint32_t work = (((uint32_t)cc) + 1) * (((uint32_t)ct) + 1) * scale;
        work /= 0x10000L;
        adj.raw[i] = work & 0xFF;
      }
    }
  }

  return adj;
#endif
}

/**
 * @brief Sets the Brightness of the LEDs and stores/recalculates the total color adjustment
 *
 * @param brightness brightness scaler
 *
 * @return void
 */
void IRAM_ATTR LEDGlovesRMTController::setBrightness(uint8_t brightness) {
  this->led_brightness_scaler = brightness;
  this->totalColorAdjustment = computeAdjustment(
    this->led_brightness_scaler,
    this->colorCorrection,
    this->colorTemperature);
}

/**
 * @brief Sets the Color Correction of the LEDs and stores/recalculates the total color adjustment
 *
 * @param correction color correction to apply to output
 *
 * @return void
 */
void IRAM_ATTR LEDGlovesRMTController::setCorrection(const struct CRGB& correction) {
  this->colorCorrection = correction;
  this->totalColorAdjustment = computeAdjustment(
    this->led_brightness_scaler,
    this->colorCorrection,
    this->colorTemperature);
}

/**
 * @brief Sets the Color Temp of the LEDs and stores/recalculates the total color adjustment
 *
 * @param temp color Temp to apply to output
 *
 * @return void
 */
void IRAM_ATTR LEDGlovesRMTController::setTemperature(const struct CRGB& temp) {
  this->colorTemperature = temp;
  this->totalColorAdjustment = computeAdjustment(
    this->led_brightness_scaler,
    this->colorCorrection,
    this->colorTemperature);
}

/**
 * @brief Enables/Disables the RMT TX Dithering - Not Implemented
 *
 * @param ditherMode 0 = Disabled, 1 = Enabled
 */
void IRAM_ATTR LEDGlovesRMTController::setDither(uint8_t ditherMode) {
  // NOOP();
}

/**
 * @brief Manipulates Pixels in-place to correct for per-pixel byte order
 *
 * @param pixel Pixel to manipulate in-place
 *
 * @param pixelIndex Index of the pixel in the strip
 */
void __always_inline LEDGlovesRMTController::translatePixelColorOrder(CRGB& pixel, uint16_t pixelIndex) {
  // Translate the pixel color order
  CRGB temp = pixel;
  if (pixelIndex == 1) {
    pixel.r = temp.g;
    pixel.g = temp.r;
    pixel.b = temp.b;
  } else if (pixelIndex == 0) {
    pixel.r = temp.r;
    pixel.g = temp.g;
    pixel.b = temp.b;
  }
}

/**
 * @brief Adjusts the RMT's Clock Divider to match the current CPU/APB clock
 *
 * @note This is a static function because it needs to be called from an ISR
 *
 * @return void
 */
void IRAM_ATTR LEDGlovesRMTController::updateRMTClkDiv() {
  uint32_t f_cpu = getCpuFrequencyMhz();
  uint32_t tick = 100;  // Assume default of 100 ps
  if (f_cpu >= 80)
    tick = 100;  // HACK: Assume 80Mhz for now
  else if (f_cpu == 40)
    tick = 50;  // lol
  else if (f_cpu == 20)
    tick = 25;  // Per Jarrad's email
  else if (f_cpu == 10)
    tick = 14;  // 12.5 is min so should = 1 if divided by 12.5

  // Serial.printf("updateRMTTick(), f_cpu=%d, tick=%d \n", f_cpu, tick);

  rmtSetTick(this->rmt_send, tick);  // Update the tick rate
  LEDGlovesRMTController::tx_should_reset_clk_div = false;
}

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
void IRAM_ATTR LEDGlovesRMTController::apb_change_CB(void* arg, apb_change_ev_t ev_type, uint32_t old_apb, uint32_t new_apb) {
  // Serial.println("APB Clock RMT needs update flag set!");

  // NOTE: Don't immediately update the tick in the CB,  updateRMTTick()
  // since apb_change_CB is fired from an ISR in arduino, just set a flag
  // that will signal to reset the RMT clock divider on the next TX

  // Flag for eventual RMT clock freq update if we changed APB clock
  if (ev_type == APB_BEFORE_CHANGE) {
    LEDGlovesRMTController::takeWritingBufferLock();
  } else if (ev_type == APB_AFTER_CHANGE) {
    LEDGlovesRMTController::releaseWritingBufferLock();
    LEDGlovesRMTController::tx_should_reset_clk_div = true;
  }
}

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
bool IRAM_ATTR LEDGlovesRMTController::setup(int pin, CRGB* ledArray, uint16_t led_num) {
  if (ledArray == NULL || pin < 0) return false;
  this->pixel_array_start_ptr = ledArray;
  this->_led_number = led_num;

  memset(ledArray, 0, sizeof(CRGB) * led_num);  // clear LEDS for now

  if ((this->rmt_send = rmtInit(pin, RMT_TX_MODE, RMT_MEM_64)) == NULL) {
    return false;
  }

  this->updateRMTClkDiv();  // Init clock divider based on APB frequency

  if (!__g_rmt_clkstoppoer_locks) {
    LEDGlovesRMTController::__g_rmt_clkstoppoer_locks = xSemaphoreCreateMutex();
  }

  if (!this->_cb_setup) addApbChangeCallback(NULL, LEDGlovesRMTController::apb_change_CB);  // Add the cb to update the rmt on APB clk chagnes
  this->_cb_setup = true;
  return true;
}

/**
 * @brief Stops the RMT Controller
 *
 * @note This is a static function because it needs to be called from an ISR
 *
 * @return void
 */
void IRAM_ATTR LEDGlovesRMTController::end() {
  LEDGlovesRMTController::releaseWritingBufferLock();
  if (this->_cb_setup) removeApbChangeCallback(NULL, LEDGlovesRMTController::apb_change_CB);
  this->_cb_setup = false;
  rmtDeinit(this->rmt_send);
}

/***
 * @brief delays the currnt Calling thread by [ms] milliseconds
 *        and maintains that delay time until changed
 *
 * @param ms milliseconds to delay by
 *
 */
void __always_inline LEDGlovesRMTController::delayMs(uint32_t ms) {
  __asm__ __volatile__("nop");  // required to prevent compiler from optimizing out the delay
  this->_delayTimeMs = ms;
  this->delayMs();
}

/***
 * @brief delays the currnt Calling thread by the current delay time
 */
void __always_inline LEDGlovesRMTController::delayMs() {
  // __asm__ __volatile__("nop");    // Removing Asm clean memory
  uint32_t delay = this->_delayTimeMs / portTICK_PERIOD_MS;  // get the delay time in ticks
  delay = delay > 0 ? delay : 1;

  // convert above to while loop
  while (delay > 0) {
    __asm__ __volatile__("nop");  // required to prevent compiler from caching delay

    // Do the delay and reset
    if (delay == 1) {
      // schedule more aggressively for fast delays/last delay
      this->xLastWakeTime = xTaskGetTickCount();
      vTaskDelayUntil(&this->xLastWakeTime, 1);
    } else {
      // schedule normally for all other delays
      vTaskDelay(1);
    }
    this->xLastWakeTime = xTaskGetTickCount();
    delay--;
  }
}

/**
 * @brief Copies and translates the Pixel Data from the pointed toCRGB Array to the RMT Buffer
 *
 * @note This will likely need an additional Mutex like in the LEDController itself to lock
 *      access to the pixel array when it's being read from, etc
 *
 * @return void
 */
void __always_inline LEDGlovesRMTController::copy_pixel_data() {
  int led, col, bit, value;
  int i = 0;
  CRGB tmpColor;
  // cast CRGB to raw bytes
  uint8_t color[sizeof(CRGB)] = { 0, 0, 0 };

  for (led = 0; led < this->_led_number; led++) {
    tmpColor = this->pixel_array_start_ptr[led];

    // Apply color adjustments like brightness and CC to pixel
    tmpColor.r = scale8(tmpColor.r, (fract8)this->totalColorAdjustment.r);
    tmpColor.g = scale8(tmpColor.g, (fract8)this->totalColorAdjustment.g);
    tmpColor.b = scale8(tmpColor.b, (fract8)this->totalColorAdjustment.b);

    // Swap byte-order for fucked up LEDs
    this->translatePixelColorOrder(tmpColor, led);

    // cast CRGB to raw bytes by copying to array
    memcpy8(&color, &tmpColor, sizeof(CRGB));

    // Transmit each bit of the CRGB struct
    for (col = 0; col < sizeof(CRGB); col++) {
      value = color[col];
      for (bit = 0; bit < 8; bit++) {
        if (value & (1 << (7 - bit))) {
          this->rmt_led_data[i].level0 = 1;
          this->rmt_led_data[i].duration0 = 8;
          this->rmt_led_data[i].level1 = 0;
          this->rmt_led_data[i].duration1 = 4;
        } else {
          this->rmt_led_data[i].level0 = 1;
          this->rmt_led_data[i].duration0 = 4;
          this->rmt_led_data[i].level1 = 0;
          this->rmt_led_data[i].duration1 = 8;
        }
        i++;
      }
    }
  }
}

/**
 * @brief Actually Send the RMT Buffer Data out in a Blocking way
 *
 * @return void
 */
void __always_inline LEDGlovesRMTController::tx_pixel_data() {
  LEDGlovesRMTController::takeWritingBufferLock();

  if (tx_should_reset_clk_div) this->updateRMTClkDiv();  // Change clk if needed

  rmtWriteBlocking(this->rmt_send, this->rmt_led_data, NR_OF_ALL_BITS);  // wait for RMT to finish TX

  // Wait a minimum time allowing other tasks to run while we let the line settle to latch the data
  uint64_t startMicros = micros();
  do {
    taskYIELD();
  } while (micros() < startMicros + POST_TX_END_DELAY_US);  // Loop and yield for min stable value of settling time
  // Finally allow more writes/clock changes
  LEDGlovesRMTController::releaseWritingBufferLock();
}

/**
 * @brief Displays the current data in the pointed to pixel strip
 *
 * @note this blocks/yields the calling thread until the RMT has finished writing the data
 *
 * @return void
 */
void IRAM_ATTR LEDGlovesRMTController::show() {
  if (this->pixel_array_start_ptr == NULL) return;
  this->copy_pixel_data();
  this->tx_pixel_data();

  taskYIELD();  // Loop as fast as the driver will allow, but yield for other threads
  // this->delayMs(1);  // Test more aggressive, but still FreeRTOS Tick-based Delays
  // this->delayMs(5);  // Test "sloppy" long delays
}

/**
 * @brief SGlobal LEDController Object
 */
LEDGlovesRMTController RMTController;

#endif