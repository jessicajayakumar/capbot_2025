/**
 * @file capbot_board.h
 * @author Lowie Deferme <lowie.deferme@kuleuven.be>
 * @brief CapBot library for on-board peripherals
 */

#ifndef CAPBOT_BOARD_H
#define CAPBOT_BOARD_H

#include <stdint.h>

// -----------------------------------------------------------------------------
// On-board LEDs
// -----------------------------------------------------------------------------

typedef enum {
    CB_D15,
    CB_D16,
} cb_led_t;

/** @brief Initialize robot's leds */
int cb_led_init(void);

/**
 * @brief Write to led
 *
 * @param led The led to write
 * @param val The value to write: 0 = off, 1 = on
 * @retval <0: An error occurred
 */
int cb_led_write(cb_led_t led, uint8_t val);

// -----------------------------------------------------------------------------
// On-board buttons
// -----------------------------------------------------------------------------

/** @brief Initialize robot's button */
int cb_btn_init(void);

/**
 * @brief Get button status
 *
 * @retval 0: the button isn't pressed
 * @retval 1: the button is pressed
 * @retval <0: An error occurred
 */
int cb_btn_get(void);

// -----------------------------------------------------------------------------
// On-board voltage measurements
// -----------------------------------------------------------------------------

/**
 * @brief Initialize robot's voltage measurement system
 *
 * @return int
 */
int cb_measure_init(void);

/**
 * @brief Measure supercapacitor voltage
 *
 * @retval ≥0: measured voltage in mV
 * @retval <0: An error occurred
 */
int16_t cb_measure_vcap(void);

#endif /* CAPBOT_BOARD_H */
