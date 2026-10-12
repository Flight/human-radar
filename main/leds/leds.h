/**
 * @file leds.h
 * @brief WS2812 LEDs in the pumpkin: one animated pattern per scene stage
 */

#ifndef LEDS_H
#define LEDS_H

#include <esp_err.h>

typedef enum {
    LEDS_PATTERN_IDLE,    /**< Slow green breathing: nobody around */
    LEDS_PATTERN_CONTACT, /**< Faster green breathing: someone in the room */
    LEDS_PATTERN_THREAT,  /**< Amber pulse: someone is getting close */
    LEDS_PATTERN_PANIC,   /**< Red strobe with white flashes: someone is behind you */
} leds_pattern_t;

/**
 * @brief Set up the strip and start the animation task (LEDs off on failure)
 */
esp_err_t leds_init(void);

/**
 * @brief Switch to a pattern (safe from any task)
 */
void leds_set_pattern(leds_pattern_t pattern);

#endif /* LEDS_H */
