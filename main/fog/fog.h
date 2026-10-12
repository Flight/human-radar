/**
 * @file fog.h
 * @brief Fog generator trigger in the pumpkin (MOSFET/relay module on a GPIO)
 */

#ifndef FOG_H
#define FOG_H

#include <stdbool.h>
#include <esp_err.h>

/**
 * @brief Configure the trigger pin (fog off)
 */
esp_err_t fog_init(void);

/**
 * @brief Start a burst of CONFIG_FOG_BURST_MS, unless one ran less than CONFIG_FOG_MIN_INTERVAL_MS ago
 *
 * @return true when a burst was started
 */
bool fog_burst(void);

/**
 * @brief Switch the fog off now
 */
void fog_stop(void);

/**
 * @brief Whether the fog is running
 */
bool fog_is_on(void);

#endif /* FOG_H */
