/**
 * @file ld2450.h
 * @brief HLK-LD2450 24 GHz mmWave sensor: up to three people, x/y/speed at 10 Hz over UART
 */

#ifndef LD2450_H
#define LD2450_H

#include <esp_err.h>

/**
 * @brief Open the UART and start publishing the sensor's targets (no-op when CONFIG_LD2450_ENABLED is off)
 */
esp_err_t ld2450_start(void);

#endif /* LD2450_H */
