/**
 * @file targets.h
 * @brief Shared list of tracked people: the radar's data model
 *
 * Coordinates follow the HLK-LD2450 convention: the sensor sits at the origin,
 * x grows to its right and y straight ahead, both in millimetres. A producer
 * (the LD2450 driver, or the simulator until it is wired up) publishes whole
 * frames with targets_publish(); the scope and the scene take a consistent
 * copy with targets_get(). Slots keep their index between frames, so slot i in
 * two consecutive frames is the same person; an empty slot has valid = false.
 */

#ifndef TARGETS_H
#define TARGETS_H

#include <stdbool.h>
#include <stdint.h>

#define TARGETS_MAX 8

typedef struct {
    bool valid;         /**< Somebody is tracked in this slot */
    int16_t x_mm;       /**< Lateral offset, positive to the right of the sensor */
    int16_t y_mm;       /**< Distance straight ahead of the sensor */
    int16_t speed_cm_s; /**< Radial speed, negative while approaching (LD2450 convention) */
} target_t;

typedef struct {
    target_t list[TARGETS_MAX];
    int count;            /**< Slots in use (including empty ones) */
    int64_t timestamp_us; /**< esp_timer time of the frame */
} target_frame_t;

/**
 * @brief Create the lock and clear the list
 */
void targets_init(void);

/**
 * @brief Replace the whole list (called by the sensor driver or the simulator)
 */
void targets_publish(const target_t *list, int count);

/**
 * @brief Copy the latest frame
 */
void targets_get(target_frame_t *frame);

/**
 * @brief Straight-line distance from the sensor in mm
 */
int32_t target_distance_mm(const target_t *target);

/**
 * @brief Bearing in degrees, 0 straight ahead, clockwise, 0..359
 */
int32_t target_bearing_deg(const target_t *target);

#endif /* TARGETS_H */
