/**
 * @file servo.h
 * @brief Pan servo that turns the pumpkin toward the nearest person
 */

#ifndef SERVO_H
#define SERVO_H

#include <esp_err.h>

/**
 * @brief Start the 50 Hz PWM (centred) and the slew task
 */
esp_err_t servo_init(void);

/**
 * @brief Turn toward a bearing: 0 faces the door, positive to the right, clamped to +/-90 degrees
 *
 * The servo slews there at CONFIG_SERVO_SPEED_DEG_S.
 */
void servo_point(int bearing_deg);

/**
 * @brief Back to the centre
 */
void servo_center(void);

/**
 * @brief The angle the servo is at right now (degrees)
 */
int servo_get_angle(void);

#endif /* SERVO_H */
