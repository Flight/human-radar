/**
 * @file scene.h
 * @brief The Halloween scene: escalation stages driven by the nearest person
 */

#ifndef SCENE_H
#define SCENE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    SCENE_CLEAR,   /**< Nobody in the room */
    SCENE_CONTACT, /**< Someone came through the door */
    SCENE_THREAT,  /**< Someone is close: warning, pumpkin turns toward them */
    SCENE_BREACH,  /**< Someone is right behind you: scream, fog, strobe */
} scene_stage_t;

typedef struct {
    scene_stage_t stage;
    int contacts;               /**< People in range */
    bool nearest_valid;
    int32_t nearest_mm;         /**< Distance of the nearest person */
    int32_t nearest_bearing_deg;
    bool fog_on;
    int pumpkin_deg;            /**< Where the pumpkin is looking */
    bool test;                  /**< A tap on the scope is simulating a breach */
} scene_state_t;

/**
 * @brief Create the state lock; call before the scope starts asking for the state
 */
void scene_init(void);

/**
 * @brief Start the scene task (effects and sound must be initialised first)
 */
void scene_start(void);

/**
 * @brief Copy the current state (for the scope)
 */
void scene_get(scene_state_t *state);

/**
 * @brief Simulate a breach for CONFIG_SCENE_TEST_BREACH_MS (a tap on the scope)
 */
void scene_test_breach(void);

/**
 * @brief Upper-case name of a stage for the display
 */
const char *scene_stage_name(scene_stage_t stage);

#endif /* SCENE_H */
