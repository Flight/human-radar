/**
 * @file target_sim.h
 * @brief Fake wandering people until the LD2450 sensor is wired up
 */

#ifndef TARGET_SIM_H
#define TARGET_SIM_H

/**
 * @brief Start publishing simulated targets (no-op when CONFIG_TARGET_SIM_ENABLED is off)
 */
void target_sim_start(void);

#endif /* TARGET_SIM_H */
