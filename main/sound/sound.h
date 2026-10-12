/**
 * @file sound.h
 * @brief Radar sounds on the built-in speaker: synthesised effects and voice/scream clips
 */

#ifndef SOUND_H
#define SOUND_H

#include <stdbool.h>
#include <stdint.h>
#include <esp_err.h>

typedef enum {
    SOUND_STARTUP,      /**< Rising chirp: the radar powers up */
    SOUND_SWEEP,        /**< Soft sonar ping with an echo, once per revolution */
    SOUND_CONTACT_FAR,  /**< Low blip: the sweep painted someone far away */
    SOUND_CONTACT_MID,  /**< Mid blip */
    SOUND_CONTACT_NEAR, /**< High double blip: someone is close */
    SOUND_ALERT,        /**< Two-tone alarm */
    SOUND_THREAT,       /**< Three rising tones, fallback for the "threat detected" clip */
    SOUND_SCREAM,       /**< Siren sweeps, fallback for the scream clips */
    SOUND_COUNT,
} sound_id_t;

typedef enum {
    SAMPLE_THREAT,  /**< assets/threat.wav: "Threat detected" */
    SAMPLE_SCREAM,  /**< assets/scream.wav */
    SAMPLE_GET_OUT, /**< assets/getout.wav: "Get out" */
    SAMPLE_COUNT,
} sample_id_t;

/**
 * @brief Open the speaker codec, load the clips from the storage partition and start the mixer task
 *
 * @return ESP_OK, or an error when the codec could not be opened (sounds are then dropped)
 */
esp_err_t sound_init(void);

/**
 * @brief Queue a synthesised sound; several sounds play at once, the call never blocks
 */
void sound_play(sound_id_t id);

/**
 * @brief Queue a clip; a missing clip plays its synthesised fallback instead
 */
void sound_play_sample(sample_id_t id);

/**
 * @brief Play the contact blip that matches the distance of the painted person
 */
void sound_play_contact(int32_t distance_mm);

/**
 * @brief Change the speaker volume (0..100 %)
 */
void sound_set_volume(int percent);

/**
 * @brief Enable or mute all sounds (the SOUND ON/OFF button on the screen)
 *
 * Starts muted unless CONFIG_SOUND_ON_AT_BOOT is set. Muting stops the sounds
 * that are playing and drops anything queued afterwards.
 */
void sound_set_enabled(bool enabled);

/**
 * @brief Whether sounds are currently enabled
 */
bool sound_is_enabled(void);

#endif /* SOUND_H */
