/**
 * @file scene.c
 * @brief The Halloween scene: escalation stages driven by the nearest person
 *
 * Twenty times a second the scene reads the target list, finds the nearest
 * person and picks a stage: CLEAR, CONTACT (someone in the room), THREAT
 * (inside CONFIG_SCENE_THREAT_RANGE_MM) or BREACH (inside
 * CONFIG_SCENE_BREACH_RANGE_MM, right behind you). Escalation is immediate;
 * stepping down needs the person to back off by the hysteresis distance and
 * stay away for CONFIG_SCENE_HOLD_MS. Each stage drives the LEDs, the servo,
 * the fog and the sounds; the scope only displays the result.
 */

#include <sdkconfig.h>
#include <math.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_log.h>
#include <esp_timer.h>

#include "targets/targets.h"
#include "sound/sound.h"
#include "fog/fog.h"
#include "leds/leds.h"
#include "servo/servo.h"
#include "scene.h"

static const char *TAG = "scene";

#define TICK_MS            50
#define DEGREES_PER_RADIAN 57.2957795f
#define TEST_DISTANCE_MM   600 /* where the pretend intruder stands during a test */

static const char *s_stage_names[] = { "CLEAR", "CONTACT", "THREAT", "BREACH" };

static SemaphoreHandle_t s_lock;
static scene_state_t s_state;
static int32_t s_nearest_x_mm;
static int32_t s_nearest_y_mm;
static uint32_t s_test_until_ms;
static uint32_t s_below_since_ms; /* when the wanted stage dropped below the current one, 0 = it has not */
static uint32_t s_last_voice_ms;
static bool s_scream_turn;        /* BREACH alternates scream and "get out" */

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/* Which stage the nearest distance asks for; a stage already reached is kept a little longer (hysteresis) */
static scene_stage_t wanted_stage(const scene_state_t *s, scene_stage_t current)
{
    if (!s->nearest_valid) {
        return SCENE_CLEAR;
    }
    int32_t breach = CONFIG_SCENE_BREACH_RANGE_MM + (current >= SCENE_BREACH ? CONFIG_SCENE_HYSTERESIS_MM : 0);
    int32_t threat = CONFIG_SCENE_THREAT_RANGE_MM + (current >= SCENE_THREAT ? CONFIG_SCENE_HYSTERESIS_MM : 0);
    if (s->nearest_mm <= breach) {
        return SCENE_BREACH;
    }
    if (s->nearest_mm <= threat) {
        return SCENE_THREAT;
    }
    return SCENE_CONTACT;
}

/* Bearing of the nearest person as seen from the pumpkin, which may sit away from the sensor */
static int pumpkin_bearing_deg(void)
{
    float dx = (float)(s_nearest_x_mm - CONFIG_PUMPKIN_X_MM);
    float dy = (float)(s_nearest_y_mm - CONFIG_PUMPKIN_Y_MM);
    return (int)lroundf(atan2f(dx, dy) * DEGREES_PER_RADIAN);
}

static void enter_stage(scene_state_t *s, scene_stage_t stage, uint32_t now)
{
    ESP_LOGI(TAG, "Stage %s -> %s (nearest %d mm)", s_stage_names[s->stage], s_stage_names[stage],
             s->nearest_valid ? (int)s->nearest_mm : -1);
    s->stage = stage;
    switch (stage) {
        case SCENE_CLEAR:
            leds_set_pattern(LEDS_PATTERN_IDLE);
            servo_center();
            fog_stop();
            break;
        case SCENE_CONTACT:
            leds_set_pattern(LEDS_PATTERN_CONTACT);
            servo_center();
            fog_stop();
            break;
        case SCENE_THREAT:
            leds_set_pattern(LEDS_PATTERN_THREAT);
            fog_stop();
            sound_play(SOUND_ALERT);
            sound_play_sample(SAMPLE_THREAT);
            s_last_voice_ms = now;
            break;
        case SCENE_BREACH:
            leds_set_pattern(LEDS_PATTERN_PANIC);
            sound_play_sample(SAMPLE_SCREAM);
            s_scream_turn = false; /* "get out" comes next */
            s_last_voice_ms = now;
            fog_burst();
            break;
    }
}

static void scene_tick(void)
{
    uint32_t now = now_ms();
    scene_state_t s;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s = s_state;
    xSemaphoreGive(s_lock);

    /* Nearest person in range */
    target_frame_t frame;
    targets_get(&frame);
    s.contacts = 0;
    s.nearest_valid = false;
    s.nearest_mm = 0;
    for (int i = 0; i < frame.count && i < TARGETS_MAX; i++) {
        const target_t *t = &frame.list[i];
        if (!t->valid) {
            continue;
        }
        int32_t distance_mm = target_distance_mm(t);
        if (distance_mm > CONFIG_RADAR_RANGE_MM) {
            continue;
        }
        s.contacts++;
        if (!s.nearest_valid || distance_mm < s.nearest_mm) {
            s.nearest_valid = true;
            s.nearest_mm = distance_mm;
            s.nearest_bearing_deg = target_bearing_deg(t);
            s_nearest_x_mm = t->x_mm;
            s_nearest_y_mm = t->y_mm;
        }
    }

    /* A test pretends someone stands right behind you */
    s.test = (now < s_test_until_ms);
    if (s.test && !s.nearest_valid) {
        s.nearest_valid = true;
        s.nearest_mm = TEST_DISTANCE_MM;
        s.nearest_bearing_deg = 0;
        s_nearest_x_mm = 0;
        s_nearest_y_mm = TEST_DISTANCE_MM;
    }

    /* Escalate at once, calm down only after the hold time */
    scene_stage_t wanted = s.test ? SCENE_BREACH : wanted_stage(&s, s.stage);
    if (wanted > s.stage) {
        enter_stage(&s, wanted, now);
        s_below_since_ms = 0;
    } else if (wanted < s.stage) {
        if (s_below_since_ms == 0) {
            s_below_since_ms = now ? now : 1;
        } else if (now - s_below_since_ms >= CONFIG_SCENE_HOLD_MS) {
            enter_stage(&s, wanted, now);
            s_below_since_ms = 0;
        }
    } else {
        s_below_since_ms = 0;
    }

    /* Keep warning and screaming while they stay */
    if (s.stage == SCENE_THREAT && now - s_last_voice_ms >= CONFIG_SCENE_THREAT_REPEAT_MS) {
        sound_play_sample(SAMPLE_THREAT);
        s_last_voice_ms = now;
    }
    if (s.stage == SCENE_BREACH) {
        if (now - s_last_voice_ms >= CONFIG_SCENE_SCREAM_REPEAT_MS) {
            sound_play_sample(s_scream_turn ? SAMPLE_SCREAM : SAMPLE_GET_OUT);
            s_scream_turn = !s_scream_turn;
            s_last_voice_ms = now;
        }
        fog_burst(); /* fires again whenever the minimum interval allows */
    }

    /* The pumpkin stares at the nearest person once things get serious */
    if (s.stage >= SCENE_THREAT && s.nearest_valid) {
        s.pumpkin_deg = pumpkin_bearing_deg();
        servo_point(s.pumpkin_deg);
    } else {
        s.pumpkin_deg = 0;
    }
    s.fog_on = fog_is_on();

    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_state = s;
    xSemaphoreGive(s_lock);
}

static void scene_task(void *arg)
{
    while (true) {
        scene_tick();
        vTaskDelay(pdMS_TO_TICKS(TICK_MS));
    }
}

void scene_init(void)
{
    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateMutex();
    }
    memset(&s_state, 0, sizeof(s_state));
}

void scene_start(void)
{
    scene_init();
    leds_set_pattern(LEDS_PATTERN_IDLE);
    ESP_LOGI(TAG, "Threat inside %d mm, breach inside %d mm, hold %d ms", CONFIG_SCENE_THREAT_RANGE_MM,
             CONFIG_SCENE_BREACH_RANGE_MM, CONFIG_SCENE_HOLD_MS);
    xTaskCreate(&scene_task, "Scene", configMINIMAL_STACK_SIZE * 3, NULL, 4, NULL);
}

void scene_get(scene_state_t *state)
{
    if (s_lock == NULL) {
        memset(state, 0, sizeof(*state)); /* asked before scene_init(): everything is CLEAR */
        return;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    *state = s_state;
    xSemaphoreGive(s_lock);
}

void scene_test_breach(void)
{
    s_test_until_ms = now_ms() + CONFIG_SCENE_TEST_BREACH_MS;
    ESP_LOGI(TAG, "Test breach for %d ms", CONFIG_SCENE_TEST_BREACH_MS);
}

const char *scene_stage_name(scene_stage_t stage)
{
    return s_stage_names[stage];
}
