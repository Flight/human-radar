/**
 * @file target_sim.c
 * @brief Fake visitors until the LD2450 sensor is wired up
 *
 * The sensor sits next to you facing the door, so a visitor appears at the far
 * edge of the sensor's cone (the door), walks up to a spot right behind you,
 * lingers there, walks back out and is gone for a while before the next entry.
 */

#include <sdkconfig.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_random.h>
#include <esp_log.h>

#include "targets/targets.h"
#include "target_sim.h"

static const char *TAG = "target_sim";

#ifdef CONFIG_TARGET_SIM_ENABLED

#define SIM_STEP_MS         100
#define SIM_COUNT           CONFIG_TARGET_SIM_COUNT
#define SIM_RANGE_MM        CONFIG_RADAR_RANGE_MM
#define SIM_DOOR_HALF_ANGLE 50.0f /* the LD2450 sees +/-60 degrees */
#define SIM_MIN_SPEED_MM_S  400   /* slow shuffle */
#define SIM_MAX_SPEED_MM_S  1100  /* brisk walk */
#define SIM_ARRIVE_MM       150
#define SIM_PI              3.14159265f

typedef enum {
    WALK_ENTER,  /* from the door toward the spot behind you */
    WALK_LINGER, /* standing behind you */
    WALK_LEAVE,  /* back to the door */
    WALK_GONE,   /* out of the room */
} walk_phase_t;

typedef struct {
    walk_phase_t phase;
    float x, y;           /* mm */
    float goal_x, goal_y; /* where this phase walks to */
    float speed;          /* mm/s */
    int32_t phase_ms;     /* time left in LINGER / GONE */
    float radial_mm_s;    /* for the speed field, negative while approaching */
} walker_t;

static float rand_range(float lo, float hi)
{
    return lo + (hi - lo) * (float)(esp_random() % 10000) / 10000.0f;
}

static void door_point(float *x, float *y, float radius_factor)
{
    float bearing = rand_range(-SIM_DOOR_HALF_ANGLE, SIM_DOOR_HALF_ANGLE) * SIM_PI / 180.0f;
    *x = radius_factor * SIM_RANGE_MM * sinf(bearing);
    *y = radius_factor * SIM_RANGE_MM * cosf(bearing);
}

static void walker_enter(walker_t *w)
{
    door_point(&w->x, &w->y, 0.95f);
    /* Heads for a spot right behind you, inside the breach ring */
    float bearing = rand_range(-40.0f, 40.0f) * SIM_PI / 180.0f;
    float distance = rand_range(CONFIG_SCENE_BREACH_RANGE_MM * 0.4f, CONFIG_SCENE_BREACH_RANGE_MM * 0.8f);
    w->goal_x = distance * sinf(bearing);
    w->goal_y = distance * cosf(bearing);
    w->speed = rand_range(SIM_MIN_SPEED_MM_S, SIM_MAX_SPEED_MM_S);
    w->phase = WALK_ENTER;
}

static void walker_leave(walker_t *w)
{
    door_point(&w->goal_x, &w->goal_y, 1.05f);
    w->speed = rand_range(SIM_MIN_SPEED_MM_S, SIM_MAX_SPEED_MM_S);
    w->phase = WALK_LEAVE;
}

static void walker_gone(walker_t *w, float min_s, float max_s)
{
    w->phase = WALK_GONE;
    w->phase_ms = (int32_t)rand_range(min_s * 1000.0f, max_s * 1000.0f);
}

/* Walk toward the goal with a little wobble; true once there */
static bool walker_move(walker_t *w)
{
    float dx = w->goal_x - w->x;
    float dy = w->goal_y - w->y;
    float dist = sqrtf(dx * dx + dy * dy);
    if (dist <= SIM_ARRIVE_MM) {
        return true;
    }
    float step = w->speed * SIM_STEP_MS / 1000.0f;
    float wobble = rand_range(-0.3f, 0.3f);
    float c = cosf(wobble);
    float s = sinf(wobble);
    float ux = dx / dist;
    float uy = dy / dist;
    float vx = (ux * c - uy * s) * w->speed;
    float vy = (ux * s + uy * c) * w->speed;
    w->x += vx * SIM_STEP_MS / 1000.0f;
    w->y += vy * SIM_STEP_MS / 1000.0f;
    float r = sqrtf(w->x * w->x + w->y * w->y);
    w->radial_mm_s = (r > 1.0f) ? (w->x * vx + w->y * vy) / r : 0.0f;
    return step >= dist;
}

static void walker_step(walker_t *w)
{
    switch (w->phase) {
        case WALK_ENTER:
            if (walker_move(w)) {
                w->phase = WALK_LINGER;
                w->phase_ms = (int32_t)rand_range(3000, 9000);
                w->radial_mm_s = 0.0f;
            }
            break;
        case WALK_LINGER:
            /* Shuffle on the spot */
            w->x += rand_range(-40.0f, 40.0f);
            w->y += rand_range(-40.0f, 40.0f);
            w->phase_ms -= SIM_STEP_MS;
            if (w->phase_ms <= 0) {
                walker_leave(w);
            }
            break;
        case WALK_LEAVE:
            if (walker_move(w) || sqrtf(w->x * w->x + w->y * w->y) > SIM_RANGE_MM) {
                walker_gone(w, 2.0f, 12.0f);
            }
            break;
        case WALK_GONE:
            w->phase_ms -= SIM_STEP_MS;
            if (w->phase_ms <= 0) {
                walker_enter(w);
            }
            break;
    }
}

static void target_sim_task(void *arg)
{
    walker_t walkers[SIM_COUNT];
    target_t frame[SIM_COUNT];

    /* Stagger the first entries so the visitors do not arrive as a group */
    for (int i = 0; i < SIM_COUNT; i++) {
        walker_gone(&walkers[i], 1.0f + 6.0f * i, 4.0f + 6.0f * i);
    }

    while (true) {
        for (int i = 0; i < SIM_COUNT; i++) {
            walker_t *w = &walkers[i];
            walker_step(w);
            frame[i].valid = (w->phase != WALK_GONE);
            frame[i].x_mm = (int16_t)w->x;
            frame[i].y_mm = (int16_t)w->y;
            frame[i].speed_cm_s = (int16_t)(w->radial_mm_s / 10.0f);
        }
        targets_publish(frame, SIM_COUNT);
        vTaskDelay(pdMS_TO_TICKS(SIM_STEP_MS));
    }
}

void target_sim_start(void)
{
    ESP_LOGI(TAG, "Simulating %d visitors walking in through the door", SIM_COUNT);
    xTaskCreate(&target_sim_task, "Target Sim", configMINIMAL_STACK_SIZE * 3, NULL, 2, NULL);
}

#else /* CONFIG_TARGET_SIM_ENABLED */

void target_sim_start(void)
{
    ESP_LOGI(TAG, "Target simulator disabled");
}

#endif /* CONFIG_TARGET_SIM_ENABLED */
