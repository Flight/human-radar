/**
 * @file targets.c
 * @brief Shared list of tracked people: the radar's data model
 */

#include <string.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_timer.h>
#include <esp_log.h>

#include "targets.h"

static const char *TAG = "targets";

#define DEGREES_PER_RADIAN 57.2957795f

static target_frame_t s_frame;
static SemaphoreHandle_t s_lock;

void targets_init(void)
{
    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateMutex();
    }
    memset(&s_frame, 0, sizeof(s_frame));
    ESP_LOGI(TAG, "Target list ready (%d slots)", TARGETS_MAX);
}

void targets_publish(const target_t *list, int count)
{
    if (count > TARGETS_MAX) {
        count = TARGETS_MAX;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (count > 0) {
        memcpy(s_frame.list, list, count * sizeof(target_t));
    }
    s_frame.count = count;
    s_frame.timestamp_us = esp_timer_get_time();
    xSemaphoreGive(s_lock);
}

void targets_get(target_frame_t *frame)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    *frame = s_frame;
    xSemaphoreGive(s_lock);
}

int32_t target_distance_mm(const target_t *target)
{
    float x = target->x_mm;
    float y = target->y_mm;
    return (int32_t)lroundf(sqrtf(x * x + y * y));
}

int32_t target_bearing_deg(const target_t *target)
{
    int32_t deg = (int32_t)lroundf(atan2f((float)target->x_mm, (float)target->y_mm) * DEGREES_PER_RADIAN);
    return (deg + 360) % 360;
}
