/**
 * @file servo.c
 * @brief Pan servo that turns the pumpkin toward the nearest person
 *
 * A hobby servo on LEDC timer 2 / channel 4 (the BSP backlight uses timer 1 /
 * channel 1): 50 Hz, pulse CONFIG_SERVO_MIN_US at -90 degrees to
 * CONFIG_SERVO_MAX_US at +90. A task slews toward the requested angle at a
 * limited speed so the pumpkin turns its head rather than snapping.
 */

#include <sdkconfig.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_check.h>

#include "servo.h"

static const char *TAG = "servo";

#ifdef CONFIG_SERVO_ENABLED

#include <driver/ledc.h>

#define SERVO_TIMER      LEDC_TIMER_2
#define SERVO_CHANNEL    LEDC_CHANNEL_4
#define SERVO_MODE       LEDC_LOW_SPEED_MODE
#define SERVO_RESOLUTION LEDC_TIMER_14_BIT
#define SERVO_DUTY_MAX   (1 << 14)
#define SERVO_FREQ_HZ    50
#define SERVO_PERIOD_US  20000
#define SERVO_LIMIT_DEG  90
#define SLEW_STEP_MS     20

static volatile int s_target_deg;
static float s_current_deg;

static int clamp_deg(int deg)
{
    if (deg > SERVO_LIMIT_DEG) {
        return SERVO_LIMIT_DEG;
    }
    if (deg < -SERVO_LIMIT_DEG) {
        return -SERVO_LIMIT_DEG;
    }
    return deg;
}

static void servo_write(float deg)
{
    deg = (float)clamp_deg((int)deg + CONFIG_SERVO_TRIM_DEG);
#ifdef CONFIG_SERVO_REVERSED
    deg = -deg;
#endif
    float pulse_us = CONFIG_SERVO_MIN_US + (deg + SERVO_LIMIT_DEG) / (2.0f * SERVO_LIMIT_DEG) *
                     (CONFIG_SERVO_MAX_US - CONFIG_SERVO_MIN_US);
    uint32_t duty = (uint32_t)(pulse_us * SERVO_DUTY_MAX / SERVO_PERIOD_US);
    ledc_set_duty(SERVO_MODE, SERVO_CHANNEL, duty);
    ledc_update_duty(SERVO_MODE, SERVO_CHANNEL);
}

static void servo_task(void *arg)
{
    const float step = (float)CONFIG_SERVO_SPEED_DEG_S * SLEW_STEP_MS / 1000.0f;
    while (true) {
        float target = (float)s_target_deg;
        if (s_current_deg != target) {
            float delta = target - s_current_deg;
            if (delta > step) {
                delta = step;
            } else if (delta < -step) {
                delta = -step;
            }
            s_current_deg += delta;
            servo_write(s_current_deg);
        }
        vTaskDelay(pdMS_TO_TICKS(SLEW_STEP_MS));
    }
}

esp_err_t servo_init(void)
{
    const ledc_timer_config_t timer_config = {
        .speed_mode = SERVO_MODE,
        .duty_resolution = SERVO_RESOLUTION,
        .timer_num = SERVO_TIMER,
        .freq_hz = SERVO_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer_config), TAG, "LEDC timer config failed");
    const ledc_channel_config_t channel_config = {
        .gpio_num = CONFIG_SERVO_GPIO,
        .speed_mode = SERVO_MODE,
        .channel = SERVO_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = SERVO_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&channel_config), TAG, "LEDC channel config failed");
    s_current_deg = 0.0f;
    s_target_deg = 0;
    servo_write(0.0f);

    BaseType_t ok = xTaskCreate(&servo_task, "Servo", 2048, NULL, 3, NULL);
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_FAIL, TAG, "Servo task creation failed");
    ESP_LOGI(TAG, "Servo on GPIO%d, %d..%d us, %d deg/s", CONFIG_SERVO_GPIO, CONFIG_SERVO_MIN_US,
             CONFIG_SERVO_MAX_US, CONFIG_SERVO_SPEED_DEG_S);
    return ESP_OK;
}

void servo_point(int bearing_deg)
{
    s_target_deg = clamp_deg(bearing_deg);
}

void servo_center(void)
{
    s_target_deg = 0;
}

int servo_get_angle(void)
{
    return (int)s_current_deg;
}

#else /* CONFIG_SERVO_ENABLED */

esp_err_t servo_init(void)
{
    ESP_LOGI(TAG, "Servo disabled");
    return ESP_OK;
}
void servo_point(int bearing_deg) { (void)bearing_deg; }
void servo_center(void) {}
int servo_get_angle(void) { return 0; }

#endif /* CONFIG_SERVO_ENABLED */
