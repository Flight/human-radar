/**
 * @file fog.c
 * @brief Fog generator trigger in the pumpkin (MOSFET/relay module on a GPIO)
 *
 * The ultrasonic fog maker starts within a second, so a burst is simply the
 * pin held active for CONFIG_FOG_BURST_MS; a one-shot timer switches it off.
 */

#include <sdkconfig.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>

#include "fog.h"

static const char *TAG = "fog";

#ifdef CONFIG_FOG_ENABLED

#include <driver/gpio.h>

#ifdef CONFIG_FOG_ACTIVE_LOW
#define FOG_ON_LEVEL  0
#define FOG_OFF_LEVEL 1
#else
#define FOG_ON_LEVEL  1
#define FOG_OFF_LEVEL 0
#endif

static esp_timer_handle_t s_off_timer;
static volatile bool s_on;
static int64_t s_last_burst_us = -1;

static void fog_set(bool on)
{
    gpio_set_level(CONFIG_FOG_GPIO, on ? FOG_ON_LEVEL : FOG_OFF_LEVEL);
    s_on = on;
}

static void off_timer_cb(void *arg)
{
    fog_set(false);
    ESP_LOGI(TAG, "Fog off");
}

esp_err_t fog_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << CONFIG_FOG_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "GPIO config failed");
    fog_set(false);

    const esp_timer_create_args_t timer_args = { .callback = off_timer_cb, .name = "fog_off" };
    ESP_RETURN_ON_ERROR(esp_timer_create(&timer_args, &s_off_timer), TAG, "Timer creation failed");
    ESP_LOGI(TAG, "Fog trigger on GPIO%d (active %s), bursts of %d ms at most every %d ms", CONFIG_FOG_GPIO,
             FOG_ON_LEVEL ? "HIGH" : "LOW", CONFIG_FOG_BURST_MS, CONFIG_FOG_MIN_INTERVAL_MS);
    return ESP_OK;
}

bool fog_burst(void)
{
    int64_t now = esp_timer_get_time();
    if (s_last_burst_us >= 0 && now - s_last_burst_us < (int64_t)CONFIG_FOG_MIN_INTERVAL_MS * 1000) {
        return false;
    }
    s_last_burst_us = now;
    fog_set(true);
    esp_timer_stop(s_off_timer); /* harmless when it is not running */
    esp_timer_start_once(s_off_timer, (uint64_t)CONFIG_FOG_BURST_MS * 1000);
    ESP_LOGI(TAG, "Fog on for %d ms", CONFIG_FOG_BURST_MS);
    return true;
}

void fog_stop(void)
{
    if (s_on) {
        esp_timer_stop(s_off_timer);
        fog_set(false);
        ESP_LOGI(TAG, "Fog off");
    }
}

bool fog_is_on(void)
{
    return s_on;
}

#else /* CONFIG_FOG_ENABLED */

esp_err_t fog_init(void)
{
    ESP_LOGI(TAG, "Fog trigger disabled");
    return ESP_OK;
}
bool fog_burst(void) { return false; }
void fog_stop(void) {}
bool fog_is_on(void) { return false; }

#endif /* CONFIG_FOG_ENABLED */
