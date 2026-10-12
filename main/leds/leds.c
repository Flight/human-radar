/**
 * @file leds.c
 * @brief WS2812 LEDs in the pumpkin: one animated pattern per scene stage
 *
 * A small task repaints the whole strip 40 times a second from the pattern's
 * colour function of time. Patterns are uniform across the strip, which is
 * what a glowing pumpkin wants; the brightness ceiling comes from Kconfig.
 */

#include <sdkconfig.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>

#include "leds.h"

static const char *TAG = "leds";

#ifdef CONFIG_LEDS_ENABLED

#include <led_strip.h>

#define FRAME_MS  25
#define TWO_PI    6.28318530f

typedef struct {
    uint8_t r, g, b;
} rgb_t;

static led_strip_handle_t s_strip;
static volatile leds_pattern_t s_pattern = LEDS_PATTERN_IDLE;
static volatile uint32_t s_pattern_epoch_ms;

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static rgb_t scaled(rgb_t color, float k)
{
    k *= CONFIG_LEDS_BRIGHTNESS / 100.0f;
    if (k < 0.0f) {
        k = 0.0f;
    }
    return (rgb_t){ (uint8_t)(color.r * k), (uint8_t)(color.g * k), (uint8_t)(color.b * k) };
}

static float breathe(uint32_t t_ms, uint32_t period_ms, float low, float high)
{
    return low + (high - low) * (0.5f + 0.5f * sinf((float)(t_ms % period_ms) * TWO_PI / period_ms));
}

static rgb_t pattern_color(leds_pattern_t pattern, uint32_t t_ms)
{
    const rgb_t green = { 40, 255, 20 };
    const rgb_t amber = { 255, 140, 0 };
    const rgb_t red = { 255, 15, 0 };
    const rgb_t white = { 255, 255, 255 };

    switch (pattern) {
        case LEDS_PATTERN_CONTACT:
            return scaled(green, breathe(t_ms, 1500, 0.2f, 0.8f));
        case LEDS_PATTERN_THREAT: {
            float phase = (float)(t_ms % 1000) / 1000.0f; /* sharp attack, slow decay, once a second */
            return scaled(amber, expf(-4.0f * phase));
        }
        case LEDS_PATTERN_PANIC: {
            uint32_t flash = t_ms / 60; /* about 8 Hz */
            if (flash % 2) {
                return (rgb_t){ 0, 0, 0 };
            }
            return scaled(((flash / 2) % 4 == 3) ? white : red, 1.0f);
        }
        case LEDS_PATTERN_IDLE:
        default:
            return scaled(green, breathe(t_ms, 3000, 0.08f, 0.4f));
    }
}

static void leds_task(void *arg)
{
    while (true) {
        rgb_t c = pattern_color(s_pattern, now_ms() - s_pattern_epoch_ms);
        for (int i = 0; i < CONFIG_LEDS_COUNT; i++) {
            led_strip_set_pixel(s_strip, i, c.r, c.g, c.b);
        }
        led_strip_refresh(s_strip);
        vTaskDelay(pdMS_TO_TICKS(FRAME_MS));
    }
}

esp_err_t leds_init(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = CONFIG_LEDS_GPIO,
        .max_leds = CONFIG_LEDS_COUNT,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags.invert_out = false,
    };
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };
    ESP_RETURN_ON_ERROR(led_strip_new_rmt_device(&strip_config, &rmt_config, &s_strip), TAG, "LED strip init failed");
    led_strip_clear(s_strip);
    s_pattern_epoch_ms = now_ms();

    BaseType_t ok = xTaskCreate(&leds_task, "LEDs", 3072, NULL, 3, NULL);
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_FAIL, TAG, "LED task creation failed");
    ESP_LOGI(TAG, "%d LEDs on GPIO%d at %d %%", CONFIG_LEDS_COUNT, CONFIG_LEDS_GPIO, CONFIG_LEDS_BRIGHTNESS);
    return ESP_OK;
}

void leds_set_pattern(leds_pattern_t pattern)
{
    if (pattern != s_pattern) {
        s_pattern = pattern;
        s_pattern_epoch_ms = now_ms();
    }
}

#else /* CONFIG_LEDS_ENABLED */

esp_err_t leds_init(void)
{
    ESP_LOGI(TAG, "LEDs disabled");
    return ESP_OK;
}
void leds_set_pattern(leds_pattern_t pattern) { (void)pattern; }

#endif /* CONFIG_LEDS_ENABLED */
