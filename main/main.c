/**
 * @file main.c
 * @brief Human Radar - Halloween radar scope with a screaming pumpkin
 *        (Waveshare ESP32-P4-WIFI6-Touch-LCD-4B)
 *
 * Boot order: board info, the shared target list, display and LVGL (BSP), the
 * radar scope, backlight, speaker, the pumpkin effects (fog, LEDs, servo), the
 * scene state machine, and finally the target source: the LD2450 sensor
 * and/or the visitor simulator, as configured.
 */

#include <sdkconfig.h>
#include <inttypes.h>
#include <esp_log.h>
#include <esp_chip_info.h>
#include <esp_flash.h>
#include <esp_system.h>
#include <esp_app_desc.h>
#ifdef CONFIG_SPIRAM
#include <esp_psram.h>
#endif
#include "bsp/esp-bsp.h"

#include "targets/targets.h"
#include "target_sim/target_sim.h"
#include "ld2450/ld2450.h"
#include "sound/sound.h"
#include "fog/fog.h"
#include "leds/leds.h"
#include "servo/servo.h"
#include "scene/scene.h"
#include "radar_ui/radar_ui.h"

static const char *TAG = "human_radar";

#define BYTES_PER_MB (1024 * 1024)

/* Log the chip, flash and PSRAM the firmware sees, so a wrong sdkconfig.defaults
 * (flash size, PSRAM mode) shows up on the very first boot. */
static void log_board_info(void)
{
    const esp_app_desc_t *app = esp_app_get_description();
    ESP_LOGI(TAG, "%s %s, built with ESP-IDF %s", app->project_name, app->version, app->idf_ver);

    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "Chip: %s, %d core(s), silicon revision v%d.%d", CONFIG_IDF_TARGET, chip_info.cores,
             chip_info.revision / 100, chip_info.revision % 100);

    uint32_t flash_size = 0;
    if (esp_flash_get_size(NULL, &flash_size) == ESP_OK) {
        ESP_LOGI(TAG, "Flash: %" PRIu32 " MB", flash_size / BYTES_PER_MB);
    }
#ifdef CONFIG_SPIRAM
    /* The P4NRW32 stacks the PSRAM in the package, but esp_chip_info() does not flag it as embedded */
    ESP_LOGI(TAG, "PSRAM: %u MB", (unsigned)(esp_psram_get_size() / BYTES_PER_MB));
#endif
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Human Radar");
    log_board_info();

    targets_init();
    scene_init(); /* the scope reads the scene state from its first frame on */

    /* MIPI DSI panel, GT911 touch and the LVGL task come from the BSP. Triple
     * partial buffering measured about 30 fps for the scope against 17 fps in
     * direct mode. The custom draw callback needs a roomy LVGL task stack. */
    bsp_display_cfg_t display_cfg = {
        .lv_adapter_cfg = ESP_LV_ADAPTER_DEFAULT_CONFIG(),
        .rotation = ESP_LV_ADAPTER_ROTATE_0,
        .tear_avoid_mode = ESP_LV_ADAPTER_TEAR_AVOID_MODE_TRIPLE_PARTIAL,
    };
    display_cfg.lv_adapter_cfg.task_stack_size = 16 * 1024;
    if (bsp_display_start_with_config(&display_cfg) == NULL) {
        ESP_LOGE(TAG, "Display start failed");
        return;
    }
    if (radar_ui_start() != ESP_OK) {
        return;
    }
    /* Backlight on only now, so the panel never shows garbage before the first frame */
    bsp_display_brightness_set(CONFIG_DISPLAY_BRIGHTNESS);

    if (sound_init() == ESP_OK) {
        sound_play(SOUND_STARTUP);
    } else {
        ESP_LOGW(TAG, "Running without sound");
    }

    /* Pumpkin effects: each one logs and carries on when its hardware is missing */
    if (fog_init() != ESP_OK) {
        ESP_LOGW(TAG, "Running without fog");
    }
    if (leds_init() != ESP_OK) {
        ESP_LOGW(TAG, "Running without LEDs");
    }
    if (servo_init() != ESP_OK) {
        ESP_LOGW(TAG, "Running without the servo");
    }
    scene_start();

#ifdef CONFIG_LD2450_ENABLED
    if (ld2450_start() != ESP_OK) {
        ESP_LOGW(TAG, "LD2450 not started");
    }
#endif
    target_sim_start();
    ESP_LOGI(TAG, "Minimum free heap: %" PRIu32 " bytes", esp_get_minimum_free_heap_size());
}
