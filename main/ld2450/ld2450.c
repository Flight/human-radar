/**
 * @file ld2450.c
 * @brief HLK-LD2450 24 GHz mmWave sensor: up to three people, x/y/speed at 10 Hz over UART
 *
 * Wiring (header P3): sensor TX -> CONFIG_LD2450_RX_GPIO, sensor RX -> CONFIG_LD2450_TX_GPIO,
 * 5V and GND. The sensor streams 30-byte frames at 256000 baud:
 * AA FF 03 00 | 3 x (x, y, speed, resolution; int16 LE each) | 55 CC.
 * Coordinates use a sign flag in bit 15 (set = positive); an absent target is all zeros.
 */

#include <sdkconfig.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>

#include "targets/targets.h"
#include "ld2450.h"

static const char *TAG = "ld2450";

#ifdef CONFIG_LD2450_ENABLED

#include <driver/uart.h>

#define LD2450_UART        CONFIG_LD2450_UART_NUM
#define LD2450_BAUD        256000
#define LD2450_FRAME_LEN   30
#define LD2450_TARGETS     3
#define LD2450_RX_BUF      1024
#define LD2450_SILENCE_MS  3000

static const uint8_t FRAME_HEADER[4] = { 0xAA, 0xFF, 0x03, 0x00 };
static const uint8_t FRAME_TAIL[2] = { 0x55, 0xCC };

static int16_t decode(const uint8_t *p)
{
    uint16_t raw = (uint16_t)(p[0] | (p[1] << 8));
    int16_t value = (int16_t)(raw & 0x7FFF);
    return (raw & 0x8000) ? value : (int16_t)-value;
}

static void publish_frame(const uint8_t *payload)
{
    target_t list[LD2450_TARGETS];
    for (int i = 0; i < LD2450_TARGETS; i++) {
        const uint8_t *p = payload + i * 8;
        int16_t x = decode(p);
        int16_t y = decode(p + 2);
#ifdef CONFIG_LD2450_MIRROR_X
        x = (int16_t)-x;
#endif
        list[i].valid = (x != 0 || y != 0);
        list[i].x_mm = x;
        list[i].y_mm = y;
        list[i].speed_cm_s = decode(p + 4);
    }
    targets_publish(list, LD2450_TARGETS);
}

static void ld2450_task(void *arg)
{
    uint8_t frame[LD2450_FRAME_LEN];
    int filled = 0;
    int64_t last_frame_us = esp_timer_get_time();
    bool silent = false;
    uint32_t frames = 0;

    while (true) {
        uint8_t byte;
        if (uart_read_bytes(LD2450_UART, &byte, 1, pdMS_TO_TICKS(100)) != 1) {
            if (!silent && esp_timer_get_time() - last_frame_us > (int64_t)LD2450_SILENCE_MS * 1000) {
                ESP_LOGW(TAG, "No data from the LD2450 (sensor TX -> GPIO%d, 256000 baud)", CONFIG_LD2450_RX_GPIO);
                targets_publish(NULL, 0);
                silent = true;
            }
            continue;
        }

        /* Resynchronise on the header, then collect a whole frame */
        if (filled < 4) {
            if (byte == FRAME_HEADER[filled]) {
                frame[filled++] = byte;
            } else {
                filled = (byte == FRAME_HEADER[0]) ? 1 : 0;
            }
            continue;
        }
        frame[filled++] = byte;
        if (filled < LD2450_FRAME_LEN) {
            continue;
        }
        filled = 0;
        if (memcmp(frame + LD2450_FRAME_LEN - 2, FRAME_TAIL, 2) != 0) {
            ESP_LOGD(TAG, "Bad frame tail");
            continue;
        }

        publish_frame(frame + 4);
        last_frame_us = esp_timer_get_time();
        if (silent || frames == 0) {
            ESP_LOGI(TAG, "LD2450 streaming");
            silent = false;
        }
        frames++;
    }
}

esp_err_t ld2450_start(void)
{
    const uart_config_t uart_config = {
        .baud_rate = LD2450_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(uart_driver_install(LD2450_UART, LD2450_RX_BUF, 0, 0, NULL, 0), TAG, "UART install failed");
    ESP_RETURN_ON_ERROR(uart_param_config(LD2450_UART, &uart_config), TAG, "UART config failed");
    ESP_RETURN_ON_ERROR(uart_set_pin(LD2450_UART, CONFIG_LD2450_TX_GPIO, CONFIG_LD2450_RX_GPIO,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE), TAG, "UART pins failed");

    BaseType_t ok = xTaskCreate(&ld2450_task, "LD2450", configMINIMAL_STACK_SIZE * 3, NULL, 4, NULL);
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_FAIL, TAG, "LD2450 task creation failed");
    ESP_LOGI(TAG, "LD2450 on UART%d: RX GPIO%d, TX GPIO%d", LD2450_UART, CONFIG_LD2450_RX_GPIO, CONFIG_LD2450_TX_GPIO);
    return ESP_OK;
}

#else /* CONFIG_LD2450_ENABLED */

esp_err_t ld2450_start(void)
{
    ESP_LOGI(TAG, "LD2450 driver disabled");
    return ESP_OK;
}

#endif /* CONFIG_LD2450_ENABLED */
