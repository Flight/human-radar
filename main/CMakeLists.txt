idf_component_register(
  SRC_DIRS "." "radar_ui" "sound" "targets" "target_sim" "ld2450" "scene" "fog" "leds" "servo"
  INCLUDE_DIRS "."
  PRIV_REQUIRES spi_flash esp_psram esp_app_format esp_timer esp_driver_gpio esp_driver_ledc esp_driver_uart
                esp32_p4_wifi6_touch_lcd_4b lvgl led_strip
)

# Voice and scream clips (assets/*.wav) are packed into the "storage" SPIFFS partition and flashed with the app
spiffs_create_partition_image(storage ../assets FLASH_IN_PROJECT)
