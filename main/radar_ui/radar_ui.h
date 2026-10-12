/**
 * @file radar_ui.h
 * @brief Green phosphor radar scope on the 720x720 panel (LVGL)
 */

#ifndef RADAR_UI_H
#define RADAR_UI_H

#include <esp_err.h>

/**
 * @brief Build the scope on the active screen and start its animation timer
 *
 * The display must already be running (bsp_display_start_with_config()).
 * A tap on the scope asks the scene for a test breach; the SOUND button
 * toggles the sounds.
 */
esp_err_t radar_ui_start(void);

#endif /* RADAR_UI_H */
