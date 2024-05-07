/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include "bsp/esp-bsp.h"
#include "lvgl.h"
#include "esp_log.h"

extern void example_lvgl_demo_ui(lv_obj_t *scr);

void app_main(void)
{
    bsp_display_start();

    ESP_LOGI("example", "Display LVGL animation");
    bsp_display_lock(0);
    lv_obj_t *scr = lv_disp_get_scr_act(NULL);
    lv_obj_set_style_bg_color(scr, lv_color_make(0x00, 0xFF, 0x00), 0);
    example_lvgl_demo_ui(scr);

    bsp_display_unlock();
    bsp_display_backlight_on();
}
