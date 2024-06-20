/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp_idf_version.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lvgl_port.h"
#include "esp_lvgl_port_priv.h"

#if SOC_PPA_SUPPORTED
#include "driver/ppa.h"
#define ALIGN_UP(num, align)    (((num) + ((align) - 1)) & ~((align) - 1))
#endif

#if CONFIG_IDF_TARGET_ESP32S3 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
#include "esp_lcd_panel_rgb.h"
#endif

#if (CONFIG_IDF_TARGET_ESP32P4 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0))
#include "esp_lcd_mipi_dsi.h"
#endif

#if (ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4, 4, 4)) || (ESP_IDF_VERSION == ESP_IDF_VERSION_VAL(5, 0, 0))
#define LVGL_PORT_HANDLE_FLUSH_READY 0
#else
#define LVGL_PORT_HANDLE_FLUSH_READY 1
#endif

static const char *TAG = "LVGL";

/*******************************************************************************
* Types definitions
*******************************************************************************/

typedef struct {
    lvgl_port_disp_type_t     disp_type;    /* Display type */
    esp_lcd_panel_io_handle_t io_handle;      /* LCD panel IO handle */
    esp_lcd_panel_handle_t    panel_handle;   /* LCD panel handle */
    esp_lcd_panel_handle_t    control_handle; /* LCD panel control handle */
    lvgl_port_rotation_cfg_t  rotation;       /* Default values of the screen rotation */
    lv_color_t                *draw_buffs[2]; /* Display draw buffers */
    lv_display_t              *disp_drv;      /* LVGL display driver */
#if SOC_PPA_SUPPORTED
    SemaphoreHandle_t         ppa_sem;
    lv_display_rotation_t     ppa_rotation;
    uint8_t                   *ppa_buffer;
    uint32_t                  ppa_buffer_size;
    ppa_client_handle_t       ppa_srm_handle;
#endif //SOC_PPA_SUPPORTED
    struct {
        unsigned int monochrome: 1;  /* True, if display is monochrome and using 1bit for 1px */
        unsigned int swap_bytes: 1;  /* Swap bytes in RGB656 (16-bit) before send to LCD driver */
        unsigned int full_refresh: 1;   /* Always make the whole screen redrawn */
        unsigned int direct_mode: 1;    /* Use screen-sized buffers and draw to absolute coordinates */
    } flags;
} lvgl_port_display_ctx_t;

/*******************************************************************************
* Function definitions
*******************************************************************************/
static lv_display_t *lvgl_port_add_disp_priv(const lvgl_port_display_cfg_t *disp_cfg, const lvgl_port_disp_priv_cfg_t *priv_cfg);
#if LVGL_PORT_HANDLE_FLUSH_READY
static bool lvgl_port_flush_io_ready_callback(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx);
#if CONFIG_IDF_TARGET_ESP32S3 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static bool lvgl_port_flush_vsync_ready_callback(esp_lcd_panel_handle_t panel_io, const esp_lcd_rgb_panel_event_data_t *edata, void *user_ctx);
#endif
#if (CONFIG_IDF_TARGET_ESP32P4 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0))
static bool lvgl_port_flush_panel_ready_callback(esp_lcd_panel_handle_t panel_io, esp_lcd_dpi_panel_event_data_t *edata, void *user_ctx);
#endif
#endif
static void lvgl_port_flush_callback(lv_display_t *drv, const lv_area_t *area, uint8_t *color_map);
static void lvgl_port_disp_size_update_callback(lv_event_t *e);
static void lvgl_port_disp_rotation_update(lvgl_port_display_ctx_t *disp_ctx);
static void lvgl_port_display_invalidate_callback(lv_event_t *e);
#if SOC_PPA_SUPPORTED
static bool lvgl_port_ppa_callback(ppa_client_handle_t ppa_client, ppa_event_data_t *event_data, void *user_data);
#endif

/*******************************************************************************
* Public API functions
*******************************************************************************/

lv_display_t *lvgl_port_add_disp(const lvgl_port_display_cfg_t *disp_cfg)
{
    lvgl_port_lock(0);
    lv_disp_t *disp = lvgl_port_add_disp_priv(disp_cfg, NULL);

    if (disp != NULL) {
        lvgl_port_display_ctx_t *disp_ctx = (lvgl_port_display_ctx_t *)lv_display_get_user_data(disp);
        /* Set display type */
        disp_ctx->disp_type = LVGL_PORT_DISP_TYPE_OTHER;

        assert(disp_cfg->io_handle != NULL);

#if LVGL_PORT_HANDLE_FLUSH_READY
        const esp_lcd_panel_io_callbacks_t cbs = {
            .on_color_trans_done = lvgl_port_flush_io_ready_callback,
        };
        /* Register done callback */
        esp_lcd_panel_io_register_event_callbacks(disp_ctx->io_handle, &cbs, disp);
#endif

        /* Apply rotation from initial display configuration */
        lvgl_port_disp_rotation_update(disp_ctx);
    }
    lvgl_port_unlock();

    return disp;
}

lv_display_t *lvgl_port_add_disp_dsi(lvgl_port_display_cfg_t *disp_cfg, const lvgl_port_display_dsi_cfg_t *dsi_cfg)
{
    esp_err_t ret = ESP_OK;
    lvgl_port_lock(0);
    //disp_cfg->buffer_size = ALIGN_UP(disp_cfg->buffer_size, 64);
    lv_disp_t *disp = lvgl_port_add_disp_priv(disp_cfg, NULL);

    if (disp != NULL) {
        lvgl_port_display_ctx_t *disp_ctx = (lvgl_port_display_ctx_t *)lv_display_get_user_data(disp);
        /* Set display type */
        disp_ctx->disp_type = LVGL_PORT_DISP_TYPE_DSI;

#if (CONFIG_IDF_TARGET_ESP32P4 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0))
        const esp_lcd_dpi_panel_event_callbacks_t cbs = {
            .on_color_trans_done = lvgl_port_flush_panel_ready_callback,
        };
        /* Register done callback */
        esp_lcd_dpi_panel_register_event_callbacks(disp_ctx->panel_handle, &cbs, disp);

#if SOC_PPA_SUPPORTED
        disp_ctx->ppa_buffer_size = ALIGN_UP(2 * disp_cfg->buffer_size * lv_color_format_get_bpp(disp_cfg->color_format) / 8, 64);
        disp_ctx->ppa_buffer = heap_caps_aligned_calloc(64, disp_ctx->ppa_buffer_size, sizeof(uint8_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
        assert(disp_ctx->ppa_buffer != NULL);

        ppa_client_config_t ppa_client_config = {
            .oper_type = PPA_OPERATION_SRM,
        };
        ESP_GOTO_ON_ERROR(ppa_register_client(&ppa_client_config, &disp_ctx->ppa_srm_handle), err, TAG, "Error when registering PPA client!");

        ppa_event_callbacks_t ppa_cbs = {
            .on_trans_done = lvgl_port_ppa_callback,
        };
        ESP_GOTO_ON_ERROR(ppa_client_register_event_callbacks(disp_ctx->ppa_srm_handle, &ppa_cbs), err, TAG, "Error when registering PPA callbacks!");

        disp_ctx->ppa_sem = xSemaphoreCreateBinary();
        assert(disp_ctx->ppa_sem != NULL);

#endif //SOC_PPA_SUPPORTED

        /* Apply rotation from initial display configuration */
        lvgl_port_disp_rotation_update(disp_ctx);

#else
        ESP_RETURN_ON_FALSE(false, NULL, TAG, "MIPI-DSI is supported only on ESP32P4 and from IDF 5.3!");
#endif
    }
    lvgl_port_unlock();


#if SOC_PPA_SUPPORTED
err:
#endif
    if (ret != ESP_OK) {
        lvgl_port_remove_disp(disp);
    }

    return disp;
}

lv_display_t *lvgl_port_add_disp_rgb(const lvgl_port_display_cfg_t *disp_cfg, const lvgl_port_display_rgb_cfg_t *rgb_cfg)
{
    lvgl_port_lock(0);
    assert(rgb_cfg != NULL);
    const lvgl_port_disp_priv_cfg_t priv_cfg = {
        .avoid_tearing = rgb_cfg->flags.avoid_tearing,
    };
    lv_disp_t *disp = lvgl_port_add_disp_priv(disp_cfg, &priv_cfg);

    if (disp != NULL) {
        lvgl_port_display_ctx_t *disp_ctx = (lvgl_port_display_ctx_t *)lv_display_get_user_data(disp);
        /* Set display type */
        disp_ctx->disp_type = LVGL_PORT_DISP_TYPE_RGB;

#if (CONFIG_IDF_TARGET_ESP32S3 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0))
        /* Register done callback */
        const esp_lcd_rgb_panel_event_callbacks_t vsync_cbs = {
            .on_vsync = lvgl_port_flush_vsync_ready_callback,
        };

        const esp_lcd_rgb_panel_event_callbacks_t bb_cbs = {
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 1, 2)
            .on_bounce_frame_finish = lvgl_port_flush_vsync_ready_callback,
#endif
        };

        if (rgb_cfg->flags.bb_mode && (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 1, 2))) {
            ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(disp_ctx->panel_handle, &bb_cbs, &disp_ctx->disp_drv));
        } else {
            ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(disp_ctx->panel_handle, &vsync_cbs, &disp_ctx->disp_drv));
        }
#else
        ESP_RETURN_ON_FALSE(false, NULL, TAG, "RGB is supported only on ESP32S3 and from IDF 5.0!");
#endif

        /* Apply rotation from initial display configuration */
        lvgl_port_disp_rotation_update(disp_ctx);
    }
    lvgl_port_unlock();

    return disp;
}

esp_err_t lvgl_port_remove_disp(lv_display_t *disp)
{
    assert(disp);
    lvgl_port_display_ctx_t *disp_ctx = (lvgl_port_display_ctx_t *)lv_display_get_user_data(disp);

    lvgl_port_lock(0);
    lv_disp_remove(disp);
    lvgl_port_unlock();

    if (disp_ctx->draw_buffs[0]) {
        free(disp_ctx->draw_buffs[0]);
    }

    if (disp_ctx->draw_buffs[1]) {
        free(disp_ctx->draw_buffs[1]);
    }


#if SOC_PPA_SUPPORTED

    vSemaphoreDelete(disp_ctx->ppa_sem);

    if (disp_ctx->ppa_buffer) {
        free(disp_ctx->ppa_buffer);
    }

    ppa_unregister_client(disp_ctx->ppa_srm_handle);
#endif //SOC_PPA_SUPPORTED

    free(disp_ctx);

    return ESP_OK;
}

void lvgl_port_flush_ready(lv_display_t *disp)
{
    assert(disp);
    lv_disp_flush_ready(disp);
}

/*******************************************************************************
* Private functions
*******************************************************************************/

static lv_display_t *lvgl_port_add_disp_priv(const lvgl_port_display_cfg_t *disp_cfg, const lvgl_port_disp_priv_cfg_t *priv_cfg)
{
    esp_err_t ret = ESP_OK;
    lv_display_t *disp = NULL;
    lv_color_t *buf1 = NULL;
    lv_color_t *buf2 = NULL;
    uint32_t buffer_size = 0;
    assert(disp_cfg != NULL);
    assert(disp_cfg->panel_handle != NULL);
    assert(disp_cfg->buffer_size > 0);
    assert(disp_cfg->hres > 0);
    assert(disp_cfg->vres > 0);

    buffer_size = disp_cfg->buffer_size;

    /* Check supported display color formats */
    ESP_RETURN_ON_FALSE(disp_cfg->color_format == 0 || disp_cfg->color_format == LV_COLOR_FORMAT_RGB565 || disp_cfg->color_format == LV_COLOR_FORMAT_RGB888 || disp_cfg->color_format == LV_COLOR_FORMAT_XRGB8888 || disp_cfg->color_format == LV_COLOR_FORMAT_ARGB8888, NULL, TAG, "Not supported display color format!");

    lv_color_format_t display_color_format = (disp_cfg->color_format != 0 ? disp_cfg->color_format : LV_COLOR_FORMAT_RGB565);
    if (disp_cfg->flags.swap_bytes) {
        /* Swap bytes can be used only in RGB656 color format */
        ESP_RETURN_ON_FALSE(display_color_format == LV_COLOR_FORMAT_RGB565, NULL, TAG, "Swap bytes can be used only in display color format RGB565!");
    }

    if (disp_cfg->flags.buff_dma) {
        /* DMA buffer can be used only in RGB656 color format */
        ESP_RETURN_ON_FALSE(display_color_format == LV_COLOR_FORMAT_RGB565, NULL, TAG, "DMA buffer can be used only in display color format RGB565 (not alligned copy)!");
    }

    /* Display context */
    lvgl_port_display_ctx_t *disp_ctx = malloc(sizeof(lvgl_port_display_ctx_t));
    ESP_GOTO_ON_FALSE(disp_ctx, ESP_ERR_NO_MEM, err, TAG, "Not enough memory for display context allocation!");
    memset(disp_ctx, 0, sizeof(lvgl_port_display_ctx_t));
    disp_ctx->io_handle = disp_cfg->io_handle;
    disp_ctx->panel_handle = disp_cfg->panel_handle;
    disp_ctx->control_handle = disp_cfg->control_handle;
    disp_ctx->rotation.swap_xy = disp_cfg->rotation.swap_xy;
    disp_ctx->rotation.mirror_x = disp_cfg->rotation.mirror_x;
    disp_ctx->rotation.mirror_y = disp_cfg->rotation.mirror_y;
    disp_ctx->flags.swap_bytes = disp_cfg->flags.swap_bytes;

    /* Use RGB internal buffers for avoid tearing effect */
    if (priv_cfg && priv_cfg->avoid_tearing) {
#if CONFIG_IDF_TARGET_ESP32S3 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        buffer_size = disp_cfg->hres * disp_cfg->vres;
        ESP_GOTO_ON_ERROR(esp_lcd_rgb_panel_get_frame_buffer(disp_cfg->panel_handle, 2, (void *)&buf1, (void *)&buf2), err, TAG, "Get RGB buffers failed");
#endif
    } else {
        uint32_t buff_caps = MALLOC_CAP_DEFAULT;
        if (disp_cfg->flags.buff_dma && disp_cfg->flags.buff_spiram) {
            ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "Alloc DMA capable buffer in SPIRAM is not supported!");
        } else if (disp_cfg->flags.buff_dma) {
            buff_caps = MALLOC_CAP_DMA;
        } else if (disp_cfg->flags.buff_spiram) {
            buff_caps = MALLOC_CAP_SPIRAM;
        }

        /* alloc draw buffers used by LVGL */
        /* it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized */
        buf1 = heap_caps_malloc(buffer_size * sizeof(lv_color_t), buff_caps);
        ESP_GOTO_ON_FALSE(buf1, ESP_ERR_NO_MEM, err, TAG, "Not enough memory for LVGL buffer (buf1) allocation!");
        if (disp_cfg->double_buffer) {
            buf2 = heap_caps_malloc(buffer_size * sizeof(lv_color_t), buff_caps);
            ESP_GOTO_ON_FALSE(buf2, ESP_ERR_NO_MEM, err, TAG, "Not enough memory for LVGL buffer (buf2) allocation!");
        }

        disp_ctx->draw_buffs[0] = buf1;
        disp_ctx->draw_buffs[1] = buf2;
    }

    disp = lv_display_create(disp_cfg->hres, disp_cfg->vres);

    /* Monochrome display settings */
    if (disp_cfg->monochrome) {
        /* When using monochromatic display, there must be used full bufer! */
        ESP_GOTO_ON_FALSE((disp_cfg->hres * disp_cfg->vres == buffer_size), ESP_ERR_INVALID_ARG, err, TAG, "Monochromatic display must using full buffer!");

        disp_ctx->flags.monochrome = 1;
        lv_display_set_buffers(disp, buf1, buf2, buffer_size * sizeof(lv_color_t), LV_DISPLAY_RENDER_MODE_FULL);
    } else if (disp_cfg->flags.direct_mode) {
        /* When using direct_mode, there must be used full bufer! */
        ESP_GOTO_ON_FALSE((disp_cfg->hres * disp_cfg->vres == buffer_size), ESP_ERR_INVALID_ARG, err, TAG, "Direct mode must using full buffer!");

        disp_ctx->flags.direct_mode = 1;
        lv_display_set_buffers(disp, buf1, buf2, buffer_size * sizeof(lv_color_t), LV_DISPLAY_RENDER_MODE_DIRECT);
    } else if (disp_cfg->flags.full_refresh) {
        /* When using full_refresh, there must be used full bufer! */
        ESP_GOTO_ON_FALSE((disp_cfg->hres * disp_cfg->vres == buffer_size), ESP_ERR_INVALID_ARG, err, TAG, "Full refresh must using full buffer!");

        disp_ctx->flags.full_refresh = 1;
        lv_display_set_buffers(disp, buf1, buf2, buffer_size * sizeof(lv_color_t), LV_DISPLAY_RENDER_MODE_FULL);
    } else {
        lv_display_set_buffers(disp, buf1, buf2, buffer_size * sizeof(lv_color_t), LV_DISPLAY_RENDER_MODE_PARTIAL);
    }

    lv_display_set_color_format(disp, display_color_format);
    lv_display_set_flush_cb(disp, lvgl_port_flush_callback);
    lv_display_add_event_cb(disp, lvgl_port_disp_size_update_callback, LV_EVENT_RESOLUTION_CHANGED, disp_ctx);
    lv_display_add_event_cb(disp, lvgl_port_display_invalidate_callback, LV_EVENT_INVALIDATE_AREA, disp_ctx);
    lv_display_add_event_cb(disp, lvgl_port_display_invalidate_callback, LV_EVENT_REFR_REQUEST, disp_ctx);

    lv_display_set_user_data(disp, disp_ctx);
    disp_ctx->disp_drv = disp;

err:
    if (ret != ESP_OK) {
        if (buf1) {
            free(buf1);
        }
        if (buf2) {
            free(buf2);
        }
        if (disp_ctx) {
            free(disp_ctx);
        }
    }

    return disp;
}

#if LVGL_PORT_HANDLE_FLUSH_READY
static bool lvgl_port_flush_io_ready_callback(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_display_t *disp_drv = (lv_display_t *)user_ctx;
    assert(disp_drv != NULL);
    lv_disp_flush_ready(disp_drv);
    return false;
}

#if (CONFIG_IDF_TARGET_ESP32P4 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0))
static bool lvgl_port_flush_panel_ready_callback(esp_lcd_panel_handle_t panel_io, esp_lcd_dpi_panel_event_data_t *edata, void *user_ctx)
{
    lv_display_t *disp_drv = (lv_display_t *)user_ctx;
    assert(disp_drv != NULL);
    lv_disp_flush_ready(disp_drv);
    return false;
}
#endif

#if CONFIG_IDF_TARGET_ESP32S3 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static bool lvgl_port_flush_vsync_ready_callback(esp_lcd_panel_handle_t panel_io, const esp_lcd_rgb_panel_event_data_t *edata, void *user_ctx)
{
    BaseType_t need_yield = pdFALSE;

    lv_display_t *disp_drv = (lv_display_t *)user_ctx;
    assert(disp_drv != NULL);
    need_yield = lvgl_port_task_notify(ULONG_MAX);
    lvgl_port_task_wake(LVGL_PORT_EVENT_DISPLAY, disp_drv);

    return (need_yield == pdTRUE);
}
#endif
#endif

#if SOC_PPA_SUPPORTED
static bool lvgl_port_ppa_callback(ppa_client_handle_t ppa_client, ppa_event_data_t *event_data, void *user_data)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    lvgl_port_display_ctx_t *disp_ctx = (lvgl_port_display_ctx_t *)user_data;
    assert(disp_ctx != NULL);
    xSemaphoreGiveFromISR(disp_ctx->ppa_sem, &xHigherPriorityTaskWoken);
    return (xHigherPriorityTaskWoken == pdTRUE);
}
#endif //SOC_PPA_SUPPORTED

static void _lvgl_port_transform_monochrome(lv_display_t *display, const lv_area_t *area, uint8_t *color_map)
{
    uint8_t *buf = color_map;
    lv_color_t *color = (lv_color_t *)color_map;
    uint16_t hor_res = lv_display_get_physical_horizontal_resolution(display);
    uint16_t ver_res = lv_display_get_physical_vertical_resolution(display);
    uint16_t res = hor_res;
    bool swap_xy = (lv_display_get_rotation(display) == LV_DISPLAY_ROTATION_90 || lv_display_get_rotation(display) == LV_DISPLAY_ROTATION_270);

    int x1 = area->x1;
    int x2 = area->x2;
    int y1 = area->y1;
    int y2 = area->y2;

    int out_x, out_y;
    for (int y = y1; y <= y2; y++) {
        for (int x = x1; x <= x2; x++) {
            bool chroma_color = (color[hor_res * y + x].blue > 16);

            if (swap_xy) {
                out_x = y;
                out_y = x;
                res = ver_res;
            } else {
                out_x = x;
                out_y = y;
                res = hor_res;
            }

            /* Write to the buffer as required for the display.
            * It writes only 1-bit for monochrome displays mapped vertically.*/
            buf = color_map + res * (out_y >> 3) + (out_x);
            if (chroma_color) {
                (*buf) &= ~(1 << (out_y % 8));
            } else {
                (*buf) |= (1 << (out_y % 8));
            }
        }
    }
}

static void lvgl_port_flush_callback(lv_display_t *drv, const lv_area_t *area, uint8_t *color_map)
{
    assert(drv != NULL);
    lvgl_port_display_ctx_t *disp_ctx = (lvgl_port_display_ctx_t *)lv_display_get_user_data(drv);
    assert(disp_ctx != NULL);

    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;

#if SOC_PPA_SUPPORTED
    const uint32_t color_type_id = COLOR_TYPE_ID(COLOR_SPACE_RGB, COLOR_PIXEL_RGB565);
    const int w = offsetx2 - offsetx1 + 1;
    const int h = offsety2 - offsety1 + 1;

    /* Screen vertical size */
    int32_t hres = lv_display_get_horizontal_resolution(drv);
    int32_t vres = lv_display_get_vertical_resolution(drv);

    /* Set dimension by screen size and rotation */
    int out_w = w;
    int out_h = h;
    ppa_srm_rotation_angle_t ppa_rotation_angle = PPA_SRM_ROTATION_ANGLE_0;
    switch (disp_ctx->ppa_rotation) {
    case LV_DISPLAY_ROTATION_0:
        ppa_rotation_angle = PPA_SRM_ROTATION_ANGLE_0;
        break;
    case LV_DISPLAY_ROTATION_90:
        ppa_rotation_angle = PPA_SRM_ROTATION_ANGLE_90;
        out_w = h;
        out_h = w;
        offsetx1 = area->y1;
        offsetx2 = area->y2;
        offsety1 = hres - area->x2;
        offsety2 = hres - area->x1;
        break;
    case LV_DISPLAY_ROTATION_180:
        ppa_rotation_angle = PPA_SRM_ROTATION_ANGLE_180;
        offsetx1 = hres - area->x2 - 1;
        offsetx2 = hres - area->x1 - 1;
        offsety1 = vres - area->y2;
        offsety2 = vres - area->y1;
        break;
    case LV_DISPLAY_ROTATION_270:
        ppa_rotation_angle = PPA_SRM_ROTATION_ANGLE_270;
        out_w = h;
        out_h = w;
        offsetx1 = vres - area->y2 - 1;
        offsetx2 = vres - area->y1 - 1;
        offsety1 = area->x1;
        offsety2 = area->x2;
        break;
    }

    ppa_srm_oper_config_t srm_oper_config = {
        .in.buffer = color_map,
        .in.pic_w = w,
        .in.pic_h = h,
        .in.block_w = w,
        .in.block_h = h,
        .in.block_offset_x = 0,
        .in.block_offset_y = 0,
        .in.srm_cm = color_type_id,

        .out.buffer = disp_ctx->ppa_buffer,
        .out.buffer_size = disp_ctx->ppa_buffer_size,
        .out.pic_w = out_w,
        .out.pic_h = out_h,
        .out.block_offset_x = 0,
        .out.block_offset_y = 0,
        .out.srm_cm = color_type_id,

        .rotation_angle = ppa_rotation_angle,
        .scale_x = 1.0,
        .scale_y = 1.0,

        .byte_swap = (disp_ctx->flags.swap_bytes ? true : false),

        .mode = PPA_TRANS_MODE_NON_BLOCKING,
        .user_data = disp_ctx,
    };
    ppa_do_scale_rotate_mirror(disp_ctx->ppa_srm_handle, &srm_oper_config);
    //ESP_LOGW(TAG, "w: %d, h: %d, buff: %ld", w, h, disp_ctx->ppa_buffer_size);

    /* Waiting for the PPA to complete transmission */
    xSemaphoreTake(disp_ctx->ppa_sem, portMAX_DELAY);

    color_map = disp_ctx->ppa_buffer;
#else //SOC_PPA_SUPPORTED
    if (disp_ctx->flags.swap_bytes) {
        size_t len = lv_area_get_size(area);
        lv_draw_sw_rgb565_swap(color_map, len);
    }
#endif //SOC_PPA_SUPPORTED

    /* Transfor data in buffer for monochromatic screen */
    if (disp_ctx->flags.monochrome) {
        _lvgl_port_transform_monochrome(drv, area, color_map);
    }

    if (disp_ctx->disp_type == LVGL_PORT_DISP_TYPE_RGB && (disp_ctx->flags.full_refresh || disp_ctx->flags.direct_mode)) {
        if (lv_disp_flush_is_last(drv)) {
            /* If the interface is I80 or SPI, this step cannot be used for drawing. */
            esp_lcd_panel_draw_bitmap(disp_ctx->panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
            /* Waiting for the last frame buffer to complete transmission */
            ulTaskNotifyValueClear(NULL, ULONG_MAX);
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }
    } else {
        esp_lcd_panel_draw_bitmap(disp_ctx->panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
    }

    if (disp_ctx->disp_type == LVGL_PORT_DISP_TYPE_RGB) {
        lv_disp_flush_ready(drv);
    }
}

static void lvgl_port_disp_rotation_update(lvgl_port_display_ctx_t *disp_ctx)
{
    assert(disp_ctx != NULL);

#if SOC_PPA_SUPPORTED
    disp_ctx->ppa_rotation = lv_display_get_rotation(disp_ctx->disp_drv);
#else
    esp_lcd_panel_handle_t control_handle = (disp_ctx->control_handle ? disp_ctx->control_handle : disp_ctx->panel_handle);
    /* Solve rotation screen and touch */
    switch (lv_display_get_rotation(disp_ctx->disp_drv)) {
    case LV_DISPLAY_ROTATION_0:
        /* Rotate LCD display */
        esp_lcd_panel_swap_xy(control_handle, disp_ctx->rotation.swap_xy);
        esp_lcd_panel_mirror(control_handle, disp_ctx->rotation.mirror_x, disp_ctx->rotation.mirror_y);
        break;
    case LV_DISPLAY_ROTATION_90:
        /* Rotate LCD display */
        esp_lcd_panel_swap_xy(control_handle, !disp_ctx->rotation.swap_xy);
        if (disp_ctx->rotation.swap_xy) {
            esp_lcd_panel_mirror(control_handle, !disp_ctx->rotation.mirror_x, disp_ctx->rotation.mirror_y);
        } else {
            esp_lcd_panel_mirror(control_handle, disp_ctx->rotation.mirror_x, !disp_ctx->rotation.mirror_y);
        }
        break;
    case LV_DISPLAY_ROTATION_180:
        /* Rotate LCD display */
        esp_lcd_panel_swap_xy(control_handle, disp_ctx->rotation.swap_xy);
        esp_lcd_panel_mirror(control_handle, !disp_ctx->rotation.mirror_x, !disp_ctx->rotation.mirror_y);
        break;
    case LV_DISPLAY_ROTATION_270:
        /* Rotate LCD display */
        esp_lcd_panel_swap_xy(control_handle, !disp_ctx->rotation.swap_xy);
        if (disp_ctx->rotation.swap_xy) {
            esp_lcd_panel_mirror(control_handle, disp_ctx->rotation.mirror_x, !disp_ctx->rotation.mirror_y);
        } else {
            esp_lcd_panel_mirror(control_handle, !disp_ctx->rotation.mirror_x, disp_ctx->rotation.mirror_y);
        }
        break;
    }
#endif
    /* Wake LVGL task, if needed */
    lvgl_port_task_wake(LVGL_PORT_EVENT_DISPLAY, disp_ctx->disp_drv);
}

static void lvgl_port_disp_size_update_callback(lv_event_t *e)
{
    assert(e);
    lvgl_port_display_ctx_t *disp_ctx = (lvgl_port_display_ctx_t *)e->user_data;
    lvgl_port_disp_rotation_update(disp_ctx);
}

static void lvgl_port_display_invalidate_callback(lv_event_t *e)
{
    /* Wake LVGL task, if needed */
    lvgl_port_task_wake(LVGL_PORT_EVENT_DISPLAY, NULL);
}
