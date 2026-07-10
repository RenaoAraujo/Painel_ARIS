#ifndef _BSP_ILLUMINATE_H_
#define _BSP_ILLUMINATE_H_

/* LCD MIPI-DSI JD9165 — Guition JC1060P470C (7", 1024×600 paisagem nativa). */
#include "esp_log.h"
#include "esp_err.h"
#include "esp_ldo_regulator.h"
#include "esp_lcd_jd9165.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_io.h"
#include "esp_lvgl_port.h"
#include "driver/gpio.h"
#include "lvgl.h"
#include "bsp_display.h"

#define ILLUMINATE_TAG "ILLUMINATE"
#define ILLUMINATE_INFO(fmt, ...)  ESP_LOGI(ILLUMINATE_TAG, fmt, ##__VA_ARGS__)
#define ILLUMINATE_DEBUG(fmt, ...) ESP_LOGD(ILLUMINATE_TAG, fmt, ##__VA_ARGS__)
#define ILLUMINATE_ERROR(fmt, ...) ESP_LOGE(ILLUMINATE_TAG, fmt, ##__VA_ARGS__)

/* 1024 (L) × 600 (A) — orientação em que o módulo é montado na maioria dos kits Guition 7". */
#define LCD_NATIVE_H_SIZE   1024
#define LCD_NATIVE_V_SIZE   600
#define LCD_BITS_PER_PIXEL  16

/* ---------------------------------------------------------------------------
 * RST do LCD: referências da comunidade para JC1060 usam GPIO 5.
 * Se a tua revisão usar GPIO 27 (como em kits 10"), altera aqui.
 * --------------------------------------------------------------------------- */
#define LCD_GPIO_RST     5
#define LCD_GPIO_BLIGHT  23

/* Backlight: Guition costuma ser nível fixo (alto = aceso). 0 = tentar LEDC. */
#define BSP_LCD_BL_USE_GPIO     1

#define LCD_BLIGHT_ON_LEVEL     1
#define LCD_BLIGHT_OFF_LEVEL    (!LCD_BLIGHT_ON_LEVEL)

#define BLIGHT_PWM_HZ    1220

#ifndef BSP_LCD_TEST_PATTERN_MS
#define BSP_LCD_TEST_PATTERN_MS  0
#endif

esp_err_t display_init(void);
esp_err_t set_lcd_blight(uint32_t brightness);

#endif
