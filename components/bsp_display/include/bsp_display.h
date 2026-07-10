#ifndef _BSP_DISPLAY_H_
#define _BSP_DISPLAY_H_

/* Touch GT911 — Guition JC1060P470C 7". */
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_touch_gt911.h"
#include "bsp_i2c.h"

#define DISPLAY_TAG "DISPLAY"
#define DISPLAY_INFO(fmt, ...)  ESP_LOGI(DISPLAY_TAG, fmt, ##__VA_ARGS__)
#define DISPLAY_DEBUG(fmt, ...) ESP_LOGD(DISPLAY_TAG, fmt, ##__VA_ARGS__)
#define DISPLAY_ERROR(fmt, ...) ESP_LOGE(DISPLAY_TAG, fmt, ##__VA_ARGS__)

/* Mesmo espaço do painel nativo 1024×600. */
#define LCD_NATIVE_H_RES   1024
#define LCD_NATIVE_V_RES   600

extern esp_lcd_touch_handle_t tp;

void get_coor(uint16_t *x, uint16_t *y, bool *press);
void set_coor(uint16_t x, uint16_t y, bool press);

esp_err_t touch_init(void);
esp_err_t touch_read(void);

#endif
