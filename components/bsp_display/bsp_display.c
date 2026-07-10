#include "bsp_display.h"
#include "esp_check.h"
#include "driver/gpio.h"

esp_lcd_touch_handle_t tp = NULL;
static esp_lcd_panel_io_handle_t tp_io_handle = NULL;
static esp_lcd_touch_io_gt911_config_t s_gt911_drv_cfg;

static uint16_t s_touch_x = 0xFFFF;
static uint16_t s_touch_y = 0xFFFF;
static bool     s_touch_pressed = false;

void set_coor(uint16_t x, uint16_t y, bool press)
{
    s_touch_x = x;
    s_touch_y = y;
    s_touch_pressed = press;
}

void get_coor(uint16_t *x, uint16_t *y, bool *press)
{
    *x = s_touch_x;
    *y = s_touch_y;
    *press = s_touch_pressed;
}

esp_err_t touch_init(void)
{
    esp_lcd_panel_io_i2c_config_t io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
    io_config.scl_speed_hz = 400000;

    s_gt911_drv_cfg = (esp_lcd_touch_io_gt911_config_t){
        .dev_addr = io_config.dev_addr,
    };

    esp_lcd_touch_config_t tp_cfg = {
        .x_max = LCD_NATIVE_H_RES,
        .y_max = LCD_NATIVE_V_RES,
        .rst_gpio_num = GPIO_NUM_NC,
        .int_gpio_num = GPIO_NUM_NC,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
        .driver_data = &s_gt911_drv_cfg,
    };

    ESP_RETURN_ON_ERROR(
        esp_lcd_new_panel_io_i2c((i2c_master_bus_handle_t)i2c_bus_handle, &io_config, &tp_io_handle),
        DISPLAY_TAG, "panel_io_i2c GT911 fail");

    ESP_RETURN_ON_ERROR(
        esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, &tp),
        DISPLAY_TAG, "touch_new gt911 fail");

    return ESP_OK;
}

esp_err_t touch_read(void)
{
    esp_err_t err = esp_lcd_touch_read_data(tp);
    if (err != ESP_OK) {
        DISPLAY_DEBUG("GT911 read error: %s", esp_err_to_name(err));
        return err;
    }

    esp_lcd_touch_point_data_t pt;
    uint8_t point_cnt = 0;

    err = esp_lcd_touch_get_data(tp, &pt, &point_cnt, 1);
    if (err != ESP_OK) {
        DISPLAY_DEBUG("GT911 get_data erro: %s", esp_err_to_name(err));
        return err;
    }
    if (point_cnt > 0) {
        set_coor(pt.x, pt.y, true);
    } else {
        set_coor(0xFFFF, 0xFFFF, false);
    }
    return ESP_OK;
}
