/* bsp_illuminate.c — MIPI-DSI JD9165 + LVGL (Guition JC1060P470C 7" 1024×600). */
#include "bsp_illuminate.h"
#include "bsp_jd9165_guition_init.h"
#if !BSP_LCD_BL_USE_GPIO
#include "driver/ledc.h"
#endif

static esp_lcd_panel_handle_t   s_panel       = NULL;
static esp_lcd_dsi_bus_handle_t s_dsi_bus     = NULL;
static esp_lcd_panel_io_handle_t s_dbi_io     = NULL;
static lv_disp_t               *s_lvgl_disp   = NULL;
static lv_indev_t              *s_lvgl_touch  = NULL;

#define BSP_LVGL_PARTIAL_LINES  110

static esp_err_t blight_init(void)
{
#if BSP_LCD_BL_USE_GPIO
    const gpio_config_t gpio_cfg = {
        .pin_bit_mask = 1ULL << LCD_GPIO_BLIGHT,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&gpio_cfg);
    if (err != ESP_OK) {
        return err;
    }
    gpio_set_level(LCD_GPIO_BLIGHT, LCD_BLIGHT_OFF_LEVEL);
    return ESP_OK;
#else
    const ledc_timer_config_t timer_cfg = {
        .clk_cfg = LEDC_USE_PLL_DIV_CLK,
        .duty_resolution = LEDC_TIMER_11_BIT,
        .freq_hz = BLIGHT_PWM_HZ,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
    };
    esp_err_t err = ledc_timer_config(&timer_cfg);
    if (err != ESP_OK) {
        return err;
    }
    const ledc_channel_config_t ch_cfg = {
        .gpio_num = LCD_GPIO_BLIGHT,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    return ledc_channel_config(&ch_cfg);
#endif
}

esp_err_t set_lcd_blight(uint32_t brightness)
{
    if (brightness > 100) {
        brightness = 100;
    }
#if BSP_LCD_BL_USE_GPIO
    gpio_set_level(LCD_GPIO_BLIGHT, brightness > 0 ? LCD_BLIGHT_ON_LEVEL : LCD_BLIGHT_OFF_LEVEL);
    return ESP_OK;
#else
    uint32_t duty = (brightness == 0) ? 0 : ((brightness * 18) + 200);
    esp_err_t err = ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    if (err != ESP_OK) {
        return err;
    }
    return ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
#endif
}

static esp_err_t display_port_init(void)
{
    esp_lcd_dsi_bus_config_t bus_cfg = JD9165_PANEL_BUS_DSI_2CH_CONFIG();
    esp_err_t err = esp_lcd_new_dsi_bus(&bus_cfg, &s_dsi_bus);
    if (err != ESP_OK) {
        return err;
    }

    esp_lcd_dbi_io_config_t dbi_cfg = JD9165_PANEL_IO_DBI_CONFIG();
    err = esp_lcd_new_panel_io_dbi(s_dsi_bus, &dbi_cfg, &s_dbi_io);
    if (err != ESP_OK) {
        return err;
    }

    lcd_color_format_t color_format = (LCD_BITS_PER_PIXEL == 24)
                                          ? LCD_COLOR_FMT_RGB888
                                          : LCD_COLOR_FMT_RGB565;

    /* Timings alinhados a setups reportados como OK no JC1060 (MiniWebRadio / comunidade):
     * ~52 MHz, HSYNC 20/160/160, VSYNC 10/23/12. */
    esp_lcd_dpi_panel_config_t dpi_cfg = {
        .dpi_clk_src        = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
        .dpi_clock_freq_mhz = 52,
        .virtual_channel    = 0,
        .in_color_format    = color_format,
        .num_fbs            = 1,
        .video_timing = {
            .h_size            = LCD_NATIVE_H_SIZE,
            .v_size            = LCD_NATIVE_V_SIZE,
            .hsync_pulse_width = 20,
            .hsync_back_porch  = 160,
            .hsync_front_porch = 160,
            .vsync_pulse_width = 10,
            .vsync_back_porch  = 23,
            .vsync_front_porch = 12,
        },
    };

    jd9165_vendor_config_t vendor_cfg = {
        .init_cmds      = bsp_jd9165_guition_init_cmds,
        .init_cmds_size = sizeof(bsp_jd9165_guition_init_cmds) / sizeof(bsp_jd9165_guition_init_cmds[0]),
        .mipi_config = {
            .dsi_bus    = s_dsi_bus,
            .dpi_config = &dpi_cfg,
        },
    };

    const esp_lcd_panel_dev_config_t panel_dev_cfg = {
        .reset_gpio_num = LCD_GPIO_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = LCD_BITS_PER_PIXEL,
        .vendor_config = &vendor_cfg,
    };

    err = esp_lcd_new_panel_jd9165(s_dbi_io, &panel_dev_cfg, &s_panel);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_lcd_panel_reset(s_panel);
    if (err != ESP_OK) {
        return err;
    }
    err = esp_lcd_panel_init(s_panel);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_lcd_panel_disp_on_off(s_panel, true);
    if (err != ESP_OK) {
        return err;
    }

    esp_err_t dma2d_err = esp_lcd_dpi_panel_enable_dma2d(s_panel);
    if (dma2d_err != ESP_OK) {
        ILLUMINATE_INFO("esp_lcd_dpi_panel_enable_dma2d: %s (continua sem DMA2D)",
                        esp_err_to_name(dma2d_err));
    } else {
        ILLUMINATE_INFO("DPI DMA2D habilitado");
    }

    err = set_lcd_blight(100);
    if (err != ESP_OK) {
        return err;
    }

#if BSP_LCD_TEST_PATTERN_MS > 0
    ILLUMINATE_INFO("Pattern de teste %d ms", BSP_LCD_TEST_PATTERN_MS);
    esp_lcd_dpi_panel_set_pattern(s_panel, MIPI_DSI_PATTERN_BAR_VERTICAL);
    vTaskDelay(pdMS_TO_TICKS(BSP_LCD_TEST_PATTERN_MS));
    esp_lcd_dpi_panel_set_pattern(s_panel, MIPI_DSI_PATTERN_NONE);
#endif

    return ESP_OK;
}

static esp_err_t lvgl_init(void)
{
    const lvgl_port_cfg_t lvgl_cfg = {
        .task_priority = 4,
        .task_stack = 8192 * 4,
        .task_affinity = 1,
        .task_max_sleep_ms = 500,
        .timer_period_ms = 5,
    };
    esp_err_t err = lvgl_port_init(&lvgl_cfg);
    if (err != ESP_OK) {
        ILLUMINATE_ERROR("lvgl_port_init falhou");
        return err;
    }

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle      = s_dbi_io,
        .panel_handle   = s_panel,
        .control_handle = s_panel,
        .buffer_size    = (uint32_t)LCD_NATIVE_H_SIZE * BSP_LVGL_PARTIAL_LINES,
        .double_buffer  = true,
        .hres           = LCD_NATIVE_H_SIZE,
        .vres           = LCD_NATIVE_V_SIZE,
        .monochrome     = false,
#if LVGL_VERSION_MAJOR >= 9
        .color_format   = LV_COLOR_FORMAT_RGB565,
#endif
        .rotation = {
            .swap_xy  = false,
            .mirror_x = false,
            .mirror_y = false,
        },
        .flags = {
            .buff_dma     = false,
            .buff_spiram  = true,
            .sw_rotate    = false,
#if LVGL_VERSION_MAJOR >= 9
            .swap_bytes   = true,
#endif
            .full_refresh = false,
            .direct_mode  = false,
            .skip_panel_swap_xy = true,
        },
    };

    const lvgl_port_display_dsi_cfg_t lvgl_dpi_cfg = {
        .flags = { .avoid_tearing = false },
    };

    s_lvgl_disp = lvgl_port_add_disp_dsi(&disp_cfg, &lvgl_dpi_cfg);
    if (!s_lvgl_disp) {
        ILLUMINATE_ERROR("lvgl_port_add_disp_dsi falhou");
        return ESP_FAIL;
    }

    if (tp) {
        const lvgl_port_touch_cfg_t touch_cfg = {
            .disp = s_lvgl_disp,
            .handle = tp,
        };
        s_lvgl_touch = lvgl_port_add_touch(&touch_cfg);
        if (!s_lvgl_touch) {
            ILLUMINATE_ERROR("lvgl_port_add_touch falhou");
            return ESP_FAIL;
        }
    } else {
        ILLUMINATE_ERROR("Touch nao inicializado antes do display_init");
    }

    return ESP_OK;
}

esp_err_t display_init(void)
{
    esp_err_t err = blight_init();
    if (err != ESP_OK) {
        return err;
    }

    err = display_port_init();
    if (err != ESP_OK) {
        return err;
    }

    err = lvgl_init();
    if (err != ESP_OK) {
        return err;
    }

    return ESP_OK;
}
