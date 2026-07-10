FILE(GLOB_RECURSE component_sources "*.c")

idf_component_register(
    SRCS ${component_sources}
    INCLUDE_DIRS "include"
    REQUIRES
        esp_lcd
        esp_lcd_jd9165
        esp_lvgl_port
        lvgl
        bsp_display
        esp_driver_gpio
        esp_driver_ledc
)
