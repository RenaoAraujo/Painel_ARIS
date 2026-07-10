FILE(GLOB_RECURSE component_sources "*.c")

idf_component_register(
    SRCS ${component_sources}
    INCLUDE_DIRS "include"
    REQUIRES
        esp_lcd
        esp_lcd_touch
        esp_lcd_touch_gt911
        bsp_i2c
        esp_driver_gpio
        esp_driver_i2c
)
