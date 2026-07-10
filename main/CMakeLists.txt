FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/main/*.c)

idf_component_register(
    SRCS ${app_sources}
    INCLUDE_DIRS "include"
    REQUIRES
        bsp_display
        bsp_illuminate
        bsp_i2c
        esp_timer
        esp_hw_support
        esp_app_format
        lvgl
        esp_lvgl_port
)
