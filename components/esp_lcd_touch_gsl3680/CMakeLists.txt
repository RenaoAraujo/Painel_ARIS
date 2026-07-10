FILE(GLOB_RECURSE component_sources "*.c")

idf_component_register(
    SRCS ${component_sources}
    INCLUDE_DIRS "include"
    REQUIRES
        esp_lcd
        esp_lcd_touch
        esp_driver_gpio
        esp_driver_i2c
)

# Driver GSL3680 portado do CelliesProjects: contém funções/variáveis nao
# utilizadas em algumas configurações. Como é codigo de terceiros, em vez
# de editar o fonte, suprimimos esses avisos apenas para este componente.
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-error=unused-function
    -Wno-error=unused-variable
    -Wno-error=unused-but-set-variable
    -Wno-unused-function
    -Wno-unused-variable
    -Wno-unused-but-set-variable
)
