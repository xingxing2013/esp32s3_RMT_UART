idf_component_register(
    SRCS "src/soft_uart_core.c"
    INCLUDE_DIRS "include" "src"
    REQUIRES esp_driver_rmt esp_driver_gpio
    PRIV_REQUIRES unity
)
