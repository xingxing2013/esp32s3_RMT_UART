set(srcs "src/soft_uart_codec.c" "src/soft_uart_core.c")

if(CONFIG_RMT_SOFT_UART_RUN_TESTS)
    list(APPEND srcs "test/su_test_codec.c")
endif()

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "include" "src" "test"
    REQUIRES esp_driver_rmt esp_driver_gpio
    PRIV_REQUIRES unity
)
