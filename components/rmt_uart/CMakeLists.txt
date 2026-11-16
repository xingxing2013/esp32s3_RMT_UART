set(srcs "src/rmt_uart_frame.c" "src/rmt_uart_ring.c" "src/rmt_uart_tx.c" "src/rmt_uart_rx.c" "src/rmt_uart_core.c")

if(CONFIG_RMT_UART_RUN_TESTS)
    list(APPEND srcs "test/test_frame.c" "test/test_ring.c")
endif()

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "include" "src" "test"
    REQUIRES esp_driver_rmt esp_driver_gpio esp_timer
    PRIV_REQUIRES unity esp_hw_support
)

# 解码热路径固定用 -O2，避免宿主工程用 -Og 时性能腰斩
target_compile_options(${COMPONENT_LIB} PRIVATE -O2)
