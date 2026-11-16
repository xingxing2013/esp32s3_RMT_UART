idf_component_register(SRCS "main.c" "ru_selftest.c" "ru_diag.c"
                       INCLUDE_DIRS "."
                       REQUIRES rmt_uart esp_driver_gptimer esp_timer)
