idf_component_register(SRCS "main.c" "su_selftest.c" "su_diag.c"
                       INCLUDE_DIRS "."
                       REQUIRES rmt_soft_uart esp_driver_gptimer esp_timer)
