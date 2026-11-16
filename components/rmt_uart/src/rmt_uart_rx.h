#pragma once

#include "esp_err.h"

struct rmt_uart_s;

esp_err_t ru_rx_create(struct rmt_uart_s *u);
void ru_rx_destroy(struct rmt_uart_s *u);
