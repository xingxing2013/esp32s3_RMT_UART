#pragma once

#include "esp_err.h"

struct rmt_uart_s;

esp_err_t ru_tx_create(struct rmt_uart_s *u);
void ru_tx_destroy(struct rmt_uart_s *u);
