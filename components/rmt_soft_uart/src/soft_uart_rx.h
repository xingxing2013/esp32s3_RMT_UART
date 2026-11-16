#pragma once

#include "esp_err.h"

struct rmt_soft_uart_s;

esp_err_t su_rx_create(struct rmt_soft_uart_s *u);
void su_rx_destroy(struct rmt_soft_uart_s *u);
