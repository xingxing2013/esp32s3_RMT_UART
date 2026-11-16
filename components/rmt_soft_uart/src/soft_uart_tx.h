#pragma once

#include "esp_err.h"

struct rmt_soft_uart_s;

esp_err_t su_tx_create(struct rmt_soft_uart_s *u);
void su_tx_destroy(struct rmt_soft_uart_s *u);
