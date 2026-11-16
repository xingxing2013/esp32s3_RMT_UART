#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include "rmt_uart.h"
#include "rmt_uart_frame.h"
#include "rmt_uart_ring.h"

#define RU_TX_CHUNK_BYTES   64
#define RU_RX_USER_SYMBOLS  96
#define RU_RING_SYMBOLS     1024
#define RU_MEM_BLOCK_SYMS   48
#define RU_RX_STACK         4096
#define RU_TX_STACK         3584
#define RU_RX_PRIO          12
#define RU_TX_PRIO          10
#define RU_DEFAULT_RX_BYTES 1024
#define RU_DEFAULT_TX_BYTES 512

struct rmt_uart_s {
    rmt_uart_config_t cfg;
    uint32_t resolution_hz;
    ru_timing_t timing;

    rmt_channel_handle_t tx_ch;
    rmt_channel_handle_t rx_ch;
    rmt_encoder_handle_t encoder;

    StreamBufferHandle_t tx_sb;
    StreamBufferHandle_t rx_sb;
    ru_ring_t *ring;
    ru_decoder_t *dec;

    TaskHandle_t tx_task;
    TaskHandle_t rx_task;
    SemaphoreHandle_t api_lock;
    SemaphoreHandle_t tx_exited;
    SemaphoreHandle_t rx_exited;

    rmt_symbol_word_t *rx_buf;
    rmt_symbol_word_t *tx_syms;
    uint8_t *tx_bytes;

    uint32_t rx_min_ns;
    uint32_t rx_max_ns;

    volatile bool closing;
    volatile bool rx_need_rearm;
    volatile bool rx_stopped;

    bool valid;
    rmt_uart_stats_t stats;
};
