#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/rmt_types.h"
#include "esp_err.h"
#include "rmt_uart.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RU_MAX_BIT_TICKS 32767u
#define RU_MAX_SAMPLES   10
#define RU_WORDS_PER_BYTE 6

typedef struct {
    uint8_t  data_bits;      /* 5..8 */
    uint8_t  parity_bits;    /* 0 / 1 */
    uint8_t  stop_bits_x2;   /* 2 / 3 / 4 */
    rmt_uart_parity_t parity;
    uint32_t bit_ticks;
    uint32_t frame_ticks;
    uint8_t  sample_count;
    uint32_t sample_off[RU_MAX_SAMPLES];
} ru_timing_t;

typedef struct {
    bool framing_error;
    bool parity_error;
    bool break_error;
} ru_rx_flags_t;

typedef void (*ru_byte_cb_t)(void *ctx, uint8_t byte, ru_rx_flags_t flags);

typedef struct ru_decoder_s ru_decoder_t;

esp_err_t ru_timing_init(uint32_t baud, uint8_t data_bits, rmt_uart_parity_t parity,
                         rmt_uart_stop_bits_t stop, uint32_t *out_resolution_hz,
                         ru_timing_t *out_timing);

/** 计算 RMT RX 的 signal_range_min_ns / max_ns，并按硬件寄存器上限收敛。 */
esp_err_t ru_rx_window_ns(uint32_t baud, uint32_t channel_resolution_hz,
                          uint32_t filter_clock_hz, uint32_t *out_min_ns,
                          uint32_t *out_max_ns);

size_t ru_encode(const uint8_t *data, size_t len, const ru_timing_t *t,
                 rmt_symbol_word_t *out, size_t out_words);

ru_decoder_t *ru_decoder_create(const ru_timing_t *t, ru_byte_cb_t cb, void *ctx);
void ru_decoder_reset(ru_decoder_t *d);
void ru_decoder_feed(ru_decoder_t *d, const rmt_symbol_word_t *syms, size_t n);
void ru_decoder_destroy(ru_decoder_t *d);

#ifdef __cplusplus
}
#endif
