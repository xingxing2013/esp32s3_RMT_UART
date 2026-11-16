#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/rmt_types.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ru_MAX_BIT_TICKS 32767u

typedef struct {
    uint32_t bit_ticks;
    uint32_t frame_ticks;
    uint32_t sample_off[9];
} ru_timing_t;

typedef struct {
    bool framing_error;
    bool break_error;
} ru_rx_flags_t;

typedef void (*ru_byte_cb_t)(void *ctx, uint8_t byte, ru_rx_flags_t flags);

typedef struct ru_decoder_s ru_decoder_t;

esp_err_t ru_timing_init(uint32_t baud, uint32_t *out_resolution_hz,
                         ru_timing_t *out_timing);

/**
 * 计算 RMT RX 的 signal_range_min_ns / signal_range_max_ns。
 * 结果已按硬件寄存器上限收敛：
 *   - filter 时钟 ≤ 255 tick（filter_clock_hz 为 RMT group 时钟，默认 APB 80 MHz）
 *   - idle 计数 ≤ 32767 tick（按 channel_resolution_hz 换算）
 * 低波特率下毛刺过滤能力会被削弱，高波特率下空闲窗口会缩短，均属预期。
 */
esp_err_t ru_rx_window_ns(uint32_t baud, uint32_t channel_resolution_hz,
                          uint32_t filter_clock_hz,
                          uint32_t *out_min_ns, uint32_t *out_max_ns);

size_t ru_encode(const uint8_t *data, size_t len, const ru_timing_t *t,
                 rmt_symbol_word_t *out, size_t out_words);

ru_decoder_t *ru_decoder_create(const ru_timing_t *t, ru_byte_cb_t cb, void *ctx);
void ru_decoder_reset(ru_decoder_t *d);

void ru_decoder_feed(ru_decoder_t *d, const rmt_symbol_word_t *syms, size_t n);
void ru_decoder_destroy(ru_decoder_t *d);

#ifdef __cplusplus
}
#endif
