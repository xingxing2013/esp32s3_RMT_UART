#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/rmt_types.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SU_MAX_BIT_TICKS 32767u

typedef struct {
    uint32_t bit_ticks;
    uint32_t frame_ticks;
    uint32_t sample_off[9];
} su_timing_t;

typedef struct {
    bool framing_error;
    bool break_error;
} su_rx_flags_t;

typedef void (*su_byte_cb_t)(void *ctx, uint8_t byte, su_rx_flags_t flags);

typedef struct su_decoder_s su_decoder_t;

esp_err_t su_timing_init(uint32_t baud, uint32_t *out_resolution_hz,
                         su_timing_t *out_timing);

/**
 * 计算 RMT RX 的 signal_range_min_ns / signal_range_max_ns。
 * 结果已按硬件寄存器上限收敛：
 *   - filter 时钟 ≤ 255 tick（filter_clock_hz 为 RMT group 时钟，默认 APB 80 MHz）
 *   - idle 计数 ≤ 32767 tick（按 channel_resolution_hz 换算）
 * 低波特率下毛刺过滤能力会被削弱，高波特率下空闲窗口会缩短，均属预期。
 */
esp_err_t su_rx_window_ns(uint32_t baud, uint32_t channel_resolution_hz,
                          uint32_t filter_clock_hz,
                          uint32_t *out_min_ns, uint32_t *out_max_ns);

size_t su_encode(const uint8_t *data, size_t len, const su_timing_t *t,
                 rmt_symbol_word_t *out, size_t out_words);

su_decoder_t *su_decoder_create(const su_timing_t *t, su_byte_cb_t cb, void *ctx);
void su_decoder_reset(su_decoder_t *d);
void su_decoder_feed(su_decoder_t *d, const rmt_symbol_word_t *syms, size_t n);
void su_decoder_destroy(su_decoder_t *d);

#ifdef __cplusplus
}
#endif
