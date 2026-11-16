#include "soft_uart_codec.h"

#include <string.h>

#include "esp_heap_caps.h"

#define SU_RMT_SRC_HZ 80000000u

esp_err_t su_timing_init(uint32_t baud, uint32_t *out_resolution_hz,
                         su_timing_t *out_timing)
{
    if (!out_timing) {
        return ESP_ERR_INVALID_ARG;
    }
    if (baud < 300 || baud > 1000000) {
        return ESP_ERR_INVALID_ARG;
    }

    /* N = round(80MHz / (baud * 80))，限制在 1..255 */
    uint32_t denom = baud * 80u;
    uint32_t n = (SU_RMT_SRC_HZ + denom / 2u) / denom;
    if (n < 1) {
        n = 1;
    }
    if (n > 255) {
        n = 255;
    }

    uint32_t resolution = SU_RMT_SRC_HZ / n;
    uint32_t bit_ticks = (resolution + baud / 2u) / baud;
    if (bit_ticks == 0) {
        bit_ticks = 1;
    }
    if (bit_ticks > SU_MAX_BIT_TICKS) {
        return ESP_ERR_INVALID_ARG;
    }

    out_timing->bit_ticks = bit_ticks;
    out_timing->frame_ticks = bit_ticks * 10u;
    for (int k = 0; k < 9; k++) {
        out_timing->sample_off[k] = (bit_ticks * (uint32_t)(2 * k + 3) + 1u) / 2u;
    }
    if (out_resolution_hz) {
        *out_resolution_hz = resolution;
    }
    return ESP_OK;
}

esp_err_t su_rx_window_ns(uint32_t baud, uint32_t channel_resolution_hz,
                          uint32_t filter_clock_hz,
                          uint32_t *out_min_ns, uint32_t *out_max_ns)
{
    if (!out_min_ns || !out_max_ns || baud < 300 || baud > 1000000 ||
        channel_resolution_hz == 0 || filter_clock_hz == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    uint64_t bit_ns = 1000000000ull / baud;
    uint64_t min_ns = bit_ns / 5u;
    uint64_t max_ns = bit_ns * 40u;
    if (max_ns < 8000000ull) {
        max_ns = 8000000ull;
    }

    /* 硬件寄存器上限：filter ≤ 255 tick，idle ≤ 32767 tick（向下取整保证不越界） */
    uint64_t filter_limit_ns = (255ull * 1000000000ull) / filter_clock_hz;
    uint64_t idle_limit_ns = (32767ull * 1000000000ull) / channel_resolution_hz;

    if (min_ns > filter_limit_ns) {
        min_ns = filter_limit_ns;
    }
    if (max_ns > idle_limit_ns) {
        max_ns = idle_limit_ns;
    }
    if (min_ns == 0) {
        min_ns = 1;
    }
    if (max_ns <= min_ns) {
        max_ns = min_ns + 1;
    }

    *out_min_ns = (uint32_t)min_ns;
    *out_max_ns = (uint32_t)max_ns;
    return ESP_OK;
}

size_t su_encode(const uint8_t *data, size_t len, const su_timing_t *t,
                 rmt_symbol_word_t *out, size_t out_words)
{    if (!data || !t || !out || t->bit_ticks == 0) {
        return 0;
    }
    size_t capacity_bytes = out_words / 5u;
    if (len > capacity_bytes) {
        len = capacity_bytes;
    }

    size_t wi = 0;
    for (size_t i = 0; i < len; i++) {
        uint32_t level[10];
        level[0] = 0; /* start */
        for (int b = 0; b < 8; b++) {
            level[1 + b] = (data[i] >> b) & 1u; /* LSB first */
        }
        level[9] = 1; /* stop */
        for (int s = 0; s < 10; s += 2) {
            out[wi].level0 = level[s];
            out[wi].duration0 = t->bit_ticks;
            out[wi].level1 = level[s + 1];
            out[wi].duration1 = t->bit_ticks;
            wi++;
        }
    }
    return wi;
}

struct su_decoder_s {
    su_timing_t t;
    su_byte_cb_t cb;
    void *ctx;

    uint64_t now;
    bool prev_valid;
    int prev_level;

    int run_level;
    uint32_t run_ticks;

    bool in_frame;
    uint64_t frame_start;
    int next_sample;
    uint8_t data;

    bool pending_valid;
    uint8_t pending_byte;

    bool have_next;
    uint64_t next_start;
};

static void dec_emit(su_decoder_t *d, uint8_t byte, bool framing_error, bool break_error)
{
    su_rx_flags_t flags = {
        .framing_error = framing_error,
        .break_error = break_error,
    };
    if (d->cb) {
        d->cb(d->ctx, byte, flags);
    }
}

/* 低电平 run 结束时判定 break / 帧错误字节 */
static void dec_run_end(su_decoder_t *d)
{
    if (d->run_level != 0) {
        return;
    }
    if (d->run_ticks > d->t.frame_ticks) {
        d->pending_valid = false;
        d->in_frame = false;
        d->have_next = false;
        dec_emit(d, 0, false, true);
    } else if (d->pending_valid) {
        d->pending_valid = false;
        dec_emit(d, d->pending_byte, true, false);
    }
}

static void dec_frame_begin(su_decoder_t *d, uint64_t t0)
{
    d->in_frame = true;
    d->frame_start = t0;
    d->next_sample = 0;
    d->data = 0;
    d->have_next = false;
}

static void dec_frame_consume(su_decoder_t *d, int level, uint64_t start, uint64_t end)
{
    (void)start;

    while (d->in_frame) {
        while (d->next_sample < 9) {
            uint64_t ts = d->frame_start + d->t.sample_off[d->next_sample];
            if (ts >= end) {
                return;
            }
            if (d->next_sample < 8) {
                if (level) {
                    d->data |= (uint8_t)(1u << d->next_sample);
                }
            } else if (level) {
                dec_emit(d, d->data, false, false);
            } else {
                d->pending_valid = true;
                d->pending_byte = d->data;
            }
            d->next_sample++;
        }

        d->in_frame = false;
        d->have_next = true;
        d->next_start = d->frame_start + d->t.frame_ticks;

        if (d->next_start < end) {
            if (level == 0) {
                d->have_next = false;
                dec_frame_begin(d, d->next_start);
                continue; /* 背靠背帧：同一段低电平里直接开始下一帧 */
            }
            d->have_next = false;
            return;
        }
        return;
    }
}

static void dec_edge(su_decoder_t *d, int level, uint32_t duration)
{
    if (duration == 0) {
        return;
    }
    uint64_t start = d->now;
    uint64_t end = start + duration;

    if (d->run_level != level) {
        dec_run_end(d);
        d->run_level = level;
        d->run_ticks = duration;
    } else if (d->run_ticks <= UINT32_MAX - duration) {
        d->run_ticks += duration;
    }

    if (!d->in_frame && d->have_next && d->next_start <= start) {
        if (level == 0 && d->next_start == start) {
            uint64_t t0 = d->next_start;
            d->have_next = false;
            dec_frame_begin(d, t0);
        } else {
            d->have_next = false;
        }
    }

    if (d->in_frame) {
        dec_frame_consume(d, level, start, end);
    }

    if (!d->in_frame && level == 0 && d->prev_valid && d->prev_level == 1) {
        dec_frame_begin(d, start);
        dec_frame_consume(d, level, start, end);
    }

    d->prev_valid = true;
    d->prev_level = level;
    d->now = end;
}

su_decoder_t *su_decoder_create(const su_timing_t *t, su_byte_cb_t cb, void *ctx)
{
    if (!t) {
        return NULL;
    }
    su_decoder_t *d = heap_caps_calloc(1, sizeof(*d), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!d) {
        return NULL;
    }
    d->t = *t;
    d->cb = cb;
    d->ctx = ctx;
    d->run_level = -1;
    return d;
}

void su_decoder_reset(su_decoder_t *d)
{
    if (!d) {
        return;
    }
    su_timing_t t = d->t;
    su_byte_cb_t cb = d->cb;
    void *ctx = d->ctx;
    memset(d, 0, sizeof(*d));
    d->t = t;
    d->cb = cb;
    d->ctx = ctx;
    d->run_level = -1;
}

void su_decoder_feed(su_decoder_t *d, const rmt_symbol_word_t *syms, size_t n)
{
    if (!d || !syms) {
        return;
    }
    for (size_t i = 0; i < n; i++) {
        dec_edge(d, syms[i].level0, syms[i].duration0);
        dec_edge(d, syms[i].level1, syms[i].duration1);
    }
}

void su_decoder_destroy(su_decoder_t *d)
{
    heap_caps_free(d);
}
