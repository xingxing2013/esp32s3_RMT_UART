#include "rmt_uart_frame.h"

#include <string.h>

#include "esp_heap_caps.h"

#define RU_RMT_SRC_HZ 80000000u

static uint8_t ru_parity_bits(rmt_uart_parity_t parity)
{
    return parity == RMT_UART_PARITY_NONE ? 0 : 1;
}

static uint8_t ru_stop_bits_x2(rmt_uart_stop_bits_t stop)
{
    switch (stop) {
    case RMT_UART_STOP_BITS_1_5:
        return 3;
    case RMT_UART_STOP_BITS_2:
        return 4;
    default:
        return 2;
    }
}

esp_err_t ru_timing_init(uint32_t baud, uint8_t data_bits, rmt_uart_parity_t parity,
                         rmt_uart_stop_bits_t stop, uint32_t *out_resolution_hz,
                         ru_timing_t *out_timing)
{
    if (!out_timing || baud < RMT_UART_BAUD_MIN || baud > RMT_UART_BAUD_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    if (data_bits < 5 || data_bits > 8) {
        return ESP_ERR_INVALID_ARG;
    }
    if (parity > RMT_UART_PARITY_ODD || stop > RMT_UART_STOP_BITS_2) {
        return ESP_ERR_INVALID_ARG;
    }

    /* N = round(80MHz / (baud * 80))，限制在 1..255 */
    uint32_t denom = baud * 80u;
    uint32_t n = (RU_RMT_SRC_HZ + denom / 2u) / denom;
    if (n < 1) {
        n = 1;
    }
    if (n > 255) {
        n = 255;
    }

    uint32_t resolution = RU_RMT_SRC_HZ / n;
    uint32_t bit_ticks = (resolution + baud / 2u) / baud;
    if (bit_ticks == 0) {
        bit_ticks = 1;
    }
    if (bit_ticks > RU_MAX_BIT_TICKS) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t pbits = ru_parity_bits(parity);
    uint8_t stop_x2 = ru_stop_bits_x2(stop);

    /* 帧长（半 bit 为单位）：2*(start + data + parity) + stop */
    uint32_t frame_x2 = 2u * (1u + data_bits + pbits) + stop_x2;
    uint64_t frame_ticks = ((uint64_t)bit_ticks * frame_x2 + 1u) / 2u;
    if (frame_ticks > 0xFFFFFFFFu) {
        return ESP_ERR_INVALID_ARG;
    }

    out_timing->data_bits = data_bits;
    out_timing->parity_bits = pbits;
    out_timing->stop_bits_x2 = stop_x2;
    out_timing->parity = parity;
    out_timing->bit_ticks = bit_ticks;
    out_timing->frame_ticks = (uint32_t)frame_ticks;
    out_timing->sample_count = (uint8_t)(data_bits + pbits + 1);

    uint8_t idx = 0;
    for (uint8_t k = 0; k < data_bits; k++) {
        out_timing->sample_off[idx++] = (bit_ticks * (uint32_t)(2 * k + 3) + 1u) / 2u;
    }
    if (pbits) {
        out_timing->sample_off[idx++] = (bit_ticks * (uint32_t)(2 * data_bits + 3) + 1u) / 2u;
    }
    out_timing->sample_off[idx++] =
        (bit_ticks * (uint32_t)(2 * (data_bits + pbits) + 3) + 1u) / 2u;

    if (out_resolution_hz) {
        *out_resolution_hz = resolution;
    }
    return ESP_OK;
}

esp_err_t ru_rx_window_ns(uint32_t baud, uint32_t channel_resolution_hz,
                          uint32_t filter_clock_hz, uint32_t *out_min_ns,
                          uint32_t *out_max_ns)
{
    if (!out_min_ns || !out_max_ns || baud < RMT_UART_BAUD_MIN || baud > RMT_UART_BAUD_MAX ||
        channel_resolution_hz == 0 || filter_clock_hz == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    uint64_t bit_ns = 1000000000ull / baud;
    uint64_t min_ns = bit_ns / 5u;
    uint64_t max_ns = bit_ns * 40u;
    if (max_ns < 8000000ull) {
        max_ns = 8000000ull;
    }

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

size_t ru_encode(const uint8_t *data, size_t len, const ru_timing_t *t,
                 rmt_symbol_word_t *out, size_t out_words)
{
    if (!data || !t || !out || t->bit_ticks == 0 || t->data_bits < 5 || t->data_bits > 8) {
        return 0;
    }
    size_t segs_per_byte = 1u + t->data_bits + t->parity_bits + 1u;
    size_t words_per_byte = (segs_per_byte + 1u) / 2u;
    size_t capacity_bytes = out_words / words_per_byte;
    if (len > capacity_bytes) {
        len = capacity_bytes;
    }

    size_t wi = 0;
    bool pending = false;
    for (size_t i = 0; i < len; i++) {
        uint32_t level[12];
        uint32_t duration[12];
        size_t segs = 0;
        uint32_t ones = 0;

        level[segs] = 0; /* start */
        duration[segs] = t->bit_ticks;
        segs++;

        for (uint8_t b = 0; b < t->data_bits; b++) {
            uint32_t bit = (data[i] >> b) & 1u;
            level[segs] = bit;
            duration[segs] = t->bit_ticks;
            segs++;
            ones += bit;
        }

        if (t->parity_bits) {
            uint32_t p = ones & 1u;
            if (t->parity == RMT_UART_PARITY_ODD) {
                p ^= 1u;
            }
            level[segs] = p;
            duration[segs] = t->bit_ticks;
            segs++;
        }

        level[segs] = 1; /* stop，可能是 1/1.5/2 bit */
        duration[segs] = (t->bit_ticks * t->stop_bits_x2) / 2u;
        segs++;

        for (size_t s = 0; s < segs; s++) {
            if (!pending) {
                out[wi].level0 = level[s];
                out[wi].duration0 = duration[s];
                pending = true;
            } else {
                out[wi].level1 = level[s];
                out[wi].duration1 = duration[s];
                pending = false;
                wi++;
            }
        }
    }
    if (pending) {
        /* 只允许在整段末尾出现 duration=0：RMT 硬件会把 0 时长当作 EOF */
        out[wi].level1 = 1;
        out[wi].duration1 = 0;
        wi++;
    }
    return wi;
}

struct ru_decoder_s {
    ru_timing_t t;
    ru_byte_cb_t cb;
    void *ctx;

    uint32_t now;
    bool prev_valid;
    int prev_level;

    int run_level;
    uint32_t run_ticks;

    bool in_frame;
    uint32_t frame_start;
    uint32_t next_sample_ts;
    int next_sample;
    uint8_t data;
    uint32_t ones;
    bool parity_level;
    bool stop_level;

    bool pending_valid;
    uint8_t pending_byte;
    bool pending_parity;

    bool have_next;
    uint32_t next_start;
};

/* 32 位 tick 的回绕安全比较：a 在 b 之前 */
#define RU_INLINE static inline __attribute__((always_inline))

RU_INLINE bool ts_before(uint32_t a, uint32_t b)
{
    return (int32_t)(a - b) < 0;
}

RU_INLINE void dec_emit(ru_decoder_t *d, uint8_t byte, bool framing, bool parity,
                            bool brk)
{
    ru_rx_flags_t flags = {
        .framing_error = framing,
        .parity_error = parity,
        .break_error = brk,
    };
    if (d->cb) {
        d->cb(d->ctx, byte, flags);
    }
}

RU_INLINE void dec_run_end(ru_decoder_t *d)
{
    if (d->run_level != 0) {
        return;
    }
    if (d->run_ticks > d->t.frame_ticks) {
        d->pending_valid = false;
        d->in_frame = false;
        d->have_next = false;
        dec_emit(d, 0, false, false, true);
    } else if (d->pending_valid) {
        d->pending_valid = false;
        dec_emit(d, d->pending_byte, true, d->pending_parity, false);
    }
}

RU_INLINE void dec_frame_begin(ru_decoder_t *d, uint32_t t0)
{
    d->in_frame = true;
    d->frame_start = t0;
    d->next_sample = 0;
    d->next_sample_ts = t0 + d->t.sample_off[0];
    d->data = 0;
    d->ones = 0;
    d->parity_level = false;
    d->stop_level = false;
    d->have_next = false;
}

RU_INLINE void dec_frame_consume(ru_decoder_t *d, int level, uint32_t end)
{
    while (d->in_frame) {
        while (d->next_sample < d->t.sample_count && ts_before(d->next_sample_ts, end)) {
            if (d->next_sample < d->t.data_bits) {
                if (level) {
                    d->data |= (uint8_t)(1u << d->next_sample);
                    d->ones++;
                }
            } else if (d->t.parity_bits && d->next_sample == d->t.data_bits) {
                d->parity_level = level != 0;
            } else {
                d->stop_level = level != 0;
            }
            d->next_sample++;
            if (d->next_sample < d->t.sample_count) {
                d->next_sample_ts = d->frame_start + d->t.sample_off[d->next_sample];
            }
        }
        if (d->next_sample < d->t.sample_count) {
            return;
        }

        bool parity_ok = true;
        if (d->t.parity_bits) {
            uint32_t expected = d->ones & 1u;
            if (d->t.parity == RMT_UART_PARITY_ODD) {
                expected ^= 1u;
            }
            parity_ok = (d->parity_level == (expected != 0));
        }

        d->in_frame = false;
        d->have_next = true;
        d->next_start = d->frame_start + d->t.frame_ticks;

        if (d->stop_level) {
            dec_emit(d, d->data, false, !parity_ok, false);
        } else {
            d->pending_valid = true;
            d->pending_byte = d->data;
            d->pending_parity = !parity_ok;
        }

        if (ts_before(d->next_start, end)) {
            if (level == 0) {
                d->have_next = false;
                dec_frame_begin(d, d->next_start);
                continue;
            }
            d->have_next = false;
            return;
        }
        return;
    }
}

RU_INLINE void dec_edge(ru_decoder_t *d, int level, uint32_t duration)
{
    uint32_t start = d->now;
    uint32_t end = start + duration;

    if (d->run_level != level) {
        dec_run_end(d);
        d->run_level = level;
        d->run_ticks = duration;
    } else if (d->run_ticks <= UINT32_MAX - duration) {
        d->run_ticks += duration;
    }

    if (!d->in_frame && d->have_next && (int32_t)(d->next_start - start) <= 0) {
        if (level == 0 && d->next_start == start) {
            uint32_t t0 = d->next_start;
            d->have_next = false;
            dec_frame_begin(d, t0);
        } else {
            d->have_next = false;
        }
    }

    if (d->in_frame) {
        dec_frame_consume(d, level, end);
    }

    if (!d->in_frame && level == 0 &&
        ((d->prev_valid && d->prev_level == 1) ||
         (!d->prev_valid && duration < d->t.frame_ticks))) {
        dec_frame_begin(d, start);
        dec_frame_consume(d, level, end);
    }

    d->prev_valid = true;
    d->prev_level = level;
    d->now = end;
}

ru_decoder_t *ru_decoder_create(const ru_timing_t *t, ru_byte_cb_t cb, void *ctx)
{
    if (!t) {
        return NULL;
    }
    ru_decoder_t *d = heap_caps_calloc(1, sizeof(*d), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!d) {
        return NULL;
    }
    d->t = *t;
    d->cb = cb;
    d->ctx = ctx;
    d->run_level = -1;
    return d;
}

void ru_decoder_reset(ru_decoder_t *d)
{
    if (!d) {
        return;
    }
    ru_timing_t t = d->t;
    ru_byte_cb_t cb = d->cb;
    void *ctx = d->ctx;
    memset(d, 0, sizeof(*d));
    d->t = t;
    d->cb = cb;
    d->ctx = ctx;
    d->run_level = -1;
}

void ru_decoder_feed(ru_decoder_t *d, const rmt_symbol_word_t *syms, size_t n)
{
    if (!d || !syms) {
        return;
    }
    for (size_t i = 0; i < n; i++) {
        if (syms[i].duration0) {
            dec_edge(d, syms[i].level0, syms[i].duration0);
        }
        if (syms[i].duration1) {
            dec_edge(d, syms[i].level1, syms[i].duration1);
        }
    }
}

void ru_decoder_destroy(ru_decoder_t *d)
{
    heap_caps_free(d);
}
