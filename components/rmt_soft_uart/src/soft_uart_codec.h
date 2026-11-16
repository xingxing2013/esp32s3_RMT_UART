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

size_t su_encode(const uint8_t *data, size_t len, const su_timing_t *t,
                 rmt_symbol_word_t *out, size_t out_words);

su_decoder_t *su_decoder_create(const su_timing_t *t, su_byte_cb_t cb, void *ctx);
void su_decoder_reset(su_decoder_t *d);
void su_decoder_feed(su_decoder_t *d, const rmt_symbol_word_t *syms, size_t n);
void su_decoder_destroy(su_decoder_t *d);

#ifdef __cplusplus
}
#endif
