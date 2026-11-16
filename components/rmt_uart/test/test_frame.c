#include "ru_tests.h"

#include <string.h>

#include "rmt_uart_frame.h"
#include "unity.h"

typedef struct {
    uint8_t buf[600];
    size_t n;
    uint32_t fe;
    uint32_t pe;
    uint32_t br;
} rx_sink_t;

static void sink_cb(void *ctx, uint8_t byte, ru_rx_flags_t flags)
{
    rx_sink_t *s = ctx;
    if (flags.break_error) {
        s->br++;
        return;
    }
    if (s->n < sizeof(s->buf)) {
        s->buf[s->n++] = byte;
    }
    if (flags.framing_error) {
        s->fe++;
    }
    if (flags.parity_error) {
        s->pe++;
    }
}

static size_t prepend_idle(rmt_symbol_word_t *out, uint32_t ticks)
{
    out[0].level0 = 1;
    out[0].duration0 = ticks;
    out[0].level1 = 0;
    out[0].duration1 = 0;
    return 1;
}

static size_t append_idle(rmt_symbol_word_t *out, size_t words, uint32_t ticks)
{
    while (ticks > 0) {
        uint32_t d = ticks > 32767u ? 32767u : ticks;
        out[words].level0 = 1;
        out[words].duration0 = d;
        out[words].level1 = 0;
        out[words].duration1 = 0;
        words++;
        ticks -= d;
    }
    return words;
}

static void test_timing_all_formats(void)
{
    for (uint8_t db = 5; db <= 8; db++) {
        for (int p = 0; p < 3; p++) {
            for (int s = 0; s < 3; s++) {
                const uint32_t bauds[] = {300, 115200, 1000000};
                for (size_t bi = 0; bi < 3; bi++) {
                    uint32_t res = 0;
                    ru_timing_t t = {0};
                    TEST_ASSERT_EQUAL(ESP_OK,
                        ru_timing_init(bauds[bi], db, (rmt_uart_parity_t)p,
                                       (rmt_uart_stop_bits_t)s, &res, &t));
                    TEST_ASSERT_TRUE(res > 0 && res <= 80000000u);
                    TEST_ASSERT_EQUAL_UINT8(db + (p ? 1 : 0) + 1, t.sample_count);
                    TEST_ASSERT_TRUE(t.frame_ticks > t.sample_off[t.sample_count - 1]);
                    for (uint8_t k = 1; k < t.sample_count; k++) {
                        TEST_ASSERT_TRUE(t.sample_off[k] > t.sample_off[k - 1]);
                    }
                    TEST_ASSERT_TRUE(t.bit_ticks >= 1 && t.bit_ticks <= 32767u);
                }
            }
        }
    }

    uint32_t res = 0;
    ru_timing_t t = {0};
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ru_timing_init(299, 8, RMT_UART_PARITY_NONE,
                                                          RMT_UART_STOP_BITS_1, &res, &t));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ru_timing_init(1000001, 8, RMT_UART_PARITY_NONE,
                                                          RMT_UART_STOP_BITS_1, &res, &t));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ru_timing_init(115200, 4, RMT_UART_PARITY_NONE,
                                                          RMT_UART_STOP_BITS_1, &res, &t));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ru_timing_init(115200, 9, RMT_UART_PARITY_NONE,
                                                          RMT_UART_STOP_BITS_1, &res, &t));
}

static void test_roundtrip_all_formats(void)
{
    uint8_t src[256];
    for (size_t i = 0; i < sizeof(src); i++) {
        src[i] = (uint8_t)(i * 37u + 11u);
    }

    for (uint8_t db = 5; db <= 8; db++) {
        uint8_t mask = (uint8_t)((1u << db) - 1u);
        uint8_t expect[256];
        for (size_t i = 0; i < sizeof(expect); i++) {
            expect[i] = src[i] & mask;
        }

        for (int p = 0; p < 3; p++) {
            for (int s = 0; s < 3; s++) {
                uint32_t res = 0;
                ru_timing_t t = {0};
                TEST_ASSERT_EQUAL(ESP_OK,
                    ru_timing_init(115200, db, (rmt_uart_parity_t)p,
                                   (rmt_uart_stop_bits_t)s, &res, &t));

                static rmt_symbol_word_t syms[1 + 256 * RU_WORDS_PER_BYTE];
                size_t words = prepend_idle(syms, 10 * t.bit_ticks);
                words += ru_encode(src, sizeof(src), &t, &syms[words], 256 * RU_WORDS_PER_BYTE);

                rx_sink_t sink = {0};
                ru_decoder_t *d = ru_decoder_create(&t, sink_cb, &sink);
                TEST_ASSERT_NOT_NULL(d);

                size_t i = 0;
                const size_t chunks[] = {1, 3, 7, 1u << 20};
                size_t ci = 0;
                while (i < words) {
                    size_t n = chunks[ci++ % 4];
                    if (n > words - i) {
                        n = words - i;
                    }
                    ru_decoder_feed(d, &syms[i], n);
                    i += n;
                }

                TEST_ASSERT_EQUAL_UINT32(sizeof(expect), sink.n);
                TEST_ASSERT_EQUAL_UINT8_ARRAY(expect, sink.buf, sizeof(expect));
                TEST_ASSERT_EQUAL_UINT32(0, sink.fe);
                TEST_ASSERT_EQUAL_UINT32(0, sink.pe);
                TEST_ASSERT_EQUAL_UINT32(0, sink.br);
                ru_decoder_destroy(d);
            }
        }
    }
}

static void test_parity_error_is_reported(void)
{
    uint32_t res = 0;
    ru_timing_t tx = {0}, rx = {0};
    TEST_ASSERT_EQUAL(ESP_OK, ru_timing_init(115200, 8, RMT_UART_PARITY_EVEN,
                                             RMT_UART_STOP_BITS_1, &res, &tx));
    TEST_ASSERT_EQUAL(ESP_OK, ru_timing_init(115200, 8, RMT_UART_PARITY_ODD,
                                             RMT_UART_STOP_BITS_1, &res, &rx));

    const uint8_t src[4] = {0x55, 0x00, 0xFF, 0x3C};
    static rmt_symbol_word_t syms[1 + 4 * RU_WORDS_PER_BYTE];
    size_t words = prepend_idle(syms, 10 * tx.bit_ticks);
    words += ru_encode(src, 4, &tx, &syms[words], 4 * RU_WORDS_PER_BYTE);

    rx_sink_t sink = {0};
    ru_decoder_t *d = ru_decoder_create(&rx, sink_cb, &sink);
    TEST_ASSERT_NOT_NULL(d);
    ru_decoder_feed(d, syms, words);

    TEST_ASSERT_EQUAL_UINT32(4, sink.n);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(src, sink.buf, 4);
    TEST_ASSERT_EQUAL_UINT32(4, sink.pe);
    TEST_ASSERT_EQUAL_UINT32(0, sink.fe);
    ru_decoder_destroy(d);
}

static void test_stop_bit_error_is_reported(void)
{
    uint32_t res = 0;
    ru_timing_t t = {0};
    TEST_ASSERT_EQUAL(ESP_OK, ru_timing_init(115200, 8, RMT_UART_PARITY_NONE,
                                             RMT_UART_STOP_BITS_1, &res, &t));

    rmt_symbol_word_t syms[5] = {0};
    const uint8_t src = 0x3C;
    TEST_ASSERT_EQUAL_size_t(5, ru_encode(&src, 1, &t, syms, 5));
    syms[4].level1 = 0; /* 停止位强制为低 */

    rmt_symbol_word_t stream[10];
    size_t words = prepend_idle(stream, 10 * t.bit_ticks);
    memcpy(&stream[words], syms, sizeof(syms));
    words += 5;
    words = append_idle(stream, words, 4 * t.bit_ticks);

    rx_sink_t sink = {0};
    ru_decoder_t *d = ru_decoder_create(&t, sink_cb, &sink);
    TEST_ASSERT_NOT_NULL(d);
    ru_decoder_feed(d, stream, words);
    TEST_ASSERT_EQUAL_UINT32(1, sink.n);
    TEST_ASSERT_EQUAL_UINT8(src, sink.buf[0]);
    TEST_ASSERT_EQUAL_UINT32(1, sink.fe);
    ru_decoder_destroy(d);
}

static void test_decode_roundtrip_chunked(void)
{
    uint32_t res = 0;
    ru_timing_t t = {0};
    TEST_ASSERT_EQUAL(ESP_OK, ru_timing_init(115200, 8, RMT_UART_PARITY_NONE,
                                             RMT_UART_STOP_BITS_1, &res, &t));

    uint8_t src[256];
    for (size_t i = 0; i < sizeof(src); i++) {
        src[i] = (uint8_t)(i * 37u + 11u);
    }
    static rmt_symbol_word_t syms[1 + 256 * RU_WORDS_PER_BYTE];
    size_t words = prepend_idle(syms, 10 * t.bit_ticks);
    words += ru_encode(src, sizeof(src), &t, &syms[words], 256 * RU_WORDS_PER_BYTE);

    rx_sink_t sink = {0};
    ru_decoder_t *d = ru_decoder_create(&t, sink_cb, &sink);
    TEST_ASSERT_NOT_NULL(d);

    const size_t chunks[] = {1, 2, 3, 5, 7, 11};
    size_t i = 0, ci = 0;
    while (i < words) {
        size_t n = chunks[ci++ % 6];
        if (n > words - i) {
            n = words - i;
        }
        ru_decoder_feed(d, &syms[i], n);
        i += n;
    }

    TEST_ASSERT_EQUAL_UINT32(256, sink.n);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(src, sink.buf, 256);
    TEST_ASSERT_EQUAL_UINT32(0, sink.fe);
    ru_decoder_destroy(d);
}

static void test_decode_clock_skew(void)
{
    const uint32_t rates[] = {9600, 115200};
    const uint32_t num[] = {97, 103};
    for (size_t r = 0; r < 2; r++) {
        for (size_t s = 0; s < 2; s++) {
            uint32_t res = 0;
            ru_timing_t t = {0};
            TEST_ASSERT_EQUAL(ESP_OK, ru_timing_init(rates[r], 8, RMT_UART_PARITY_NONE,
                                                     RMT_UART_STOP_BITS_1, &res, &t));

            uint8_t src[32];
            for (size_t i = 0; i < sizeof(src); i++) {
                src[i] = (uint8_t)(0xA5u ^ (i * 13u));
            }
            rmt_symbol_word_t syms[1 + 32 * RU_WORDS_PER_BYTE];
            size_t words = prepend_idle(syms, 10 * t.bit_ticks);
            size_t n = ru_encode(src, sizeof(src), &t, &syms[words], 32 * RU_WORDS_PER_BYTE);
            words += n;
            for (size_t i = 1; i < words; i++) {
                syms[i].duration0 = (uint16_t)((syms[i].duration0 * num[s] + 50u) / 100u);
                syms[i].duration1 = (uint16_t)((syms[i].duration1 * num[s] + 50u) / 100u);
            }

            rx_sink_t sink = {0};
            ru_decoder_t *d = ru_decoder_create(&t, sink_cb, &sink);
            TEST_ASSERT_NOT_NULL(d);
            ru_decoder_feed(d, syms, words);
            TEST_ASSERT_EQUAL_UINT32(sizeof(src), sink.n);
            TEST_ASSERT_EQUAL_UINT8_ARRAY(src, sink.buf, sizeof(src));
            TEST_ASSERT_EQUAL_UINT32(0, sink.fe);
            ru_decoder_destroy(d);
        }
    }
}

static void test_decode_with_idle_gaps_and_split_symbols(void)
{
    uint32_t res = 0;
    ru_timing_t t = {0};
    TEST_ASSERT_EQUAL(ESP_OK, ru_timing_init(9600, 8, RMT_UART_PARITY_NONE,
                                             RMT_UART_STOP_BITS_1, &res, &t));

    uint8_t src[16];
    for (size_t i = 0; i < sizeof(src); i++) {
        src[i] = (uint8_t)(i + 1);
    }

    static rmt_symbol_word_t stream[16 * RU_WORDS_PER_BYTE + 128];
    size_t words = 0;
    for (size_t i = 0; i < sizeof(src); i++) {
        rmt_symbol_word_t one[RU_WORDS_PER_BYTE];
        size_t n = ru_encode(&src[i], 1, &t, one, RU_WORDS_PER_BYTE);
        words = append_idle(stream, words, 100000);
        memcpy(&stream[words], one, n * sizeof(rmt_symbol_word_t));
        words += n;
    }
    words = append_idle(stream, words, 100000);

    rx_sink_t sink = {0};
    ru_decoder_t *d = ru_decoder_create(&t, sink_cb, &sink);
    TEST_ASSERT_NOT_NULL(d);
    ru_decoder_feed(d, stream, words);
    TEST_ASSERT_EQUAL_UINT32(sizeof(src), sink.n);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(src, sink.buf, sizeof(src));
    TEST_ASSERT_EQUAL_UINT32(0, sink.fe);
    ru_decoder_destroy(d);
}

static void test_decode_break_is_not_a_byte(void)
{
    uint32_t res = 0;
    ru_timing_t t = {0};
    TEST_ASSERT_EQUAL(ESP_OK, ru_timing_init(9600, 8, RMT_UART_PARITY_NONE,
                                             RMT_UART_STOP_BITS_1, &res, &t));

    rmt_symbol_word_t stream[2];
    stream[0].level0 = 0;
    stream[0].duration0 = 12u * t.bit_ticks;
    stream[0].level1 = 1;
    stream[0].duration1 = t.bit_ticks;

    rx_sink_t sink = {0};
    ru_decoder_t *d = ru_decoder_create(&t, sink_cb, &sink);
    TEST_ASSERT_NOT_NULL(d);
    ru_decoder_feed(d, stream, 2);
    TEST_ASSERT_EQUAL_UINT32(1, sink.br);
    TEST_ASSERT_EQUAL_UINT32(0, sink.n);
    ru_decoder_destroy(d);
}

static void test_decode_0x00_is_not_break(void)
{
    uint32_t res = 0;
    ru_timing_t t = {0};
    TEST_ASSERT_EQUAL(ESP_OK, ru_timing_init(115200, 8, RMT_UART_PARITY_NONE,
                                             RMT_UART_STOP_BITS_1, &res, &t));

    rmt_symbol_word_t syms[5] = {0};
    const uint8_t src = 0x00;
    TEST_ASSERT_EQUAL_size_t(5, ru_encode(&src, 1, &t, syms, 5));

    rmt_symbol_word_t stream[6];
    size_t words = prepend_idle(stream, 10 * t.bit_ticks);
    memcpy(&stream[words], syms, sizeof(syms));
    words += 5;

    rx_sink_t sink = {0};
    ru_decoder_t *d = ru_decoder_create(&t, sink_cb, &sink);
    TEST_ASSERT_NOT_NULL(d);
    ru_decoder_feed(d, stream, words);
    TEST_ASSERT_EQUAL_UINT32(0, sink.br);
    TEST_ASSERT_EQUAL_UINT32(1, sink.n);
    TEST_ASSERT_EQUAL_UINT8(0x00, sink.buf[0]);
    ru_decoder_destroy(d);
}

static void test_decode_recovers_after_glitch(void)
{
    uint32_t res = 0;
    ru_timing_t t = {0};
    TEST_ASSERT_EQUAL(ESP_OK, ru_timing_init(115200, 8, RMT_UART_PARITY_NONE,
                                             RMT_UART_STOP_BITS_1, &res, &t));

    rmt_symbol_word_t glitch[2] = {0};
    glitch[0].level0 = 0;
    glitch[0].duration0 = t.bit_ticks + t.bit_ticks / 2;
    glitch[0].level1 = 1;
    glitch[0].duration1 = 2 * t.bit_ticks;

    rmt_symbol_word_t syms[5] = {0};
    const uint8_t src = 0x7E;
    TEST_ASSERT_EQUAL_size_t(5, ru_encode(&src, 1, &t, syms, 5));

    rmt_symbol_word_t stream[24];
    size_t words = prepend_idle(stream, 10 * t.bit_ticks);
    memcpy(&stream[words], glitch, sizeof(glitch));
    words += 2;
    words = append_idle(stream, words, 12 * t.bit_ticks);
    memcpy(&stream[words], syms, sizeof(syms));
    words += 5;

    rx_sink_t sink = {0};
    ru_decoder_t *d = ru_decoder_create(&t, sink_cb, &sink);
    TEST_ASSERT_NOT_NULL(d);
    ru_decoder_feed(d, stream, words);
    TEST_ASSERT_TRUE(sink.n >= 1 && sink.n <= 2);
    TEST_ASSERT_EQUAL_UINT8(src, sink.buf[sink.n - 1]);
    ru_decoder_destroy(d);
}

static void test_rx_window_limits(void)
{
    const uint32_t bauds[] = {300, 9600, 115200, 460800, 1000000};
    for (size_t i = 0; i < sizeof(bauds) / sizeof(bauds[0]); i++) {
        uint32_t res = 0;
        ru_timing_t t = {0};
        TEST_ASSERT_EQUAL(ESP_OK, ru_timing_init(bauds[i], 8, RMT_UART_PARITY_NONE,
                                                 RMT_UART_STOP_BITS_1, &res, &t));

        uint32_t min_ns = 0, max_ns = 0;
        TEST_ASSERT_EQUAL(ESP_OK, ru_rx_window_ns(bauds[i], res, 80000000u,
                                                  &min_ns, &max_ns));
        uint64_t filter_ticks = (uint64_t)80000000u * min_ns / 1000000000ull;
        uint64_t idle_ticks = (uint64_t)res * max_ns / 1000000000ull;
        TEST_ASSERT_TRUE(filter_ticks <= 255);
        TEST_ASSERT_TRUE(idle_ticks <= 32767);
        TEST_ASSERT_TRUE(min_ns >= 1);
        TEST_ASSERT_TRUE((uint64_t)max_ns > 11ull * (1000000000ull / bauds[i]));
    }
}

static void test_break_boundary(void)
{
    uint32_t res = 0;
    ru_timing_t t = {0};
    TEST_ASSERT_EQUAL(ESP_OK, ru_timing_init(115200, 8, RMT_UART_PARITY_NONE,
                                             RMT_UART_STOP_BITS_1, &res, &t));

    rmt_symbol_word_t syms[5] = {0};
    const uint8_t src = 0x00;
    TEST_ASSERT_EQUAL_size_t(5, ru_encode(&src, 1, &t, syms, 5));
    syms[4].level1 = 0;

    rmt_symbol_word_t stream[10];
    size_t words = prepend_idle(stream, 10 * t.bit_ticks);
    memcpy(&stream[words], syms, sizeof(syms));
    words += 5;
    words = append_idle(stream, words, 4 * t.bit_ticks);

    rx_sink_t sink = {0};
    ru_decoder_t *d = ru_decoder_create(&t, sink_cb, &sink);
    TEST_ASSERT_NOT_NULL(d);
    ru_decoder_feed(d, stream, words);
    TEST_ASSERT_EQUAL_UINT32(0, sink.br);
    TEST_ASSERT_EQUAL_UINT32(1, sink.fe);
    TEST_ASSERT_EQUAL_UINT32(1, sink.n);
    ru_decoder_destroy(d);

    rmt_symbol_word_t syms2[5] = {0};
    TEST_ASSERT_EQUAL_size_t(5, ru_encode(&src, 1, &t, syms2, 5));
    syms2[4].level1 = 0;
    syms2[4].duration1 = (uint16_t)(t.bit_ticks + t.bit_ticks);

    rmt_symbol_word_t stream2[10];
    size_t words2 = prepend_idle(stream2, 10 * t.bit_ticks);
    memcpy(&stream2[words2], syms2, sizeof(syms2));
    words2 += 5;
    words2 = append_idle(stream2, words2, 2 * t.bit_ticks);

    rx_sink_t sink2 = {0};
    ru_decoder_t *d2 = ru_decoder_create(&t, sink_cb, &sink2);
    TEST_ASSERT_NOT_NULL(d2);
    ru_decoder_feed(d2, stream2, words2);
    TEST_ASSERT_EQUAL_UINT32(0, sink2.n);
    TEST_ASSERT_EQUAL_UINT32(1, sink2.br);
    ru_decoder_destroy(d2);
}

static void test_decode_recovers_after_symbol_loss(void)
{
    uint32_t res = 0;
    ru_timing_t t = {0};
    TEST_ASSERT_EQUAL(ESP_OK, ru_timing_init(115200, 8, RMT_UART_PARITY_NONE,
                                             RMT_UART_STOP_BITS_1, &res, &t));

    const uint8_t src[6] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66};
    static rmt_symbol_word_t syms[1 + 6 * RU_WORDS_PER_BYTE];
    size_t words = prepend_idle(syms, 10 * t.bit_ticks);
    words += ru_encode(src, 6, &t, &syms[words], 6 * RU_WORDS_PER_BYTE);

    rmt_symbol_word_t stream[1 + 6 * RU_WORDS_PER_BYTE];
    size_t out = 0;
    memcpy(&stream[out], &syms[0], sizeof(rmt_symbol_word_t));
    out += 1;
    memcpy(&stream[out], &syms[1], 2 * 5 * sizeof(rmt_symbol_word_t));
    out += 2 * 5;
    memcpy(&stream[out], &syms[1 + 3 * 5], 3 * 5 * sizeof(rmt_symbol_word_t));
    out += 3 * 5;

    rx_sink_t sink = {0};
    ru_decoder_t *d = ru_decoder_create(&t, sink_cb, &sink);
    TEST_ASSERT_NOT_NULL(d);
    ru_decoder_feed(d, stream, out);
    TEST_ASSERT_TRUE(sink.n >= 3);
    TEST_ASSERT_EQUAL_UINT8(0x44, sink.buf[sink.n - 3]);
    TEST_ASSERT_EQUAL_UINT8(0x55, sink.buf[sink.n - 2]);
    TEST_ASSERT_EQUAL_UINT8(0x66, sink.buf[sink.n - 1]);
    ru_decoder_destroy(d);
}

static void test_decode_skips_leading_low_run(void)
{
    uint32_t res = 0;
    ru_timing_t t = {0};
    TEST_ASSERT_EQUAL(ESP_OK, ru_timing_init(115200, 8, RMT_UART_PARITY_NONE,
                                             RMT_UART_STOP_BITS_1, &res, &t));

    const uint8_t src[4] = {0x75, 0xA3, 0x00, 0xFF};
    rmt_symbol_word_t syms[4 * RU_WORDS_PER_BYTE];
    size_t w = ru_encode(src, 4, &t, syms, 4 * RU_WORDS_PER_BYTE);

    rmt_symbol_word_t stream[32];
    size_t words = 0;
    stream[words].level0 = 0;
    stream[words].duration0 = 50 * t.bit_ticks;
    stream[words].level1 = 1;
    stream[words].duration1 = 10 * t.bit_ticks;
    words++;
    memcpy(&stream[words], syms, w * sizeof(rmt_symbol_word_t));
    words += w;

    rx_sink_t sink = {0};
    ru_decoder_t *d = ru_decoder_create(&t, sink_cb, &sink);
    TEST_ASSERT_NOT_NULL(d);
    ru_decoder_feed(d, stream, words);
    TEST_ASSERT_EQUAL_UINT32(4, sink.n);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(src, sink.buf, 4);
    TEST_ASSERT_EQUAL_UINT32(0, sink.fe);
    TEST_ASSERT_EQUAL_UINT32(1, sink.br);
    ru_decoder_destroy(d);
}

static void test_decode_cold_start_first_symbol_is_start_bit(void)
{
    uint32_t res = 0;
    ru_timing_t t = {0};
    TEST_ASSERT_EQUAL(ESP_OK, ru_timing_init(115200, 8, RMT_UART_PARITY_NONE,
                                             RMT_UART_STOP_BITS_1, &res, &t));

    const uint8_t src[3] = {0x75, 0x00, 0xFF};
    rmt_symbol_word_t syms[3 * RU_WORDS_PER_BYTE];
    size_t w = ru_encode(src, 3, &t, syms, 3 * RU_WORDS_PER_BYTE);

    rx_sink_t sink = {0};
    ru_decoder_t *d = ru_decoder_create(&t, sink_cb, &sink);
    TEST_ASSERT_NOT_NULL(d);
    ru_decoder_feed(d, syms, w);
    TEST_ASSERT_EQUAL_UINT32(3, sink.n);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(src, sink.buf, 3);
    TEST_ASSERT_EQUAL_UINT32(0, sink.fe);
    TEST_ASSERT_EQUAL_UINT32(0, sink.br);
    ru_decoder_destroy(d);
}

static void test_decode_flush_idle_completes_last_byte(void)
{
    uint32_t res = 0;
    ru_timing_t t = {0};
    TEST_ASSERT_EQUAL(ESP_OK, ru_timing_init(115200, 8, RMT_UART_PARITY_NONE,
                                             RMT_UART_STOP_BITS_1, &res, &t));

    rmt_symbol_word_t syms[5] = {0};
    const uint8_t src = 0x3C;
    TEST_ASSERT_EQUAL_size_t(5, ru_encode(&src, 1, &t, syms, 5));
    syms[4].level1 = 1;
    syms[4].duration1 = 0;

    rx_sink_t sink = {0};
    ru_decoder_t *d = ru_decoder_create(&t, sink_cb, &sink);
    TEST_ASSERT_NOT_NULL(d);
    ru_decoder_feed(d, syms, 5);
    TEST_ASSERT_EQUAL_UINT32(0, sink.n);

    rmt_symbol_word_t idle = {
        .level0 = 1,
        .duration0 = t.frame_ticks,
        .level1 = 1,
        .duration1 = 0,
    };
    ru_decoder_feed(d, &idle, 1);
    TEST_ASSERT_EQUAL_UINT32(1, sink.n);
    TEST_ASSERT_EQUAL_UINT8(src, sink.buf[0]);
    ru_decoder_destroy(d);
}

int ru_test_frame_run(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_timing_all_formats);
    RUN_TEST(test_roundtrip_all_formats);
    RUN_TEST(test_parity_error_is_reported);
    RUN_TEST(test_stop_bit_error_is_reported);
    RUN_TEST(test_decode_roundtrip_chunked);
    RUN_TEST(test_decode_clock_skew);
    RUN_TEST(test_decode_with_idle_gaps_and_split_symbols);
    RUN_TEST(test_decode_break_is_not_a_byte);
    RUN_TEST(test_decode_0x00_is_not_break);
    RUN_TEST(test_decode_recovers_after_glitch);
    RUN_TEST(test_rx_window_limits);
    RUN_TEST(test_break_boundary);
    RUN_TEST(test_decode_recovers_after_symbol_loss);
    RUN_TEST(test_decode_skips_leading_low_run);
    RUN_TEST(test_decode_cold_start_first_symbol_is_start_bit);
    RUN_TEST(test_decode_flush_idle_completes_last_byte);
    return UNITY_END();
}
