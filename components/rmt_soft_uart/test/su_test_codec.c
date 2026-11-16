#include "su_tests.h"

#include <string.h>

#include "soft_uart_codec.h"
#include "unity.h"

typedef struct {
    uint8_t buf[600];
    size_t n;
    uint32_t fe;
    uint32_t br;
} rx_sink_t;

static void sink_cb(void *ctx, uint8_t byte, su_rx_flags_t flags)
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
}

static void test_timing_bounds(void)
{
    const uint32_t bauds[] = {300, 9600, 115200, 460800, 1000000};
    for (size_t i = 0; i < sizeof(bauds) / sizeof(bauds[0]); i++) {
        uint32_t res = 0;
        su_timing_t t = {0};
        TEST_ASSERT_EQUAL(ESP_OK, su_timing_init(bauds[i], &res, &t));
        TEST_ASSERT_TRUE(res > 0 && res <= 80000000u);
        TEST_ASSERT_TRUE(t.bit_ticks >= 1 && t.bit_ticks <= 32767u);
        TEST_ASSERT_EQUAL_UINT32(t.bit_ticks * 10u, t.frame_ticks);
        TEST_ASSERT_TRUE(t.sample_off[8] < t.frame_ticks);
        TEST_ASSERT_TRUE(t.sample_off[8] > t.sample_off[0]);
    }

    uint32_t res = 0;
    su_timing_t t = {0};
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, su_timing_init(299, &res, &t));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, su_timing_init(1000001, &res, &t));
}

static void test_encode_0x55(void)
{
    uint32_t res = 0;
    su_timing_t t = {0};
    TEST_ASSERT_EQUAL(ESP_OK, su_timing_init(115200, &res, &t));

    rmt_symbol_word_t words[5] = {0};
    const uint8_t src = 0x55;
    TEST_ASSERT_EQUAL_size_t(5, su_encode(&src, 1, &t, words, 5));

    /* 0x55 = start(0) 1 0 1 0 1 0 1 0 stop(1) -> 每对都是 (0,1) */
    for (int i = 0; i < 5; i++) {
        TEST_ASSERT_EQUAL_UINT32(0, words[i].level0);
        TEST_ASSERT_EQUAL_UINT32(1, words[i].level1);
        TEST_ASSERT_EQUAL_UINT32(t.bit_ticks, words[i].duration0);
        TEST_ASSERT_EQUAL_UINT32(t.bit_ticks, words[i].duration1);
    }
}

static void test_encode_0xaa(void)
{
    uint32_t res = 0;
    su_timing_t t = {0};
    TEST_ASSERT_EQUAL(ESP_OK, su_timing_init(9600, &res, &t));

    rmt_symbol_word_t words[5] = {0};
    const uint8_t src = 0xAA;
    TEST_ASSERT_EQUAL_size_t(5, su_encode(&src, 1, &t, words, 5));

    /* start(0) 0 1 0 1 0 1 0 1 stop(1) -> (0,0)(1,0)(1,0)(1,0)(1,1) */
    TEST_ASSERT_EQUAL_UINT32(0, words[0].level0);
    TEST_ASSERT_EQUAL_UINT32(0, words[0].level1);
    TEST_ASSERT_EQUAL_UINT32(1, words[4].level0);
    TEST_ASSERT_EQUAL_UINT32(1, words[4].level1);
}

static void test_encode_truncates_when_buffer_small(void)
{
    uint32_t res = 0;
    su_timing_t t = {0};
    TEST_ASSERT_EQUAL(ESP_OK, su_timing_init(115200, &res, &t));

    uint8_t src[4] = {1, 2, 3, 4};
    rmt_symbol_word_t words[10] = {0}; /* 只够 2 个字节 */
    TEST_ASSERT_EQUAL_size_t(10, su_encode(src, 4, &t, words, 10));
}

static size_t prepend_idle(rmt_symbol_word_t *out, uint32_t ticks)
{
    out[0].level0 = 1;
    out[0].duration0 = ticks;
    out[0].level1 = 0;
    out[0].duration1 = 0;
    return 1;
}

static void test_decode_roundtrip_chunked(void)
{
    uint32_t res = 0;
    su_timing_t t = {0};
    TEST_ASSERT_EQUAL(ESP_OK, su_timing_init(115200, &res, &t));

    uint8_t src[256];
    for (size_t i = 0; i < sizeof(src); i++) {
        src[i] = (uint8_t)(i * 37u + 11u);
    }
    static rmt_symbol_word_t syms[1 + 256 * 5];
    size_t words = prepend_idle(syms, 10 * t.bit_ticks);
    words += su_encode(src, sizeof(src), &t, &syms[words], 256 * 5);
    TEST_ASSERT_EQUAL_size_t(1 + 256 * 5, words);

    rx_sink_t sink = {0};
    su_decoder_t *d = su_decoder_create(&t, sink_cb, &sink);
    TEST_ASSERT_NOT_NULL(d);

    const size_t chunks[] = {1, 2, 3, 5, 7, 11};
    size_t i = 0, ci = 0;
    while (i < words) {
        size_t n = chunks[ci++ % (sizeof(chunks) / sizeof(chunks[0]))];
        if (n > words - i) {
            n = words - i;
        }
        su_decoder_feed(d, &syms[i], n);
        i += n;
    }

    TEST_ASSERT_EQUAL_UINT32(256, sink.n);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(src, sink.buf, 256);
    TEST_ASSERT_EQUAL_UINT32(0, sink.fe);
    TEST_ASSERT_EQUAL_UINT32(0, sink.br);
    su_decoder_destroy(d);
}

static void test_decode_clock_skew(void)
{
    const uint32_t rates[] = {9600, 115200};
    const uint32_t num[] = {97, 103};
    for (size_t r = 0; r < 2; r++) {
        for (size_t s = 0; s < 2; s++) {
            uint32_t res = 0;
            su_timing_t t = {0};
            TEST_ASSERT_EQUAL(ESP_OK, su_timing_init(rates[r], &res, &t));

            uint8_t src[32];
            for (size_t i = 0; i < sizeof(src); i++) {
                src[i] = (uint8_t)(0xA5u ^ (i * 13u));
            }
            rmt_symbol_word_t syms[1 + 32 * 5];
            size_t words = prepend_idle(syms, 10 * t.bit_ticks);
            size_t n = su_encode(src, sizeof(src), &t, &syms[words], 32 * 5);
            words += n;
            for (size_t i = 1; i < words; i++) {
                syms[i].duration0 = (uint16_t)((syms[i].duration0 * num[s] + 50u) / 100u);
                syms[i].duration1 = (uint16_t)((syms[i].duration1 * num[s] + 50u) / 100u);
            }

            rx_sink_t sink = {0};
            su_decoder_t *d = su_decoder_create(&t, sink_cb, &sink);
            TEST_ASSERT_NOT_NULL(d);
            su_decoder_feed(d, syms, words);
            TEST_ASSERT_EQUAL_UINT32(sizeof(src), sink.n);
            TEST_ASSERT_EQUAL_UINT8_ARRAY(src, sink.buf, sizeof(src));
            TEST_ASSERT_EQUAL_UINT32(0, sink.fe);
            su_decoder_destroy(d);
        }
    }
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

static void test_decode_with_idle_gaps_and_split_symbols(void)
{
    uint32_t res = 0;
    su_timing_t t = {0};
    TEST_ASSERT_EQUAL(ESP_OK, su_timing_init(9600, &res, &t));

    uint8_t src[16];
    for (size_t i = 0; i < sizeof(src); i++) {
        src[i] = (uint8_t)(i + 1);
    }

    static rmt_symbol_word_t stream[16 * 5 + 128];
    size_t words = 0;
    for (size_t i = 0; i < sizeof(src); i++) {
        rmt_symbol_word_t one[5];
        TEST_ASSERT_EQUAL_size_t(5, su_encode(&src[i], 1, &t, one, 5));
        words = append_idle(stream, words, 100000); /* 32767 拆分的长空闲 */
        memcpy(&stream[words], one, sizeof(one));
        words += 5;
    }
    words = append_idle(stream, words, 100000);

    rx_sink_t sink = {0};
    su_decoder_t *d = su_decoder_create(&t, sink_cb, &sink);
    TEST_ASSERT_NOT_NULL(d);
    su_decoder_feed(d, stream, words);
    TEST_ASSERT_EQUAL_UINT32(sizeof(src), sink.n);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(src, sink.buf, sizeof(src));
    TEST_ASSERT_EQUAL_UINT32(0, sink.fe);
    su_decoder_destroy(d);
}

static void test_decode_framing_error_still_delivers_byte(void)
{
    uint32_t res = 0;
    su_timing_t t = {0};
    TEST_ASSERT_EQUAL(ESP_OK, su_timing_init(115200, &res, &t));

    rmt_symbol_word_t syms[5] = {0};
    const uint8_t src = 0x3C;
    TEST_ASSERT_EQUAL_size_t(5, su_encode(&src, 1, &t, syms, 5));
    syms[4].level1 = 0; /* 停止位强制为低 */

    rmt_symbol_word_t stream[16];
    size_t words = prepend_idle(stream, 10 * t.bit_ticks);
    memcpy(&stream[words], syms, sizeof(syms));
    words += 5;
    words = append_idle(stream, words, 4 * t.bit_ticks); /* 拉高结束低电平 run */

    rx_sink_t sink = {0};
    su_decoder_t *d = su_decoder_create(&t, sink_cb, &sink);
    TEST_ASSERT_NOT_NULL(d);
    su_decoder_feed(d, stream, words);
    TEST_ASSERT_EQUAL_UINT32(1, sink.n);
    TEST_ASSERT_EQUAL_UINT8(src, sink.buf[0]);
    TEST_ASSERT_EQUAL_UINT32(1, sink.fe);
    su_decoder_destroy(d);
}

static void test_decode_break_is_not_a_byte(void)
{
    uint32_t res = 0;
    su_timing_t t = {0};
    TEST_ASSERT_EQUAL(ESP_OK, su_timing_init(9600, &res, &t));

    rmt_symbol_word_t stream[2];
    stream[0].level0 = 0;
    stream[0].duration0 = 12u * t.bit_ticks;
    stream[0].level1 = 1;
    stream[0].duration1 = t.bit_ticks;

    rx_sink_t sink = {0};
    su_decoder_t *d = su_decoder_create(&t, sink_cb, &sink);
    TEST_ASSERT_NOT_NULL(d);
    su_decoder_feed(d, stream, 2);
    TEST_ASSERT_EQUAL_UINT32(1, sink.br);
    TEST_ASSERT_EQUAL_UINT32(0, sink.n);
    su_decoder_destroy(d);
}

static void test_decode_0x00_is_not_break(void)
{
    uint32_t res = 0;
    su_timing_t t = {0};
    TEST_ASSERT_EQUAL(ESP_OK, su_timing_init(115200, &res, &t));

    rmt_symbol_word_t syms[5] = {0};
    const uint8_t src = 0x00;
    TEST_ASSERT_EQUAL_size_t(5, su_encode(&src, 1, &t, syms, 5));

    rmt_symbol_word_t stream[6];
    size_t words = prepend_idle(stream, 10 * t.bit_ticks);
    memcpy(&stream[words], syms, sizeof(syms));
    words += 5;

    rx_sink_t sink = {0};
    su_decoder_t *d = su_decoder_create(&t, sink_cb, &sink);
    TEST_ASSERT_NOT_NULL(d);
    su_decoder_feed(d, stream, words);
    TEST_ASSERT_EQUAL_UINT32(0, sink.br);
    TEST_ASSERT_EQUAL_UINT32(1, sink.n);
    TEST_ASSERT_EQUAL_UINT8(0x00, sink.buf[0]);
    su_decoder_destroy(d);
}

static void test_decode_recovers_after_glitch(void)
{
    uint32_t res = 0;
    su_timing_t t = {0};
    TEST_ASSERT_EQUAL(ESP_OK, su_timing_init(115200, &res, &t));

    rmt_symbol_word_t glitch[2] = {0};
    glitch[0].level0 = 0;
    glitch[0].duration0 = t.bit_ticks + t.bit_ticks / 2;
    glitch[0].level1 = 1;
    glitch[0].duration1 = 2 * t.bit_ticks;

    rmt_symbol_word_t syms[5] = {0};
    const uint8_t src = 0x7E;
    TEST_ASSERT_EQUAL_size_t(5, su_encode(&src, 1, &t, syms, 5));

    rmt_symbol_word_t stream[24];
    size_t words = prepend_idle(stream, 10 * t.bit_ticks);
    memcpy(&stream[words], glitch, sizeof(glitch));
    words += 2;
    words = append_idle(stream, words, 12 * t.bit_ticks); /* 毛刺后线路恢复空闲 */
    memcpy(&stream[words], syms, sizeof(syms));
    words += 5;

    rx_sink_t sink = {0};
    su_decoder_t *d = su_decoder_create(&t, sink_cb, &sink);
    TEST_ASSERT_NOT_NULL(d);
    su_decoder_feed(d, stream, words);
    /* 允许毛刺产生 1 个假字节，但毛刺之后必须恢复并正确解出 0x7E */
    TEST_ASSERT_TRUE(sink.n >= 1 && sink.n <= 2);
    TEST_ASSERT_EQUAL_UINT8(src, sink.buf[sink.n - 1]);
    su_decoder_destroy(d);
}

static void test_rx_window_limits(void)
{
    const uint32_t bauds[] = {300, 9600, 115200, 460800, 1000000};
    for (size_t i = 0; i < sizeof(bauds) / sizeof(bauds[0]); i++) {
        uint32_t res = 0;
        su_timing_t t = {0};
        TEST_ASSERT_EQUAL(ESP_OK, su_timing_init(bauds[i], &res, &t));

        uint32_t min_ns = 0, max_ns = 0;
        TEST_ASSERT_EQUAL(ESP_OK, su_rx_window_ns(bauds[i], res, 80000000u,
                                                  &min_ns, &max_ns));

        uint64_t filter_ticks = (uint64_t)80000000u * min_ns / 1000000000ull;
        uint64_t idle_ticks = (uint64_t)res * max_ns / 1000000000ull;
        TEST_ASSERT_TRUE(filter_ticks <= 255);
        TEST_ASSERT_TRUE(idle_ticks <= 32767);
        TEST_ASSERT_TRUE(min_ns >= 1);
        TEST_ASSERT_TRUE(max_ns > min_ns);
        /* 空闲窗口必须长于一帧，否则接收事务会在帧中途被截断 */
        TEST_ASSERT_TRUE((uint64_t)max_ns > 11ull * (1000000000ull / bauds[i]));
    }
}

static void test_break_boundary(void)
{
    uint32_t res = 0;
    su_timing_t t = {0};
    TEST_ASSERT_EQUAL(ESP_OK, su_timing_init(115200, &res, &t));

    /* 10 bit 低电平（起始 + 8 个 0 + 停止位为低）→ 帧错误，字节仍投递 */
    rmt_symbol_word_t syms[5] = {0};
    const uint8_t src = 0x00;
    TEST_ASSERT_EQUAL_size_t(5, su_encode(&src, 1, &t, syms, 5));
    syms[4].level1 = 0;

    rmt_symbol_word_t stream[8];
    size_t words = prepend_idle(stream, 10 * t.bit_ticks);
    memcpy(&stream[words], syms, sizeof(syms));
    words += 5;
    words = append_idle(stream, words, 4 * t.bit_ticks);

    rx_sink_t sink = {0};
    su_decoder_t *d = su_decoder_create(&t, sink_cb, &sink);
    TEST_ASSERT_NOT_NULL(d);
    su_decoder_feed(d, stream, words);
    TEST_ASSERT_EQUAL_UINT32(0, sink.br);
    TEST_ASSERT_EQUAL_UINT32(1, sink.fe);
    TEST_ASSERT_EQUAL_UINT32(1, sink.n);
    TEST_ASSERT_EQUAL_UINT8(0x00, sink.buf[0]);
    su_decoder_destroy(d);

    /* 11 bit 低电平 → break，不投递字节 */
    rmt_symbol_word_t syms2[5] = {0};
    TEST_ASSERT_EQUAL_size_t(5, su_encode(&src, 1, &t, syms2, 5));
    syms2[4].level1 = 0;
    syms2[4].duration1 = (uint16_t)(t.bit_ticks + t.bit_ticks);

    rmt_symbol_word_t stream2[10];
    size_t words2 = prepend_idle(stream2, 10 * t.bit_ticks);
    memcpy(&stream2[words2], syms2, sizeof(syms2));
    words2 += 5;
    words2 = append_idle(stream2, words2, 2 * t.bit_ticks); /* break 需要上升沿才会判定 */

    rx_sink_t sink2 = {0};
    su_decoder_t *d2 = su_decoder_create(&t, sink_cb, &sink2);
    TEST_ASSERT_NOT_NULL(d2);
    su_decoder_feed(d2, stream2, words2);
    TEST_ASSERT_EQUAL_UINT32(0, sink2.n);
    TEST_ASSERT_EQUAL_UINT32(1, sink2.br);
    su_decoder_destroy(d2);
}

static void test_decode_recovers_after_symbol_loss(void)
{
    uint32_t res = 0;
    su_timing_t t = {0};
    TEST_ASSERT_EQUAL(ESP_OK, su_timing_init(115200, &res, &t));

    const uint8_t src[6] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66};
    rmt_symbol_word_t syms[1 + 6 * 5];
    size_t words = prepend_idle(syms, 10 * t.bit_ticks);
    words += su_encode(src, 6, &t, &syms[words], 6 * 5);
    TEST_ASSERT_EQUAL_size_t(1 + 30, words);

    /* 丢掉第 3 个字节的全部 symbol，模拟 ring / 字节缓冲溢出 */
    rmt_symbol_word_t stream[1 + 6 * 5];
    size_t out = 0;
    memcpy(&stream[out], &syms[0], sizeof(rmt_symbol_word_t));
    out += 1;
    memcpy(&stream[out], &syms[1], 2 * 5 * sizeof(rmt_symbol_word_t));
    out += 2 * 5;
    memcpy(&stream[out], &syms[1 + 3 * 5], 3 * 5 * sizeof(rmt_symbol_word_t));
    out += 3 * 5;

    rx_sink_t sink = {0};
    su_decoder_t *d = su_decoder_create(&t, sink_cb, &sink);
    TEST_ASSERT_NOT_NULL(d);
    su_decoder_feed(d, stream, out);

    /* 丢数据后允许出现少量垃圾，但必须重新同步并正确解出后 3 个字节 */
    TEST_ASSERT_TRUE(sink.n >= 3);
    TEST_ASSERT_EQUAL_UINT8(0x44, sink.buf[sink.n - 3]);
    TEST_ASSERT_EQUAL_UINT8(0x55, sink.buf[sink.n - 2]);
    TEST_ASSERT_EQUAL_UINT8(0x66, sink.buf[sink.n - 1]);
    su_decoder_destroy(d);
}

static void test_decode_skips_leading_low_run(void)
{
    uint32_t res = 0;
    su_timing_t t = {0};
    TEST_ASSERT_EQUAL(ESP_OK, su_timing_init(115200, &res, &t));

    /* 布防时线路被拉低（TX 未初始化）产生的长低电平，不应被当成帧或 break */
    const uint8_t src[4] = {0x75, 0xA3, 0x00, 0xFF};
    rmt_symbol_word_t syms[4 * 5];
    TEST_ASSERT_EQUAL_size_t(20, su_encode(src, 4, &t, syms, 20));

    rmt_symbol_word_t stream[32];
    size_t words = 0;
    stream[words].level0 = 0; /* 50 bit 低电平 */
    stream[words].duration0 = 50 * t.bit_ticks;
    stream[words].level1 = 1;
    stream[words].duration1 = 10 * t.bit_ticks;
    words++;
    memcpy(&stream[words], syms, sizeof(syms));
    words += 20;

    rx_sink_t sink = {0};
    su_decoder_t *d = su_decoder_create(&t, sink_cb, &sink);
    TEST_ASSERT_NOT_NULL(d);
    su_decoder_feed(d, stream, words);
    TEST_ASSERT_EQUAL_UINT32(4, sink.n);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(src, sink.buf, 4);
    TEST_ASSERT_EQUAL_UINT32(0, sink.fe);
    /* 布防时就被拉低的线路会被报告为一次 break（不投递字节） */
    TEST_ASSERT_EQUAL_UINT32(1, sink.br);
    su_decoder_destroy(d);
}

static void test_decode_cold_start_first_symbol_is_start_bit(void)
{
    uint32_t res = 0;
    su_timing_t t = {0};
    TEST_ASSERT_EQUAL(ESP_OK, su_timing_init(115200, &res, &t));

    /* RMT 不为布防前的空闲期产生 symbol，流的第一个 symbol 就是起始位 */
    const uint8_t src[3] = {0x75, 0x00, 0xFF};
    rmt_symbol_word_t syms[3 * 5];
    TEST_ASSERT_EQUAL_size_t(15, su_encode(src, 3, &t, syms, 15));

    rx_sink_t sink = {0};
    su_decoder_t *d = su_decoder_create(&t, sink_cb, &sink);
    TEST_ASSERT_NOT_NULL(d);
    su_decoder_feed(d, syms, 15);
    TEST_ASSERT_EQUAL_UINT32(3, sink.n);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(src, sink.buf, 3);
    TEST_ASSERT_EQUAL_UINT32(0, sink.fe);
    TEST_ASSERT_EQUAL_UINT32(0, sink.br);
    su_decoder_destroy(d);
}

static void test_decode_flush_idle_completes_last_byte(void)
{
    uint32_t res = 0;
    su_timing_t t = {0};
    TEST_ASSERT_EQUAL(ESP_OK, su_timing_init(115200, &res, &t));

    /* 模拟 RMT 没交付最后一段高电平：停止位缺失，帧停在 next_sample==8 */
    rmt_symbol_word_t syms[5] = {0};
    const uint8_t src = 0x3C;
    TEST_ASSERT_EQUAL_size_t(5, su_encode(&src, 1, &t, syms, 5));
    syms[4].level1 = 1;
    syms[4].duration1 = 0;

    rx_sink_t sink = {0};
    su_decoder_t *d = su_decoder_create(&t, sink_cb, &sink);
    TEST_ASSERT_NOT_NULL(d);
    su_decoder_feed(d, syms, 5);
    TEST_ASSERT_EQUAL_UINT32(0, sink.n);

    /* 驱动在事务结束时由 ISR 补入的空闲符号 */
    rmt_symbol_word_t idle = {
        .level0 = 1,
        .duration0 = t.frame_ticks,
        .level1 = 1,
        .duration1 = 0,
    };
    su_decoder_feed(d, &idle, 1);
    TEST_ASSERT_EQUAL_UINT32(1, sink.n);
    TEST_ASSERT_EQUAL_UINT8(src, sink.buf[0]);
    TEST_ASSERT_EQUAL_UINT32(0, sink.fe);
    su_decoder_destroy(d);
}

int su_test_codec_run(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_timing_bounds);
    RUN_TEST(test_encode_0x55);
    RUN_TEST(test_encode_0xaa);
    RUN_TEST(test_encode_truncates_when_buffer_small);
    RUN_TEST(test_decode_roundtrip_chunked);
    RUN_TEST(test_decode_clock_skew);
    RUN_TEST(test_decode_with_idle_gaps_and_split_symbols);
    RUN_TEST(test_decode_framing_error_still_delivers_byte);
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
