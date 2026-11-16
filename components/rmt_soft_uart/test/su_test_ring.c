#include "su_tests.h"

#include "symbol_ring.h"
#include "unity.h"

static rmt_symbol_word_t make_sym(uint32_t v)
{
    rmt_symbol_word_t s = {0};
    s.duration0 = (uint16_t)(v & 0x7FFFu);
    s.level0 = (uint16_t)((v >> 15) & 1u);
    return s;
}

static void test_ring_push_pop_and_wrap(void)
{
    su_symbol_ring_t *r = su_symbol_ring_create(8);
    TEST_ASSERT_NOT_NULL(r);
    TEST_ASSERT_EQUAL_UINT32(0, su_symbol_ring_count(r));

    rmt_symbol_word_t in[8];
    for (int i = 0; i < 8; i++) {
        in[i] = make_sym((uint32_t)i);
    }
    TEST_ASSERT_EQUAL_size_t(8, su_symbol_ring_push_from_isr(r, in, 8));
    TEST_ASSERT_EQUAL_UINT32(8, su_symbol_ring_count(r));
    TEST_ASSERT_EQUAL_size_t(0, su_symbol_ring_push_from_isr(r, in, 1));

    rmt_symbol_word_t out[8] = {0};
    TEST_ASSERT_EQUAL_size_t(5, su_symbol_ring_pop(r, out, 5));
    for (int i = 0; i < 5; i++) {
        TEST_ASSERT_EQUAL_UINT32((uint32_t)i, out[i].duration0);
    }

    rmt_symbol_word_t more[5];
    for (int i = 0; i < 5; i++) {
        more[i] = make_sym((uint32_t)(100 + i));
    }
    TEST_ASSERT_EQUAL_size_t(5, su_symbol_ring_push_from_isr(r, more, 5));
    TEST_ASSERT_EQUAL_UINT32(8, su_symbol_ring_count(r));

    TEST_ASSERT_EQUAL_size_t(8, su_symbol_ring_pop(r, out, 8));
    TEST_ASSERT_EQUAL_UINT32(5, out[0].duration0);
    TEST_ASSERT_EQUAL_UINT32(7, out[2].duration0);
    TEST_ASSERT_EQUAL_UINT32(100, out[3].duration0);
    TEST_ASSERT_EQUAL_UINT32(104, out[7].duration0);
    TEST_ASSERT_EQUAL_UINT32(0, su_symbol_ring_count(r));

    su_symbol_ring_destroy(r);
}

static void test_ring_reset(void)
{
    su_symbol_ring_t *r = su_symbol_ring_create(4);
    TEST_ASSERT_NOT_NULL(r);
    rmt_symbol_word_t s = make_sym(1);
    TEST_ASSERT_EQUAL_size_t(1, su_symbol_ring_push_from_isr(r, &s, 1));
    TEST_ASSERT_EQUAL_UINT32(1, su_symbol_ring_count(r));
    su_symbol_ring_reset(r);
    TEST_ASSERT_EQUAL_UINT32(0, su_symbol_ring_count(r));
    su_symbol_ring_destroy(r);
}

int su_test_ring_run(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_ring_push_pop_and_wrap);
    RUN_TEST(test_ring_reset);
    return UNITY_END();
}
