#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/rmt_types.h"
#include "esp_attr.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    rmt_symbol_word_t *buf;
    uint32_t cap;
    uint32_t mask;
    volatile uint32_t head;
    volatile uint32_t tail;
} su_symbol_ring_t;

su_symbol_ring_t *su_symbol_ring_create(uint32_t capacity);

void su_symbol_ring_destroy(su_symbol_ring_t *r);

/* 单生产者（ISR）批量入队，返回实际写入数量 */
IRAM_ATTR size_t su_symbol_ring_push_from_isr(su_symbol_ring_t *r,
                                              const rmt_symbol_word_t *src, size_t n);

/* 单消费者（任务）批量出队，返回实际读出数量 */
size_t su_symbol_ring_pop(su_symbol_ring_t *r, rmt_symbol_word_t *dst, size_t n);

size_t su_symbol_ring_count(const su_symbol_ring_t *r);

void su_symbol_ring_reset(su_symbol_ring_t *r);

#ifdef __cplusplus
}
#endif
