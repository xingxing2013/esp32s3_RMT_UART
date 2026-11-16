#include "rmt_uart_ring.h"

#include <string.h>

#include "esp_heap_caps.h"
#include "sdkconfig.h"

#if CONFIG_RMT_ISR_IRAM_SAFE
#error "rmt_uart 依赖 flash 中的 memcpy，必须关闭 CONFIG_RMT_ISR_IRAM_SAFE"
#endif

static inline uint32_t ring_load(const volatile uint32_t *p)
{
    return __atomic_load_n(p, __ATOMIC_ACQUIRE);
}

static inline void ring_store(volatile uint32_t *p, uint32_t v)
{
    __atomic_store_n(p, v, __ATOMIC_RELEASE);
}

ru_ring_t *ru_ring_create(uint32_t capacity)
{
    if (capacity == 0) {
        return NULL;
    }
    uint32_t cap = 1;
    while (cap < capacity) {
        cap <<= 1;
    }

    ru_ring_t *r = heap_caps_calloc(1, sizeof(*r), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!r) {
        return NULL;
    }
    r->buf = heap_caps_malloc(cap * sizeof(rmt_symbol_word_t),
                              MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!r->buf) {
        heap_caps_free(r);
        return NULL;
    }
    r->cap = cap;
    r->mask = cap - 1;
    return r;
}

void ru_ring_destroy(ru_ring_t *r)
{
    if (!r) {
        return;
    }
    heap_caps_free(r->buf);
    heap_caps_free(r);
}

size_t ru_ring_push_from_isr(ru_ring_t *r,
                                    const rmt_symbol_word_t *src, size_t n)
{
    if (!r || !src || n == 0) {
        return 0;
    }
    uint32_t head = r->head;
    uint32_t tail = ring_load(&r->tail);
    uint32_t used = head - tail;
    uint32_t free_cnt = r->cap - used;
    if (n > free_cnt) {
        n = free_cnt;
    }
    if (n == 0) {
        return 0;
    }

    uint32_t idx = head & r->mask;
    uint32_t first = r->cap - idx;
    if (first > n) {
        first = (uint32_t)n;
    }
    memcpy(&r->buf[idx], src, first * sizeof(rmt_symbol_word_t));
    if (n > first) {
        memcpy(&r->buf[0], src + first, (n - first) * sizeof(rmt_symbol_word_t));
    }
    ring_store(&r->head, head + (uint32_t)n);
    return n;
}

size_t ru_ring_pop(ru_ring_t *r, rmt_symbol_word_t *dst, size_t n)
{
    if (!r || !dst || n == 0) {
        return 0;
    }
    uint32_t tail = r->tail;
    uint32_t head = ring_load(&r->head);
    uint32_t avail = head - tail;
    if (n > avail) {
        n = avail;
    }
    if (n == 0) {
        return 0;
    }

    uint32_t idx = tail & r->mask;
    uint32_t first = r->cap - idx;
    if (first > n) {
        first = (uint32_t)n;
    }
    memcpy(dst, &r->buf[idx], first * sizeof(rmt_symbol_word_t));
    if (n > first) {
        memcpy(dst + first, &r->buf[0], (n - first) * sizeof(rmt_symbol_word_t));
    }
    ring_store(&r->tail, tail + (uint32_t)n);
    return n;
}

size_t ru_ring_count(const ru_ring_t *r)
{
    if (!r) {
        return 0;
    }
    return ring_load(&r->head) - ring_load(&r->tail);
}

void ru_ring_reset(ru_ring_t *r)
{
    if (!r) {
        return;
    }
    ring_store(&r->tail, ring_load(&r->head));
}
