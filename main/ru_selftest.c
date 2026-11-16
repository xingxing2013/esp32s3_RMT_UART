#include "ru_selftest.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "rmt_uart.h"
#include "sdkconfig.h"

static const char *TAG = "ru_selftest";

#define SELFTEST_CHANNELS 3
#define SELFTEST_LEN      1024

typedef struct {
    const char *name;
    int tx_gpio;
    int rx_gpio;
} ch_desc_t;

typedef struct {
    SemaphoreHandle_t sem;
    rmt_uart_async_result_t result;
    size_t transferred;
} async_wait_t;

static uint32_t lcg_state = 0x12345678u;

static uint8_t lcg_next(void)
{
    lcg_state = lcg_state * 1664525u + 1013904223u;
    return (uint8_t)(lcg_state >> 24);
}

static void fill_random(uint8_t *buf, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        buf[i] = lcg_next();
    }
}

static void async_done_cb(rmt_uart_handle_t h, rmt_uart_async_result_t result,
                          size_t transferred, void *ctx)
{
    (void)h;
    async_wait_t *w = ctx;
    w->result = result;
    w->transferred = transferred;
    xSemaphoreGive(w->sem);
}

static void dump_mismatch(const char *name, const uint8_t *tx, const uint8_t *rx,
                          size_t got, size_t len)
{
    size_t first = len;
    for (size_t i = 0; i < len && i < got; i++) {
        if (tx[i] != rx[i]) {
            first = i;
            break;
        }
    }
    if (first == len && got != len) {
        first = got;
    }
    ESP_LOGE(TAG, "%s first_diff=%d of %u", name, (int)first, (unsigned)len);
}

/* 同步：写满 -> 读满 -> 比对 */
static bool run_case(const ch_desc_t *ch, uint32_t baud, size_t len, uint8_t db,
                     rmt_uart_parity_t par, rmt_uart_stop_bits_t stop)
{
    rmt_uart_config_t cfg = {
        .tx_gpio = ch->tx_gpio,
        .rx_gpio = ch->rx_gpio,
        .baud = baud,
        .data_bits = db,
        .parity = par,
        .stop_bits = stop,
        .rx_buffer_bytes = 4096,
        .tx_buffer_bytes = 2048,
    };

    rmt_uart_handle_t h = NULL;
    esp_err_t err = rmt_uart_init(&cfg, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s @%u: init failed: %s", ch->name, (unsigned)baud,
                 esp_err_to_name(err));
        return false;
    }

    uint8_t *tx = malloc(len);
    uint8_t *rx = malloc(len);
    if (!tx || !rx) {
        free(tx);
        free(rx);
        rmt_uart_deinit(h);
        return false;
    }
    fill_random(tx, len);
    memset(rx, 0, len);

    size_t written = 0;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(3000);
    while (written < len) {
        int n = rmt_uart_write(h, tx + written, len - written, pdMS_TO_TICKS(100));
        if (n <= 0) {
            break;
        }
        written += (size_t)n;
    }

    size_t got = 0;
    while (got < len && xTaskGetTickCount() < deadline) {
        int n = rmt_uart_read(h, rx + got, len - got, pdMS_TO_TICKS(200));
        if (n <= 0) {
            break;
        }
        got += (size_t)n;
    }

    rmt_uart_stats_t st = {0};
    rmt_uart_get_stats(h, &st);
    uint8_t mask = (db >= 8) ? 0xFF : (uint8_t)((1u << db) - 1u);
    bool match = (got == len);
    for (size_t i = 0; match && i < len; i++) {
        if (rx[i] != (uint8_t)(tx[i] & mask)) {
            match = false;
        }
    }
    bool ok = (written == len) && match && (st.framing_errors == 0) &&
              (st.parity_errors == 0) && (st.rx_overflow == 0);

    ESP_LOGI(TAG, "%s @%u %u%c%u %s: wrote=%u got=%u match=%d fe=%u pe=%u ovf=%u",
             ch->name, (unsigned)baud, (unsigned)db,
             par == RMT_UART_PARITY_EVEN ? 'E' : (par == RMT_UART_PARITY_ODD ? 'O' : 'N'),
             (unsigned)(stop == RMT_UART_STOP_BITS_1 ? 1 : (stop == RMT_UART_STOP_BITS_1_5 ? 3 : 2)),
             ok ? "PASS" : "FAIL", (unsigned)written, (unsigned)got, match,
             (unsigned)st.framing_errors, (unsigned)st.parity_errors,
             (unsigned)st.rx_overflow);
    ESP_LOGI(TAG, "  txerr=%u restarts=%u", (unsigned)st.tx_errors,
             (unsigned)st.rx_restarts);
    if (!ok) {
        ESP_LOGI(TAG, "  tx: %02X %02X %02X %02X %02X %02X %02X %02X", tx[0], tx[1], tx[2], tx[3], tx[4], tx[5], tx[6], tx[7]);
        ESP_LOGI(TAG, "  rx: %02X %02X %02X %02X %02X %02X %02X %02X", rx[0], rx[1], rx[2], rx[3], rx[4], rx[5], rx[6], rx[7]);
    }
    if (!ok) {
        dump_mismatch(ch->name, tx, rx, got, len);
    }

    free(tx);
    free(rx);
    rmt_uart_deinit(h);
    return ok;
}

/* 异步：write_async 完成 -> read_async 完成 -> 比对 */
static bool run_async_case(const ch_desc_t *ch)
{
    rmt_uart_config_t cfg = {
        .tx_gpio = ch->tx_gpio,
        .rx_gpio = ch->rx_gpio,
        .baud = 115200,
        .rx_buffer_bytes = 4096,
        .tx_buffer_bytes = 2048,
    };
    rmt_uart_handle_t h = NULL;
    if (rmt_uart_init(&cfg, &h) != ESP_OK) {
        return false;
    }

    uint8_t tx[256], rx[256];
    fill_random(tx, sizeof(tx));
    memset(rx, 0, sizeof(rx));

    async_wait_t w = {.sem = xSemaphoreCreateBinary()};
    bool ok = (w.sem != NULL);

    if (ok) {
        ok = (rmt_uart_write_async(h, tx, sizeof(tx), async_done_cb, &w) == ESP_OK) &&
             (xSemaphoreTake(w.sem, pdMS_TO_TICKS(2000)) == pdTRUE) &&
             (w.result == RMT_UART_ASYNC_OK) && (w.transferred == sizeof(tx));
    }
    if (ok) {
        ok = (rmt_uart_read_async(h, rx, sizeof(rx), 2000, async_done_cb, &w) == ESP_OK) &&
             (xSemaphoreTake(w.sem, pdMS_TO_TICKS(3000)) == pdTRUE) &&
             (w.result == RMT_UART_ASYNC_OK) && (w.transferred == sizeof(rx)) &&
             (memcmp(tx, rx, sizeof(tx)) == 0);
    }

    rmt_uart_stats_t st = {0};
    rmt_uart_get_stats(h, &st);
    ESP_LOGI(TAG, "async @115200 %s: result=%d transferred=%u fe=%u ovf=%u",
             ok ? "PASS" : "FAIL", (int)w.result, (unsigned)w.transferred,
             (unsigned)st.framing_errors, (unsigned)st.rx_overflow);

    if (w.sem) {
        vSemaphoreDelete(w.sem);
    }
    rmt_uart_deinit(h);
    return ok;
}

typedef struct {
    uint8_t buf[256];
    size_t n;
    SemaphoreHandle_t sem;
} rx_cb_ctx_t;

static void rx_data_cb(rmt_uart_handle_t h, const uint8_t *data, size_t len, void *ctx)
{
    (void)h;
    rx_cb_ctx_t *c = ctx;
    if (c->n + len <= sizeof(c->buf)) {
        memcpy(c->buf + c->n, data, len);
        c->n += len;
    }
    if (c->n >= sizeof(c->buf)) {
        xSemaphoreGive(c->sem);
    }
}

/* 流式回调：on_rx_data 收到完整 256 字节 */
static bool run_callback_case(const ch_desc_t *ch)
{
    rmt_uart_config_t cfg = {
        .tx_gpio = ch->tx_gpio,
        .rx_gpio = ch->rx_gpio,
        .baud = 115200,
        .rx_buffer_bytes = 4096,
        .tx_buffer_bytes = 2048,
    };
    rmt_uart_handle_t h = NULL;
    if (rmt_uart_init(&cfg, &h) != ESP_OK) {
        return false;
    }

    rx_cb_ctx_t ctx = {.n = 0, .sem = xSemaphoreCreateBinary()};
    uint8_t tx[256];
    fill_random(tx, sizeof(tx));

    rmt_uart_callbacks_t cbs = {.on_rx_data = rx_data_cb};
    bool ok = (ctx.sem != NULL) &&
              (rmt_uart_register_callbacks(h, &cbs, &ctx) == ESP_OK);
    if (ok) {
        int sent = rmt_uart_write(h, tx, sizeof(tx), pdMS_TO_TICKS(200));
        ok = (sent == (int)sizeof(tx)) &&
             (xSemaphoreTake(ctx.sem, pdMS_TO_TICKS(2000)) == pdTRUE) &&
             (ctx.n == sizeof(tx)) && (memcmp(tx, ctx.buf, sizeof(tx)) == 0);
    }

    ESP_LOGI(TAG, "rx-callback @115200 %s: n=%u", ok ? "PASS" : "FAIL", (unsigned)ctx.n);
    if (ctx.sem) {
        vSemaphoreDelete(ctx.sem);
    }
    rmt_uart_deinit(h);
    return ok;
}

/* 端口容量：3 路占满后第 4 路必须失败 */
static bool run_port_limit(const ch_desc_t chs[SELFTEST_CHANNELS])
{
    rmt_uart_handle_t h[SELFTEST_CHANNELS] = {0};
    bool ok = true;

    for (int i = 0; i < SELFTEST_CHANNELS; i++) {
        rmt_uart_config_t cfg = {
            .tx_gpio = chs[i].tx_gpio,
            .rx_gpio = chs[i].rx_gpio,
            .baud = 115200,
        };
        if (rmt_uart_init(&cfg, &h[i]) != ESP_OK) {
            ok = false;
            break;
        }
    }

    if (ok) {
        rmt_uart_config_t extra = {
            .tx_gpio = CONFIG_RMT_UART_CH0_TX_GPIO,
            .rx_gpio = CONFIG_RMT_UART_CH0_RX_GPIO,
            .baud = 115200,
        };
        rmt_uart_handle_t h4 = NULL;
        esp_err_t err = rmt_uart_init(&extra, &h4);
        ok = (err == ESP_ERR_NOT_FOUND) || (err == ESP_ERR_INVALID_STATE);
        if (err == ESP_OK) {
            rmt_uart_deinit(h4);
        }
        ESP_LOGI(TAG, "port-limit %s: 4th init -> %s", ok ? "PASS" : "FAIL",
                 esp_err_to_name(err));
    }

    for (int i = 0; i < SELFTEST_CHANNELS; i++) {
        if (h[i]) {
            rmt_uart_deinit(h[i]);
        }
    }
    return ok;
}

/* 三路同时收发 */
static bool run_concurrent(const ch_desc_t chs[SELFTEST_CHANNELS], uint32_t baud,
                          const uint32_t *bauds, size_t len)
{
    rmt_uart_handle_t handles[SELFTEST_CHANNELS] = {0};
    uint8_t *tx[SELFTEST_CHANNELS] = {0};
    uint8_t *rx[SELFTEST_CHANNELS] = {0};
    bool ok = false;

    for (int i = 0; i < SELFTEST_CHANNELS; i++) {
        uint32_t b = bauds ? bauds[i] : baud;
        rmt_uart_config_t cfg = {
            .tx_gpio = chs[i].tx_gpio,
            .rx_gpio = chs[i].rx_gpio,
            .baud = b,
            .rx_buffer_bytes = 4096,
            .tx_buffer_bytes = 2048,
        };
        if (rmt_uart_init(&cfg, &handles[i]) != ESP_OK) {
            ESP_LOGE(TAG, "concurrent %s: init failed", chs[i].name);
            goto cleanup;
        }
        tx[i] = malloc(len);
        rx[i] = malloc(len);
        if (!tx[i] || !rx[i]) {
            goto cleanup;
        }
        fill_random(tx[i], len);
        memset(rx[i], 0, len);
    }

    for (int i = 0; i < SELFTEST_CHANNELS; i++) {
        size_t written = 0;
        while (written < len) {
            int n = rmt_uart_write(handles[i], tx[i] + written, len - written,
                                   pdMS_TO_TICKS(100));
            if (n <= 0) {
                break;
            }
            written += (size_t)n;
        }
        if (written != len) {
            goto cleanup;
        }
    }

    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(8000);
    size_t got[SELFTEST_CHANNELS] = {0};
    for (int i = 0; i < SELFTEST_CHANNELS; i++) {
        while (got[i] < len && xTaskGetTickCount() < deadline) {
            int n = rmt_uart_read(handles[i], rx[i] + got[i], len - got[i],
                                  pdMS_TO_TICKS(200));
            if (n <= 0) {
                break;
            }
            got[i] += (size_t)n;
        }
    }

    ok = true;
    for (int i = 0; i < SELFTEST_CHANNELS; i++) {
        rmt_uart_stats_t st = {0};
        rmt_uart_get_stats(handles[i], &st);
        bool match = (got[i] == len) && (memcmp(tx[i], rx[i], len) == 0);
        bool ch_ok = match && (st.framing_errors == 0) && (st.parity_errors == 0) &&
                     (st.rx_overflow == 0);
        ESP_LOGI(TAG, "concurrent %s @%u %s: got=%u match=%d fe=%u pe=%u ovf=%u",
                 chs[i].name, (unsigned)(bauds ? bauds[i] : baud), ch_ok ? "PASS" : "FAIL",
                 (unsigned)got[i], got[i] == len, (unsigned)st.framing_errors,
                 (unsigned)st.parity_errors, (unsigned)st.rx_overflow);
        ok = ok && ch_ok;
    }

cleanup:
    for (int i = 0; i < SELFTEST_CHANNELS; i++) {
        free(tx[i]);
        free(rx[i]);
        if (handles[i]) {
            rmt_uart_deinit(handles[i]);
        }
    }
    if (!ok) {
        ESP_LOGE(TAG, "concurrent @%u FAIL", (unsigned)baud);
    }
    return ok;
}

static bool run_baud_switch(uint32_t baud_a, uint32_t baud_b)
{
    rmt_uart_config_t cfg = {
        .tx_gpio = CONFIG_RMT_UART_CH0_TX_GPIO,
        .rx_gpio = CONFIG_RMT_UART_CH0_RX_GPIO,
        .baud = baud_a,
        .rx_buffer_bytes = 2048,
        .tx_buffer_bytes = 1024,
    };
    rmt_uart_handle_t h = NULL;
    if (rmt_uart_init(&cfg, &h) != ESP_OK) {
        return false;
    }

    bool ok = true;
    for (int round = 0; round < 5 && ok; round++) {
        const uint32_t seq[3] = {baud_b, baud_a, baud_b};
        for (size_t i = 0; i < 3 && ok; i++) {
            if (rmt_uart_set_baud(h, seq[i]) != ESP_OK) {
                ok = false;
                break;
            }
            const uint8_t probe[2] = {'O', 'K'};
            uint8_t rx[2] = {0};
            int sent = rmt_uart_write(h, probe, 2, pdMS_TO_TICKS(200));
            int got = rmt_uart_read(h, rx, 2, pdMS_TO_TICKS(1000));
            ok = (sent == 2) && (got == 2) && (memcmp(probe, rx, 2) == 0);
        }
    }
    ESP_LOGI(TAG, "baud-switch @%u<->%u %s", (unsigned)baud_a, (unsigned)baud_b,
             ok ? "PASS" : "FAIL");
    rmt_uart_deinit(h);
    return ok;
}

void ru_selftest_run(void)
{
    ESP_LOGI(TAG, "selftest build %s %s", __DATE__, __TIME__);

    const ch_desc_t chs[SELFTEST_CHANNELS] = {
        {"ch0", CONFIG_RMT_UART_CH0_TX_GPIO, CONFIG_RMT_UART_CH0_RX_GPIO},
        {"ch1", CONFIG_RMT_UART_CH1_TX_GPIO, CONFIG_RMT_UART_CH1_RX_GPIO},
        {"ch2", CONFIG_RMT_UART_CH2_TX_GPIO, CONFIG_RMT_UART_CH2_RX_GPIO},
    };
    const uint32_t baud = CONFIG_RMT_UART_DEMO_BAUD;

    int pass = 0, total = 0;

    /* 三路 8N1 */
    for (int i = 0; i < SELFTEST_CHANNELS; i++) {
        total++;
        if (run_case(&chs[i], baud, SELFTEST_LEN, 8, RMT_UART_PARITY_NONE,
                     RMT_UART_STOP_BITS_1)) {
            pass++;
        }
    }

    /* 帧格式矩阵（ch0） */
    const struct {
        uint8_t db;
        rmt_uart_parity_t par;
        rmt_uart_stop_bits_t stop;
    } formats[] = {
        {8, RMT_UART_PARITY_EVEN, RMT_UART_STOP_BITS_1},
        {8, RMT_UART_PARITY_ODD, RMT_UART_STOP_BITS_1},
        {8, RMT_UART_PARITY_NONE, RMT_UART_STOP_BITS_2},
        {7, RMT_UART_PARITY_EVEN, RMT_UART_STOP_BITS_1},
        {5, RMT_UART_PARITY_NONE, RMT_UART_STOP_BITS_1_5},
        {6, RMT_UART_PARITY_ODD, RMT_UART_STOP_BITS_2},
    };
    for (size_t i = 0; i < sizeof(formats) / sizeof(formats[0]); i++) {
        total++;
        if (run_case(&chs[0], baud, 256, formats[i].db, formats[i].par, formats[i].stop)) {
            pass++;
        }
    }

    total++;
    if (run_async_case(&chs[0])) {
        pass++;
    }
    total++;
    if (run_callback_case(&chs[1])) {
        pass++;
    }
    total++;
    if (run_port_limit(chs)) {
        pass++;
    }
    total++;
    if (run_concurrent(chs, baud, NULL, SELFTEST_LEN)) {
        pass++;
    }

    /* 三路不同波特率并发 */
    total++;
    {
        const uint32_t mixed[SELFTEST_CHANNELS] = {9600, 115200, 460800};
        if (run_concurrent(chs, baud, mixed, 256)) {
            pass++;
        }
    }
    total++;
    if (run_baud_switch(baud, 9600)) {
        pass++;
    }

    ESP_LOGI(TAG, "selftest summary: %d/%d passed", pass, total);
}
