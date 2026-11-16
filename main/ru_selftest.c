#include "ru_selftest.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
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

    size_t start = (first > 4) ? (first - 4) : 0;
    char tx_hex[3 * 8 + 1] = {0};
    char rx_hex[3 * 8 + 1] = {0};
    for (size_t i = 0; i < 8 && start + i < len; i++) {
        sprintf(&tx_hex[i * 3], "%02X ", tx[start + i]);
        sprintf(&rx_hex[i * 3], "%02X ", (start + i < got) ? rx[start + i] : 0);
    }
    ESP_LOGE(TAG, "%s first_diff=%d of %u", name, (int)first, (unsigned)len);
    ESP_LOGE(TAG, "  @%u tx: %s", (unsigned)start, tx_hex);
    ESP_LOGE(TAG, "  @%u rx: %s", (unsigned)start, rx_hex);
}

/* 单路：写满 -> 读满 -> 比对 */
static bool run_case(const ch_desc_t *ch, uint32_t baud, size_t len)
{
    rmt_uart_config_t cfg = {
        .tx_gpio = ch->tx_gpio,
        .rx_gpio = ch->rx_gpio,
        .baud = baud,
        .invert = false,
        .rx_buffer_bytes = 4096,
        .tx_buffer_bytes = 2048,
    };

    rmt_uart_handle_t u = NULL;
    esp_err_t err = rmt_uart_init(&cfg, &u);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s @%u: new failed: %s", ch->name, (unsigned)baud,
                 esp_err_to_name(err));
        return false;
    }

    uint8_t *tx = malloc(len);
    uint8_t *rx = malloc(len);
    if (!tx || !rx) {
        free(tx);
        free(rx);
        rmt_uart_deinit(u);
        return false;
    }
    fill_random(tx, len);
    memset(rx, 0, len);

    size_t written = 0;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(3000);
    while (written < len) {
        int n = rmt_uart_write(u, tx + written, len - written, pdMS_TO_TICKS(100));
        if (n <= 0) {
            break;
        }
        written += (size_t)n;
    }

    size_t got = 0;
    while (got < len && xTaskGetTickCount() < deadline) {
        int n = rmt_uart_read(u, rx + got, len - got, pdMS_TO_TICKS(200));
        if (n <= 0) {
            break;
        }
        got += (size_t)n;
    }

    rmt_uart_stats_t st = {0};
    rmt_uart_get_stats(u, &st);
    bool match = (got == len) && (memcmp(tx, rx, len) == 0);
    bool ok = (written == len) && match && (st.framing_errors == 0) && (st.rx_overflow == 0);

    ESP_LOGI(TAG, "%s @%u %s: wrote=%u got=%u txb=%u rxb=%u match=%d fe=%u ovf=%u restarts=%u txerr=%u",
             ch->name, (unsigned)baud, ok ? "PASS" : "FAIL",
             (unsigned)written, (unsigned)got,
             (unsigned)st.tx_bytes, (unsigned)st.rx_bytes, match,
             (unsigned)st.framing_errors, (unsigned)st.rx_overflow,
             (unsigned)st.rx_restarts, (unsigned)st.tx_errors);
    if (!ok) {
        dump_mismatch(ch->name, tx, rx, got, len);
    }

    free(tx);
    free(rx);
    rmt_uart_deinit(u);
    return ok;
}

/* 三路同时收发（3 个 TX 任务 + 3 个 RX 任务并行），需要 3 组短接线 */
static bool run_concurrent(const ch_desc_t chs[SELFTEST_CHANNELS], uint32_t baud, size_t len)
{
    rmt_uart_handle_t handles[SELFTEST_CHANNELS] = {0};
    uint8_t *tx[SELFTEST_CHANNELS] = {0};
    uint8_t *rx[SELFTEST_CHANNELS] = {0};
    bool ok = false;

    for (int i = 0; i < SELFTEST_CHANNELS; i++) {
        rmt_uart_config_t cfg = {
            .tx_gpio = chs[i].tx_gpio,
            .rx_gpio = chs[i].rx_gpio,
            .baud = baud,
            .rx_buffer_bytes = 4096,
            .tx_buffer_bytes = 2048,
        };
        if (rmt_uart_init(&cfg, &handles[i]) != ESP_OK) {
            ESP_LOGE(TAG, "concurrent %s: new failed", chs[i].name);
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
            ESP_LOGE(TAG, "concurrent %s: wrote %u/%u", chs[i].name,
                     (unsigned)written, (unsigned)len);
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
        bool ch_ok = match && (st.framing_errors == 0) && (st.rx_overflow == 0);
        ESP_LOGI(TAG, "concurrent %s @%u %s: got=%u match=%d fe=%u ovf=%u restarts=%u",
                 chs[i].name, (unsigned)baud, ch_ok ? "PASS" : "FAIL",
                 (unsigned)got[i], got[i] == len, (unsigned)st.framing_errors,
                 (unsigned)st.rx_overflow, (unsigned)st.rx_restarts);
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

/* set_baud 在收发进行中调用：允许丢弃在途数据，但改完必须还能收发 */
static bool run_baud_switch(uint32_t baud_a, uint32_t baud_b)
{
    rmt_uart_config_t cfg = {
        .tx_gpio = CONFIG_RMT_UART_CH0_TX_GPIO,
        .rx_gpio = CONFIG_RMT_UART_CH0_RX_GPIO,
        .baud = baud_a,
        .rx_buffer_bytes = 2048,
        .tx_buffer_bytes = 1024,
    };

    rmt_uart_handle_t u = NULL;
    esp_err_t err = rmt_uart_init(&cfg, &u);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "baud-switch: new failed: %s", esp_err_to_name(err));
        return false;
    }

    bool ok = true;
    for (int round = 0; round < 5 && ok; round++) {
        const uint32_t seq[3] = {baud_b, baud_a, baud_b};
        for (size_t i = 0; i < 3 && ok; i++) {
            if (rmt_uart_set_baud(u, seq[i]) != ESP_OK) {
                ESP_LOGE(TAG, "baud-switch: set_baud(%u) failed", (unsigned)seq[i]);
                ok = false;
                break;
            }
            const uint8_t probe[2] = {'O', 'K'};
            uint8_t rx[2] = {0};
            int sent = rmt_uart_write(u, probe, 2, pdMS_TO_TICKS(200));
            int got = rmt_uart_read(u, rx, 2, pdMS_TO_TICKS(1000));
            bool match = (sent == 2) && (got == 2) && (memcmp(probe, rx, 2) == 0);
            if (!match) {
                ESP_LOGE(TAG, "baud-switch round=%d baud=%u: sent=%d got=%d match=%d",
                         round, (unsigned)seq[i], sent, got, match);
                ok = false;
            }
        }
    }

    rmt_uart_stats_t st = {0};
    rmt_uart_get_stats(u, &st);
    ESP_LOGI(TAG, "baud-switch @%u<->%u %s: fe=%u ovf=%u restarts=%u",
             (unsigned)baud_a, (unsigned)baud_b, ok ? "PASS" : "FAIL",
             (unsigned)st.framing_errors, (unsigned)st.rx_overflow,
             (unsigned)st.rx_restarts);

    rmt_uart_deinit(u);
    return ok;
}

void ru_selftest_run(void)
{
    ESP_LOGI(TAG, "selftest build %s %s", __DATE__, __TIME__);

    const ch_desc_t wired[SELFTEST_CHANNELS] = {
        {"wire-ch0", CONFIG_RMT_UART_CH0_TX_GPIO, CONFIG_RMT_UART_CH0_RX_GPIO},
        {"wire-ch1", CONFIG_RMT_UART_CH1_TX_GPIO, CONFIG_RMT_UART_CH1_RX_GPIO},
        {"wire-ch2", CONFIG_RMT_UART_CH2_TX_GPIO, CONFIG_RMT_UART_CH2_RX_GPIO},
    };

    int pass = 0, total = 0;
    for (int i = 0; i < SELFTEST_CHANNELS; i++) {
        total++;
        if (run_case(&wired[i], CONFIG_RMT_UART_DEMO_BAUD, SELFTEST_LEN)) {
            pass++;
        }
    }
    total++;
    if (run_concurrent(wired, CONFIG_RMT_UART_DEMO_BAUD, SELFTEST_LEN)) {
        pass++;
    }
    total++;
    if (run_baud_switch(115200, 9600)) {
        pass++;
    }

    ESP_LOGI(TAG, "selftest summary: %d/%d passed", pass, total);
}
