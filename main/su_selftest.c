#include "su_selftest.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rmt_soft_uart.h"
#include "sdkconfig.h"

static const char *TAG = "su_selftest";

#define SELFTEST_CHANNELS 3
#define SELFTEST_LEN      1024

typedef struct {
    const char *name;
    int tx_gpio;
    int rx_gpio;
    bool internal;
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

/* 单路：写满 -> 读满 -> 比对 */
static bool run_case(const ch_desc_t *ch, uint32_t baud, size_t len)
{
    rmt_soft_uart_config_t cfg = {
        .tx_gpio = ch->tx_gpio,
        .rx_gpio = ch->rx_gpio,
        .baud = baud,
        .invert = false,
        .internal_loopback = ch->internal,
        .rx_buffer_bytes = 2048,
        .tx_buffer_bytes = 1024,
    };

    rmt_soft_uart_handle_t u = NULL;
    esp_err_t err = rmt_soft_uart_new(&cfg, &u);
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
        rmt_soft_uart_del(u);
        return false;
    }
    fill_random(tx, len);
    memset(rx, 0, len);

    size_t written = 0;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(2000);
    while (written < len) {
        int n = rmt_soft_uart_write(u, tx + written, len - written, pdMS_TO_TICKS(100));
        if (n <= 0) {
            break;
        }
        written += (size_t)n;
    }

    size_t got = 0;
    while (got < len && xTaskGetTickCount() < deadline) {
        int n = rmt_soft_uart_read(u, rx + got, len - got, pdMS_TO_TICKS(50));
        if (n <= 0) {
            break;
        }
        got += (size_t)n;
    }

    rmt_soft_uart_stats_t st = {0};
    rmt_soft_uart_get_stats(u, &st);
    bool match = (got == len) && (memcmp(tx, rx, len) == 0);
    bool ok = (written == len) && match && (st.framing_errors == 0) && (st.rx_overflow == 0);

    ESP_LOGI(TAG, "%s @%u %s: wrote=%u got=%u match=%d fe=%u ovf=%u restarts=%u",
             ch->name, (unsigned)baud, ok ? "PASS" : "FAIL",
             (unsigned)written, (unsigned)got, match,
             (unsigned)st.framing_errors, (unsigned)st.rx_overflow,
             (unsigned)st.rx_restarts);

    free(tx);
    free(rx);
    rmt_soft_uart_del(u);
    return ok;
}

/* 三路同时收发（3 个 TX 任务 + 3 个 RX 任务并行） */
static bool run_concurrent(const ch_desc_t chs[SELFTEST_CHANNELS], uint32_t baud, size_t len)
{
    rmt_soft_uart_handle_t handles[SELFTEST_CHANNELS] = {0};
    uint8_t *tx[SELFTEST_CHANNELS] = {0};
    uint8_t *rx[SELFTEST_CHANNELS] = {0};
    bool ok = false;

    for (int i = 0; i < SELFTEST_CHANNELS; i++) {
        rmt_soft_uart_config_t cfg = {
            .tx_gpio = chs[i].tx_gpio,
            .rx_gpio = chs[i].rx_gpio,
            .baud = baud,
            .internal_loopback = chs[i].internal,
            .rx_buffer_bytes = 4096,
            .tx_buffer_bytes = 2048,
        };
        if (rmt_soft_uart_new(&cfg, &handles[i]) != ESP_OK) {
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
            int n = rmt_soft_uart_write(handles[i], tx[i] + written, len - written,
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

    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(5000);
    for (int i = 0; i < SELFTEST_CHANNELS; i++) {
        size_t got = 0;
        while (got < len && xTaskGetTickCount() < deadline) {
            int n = rmt_soft_uart_read(handles[i], rx[i] + got, len - got, pdMS_TO_TICKS(50));
            if (n <= 0) {
                break;
            }
            got += (size_t)n;
        }
    }

    ok = true;
    for (int i = 0; i < SELFTEST_CHANNELS; i++) {
        rmt_soft_uart_stats_t st = {0};
        rmt_soft_uart_get_stats(handles[i], &st);
        bool match = (memcmp(tx[i], rx[i], len) == 0);
        bool ch_ok = match && (st.framing_errors == 0) && (st.rx_overflow == 0);
        ESP_LOGI(TAG, "concurrent %s @%u %s: match=%d fe=%u ovf=%u restarts=%u",
                 chs[i].name, (unsigned)baud, ch_ok ? "PASS" : "FAIL", match,
                 (unsigned)st.framing_errors, (unsigned)st.rx_overflow,
                 (unsigned)st.rx_restarts);
        ok = ok && ch_ok;
    }

cleanup:
    for (int i = 0; i < SELFTEST_CHANNELS; i++) {
        free(tx[i]);
        free(rx[i]);
        if (handles[i]) {
            rmt_soft_uart_del(handles[i]);
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
    rmt_soft_uart_config_t cfg = {
        .tx_gpio = CONFIG_RMT_SOFT_UART_CH0_TX_GPIO,
        .rx_gpio = CONFIG_RMT_SOFT_UART_CH0_TX_GPIO,
        .baud = baud_a,
        .internal_loopback = true,
        .rx_buffer_bytes = 2048,
        .tx_buffer_bytes = 1024,
    };

    rmt_soft_uart_handle_t u = NULL;
    esp_err_t err = rmt_soft_uart_new(&cfg, &u);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "baud-switch: new failed: %s", esp_err_to_name(err));
        return false;
    }

    bool ok = true;
    for (int round = 0; round < 5 && ok; round++) {
        const uint32_t seq[3] = {baud_b, baud_a, baud_b};
        for (size_t i = 0; i < 3 && ok; i++) {
            if (rmt_soft_uart_set_baud(u, seq[i]) != ESP_OK) {
                ESP_LOGE(TAG, "baud-switch: set_baud(%u) failed", (unsigned)seq[i]);
                ok = false;
                break;
            }
            const uint8_t probe[2] = {'O', 'K'};
            uint8_t rx[2] = {0};
            int sent = rmt_soft_uart_write(u, probe, 2, pdMS_TO_TICKS(200));
            int got = rmt_soft_uart_read(u, rx, 2, pdMS_TO_TICKS(1000));
            bool match = (sent == 2) && (got == 2) && (memcmp(probe, rx, 2) == 0);
            if (!match) {
                ESP_LOGE(TAG, "baud-switch round=%d baud=%u: sent=%d got=%d match=%d",
                         round, (unsigned)seq[i], sent, got, match);
                ok = false;
            }
        }
    }

    rmt_soft_uart_stats_t st = {0};
    rmt_soft_uart_get_stats(u, &st);
    ESP_LOGI(TAG, "baud-switch @%u<->%u %s: fe=%u ovf=%u restarts=%u",
             (unsigned)baud_a, (unsigned)baud_b, ok ? "PASS" : "FAIL",
             (unsigned)st.framing_errors, (unsigned)st.rx_overflow,
             (unsigned)st.rx_restarts);

    rmt_soft_uart_del(u);
    return ok;
}

void su_selftest_run(void)
{
    const ch_desc_t internal[SELFTEST_CHANNELS] = {
        {"internal-ch0", CONFIG_RMT_SOFT_UART_CH0_TX_GPIO,
         CONFIG_RMT_SOFT_UART_CH0_TX_GPIO, true},
        {"internal-ch1", CONFIG_RMT_SOFT_UART_CH1_TX_GPIO,
         CONFIG_RMT_SOFT_UART_CH1_TX_GPIO, true},
        {"internal-ch2", CONFIG_RMT_SOFT_UART_CH2_TX_GPIO,
         CONFIG_RMT_SOFT_UART_CH2_TX_GPIO, true},
    };
    const ch_desc_t wired[SELFTEST_CHANNELS] = {
        {"wire-ch0", CONFIG_RMT_SOFT_UART_CH0_TX_GPIO, CONFIG_RMT_SOFT_UART_CH0_RX_GPIO, false},
        {"wire-ch1", CONFIG_RMT_SOFT_UART_CH1_TX_GPIO, CONFIG_RMT_SOFT_UART_CH1_RX_GPIO, false},
        {"wire-ch2", CONFIG_RMT_SOFT_UART_CH2_TX_GPIO, CONFIG_RMT_SOFT_UART_CH2_RX_GPIO, false},
    };

    int pass = 0, total = 0;
    for (int i = 0; i < SELFTEST_CHANNELS; i++) {
        total++;
        if (run_case(&internal[i], CONFIG_RMT_SOFT_UART_DEMO_BAUD, SELFTEST_LEN)) {
            pass++;
        }
    }
    for (int i = 0; i < SELFTEST_CHANNELS; i++) {
        total++;
        if (run_case(&wired[i], CONFIG_RMT_SOFT_UART_DEMO_BAUD, SELFTEST_LEN)) {
            pass++;
        }
    }
    total++;
    if (run_concurrent(internal, CONFIG_RMT_SOFT_UART_DEMO_BAUD, SELFTEST_LEN)) {
        pass++;
    }
    total++;
    if (run_baud_switch(CONFIG_RMT_SOFT_UART_DEMO_BAUD, 9600)) {
        pass++;
    }

    ESP_LOGI(TAG, "selftest summary: %d/%d passed", pass, total);
    ESP_LOGI(TAG, "wire-* and concurrent cases require TX->RX jumper wires");
}
