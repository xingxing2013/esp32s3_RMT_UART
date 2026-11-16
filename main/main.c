#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rmt_uart.h"
#include "sdkconfig.h"
#include "ru_diag.h"
#include "ru_selftest.h"
#include "ru_tests.h"

static const char *TAG = "app";

typedef struct {
    const char *name;
    int tx_gpio;
    int rx_gpio;
} ch_cfg_t;

static const ch_cfg_t channels[rmt_uart_MAX_CHANNELS] = {
    {"ch0", CONFIG_RMT_UART_CH0_TX_GPIO, CONFIG_RMT_UART_CH0_RX_GPIO},
    {"ch1", CONFIG_RMT_UART_CH1_TX_GPIO, CONFIG_RMT_UART_CH1_RX_GPIO},
    {"ch2", CONFIG_RMT_UART_CH2_TX_GPIO, CONFIG_RMT_UART_CH2_RX_GPIO},
};

static void echo_task(void *arg)
{
    rmt_uart_handle_t u = arg;
    uint8_t buf[64];

    for (;;) {
        int n = rmt_uart_read(u, buf, sizeof(buf), pdMS_TO_TICKS(1000));
        if (n > 0) {
            rmt_uart_write(u, buf, (size_t)n, pdMS_TO_TICKS(1000));
        }
    }
}

static void stats_task(void *arg)
{
    rmt_uart_handle_t *handles = arg;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        for (int i = 0; i < rmt_uart_MAX_CHANNELS; i++) {
            rmt_uart_stats_t st = {0};
            if (rmt_uart_get_stats(handles[i], &st) == ESP_OK) {
                ESP_LOGI(TAG, "%s tx=%u rx=%u fe=%u brk=%u ovf=%u restarts=%u",
                         channels[i].name, (unsigned)st.tx_bytes,
                         (unsigned)st.rx_bytes, (unsigned)st.framing_errors,
                         (unsigned)st.break_errors, (unsigned)st.rx_overflow,
                         (unsigned)st.rx_restarts);
            }
        }
    }
}

void app_main(void)
{
#if CONFIG_SU_DIAG_ENABLE
#if CONFIG_SU_DIAG_IRQ_ONLY
    ru_diag_irq_start(CONFIG_SU_DIAG_IRQ_KHZ);
#else
    ru_diag_run(CONFIG_SU_DIAG_SECONDS, CONFIG_SU_DIAG_BAUD,
                CONFIG_SU_DIAG_IRQ_KHZ, CONFIG_SU_DIAG_HOG_PERCENT);
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
#endif
#endif

#if CONFIG_RMT_UART_RUN_TESTS
    int codec_failures = ru_test_frame_run();
    int ring_failures = ru_test_ring_run();
    ESP_LOGI(TAG, "codec tests: %d failure(s)", codec_failures);
    ESP_LOGI(TAG, "ring tests: %d failure(s)", ring_failures);
#endif

#if CONFIG_RMT_UART_SELFTEST
    ru_selftest_run();
#endif

    static rmt_uart_handle_t handles[rmt_uart_MAX_CHANNELS];

    for (int i = 0; i < rmt_uart_MAX_CHANNELS; i++) {
        rmt_uart_config_t cfg = {
            .tx_gpio = channels[i].tx_gpio,
            .rx_gpio = channels[i].rx_gpio,
            .baud = CONFIG_RMT_UART_DEMO_BAUD,
            .rx_buffer_bytes = 2048,
            .tx_buffer_bytes = 1024,
        };
        ESP_ERROR_CHECK(rmt_uart_new(&cfg, &handles[i]));
        xTaskCreate(echo_task, channels[i].name, 3072, handles[i], 8, NULL);
        ESP_LOGI(TAG, "%s started tx=%d rx=%d baud=%d", channels[i].name,
                 channels[i].tx_gpio, channels[i].rx_gpio,
                 CONFIG_RMT_UART_DEMO_BAUD);
    }

    xTaskCreate(stats_task, "stats", 3072, handles, 4, NULL);
    ESP_LOGI(TAG, "3-channel soft UART running");
}
