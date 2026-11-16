#include <inttypes.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rmt_soft_uart.h"
#include "su_tests.h"

static const char *TAG = "app";

void app_main(void)
{
#if CONFIG_RMT_SOFT_UART_RUN_TESTS
    int codec_failures = su_test_codec_run();
    int ring_failures = su_test_ring_run();
    ESP_LOGI(TAG, "codec tests: %d failure(s)", codec_failures);
    ESP_LOGI(TAG, "ring tests: %d failure(s)", ring_failures);
#endif

    rmt_soft_uart_config_t cfg = {
        .tx_gpio = 4,
        .rx_gpio = 4, /* 内部回环 */
        .baud = 115200,
        .internal_loopback = true,
    };
    rmt_soft_uart_handle_t u = NULL;
    ESP_ERROR_CHECK(rmt_soft_uart_new(&cfg, &u));

    static const uint32_t seq[] = {115200, 9600, 115200};
    for (size_t i = 0; i < sizeof(seq) / sizeof(seq[0]); i++) {
        ESP_ERROR_CHECK(rmt_soft_uart_set_baud(u, seq[i]));
        vTaskDelay(pdMS_TO_TICKS(20));
        const uint8_t probe[] = {'O', 'K'};
        int sent = rmt_soft_uart_write(u, probe, sizeof(probe), pdMS_TO_TICKS(100));
        uint8_t rxbuf[2] = {0};
        int got = rmt_soft_uart_read(u, rxbuf, sizeof(rxbuf), pdMS_TO_TICKS(1000));
        ESP_LOGI(TAG, "baud=%" PRIu32 " sent=%d got=%d match=%d", seq[i], sent, got,
                 (got == 2) && (memcmp(probe, rxbuf, 2) == 0));
    }

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
