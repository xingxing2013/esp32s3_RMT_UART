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
        .rx_gpio = 5,
        .baud = 115200,
    };
    rmt_soft_uart_handle_t u = NULL;
    ESP_ERROR_CHECK(rmt_soft_uart_new(&cfg, &u));
    const char msg[] = "U";
    int sent = rmt_soft_uart_write(u, msg, 1, pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "write returned %d", sent);

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
