#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
