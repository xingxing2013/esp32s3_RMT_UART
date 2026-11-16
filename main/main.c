#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "app";

void app_main(void)
{
    ESP_LOGI(TAG, "rmt soft uart scaffold");
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
