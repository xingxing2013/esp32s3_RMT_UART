/*
 * 最小示例：一路 8N1 @115200，收到什么回什么。
 *
 * 硬件：TX=GPIO42、RX=GPIO41 短接（或接真实设备）。
 */

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rmt_uart.h"

static const char *TAG = "basic_echo";

#define PORT_TX_GPIO 42
#define PORT_RX_GPIO 41

static void echo_task(void *arg)
{
    rmt_uart_handle_t h = arg;
    uint8_t buf[64];

    for (;;) {
        int n = rmt_uart_read(h, buf, sizeof(buf), pdMS_TO_TICKS(1000));
        if (n > 0) {
            rmt_uart_write(h, buf, (size_t)n, pdMS_TO_TICKS(1000));
        }
    }
}

void app_main(void)
{
    /* 1. 全局初始化：最多 3 路，这里只用 1 路 */
    ESP_ERROR_CHECK(rmt_uart_global_init(&(rmt_uart_global_config_t){.max_ports = 3}));

    /* 2. 实例初始化 */
    rmt_uart_config_t cfg = {
        .tx_gpio = PORT_TX_GPIO,
        .rx_gpio = PORT_RX_GPIO,
        .baud = 115200,
        .data_bits = 8,
        .parity = RMT_UART_PARITY_NONE,
        .stop_bits = RMT_UART_STOP_BITS_1,
    };

    rmt_uart_handle_t port = NULL;
    ESP_ERROR_CHECK(rmt_uart_init(&cfg, &port));

    xTaskCreate(echo_task, "echo", 3072, port, 8, NULL);

    ESP_LOGI(TAG, "8N1 @115200, TX=%d RX=%d, free ports=%u",
             PORT_TX_GPIO, PORT_RX_GPIO, rmt_uart_global_free_ports());
}
