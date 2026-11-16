/*
 * rmt_uart 最小示例：两路软件串口，8N1 @115200，收到什么回什么。
 *
 * 外部工程引用方式：
 *   1. 把 components/rmt_uart 拷贝到工程 components/ 下，或
 *   2. 在 idf_component.yml 里依赖本组件包
 * 然后在 CMakeLists 里 REQUIRES rmt_uart。
 */

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rmt_uart.h"

static const char *TAG = "example";

#define PORT0_TX_GPIO 42
#define PORT0_RX_GPIO 41
#define PORT1_TX_GPIO 40
#define PORT1_RX_GPIO 39

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
    /* 1. 全局初始化：最多 3 路，用几路初始化几路 */
    ESP_ERROR_CHECK(rmt_uart_global_init(&(rmt_uart_global_config_t){.max_ports = 3}));

    rmt_uart_config_t cfg0 = {
        .tx_gpio = PORT0_TX_GPIO,
        .rx_gpio = PORT0_RX_GPIO,
        .baud = 115200,
        .data_bits = 8,
        .parity = RMT_UART_PARITY_NONE,
        .stop_bits = RMT_UART_STOP_BITS_1,
    };
    rmt_uart_config_t cfg1 = cfg0;
    cfg1.tx_gpio = PORT1_TX_GPIO;
    cfg1.rx_gpio = PORT1_RX_GPIO;
    cfg1.data_bits = 7;
    cfg1.parity = RMT_UART_PARITY_EVEN; /* 7E1 */

    rmt_uart_handle_t port0 = NULL;
    rmt_uart_handle_t port1 = NULL;
    ESP_ERROR_CHECK(rmt_uart_init(&cfg0, &port0));
    ESP_ERROR_CHECK(rmt_uart_init(&cfg1, &port1));

    xTaskCreate(echo_task, "echo0", 3072, port0, 8, NULL);
    xTaskCreate(echo_task, "echo1", 3072, port1, 8, NULL);

    ESP_LOGI(TAG, "port0 8N1 @115200, port1 7E1 @115200, free ports=%u",
             rmt_uart_global_free_ports());
}
