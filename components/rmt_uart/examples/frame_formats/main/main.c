/*
 * 帧格式示例：改这几个宏即可切换 5-8 数据位 / 校验 / 停止位。
 *
 * 运行 10 秒后演示一次 set_baud()（只改波特率；改帧格式必须 deinit + init）。
 */

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rmt_uart.h"

static const char *TAG = "frame_formats";

#define PORT_TX_GPIO 42
#define PORT_RX_GPIO 41

/* ---------------- 在这里选帧格式 ---------------- */
#define DEMO_BAUD       115200
#define DEMO_DATA_BITS  8              /* 5..8 */
#define DEMO_PARITY     RMT_UART_PARITY_NONE  /* NONE / EVEN / ODD */
#define DEMO_STOP_BITS  RMT_UART_STOP_BITS_1  /* 1 / 1.5 / 2 */
/* ------------------------------------------------ */

static void echo_task(void *arg)
{
    rmt_uart_handle_t h = arg;
    uint8_t buf[32];

    for (;;) {
        int n = rmt_uart_read(h, buf, sizeof(buf), pdMS_TO_TICKS(1000));
        if (n > 0) {
            rmt_uart_write(h, buf, (size_t)n, pdMS_TO_TICKS(1000));
        }
    }
}

static void baud_switch_task(void *arg)
{
    rmt_uart_handle_t h = arg;
    const uint32_t seq[] = {9600, 115200};

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        for (size_t i = 0; i < sizeof(seq) / sizeof(seq[0]); i++) {
            esp_err_t err = rmt_uart_set_baud(h, seq[i]);
            ESP_LOGI(TAG, "set_baud(%u) -> %s（会丢弃在途数据）",
                     (unsigned)seq[i], esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(rmt_uart_global_init(&(rmt_uart_global_config_t){.max_ports = 3}));

    rmt_uart_config_t cfg = {
        .tx_gpio = PORT_TX_GPIO,
        .rx_gpio = PORT_RX_GPIO,
        .baud = DEMO_BAUD,
        .data_bits = DEMO_DATA_BITS,
        .parity = DEMO_PARITY,
        .stop_bits = DEMO_STOP_BITS,
        .rx_buffer_bytes = 2048,
        .tx_buffer_bytes = 1024,
    };

    rmt_uart_handle_t port = NULL;
    ESP_ERROR_CHECK(rmt_uart_init(&cfg, &port));

    ESP_LOGI(TAG, "format: %d%c%d @%d", DEMO_DATA_BITS,
             DEMO_PARITY == RMT_UART_PARITY_EVEN ? 'E' :
             (DEMO_PARITY == RMT_UART_PARITY_ODD ? 'O' : 'N'),
             DEMO_STOP_BITS == RMT_UART_STOP_BITS_1 ? 1 :
             (DEMO_STOP_BITS == RMT_UART_STOP_BITS_1_5 ? 3 : 2),
             DEMO_BAUD);

    xTaskCreate(echo_task, "echo", 3072, port, 8, NULL);
    xTaskCreate(baud_switch_task, "baud", 3072, port, 5, NULL);
}
