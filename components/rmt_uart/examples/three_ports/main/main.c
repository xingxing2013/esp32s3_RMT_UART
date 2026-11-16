/*
 * 三路示例：不同帧格式、不同波特率、同步 / 异步 / 流式回调三种用法。
 *
 * 端口分配：
 *   ch0  8N1 @115200   同步 read/write 回声
 *   ch1  7E1 @9600     异步 read_async -> write
 *   ch2  8N2 @460800   on_rx_data 回调 + 队列回声
 *
 * 硬件：三组 TX/RX 分别短接（默认 42/41、40/39、38/37）。
 */

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "rmt_uart.h"

static const char *TAG = "three_ports";

#define CH0_TX 42
#define CH0_RX 41
#define CH1_TX 40
#define CH1_RX 39
#define CH2_TX 38
#define CH2_RX 37

/* ---------------- ch0：同步回声 ---------------- */

static void sync_echo_task(void *arg)
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

/* ---------------- ch1：异步读写 ---------------- */

typedef struct {
    SemaphoreHandle_t sem;
    rmt_uart_async_result_t result;
    size_t transferred;
} async_ctx_t;

static void async_done_cb(rmt_uart_handle_t h, rmt_uart_async_result_t result,
                          size_t transferred, void *ctx)
{
    (void)h;
    async_ctx_t *c = ctx;
    c->result = result;
    c->transferred = transferred;
    xSemaphoreGive(c->sem);
}

static void async_echo_task(void *arg)
{
    rmt_uart_handle_t h = arg;
    static uint8_t buf[32];
    async_ctx_t c = {.sem = xSemaphoreCreateBinary()};

    for (;;) {
        if (rmt_uart_read_async(h, buf, sizeof(buf), 500, async_done_cb, &c) != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (xSemaphoreTake(c.sem, portMAX_DELAY) == pdTRUE && c.transferred > 0) {
            rmt_uart_write(h, buf, c.transferred, pdMS_TO_TICKS(100));
        }
    }
}

/* ---------------- ch2：流式回调 ---------------- */

static QueueHandle_t s_rx_queue;

static void rx_data_cb(rmt_uart_handle_t h, const uint8_t *data, size_t len, void *ctx)
{
    (void)h;
    (void)ctx;
    for (size_t i = 0; i < len; i++) {
        /* 回调在驱动任务里，不要阻塞；队列满就丢弃 */
        xQueueSend(s_rx_queue, &data[i], 0);
    }
}

static void callback_echo_task(void *arg)
{
    rmt_uart_handle_t h = arg;
    uint8_t byte;

    for (;;) {
        if (xQueueReceive(s_rx_queue, &byte, portMAX_DELAY) == pdTRUE) {
            rmt_uart_write(h, &byte, 1, pdMS_TO_TICKS(100));
        }
    }
}

/* ---------------- 初始化 ---------------- */

static rmt_uart_handle_t make_port(int tx, int rx, uint32_t baud, uint8_t bits,
                                   rmt_uart_parity_t parity, rmt_uart_stop_bits_t stop,
                                   const char *name)
{
    rmt_uart_config_t cfg = {
        .tx_gpio = tx,
        .rx_gpio = rx,
        .baud = baud,
        .data_bits = bits,
        .parity = parity,
        .stop_bits = stop,
        .rx_buffer_bytes = 2048,
        .tx_buffer_bytes = 1024,
    };

    rmt_uart_handle_t h = NULL;
    esp_err_t err = rmt_uart_init(&cfg, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s init failed: %s", name, esp_err_to_name(err));
        return NULL;
    }
    ESP_LOGI(TAG, "%s ready: %u%c%u @%u", name, bits,
             parity == RMT_UART_PARITY_EVEN ? 'E' : (parity == RMT_UART_PARITY_ODD ? 'O' : 'N'),
             stop == RMT_UART_STOP_BITS_1 ? 1 : (stop == RMT_UART_STOP_BITS_1_5 ? 3 : 2),
             (unsigned)baud);
    return h;
}

void app_main(void)
{
    ESP_ERROR_CHECK(rmt_uart_global_init(&(rmt_uart_global_config_t){.max_ports = 3}));

    rmt_uart_handle_t ch0 = make_port(CH0_TX, CH0_RX, 115200, 8, RMT_UART_PARITY_NONE,
                                      RMT_UART_STOP_BITS_1, "ch0 8N1");
    rmt_uart_handle_t ch1 = make_port(CH1_TX, CH1_RX, 9600, 7, RMT_UART_PARITY_EVEN,
                                      RMT_UART_STOP_BITS_1, "ch1 7E1");
    rmt_uart_handle_t ch2 = make_port(CH2_TX, CH2_RX, 460800, 8, RMT_UART_PARITY_NONE,
                                      RMT_UART_STOP_BITS_2, "ch2 8N2");

    if (ch0) {
        xTaskCreate(sync_echo_task, "echo0", 3072, ch0, 8, NULL);
    }
    if (ch1) {
        xTaskCreate(async_echo_task, "echo1", 3072, ch1, 8, NULL);
    }
    if (ch2) {
        s_rx_queue = xQueueCreate(512, sizeof(uint8_t));
        rmt_uart_callbacks_t cbs = {.on_rx_data = rx_data_cb};
        ESP_ERROR_CHECK(rmt_uart_register_callbacks(ch2, &cbs, NULL));
        xTaskCreate(callback_echo_task, "echo2", 3072, ch2, 8, NULL);
    }

    ESP_LOGI(TAG, "free ports=%u", rmt_uart_global_free_ports());
}
