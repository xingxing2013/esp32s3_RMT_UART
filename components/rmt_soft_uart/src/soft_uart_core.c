#include "rmt_soft_uart.h"

#include <inttypes.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "soft_uart_internal.h"
#include "soft_uart_rx.h"
#include "soft_uart_tx.h"

static const char *TAG = "su_core";

/* RMT group 时钟：RMT_CLK_SRC_DEFAULT 在 S3 上为 APB 80 MHz */
#define SU_FILTER_CLK_HZ 80000000u

/* 引脚占用登记，防止两个句柄悄悄抢同一个脚 */
#define SU_PIN_SLOTS 64
static bool s_pin_used[SU_PIN_SLOTS];
static portMUX_TYPE s_pin_mux = portMUX_INITIALIZER_UNLOCKED;

static bool su_pins_claim(int tx_gpio, int rx_gpio)
{
    bool ok = true;
    taskENTER_CRITICAL(&s_pin_mux);
    if (s_pin_used[tx_gpio] || s_pin_used[rx_gpio]) {
        ok = false;
    } else {
        s_pin_used[tx_gpio] = true;
        s_pin_used[rx_gpio] = true;
    }
    taskEXIT_CRITICAL(&s_pin_mux);
    return ok;
}

static void su_pins_release(int tx_gpio, int rx_gpio)
{
    taskENTER_CRITICAL(&s_pin_mux);
    s_pin_used[tx_gpio] = false;
    s_pin_used[rx_gpio] = false;
    taskEXIT_CRITICAL(&s_pin_mux);
}

static void su_stop_workers(struct rmt_soft_uart_s *u)
{
    /* 先停 RX，避免回调继续往任务里塞数据；TX 让任务自然收尾 */
    if (u->rx_ch && !u->rx_stopped) {
        rmt_disable(u->rx_ch);
        u->rx_stopped = true;
    }

    u->closing = true;

    /* 丢掉上次异常退出可能残留的令牌 */
    if (u->rx_exited) {
        xSemaphoreTake(u->rx_exited, 0);
    }
    if (u->tx_exited) {
        xSemaphoreTake(u->tx_exited, 0);
    }

    if (u->rx_task) {
        xTaskNotifyGive(u->rx_task);
        xSemaphoreTake(u->rx_exited, portMAX_DELAY);
        u->rx_task = NULL;
    }

    if (u->tx_task) {
        uint8_t dummy = 0;
        xStreamBufferSend(u->tx_sb, &dummy, 1, portMAX_DELAY);
        xSemaphoreTake(u->tx_exited, portMAX_DELAY);
        u->tx_task = NULL;
    }
}

static void su_teardown(struct rmt_soft_uart_s *u)
{
    su_rx_destroy(u);
    su_tx_destroy(u);
}

static esp_err_t su_setup(struct rmt_soft_uart_s *u)
{
    u->closing = false;
    u->rx_need_rearm = false;
    u->rx_stopped = false;

    ESP_RETURN_ON_ERROR(su_timing_init(u->cfg.baud, &u->resolution_hz, &u->timing),
                        TAG, "timing init");
    ESP_RETURN_ON_ERROR(su_rx_window_ns(u->cfg.baud, u->resolution_hz, SU_FILTER_CLK_HZ,
                                        &u->rx_min_ns, &u->rx_max_ns),
                        TAG, "rx window");

    esp_err_t err = su_tx_create(u);
    if (err != ESP_OK) {
        return err;
    }
    err = su_rx_create(u);
    if (err != ESP_OK) {
        return err;
    }

    u->valid = true;
    return ESP_OK;
}

esp_err_t rmt_soft_uart_new(const rmt_soft_uart_config_t *cfg,
                            rmt_soft_uart_handle_t *out_handle)
{
    if (!cfg || !out_handle) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!GPIO_IS_VALID_OUTPUT_GPIO(cfg->tx_gpio) || !GPIO_IS_VALID_GPIO(cfg->rx_gpio) ||
        cfg->tx_gpio >= SU_PIN_SLOTS || cfg->rx_gpio >= SU_PIN_SLOTS) {
        ESP_LOGE(TAG, "invalid gpio tx=%d rx=%d", cfg->tx_gpio, cfg->rx_gpio);
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->tx_gpio == cfg->rx_gpio) {
        /* IDF 的 gpio_config() 会把同脚的输出信号改回普通 GPIO，RMT TX 无法回环到 RX */
        ESP_LOGE(TAG, "tx and rx must use different gpio (got %d)", cfg->tx_gpio);
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->baud < RMT_SOFT_UART_BAUD_MIN || cfg->baud > RMT_SOFT_UART_BAUD_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    struct rmt_soft_uart_s *u = heap_caps_calloc(1, sizeof(*u),
                                                 MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!u) {
        return ESP_ERR_NO_MEM;
    }
    esp_err_t err = ESP_ERR_NO_MEM;
    u->cfg = *cfg;
    if (u->cfg.rx_buffer_bytes == 0) {
        u->cfg.rx_buffer_bytes = SU_DEFAULT_RX_BYTES;
    }
    if (u->cfg.tx_buffer_bytes == 0) {
        u->cfg.tx_buffer_bytes = SU_DEFAULT_TX_BYTES;
    }

    if (!su_pins_claim(u->cfg.tx_gpio, u->cfg.rx_gpio)) {
        ESP_LOGE(TAG, "gpio already in use: tx=%d rx=%d", u->cfg.tx_gpio, u->cfg.rx_gpio);
        heap_caps_free(u);
        return ESP_ERR_INVALID_STATE;
    }

    u->api_lock = xSemaphoreCreateMutex();
    u->tx_exited = xSemaphoreCreateBinary();
    u->rx_exited = xSemaphoreCreateBinary();
    if (!u->api_lock || !u->tx_exited || !u->rx_exited) {
        goto fail;
    }

    err = su_setup(u);
    if (err != ESP_OK) {
        su_stop_workers(u);
        su_teardown(u);
        goto fail;
    }

    *out_handle = u;
    return ESP_OK;

fail:
    su_pins_release(u->cfg.tx_gpio, u->cfg.rx_gpio);
    if (u->api_lock) {
        vSemaphoreDelete(u->api_lock);
    }
    if (u->tx_exited) {
        vSemaphoreDelete(u->tx_exited);
    }
    if (u->rx_exited) {
        vSemaphoreDelete(u->rx_exited);
    }
    heap_caps_free(u);
    return err;
}

esp_err_t rmt_soft_uart_del(rmt_soft_uart_handle_t u)
{
    if (!u) {
        return ESP_ERR_INVALID_ARG;
    }
    if (u->valid) {
        su_stop_workers(u);
        su_teardown(u);
        u->valid = false;
    }
    su_pins_release(u->cfg.tx_gpio, u->cfg.rx_gpio);
    vSemaphoreDelete(u->api_lock);
    vSemaphoreDelete(u->tx_exited);
    vSemaphoreDelete(u->rx_exited);
    heap_caps_free(u);
    return ESP_OK;
}

esp_err_t rmt_soft_uart_set_baud(rmt_soft_uart_handle_t u, uint32_t baud)
{
    if (!u || !u->valid) {
        return ESP_ERR_INVALID_STATE;
    }
    if (baud < RMT_SOFT_UART_BAUD_MIN || baud > RMT_SOFT_UART_BAUD_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(u->api_lock, portMAX_DELAY);
    su_stop_workers(u);
    su_teardown(u);

    uint32_t old_baud = u->cfg.baud;
    u->cfg.baud = baud;
    esp_err_t err = su_setup(u);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set_baud(%" PRIu32 ") failed: %s", baud, esp_err_to_name(err));
        su_teardown(u); /* 清掉第一轮已创建的资源，否则重试会覆盖句柄字段 */
        u->cfg.baud = old_baud;
        if (su_setup(u) != ESP_OK) {
            su_teardown(u);
            u->valid = false;
            ESP_LOGE(TAG, "restore baud %" PRIu32 " failed, handle now unusable", old_baud);
        }
    }
    xSemaphoreGive(u->api_lock);
    return err;
}

int rmt_soft_uart_write(rmt_soft_uart_handle_t u, const void *data, size_t len,
                        TickType_t timeout)
{
    if (!u || !u->valid || !data || len == 0 || !u->tx_sb) {
        return -1;
    }
    return (int)xStreamBufferSend(u->tx_sb, data, len, timeout);
}

int rmt_soft_uart_read(rmt_soft_uart_handle_t u, void *buf, size_t len,
                       TickType_t timeout)
{
    if (!u || !u->valid || !buf || len == 0 || !u->rx_sb) {
        return -1;
    }
    return (int)xStreamBufferReceive(u->rx_sb, buf, len, timeout);
}

esp_err_t rmt_soft_uart_flush_rx(rmt_soft_uart_handle_t u)
{
    if (!u || !u->valid || !u->rx_sb) {
        return ESP_ERR_INVALID_STATE;
    }
    xStreamBufferReset(u->rx_sb);
    return ESP_OK;
}

esp_err_t rmt_soft_uart_get_stats(rmt_soft_uart_handle_t u,
                                  rmt_soft_uart_stats_t *out_stats)
{
    if (!u || !out_stats) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_stats = u->stats;
    return ESP_OK;
}
