#include "rmt_soft_uart.h"

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

static void su_stop_workers(struct rmt_soft_uart_s *u)
{
    u->closing = true;

    if (u->rx_task) {
        xTaskNotifyGive(u->rx_task);
        if (xSemaphoreTake(u->rx_exited, pdMS_TO_TICKS(2000)) != pdTRUE) {
            ESP_LOGW(TAG, "rx task did not exit in time");
        }
        u->rx_task = NULL;
    }

    if (u->tx_task) {
        uint8_t dummy = 0;
        xStreamBufferSend(u->tx_sb, &dummy, 1, pdMS_TO_TICKS(50));
        if (xSemaphoreTake(u->tx_exited, pdMS_TO_TICKS(2000)) != pdTRUE) {
            ESP_LOGW(TAG, "tx task did not exit in time");
        }
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

    ESP_RETURN_ON_ERROR(su_timing_init(u->cfg.baud, &u->resolution_hz, &u->timing),
                        TAG, "timing init");

    uint32_t bit_ns = (uint32_t)(1000000000ULL / u->cfg.baud);
    uint32_t tick_ns = (uint32_t)(1000000000ULL / u->resolution_hz);
    if (tick_ns == 0) {
        tick_ns = 1;
    }
    u->rx_min_ns = bit_ns / 5u;
    if (u->rx_min_ns < tick_ns) {
        u->rx_min_ns = tick_ns;
    }
    u->rx_max_ns = bit_ns * 40u;
    if (u->rx_max_ns < 8000000u) {
        u->rx_max_ns = 8000000u;
    }

    esp_err_t err = su_tx_create(u);
    if (err != ESP_OK) {
        return err;
    }
    return su_rx_create(u);
}

esp_err_t rmt_soft_uart_new(const rmt_soft_uart_config_t *cfg,
                            rmt_soft_uart_handle_t *out_handle)
{
    if (!cfg || !out_handle) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!GPIO_IS_VALID_OUTPUT_GPIO(cfg->tx_gpio) || !GPIO_IS_VALID_GPIO(cfg->rx_gpio)) {
        ESP_LOGE(TAG, "invalid gpio tx=%d rx=%d", cfg->tx_gpio, cfg->rx_gpio);
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
    su_stop_workers(u);
    su_teardown(u);
    vSemaphoreDelete(u->api_lock);
    vSemaphoreDelete(u->tx_exited);
    vSemaphoreDelete(u->rx_exited);
    heap_caps_free(u);
    return ESP_OK;
}

esp_err_t rmt_soft_uart_set_baud(rmt_soft_uart_handle_t u, uint32_t baud)
{
    if (!u || baud < RMT_SOFT_UART_BAUD_MIN || baud > RMT_SOFT_UART_BAUD_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    xSemaphoreTake(u->api_lock, portMAX_DELAY);
    su_stop_workers(u);
    su_teardown(u);
    u->cfg.baud = baud;
    esp_err_t err = su_setup(u);
    xSemaphoreGive(u->api_lock);
    return err;
}

int rmt_soft_uart_write(rmt_soft_uart_handle_t u, const void *data, size_t len,
                        TickType_t timeout)
{
    if (!u || !data || len == 0 || !u->tx_sb) {
        return -1;
    }
    return (int)xStreamBufferSend(u->tx_sb, data, len, timeout);
}

int rmt_soft_uart_read(rmt_soft_uart_handle_t u, void *buf, size_t len,
                       TickType_t timeout)
{
    if (!u || !buf || len == 0 || !u->rx_sb) {
        return -1;
    }
    return (int)xStreamBufferReceive(u->rx_sb, buf, len, timeout);
}

esp_err_t rmt_soft_uart_flush_rx(rmt_soft_uart_handle_t u)
{
    (void)u;
    return ESP_ERR_NOT_SUPPORTED; /* 任务 7 实现 */
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
