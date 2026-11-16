#include "rmt_soft_uart.h"

#include "esp_log.h"

static const char *TAG = "su_core";

esp_err_t rmt_soft_uart_new(const rmt_soft_uart_config_t *cfg,
                            rmt_soft_uart_handle_t *out_handle)
{
    (void)cfg;
    (void)out_handle;
    ESP_LOGE(TAG, "not implemented yet");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t rmt_soft_uart_del(rmt_soft_uart_handle_t handle)
{
    (void)handle;
    return ESP_OK;
}

esp_err_t rmt_soft_uart_set_baud(rmt_soft_uart_handle_t handle, uint32_t baud)
{
    (void)handle;
    (void)baud;
    return ESP_ERR_NOT_SUPPORTED;
}

int rmt_soft_uart_write(rmt_soft_uart_handle_t handle, const void *data,
                        size_t len, TickType_t timeout)
{
    (void)handle;
    (void)data;
    (void)len;
    (void)timeout;
    return -1;
}

int rmt_soft_uart_read(rmt_soft_uart_handle_t handle, void *buf, size_t len,
                       TickType_t timeout)
{
    (void)handle;
    (void)buf;
    (void)len;
    (void)timeout;
    return -1;
}

esp_err_t rmt_soft_uart_flush_rx(rmt_soft_uart_handle_t handle)
{
    (void)handle;
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t rmt_soft_uart_get_stats(rmt_soft_uart_handle_t handle,
                                  rmt_soft_uart_stats_t *out_stats)
{
    (void)handle;
    (void)out_stats;
    return ESP_ERR_NOT_SUPPORTED;
}
