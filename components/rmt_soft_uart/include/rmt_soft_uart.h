#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct rmt_soft_uart_s *rmt_soft_uart_handle_t;

#define RMT_SOFT_UART_MAX_CHANNELS 3
#define RMT_SOFT_UART_BAUD_MIN     300
#define RMT_SOFT_UART_BAUD_MAX     1000000

typedef struct {
    int      tx_gpio;
    int      rx_gpio;
    uint32_t baud;
    bool     invert;
    bool     internal_loopback;
    size_t   rx_buffer_bytes;
    size_t   tx_buffer_bytes;
} rmt_soft_uart_config_t;

typedef struct {
    uint32_t tx_bytes;
    uint32_t rx_bytes;
    uint32_t framing_errors;
    uint32_t break_errors;
    uint32_t rx_overflow;
    uint32_t rx_restarts;
    uint32_t tx_errors;
    uint32_t rx_rearm_errors;
} rmt_soft_uart_stats_t;

esp_err_t rmt_soft_uart_new(const rmt_soft_uart_config_t *cfg,
                            rmt_soft_uart_handle_t *out_handle);
esp_err_t rmt_soft_uart_del(rmt_soft_uart_handle_t handle);

/**
 * 运行中修改波特率。
 *
 * 注意：分辨率写在 RMT 通道创建参数里，因此本函数会销毁并重建 TX/RX 通道，
 * 未发送和未读取的数据会被丢弃。
 * 不要与其他线程的 read/write 并发调用；与 del() 也必须串行。
 */
esp_err_t rmt_soft_uart_set_baud(rmt_soft_uart_handle_t handle, uint32_t baud);

/** 写入 TX 缓冲，返回实际写入字节数；-1 表示参数或句柄状态错误。 */
int rmt_soft_uart_write(rmt_soft_uart_handle_t handle,
                        const void *data, size_t len, TickType_t timeout);

/** 从 RX 缓冲读取，返回实际字节数；0 表示超时；-1 表示参数或句柄状态错误。 */
int rmt_soft_uart_read(rmt_soft_uart_handle_t handle,
                       void *buf, size_t len, TickType_t timeout);

/** 丢弃 RX 缓冲中尚未读取的字节。 */
esp_err_t rmt_soft_uart_flush_rx(rmt_soft_uart_handle_t handle);

/** 复制一份统计快照（计数为诊断用途，不保证严格原子）。 */
esp_err_t rmt_soft_uart_get_stats(rmt_soft_uart_handle_t handle,
                                  rmt_soft_uart_stats_t *out_stats);

#ifdef __cplusplus
}
#endif
