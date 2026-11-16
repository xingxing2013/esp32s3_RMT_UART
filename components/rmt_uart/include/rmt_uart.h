#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RMT_UART_MAX_PORTS 3

#define RMT_UART_BAUD_MIN 300
#define RMT_UART_BAUD_MAX 1000000

typedef struct rmt_uart_s *rmt_uart_handle_t;

/* ===================== 帧格式 ===================== */

typedef enum {
    RMT_UART_PARITY_NONE = 0,
    RMT_UART_PARITY_EVEN,
    RMT_UART_PARITY_ODD,
} rmt_uart_parity_t;

typedef enum {
    RMT_UART_STOP_BITS_1 = 0,
    RMT_UART_STOP_BITS_1_5,
    RMT_UART_STOP_BITS_2,
} rmt_uart_stop_bits_t;

typedef struct {
    int      tx_gpio;
    int      rx_gpio;
    uint32_t baud;
    uint8_t  data_bits;               /* 5..8，0 = 默认 8 */
    rmt_uart_parity_t    parity;      /* 默认 NONE */
    rmt_uart_stop_bits_t stop_bits;   /* 默认 1 */
    bool     invert;
    bool     use_dma;                 /* 仅 RMT 最后一个通道支持 DMA，最多一路可用 */
    size_t   rx_buffer_bytes;         /* 0 = 默认 2048 */
    size_t   tx_buffer_bytes;         /* 0 = 默认 1024 */
} rmt_uart_config_t;

/* ===================== 两级初始化 ===================== */

typedef struct {
    uint8_t max_ports;                /* 1..3，0 = 默认 3 */
} rmt_uart_global_config_t;

/** 进程级初始化：建立容量账本、引脚表和全局锁。必须在 rmt_uart_init 之前调用。 */
esp_err_t rmt_uart_global_init(const rmt_uart_global_config_t *cfg);
esp_err_t rmt_uart_global_deinit(void);
uint8_t   rmt_uart_global_free_ports(void);

/**
 * 初始化一路软件串口。
 *
 * 注意：分辨率写在 RMT 通道创建参数里，因此 set_baud() 会销毁并重建通道，
 * 未发送和未读取的数据会被丢弃。不要与其他线程的读写并发调用 set_baud/del。
 */
esp_err_t rmt_uart_init(const rmt_uart_config_t *cfg, rmt_uart_handle_t *out_handle);
esp_err_t rmt_uart_deinit(rmt_uart_handle_t handle);
esp_err_t rmt_uart_set_baud(rmt_uart_handle_t handle, uint32_t baud);

/* ===================== 同步读写 ===================== */

/** 写入 TX 缓冲，返回实际写入字节数；-1 表示参数或句柄状态错误。 */
int rmt_uart_write(rmt_uart_handle_t handle, const void *data, size_t len,
                   TickType_t timeout);
/** 从 RX 缓冲读取，返回实际字节数；0 表示超时；-1 表示参数或句柄状态错误。 */
int rmt_uart_read(rmt_uart_handle_t handle, void *buf, size_t len,
                  TickType_t timeout);

esp_err_t rmt_uart_flush_rx(rmt_uart_handle_t handle);
esp_err_t rmt_uart_flush_tx(rmt_uart_handle_t handle);
bool      rmt_uart_tx_idle(rmt_uart_handle_t handle);

/* ===================== 异步读写 ===================== */

typedef enum {
    RMT_UART_ASYNC_OK = 0,
    RMT_UART_ASYNC_TIMEOUT,
    RMT_UART_ASYNC_CANCELED,
    RMT_UART_ASYNC_NO_MEM,
    RMT_UART_ASYNC_INVALID_STATE,
} rmt_uart_async_result_t;

/**
 * 异步完成回调，运行在驱动任务上下文（不是 ISR）。
 * @param transferred 实际完成字节数
 */
typedef void (*rmt_uart_done_cb_t)(rmt_uart_handle_t handle,
                                   rmt_uart_async_result_t result,
                                   size_t transferred, void *ctx);

/**
 * 异步写：数据在返回前拷入内部缓冲，len 不能超过 TX 缓冲剩余空间。
 * 同一实例同一时刻只允许一个未完成异步写。
 */
esp_err_t rmt_uart_write_async(rmt_uart_handle_t handle, const void *data, size_t len,
                               rmt_uart_done_cb_t cb, void *ctx);

/**
 * 异步读：读满 len 或超时后回调；同一实例同一时刻只允许一个未完成异步读。
 * 不要与同步 read 混用。
 */
esp_err_t rmt_uart_read_async(rmt_uart_handle_t handle, void *buf, size_t len,
                              uint32_t timeout_ms, rmt_uart_done_cb_t cb, void *ctx);

/* ===================== 流式事件 ===================== */

typedef enum {
    RMT_UART_ERROR_PARITY  = 1u << 0,
    RMT_UART_ERROR_FRAMING = 1u << 1,
    RMT_UART_ERROR_OVERRUN = 1u << 2,
    RMT_UART_ERROR_BREAK   = 1u << 3,
} rmt_uart_error_t;

typedef struct {
    /** 收到数据（任务上下文）。注册后数据由回调消费，同步 read 不再返回。 */
    void (*on_rx_data)(rmt_uart_handle_t handle, const uint8_t *data, size_t len,
                       void *ctx);
    /** 错误汇总（位或 rmt_uart_error_t ），任务上下文。 */
    void (*on_error)(rmt_uart_handle_t handle, uint32_t errors, void *ctx);
} rmt_uart_callbacks_t;

esp_err_t rmt_uart_register_callbacks(rmt_uart_handle_t handle,
                                      const rmt_uart_callbacks_t *cbs, void *ctx);

/* ===================== 统计 ===================== */

typedef struct {
    uint32_t tx_bytes;
    uint32_t rx_bytes;
    uint32_t parity_errors;
    uint32_t framing_errors;
    uint32_t break_errors;
    uint32_t rx_overflow;
    uint32_t rx_restarts;
    uint32_t tx_errors;
    uint32_t rx_rearm_errors;
} rmt_uart_stats_t;

esp_err_t rmt_uart_get_stats(rmt_uart_handle_t handle, rmt_uart_stats_t *out_stats);

#ifdef __cplusplus
}
#endif
