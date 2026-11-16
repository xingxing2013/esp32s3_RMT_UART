#include "rmt_uart.h"

#include <inttypes.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "rmt_uart_internal.h"
#include "rmt_uart_rx.h"
#include "rmt_uart_tx.h"

static const char *TAG = "ru_core";

/* RMT group 时钟：RMT_CLK_SRC_DEFAULT 在 S3 上为 APB 80 MHz */
#define RU_FILTER_CLK_HZ 80000000u

/* 引脚占用登记，防止两个句柄悄悄抢同一个脚 */
#define RU_PIN_SLOTS 64
static bool s_pin_used[RU_PIN_SLOTS];
static portMUX_TYPE s_pin_mux = portMUX_INITIALIZER_UNLOCKED;

static struct {
    bool inited;
    uint8_t max_ports;
    uint8_t used_mask;
    SemaphoreHandle_t lock;
} s_global;

static int ru_port_claim(void)
{
    int slot = -1;
    xSemaphoreTake(s_global.lock, portMAX_DELAY);
    for (int i = 0; i < s_global.max_ports; i++) {
        if (!(s_global.used_mask & (1u << i))) {
            s_global.used_mask |= (uint8_t)(1u << i);
            slot = i;
            break;
        }
    }
    xSemaphoreGive(s_global.lock);
    return slot;
}

static void ru_port_release(int slot)
{
    if (slot < 0 || !s_global.inited) {
        return;
    }
    xSemaphoreTake(s_global.lock, portMAX_DELAY);
    s_global.used_mask &= (uint8_t)~(1u << slot);
    xSemaphoreGive(s_global.lock);
}

esp_err_t rmt_uart_global_init(const rmt_uart_global_config_t *cfg)
{
    uint8_t max_ports = (cfg && cfg->max_ports) ? cfg->max_ports : RMT_UART_MAX_PORTS;
    if (max_ports > RMT_UART_MAX_PORTS) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_global.inited) {
        return ESP_ERR_INVALID_STATE;
    }
    s_global.lock = xSemaphoreCreateMutex();
    if (!s_global.lock) {
        return ESP_ERR_NO_MEM;
    }
    s_global.max_ports = max_ports;
    s_global.used_mask = 0;
    s_global.inited = true;
    return ESP_OK;
}

esp_err_t rmt_uart_global_deinit(void)
{
    if (!s_global.inited) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_global.used_mask) {
        return ESP_ERR_INVALID_STATE;
    }
    vSemaphoreDelete(s_global.lock);
    s_global.lock = NULL;
    s_global.inited = false;
    return ESP_OK;
}

uint8_t rmt_uart_global_free_ports(void)
{
    if (!s_global.inited) {
        return 0;
    }
    uint8_t used = 0;
    for (int i = 0; i < s_global.max_ports; i++) {
        if (s_global.used_mask & (1u << i)) {
            used++;
        }
    }
    return (uint8_t)(s_global.max_ports - used);
}

static bool ru_pins_claim(int tx_gpio, int rx_gpio)
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

static void ru_pins_release(int tx_gpio, int rx_gpio)
{
    taskENTER_CRITICAL(&s_pin_mux);
    s_pin_used[tx_gpio] = false;
    s_pin_used[rx_gpio] = false;
    taskEXIT_CRITICAL(&s_pin_mux);
}

static void ru_stop_workers(struct rmt_uart_s *u)
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

static void ru_teardown(struct rmt_uart_s *u)
{
    ru_rx_destroy(u);
    ru_tx_destroy(u);
}

static esp_err_t ru_setup(struct rmt_uart_s *u)
{
    u->closing = false;
    u->rx_need_rearm = false;
    u->rx_stopped = false;

    ESP_RETURN_ON_ERROR(ru_timing_init(u->cfg.baud, u->cfg.data_bits, u->cfg.parity, u->cfg.stop_bits, &u->resolution_hz, &u->timing),
                        TAG, "timing init");
    ESP_RETURN_ON_ERROR(ru_rx_window_ns(u->cfg.baud, u->resolution_hz, RU_FILTER_CLK_HZ,
                                        &u->rx_min_ns, &u->rx_max_ns),
                        TAG, "rx window");

    esp_err_t err = ru_tx_create(u);
    if (err != ESP_OK) {
        return err;
    }
    err = ru_rx_create(u);
    if (err != ESP_OK) {
        return err;
    }

    u->valid = true;
    return ESP_OK;
}

static void ru_rx_timeout_cb(void *arg)
{
    struct rmt_uart_s *u = arg;
    if (u->rx_task) {
        xTaskNotifyGive(u->rx_task);
    }
}

static portMUX_TYPE s_async_mux = portMUX_INITIALIZER_UNLOCKED;

static void ru_aw_finish(struct rmt_uart_s *u, rmt_uart_async_result_t result)
{
    rmt_uart_done_cb_t cb;
    void *ctx;
    size_t n;

    taskENTER_CRITICAL(&s_async_mux);
    if (!u->aw.active) {
        taskEXIT_CRITICAL(&s_async_mux);
        return;
    }
    cb = u->aw.cb;
    ctx = u->aw.ctx;
    n = u->aw.sent;
    u->aw.active = false;
    u->aw.cb = NULL;
    u->aw.ctx = NULL;
    u->aw.len = 0;
    u->aw.sent = 0;
    taskEXIT_CRITICAL(&s_async_mux);

    if (cb) {
        cb((rmt_uart_handle_t)u, result, n, ctx);
    }
}

void ru_async_write_account(struct rmt_uart_s *u, size_t n)
{
    if (!u->aw.active) {
        return;
    }
    u->aw.sent += n;
    if (u->aw.sent >= u->aw.len) {
        ru_aw_finish(u, RMT_UART_ASYNC_OK);
    }
}

static void ru_ar_finish(struct rmt_uart_s *u, rmt_uart_async_result_t result)
{
    rmt_uart_done_cb_t cb;
    void *ctx;
    size_t n;
    esp_timer_handle_t timer;

    taskENTER_CRITICAL(&s_async_mux);
    if (!u->ar.active) {
        taskEXIT_CRITICAL(&s_async_mux);
        return;
    }
    cb = u->ar.cb;
    ctx = u->ar.ctx;
    n = u->ar.got;
    timer = u->ar.timer;
    u->ar.active = false;
    u->ar.cb = NULL;
    u->ar.ctx = NULL;
    u->ar.buf = NULL;
    u->ar.len = 0;
    u->ar.got = 0;
    u->ar.deadline_us = 0;
    taskEXIT_CRITICAL(&s_async_mux);

    if (timer) {
        esp_timer_stop(timer);
    }
    if (cb) {
        cb((rmt_uart_handle_t)u, result, n, ctx);
    }
}

void ru_async_read_service(struct rmt_uart_s *u)
{
    if (!u->ar.active || !u->ar.buf) {
        return;
    }

    uint8_t tmp[64];
    for (;;) {
        size_t room = u->ar.len - u->ar.got;
        if (room == 0) {
            break;
        }
        size_t want = room < sizeof(tmp) ? room : sizeof(tmp);
        size_t n = xStreamBufferReceive(u->rx_sb, tmp, want, 0);
        if (n == 0) {
            break;
        }
        memcpy(u->ar.buf + u->ar.got, tmp, n);
        u->ar.got += n;
    }

    if (u->ar.got >= u->ar.len) {
        ru_ar_finish(u, RMT_UART_ASYNC_OK);
        return;
    }
    if (u->ar.deadline_us > 0 && esp_timer_get_time() >= u->ar.deadline_us) {
        ru_ar_finish(u, RMT_UART_ASYNC_TIMEOUT);
    }
}

void ru_async_cancel_all(struct rmt_uart_s *u, rmt_uart_async_result_t result)
{
    if (u->aw.active) {
        ru_aw_finish(u, result);
    }
    if (u->ar.active) {
        ru_ar_finish(u, result);
    }
}

void ru_notify_error(struct rmt_uart_s *u, uint32_t errors)
{
    if (!errors || !u->cbs.on_error) {
        return;
    }
    u->cbs.on_error((rmt_uart_handle_t)u, errors, u->cb_ctx);
}

esp_err_t rmt_uart_init(const rmt_uart_config_t *cfg,
                            rmt_uart_handle_t *out_handle)
{
    if (!cfg || !out_handle) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_global.inited) {
        ESP_LOGE(TAG, "call rmt_uart_global_init() first");
        return ESP_ERR_INVALID_STATE;
    }
    if (!GPIO_IS_VALID_OUTPUT_GPIO(cfg->tx_gpio) || !GPIO_IS_VALID_GPIO(cfg->rx_gpio) ||
        cfg->tx_gpio >= RU_PIN_SLOTS || cfg->rx_gpio >= RU_PIN_SLOTS) {
        ESP_LOGE(TAG, "invalid gpio tx=%d rx=%d", cfg->tx_gpio, cfg->rx_gpio);
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->tx_gpio == cfg->rx_gpio) {
        /* IDF 的 gpio_config() 会把同脚的输出信号改回普通 GPIO，RMT TX 无法回环到 RX */
        ESP_LOGE(TAG, "tx and rx must use different gpio (got %d)", cfg->tx_gpio);
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->baud < RMT_UART_BAUD_MIN || cfg->baud > RMT_UART_BAUD_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    struct rmt_uart_s *u = heap_caps_calloc(1, sizeof(*u),
                                                 MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!u) {
        return ESP_ERR_NO_MEM;
    }
    esp_err_t err = ESP_ERR_NO_MEM;
    u->cfg = *cfg;
    if (u->cfg.rx_buffer_bytes == 0) {
        u->cfg.rx_buffer_bytes = RU_DEFAULT_RX_BYTES;
    }
    if (u->cfg.data_bits == 0) {
        u->cfg.data_bits = 8;
    }
    if (u->cfg.tx_buffer_bytes == 0) {
        u->cfg.tx_buffer_bytes = RU_DEFAULT_TX_BYTES;
    }

    if (!ru_pins_claim(u->cfg.tx_gpio, u->cfg.rx_gpio)) {
        ESP_LOGE(TAG, "gpio already in use: tx=%d rx=%d", u->cfg.tx_gpio, u->cfg.rx_gpio);
        heap_caps_free(u);
        return ESP_ERR_INVALID_STATE;
    }

    int slot = ru_port_claim();
    if (slot < 0) {
        ESP_LOGE(TAG, "no free port (max_ports=%u)", (unsigned)s_global.max_ports);
        ru_pins_release(u->cfg.tx_gpio, u->cfg.rx_gpio);
        heap_caps_free(u);
        return ESP_ERR_NOT_FOUND;
    }
    u->port_index = (int8_t)slot;

    u->api_lock = xSemaphoreCreateMutex();
    u->tx_exited = xSemaphoreCreateBinary();
    u->rx_exited = xSemaphoreCreateBinary();
    if (!u->api_lock || !u->tx_exited || !u->rx_exited) {
        goto fail;
    }

    const esp_timer_create_args_t targs = {
        .callback = ru_rx_timeout_cb,
        .arg = u,
        .name = "ru_rx_tmo",
    };
    if (esp_timer_create(&targs, &u->rx_timeout_timer) != ESP_OK) {
        goto fail;
    }

    err = ru_setup(u);
    if (err != ESP_OK) {
        ru_stop_workers(u);
        ru_teardown(u);
        goto fail;
    }

    *out_handle = u;
    return ESP_OK;

fail:
    ru_pins_release(u->cfg.tx_gpio, u->cfg.rx_gpio);
    ru_port_release(u->port_index);
    if (u->api_lock) {
        vSemaphoreDelete(u->api_lock);
    }
    if (u->tx_exited) {
        vSemaphoreDelete(u->tx_exited);
    }
    if (u->rx_exited) {
        vSemaphoreDelete(u->rx_exited);
    }
    if (u->rx_timeout_timer) {
        esp_timer_delete(u->rx_timeout_timer);
    }
    heap_caps_free(u);
    return err;
}

esp_err_t rmt_uart_deinit(rmt_uart_handle_t u)
{
    if (!u) {
        return ESP_ERR_INVALID_ARG;
    }
    if (u->valid) {
        ru_stop_workers(u);
        ru_async_cancel_all(u, RMT_UART_ASYNC_CANCELED);
        ru_teardown(u);
        u->valid = false;
    }
    ru_pins_release(u->cfg.tx_gpio, u->cfg.rx_gpio);
    ru_port_release(u->port_index);
    if (u->rx_timeout_timer) {
        esp_timer_delete(u->rx_timeout_timer);
        u->rx_timeout_timer = NULL;
    }
    vSemaphoreDelete(u->api_lock);
    vSemaphoreDelete(u->tx_exited);
    vSemaphoreDelete(u->rx_exited);
    heap_caps_free(u);
    return ESP_OK;
}

esp_err_t rmt_uart_set_baud(rmt_uart_handle_t u, uint32_t baud)
{
    if (!u || !u->valid) {
        return ESP_ERR_INVALID_STATE;
    }
    if (baud < RMT_UART_BAUD_MIN || baud > RMT_UART_BAUD_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(u->api_lock, portMAX_DELAY);
    ru_stop_workers(u);
    ru_async_cancel_all(u, RMT_UART_ASYNC_CANCELED);
    ru_teardown(u);

    uint32_t old_baud = u->cfg.baud;
    u->cfg.baud = baud;
    esp_err_t err = ru_setup(u);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set_baud(%" PRIu32 ") failed: %s", baud, esp_err_to_name(err));
        ru_teardown(u); /* 清掉第一轮已创建的资源，否则重试会覆盖句柄字段 */
        u->cfg.baud = old_baud;
        if (ru_setup(u) != ESP_OK) {
            ru_teardown(u);
            u->valid = false;
            ESP_LOGE(TAG, "restore baud %" PRIu32 " failed, handle now unusable", old_baud);
        }
    }
    xSemaphoreGive(u->api_lock);
    return err;
}

int rmt_uart_write(rmt_uart_handle_t u, const void *data, size_t len,
                        TickType_t timeout)
{
    if (!u || !u->valid || !data || len == 0 || !u->tx_sb) {
        return -1;
    }
    return (int)xStreamBufferSend(u->tx_sb, data, len, timeout);
}

int rmt_uart_read(rmt_uart_handle_t u, void *buf, size_t len,
                       TickType_t timeout)
{
    if (!u || !u->valid || !buf || len == 0 || !u->rx_sb) {
        return -1;
    }
    return (int)xStreamBufferReceive(u->rx_sb, buf, len, timeout);
}

esp_err_t rmt_uart_write_async(rmt_uart_handle_t u, const void *data, size_t len,
                               rmt_uart_done_cb_t cb, void *ctx)
{
    if (!u || !u->valid || !u->tx_sb || !data || !cb || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (u->aw.active) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xStreamBufferSpacesAvailable(u->tx_sb) < len) {
        return ESP_ERR_INVALID_SIZE;
    }

    u->aw.cb = cb;
    u->aw.ctx = ctx;
    u->aw.len = len;
    u->aw.sent = 0;
    u->aw.active = true;

    size_t sent = xStreamBufferSend(u->tx_sb, data, len, 0);
    if (sent != len) {
        u->aw.active = false;
        u->aw.cb = NULL;
        u->aw.ctx = NULL;
        u->aw.len = 0;
        return ESP_ERR_INVALID_SIZE;
    }
    return ESP_OK;
}

esp_err_t rmt_uart_read_async(rmt_uart_handle_t u, void *buf, size_t len,
                              uint32_t timeout_ms, rmt_uart_done_cb_t cb, void *ctx)
{
    if (!u || !u->valid || !u->rx_sb || !buf || !cb || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (u->ar.active || !u->rx_timeout_timer) {
        return ESP_ERR_INVALID_STATE;
    }
    if (u->cbs.on_rx_data) {
        /* 流式回调已经消费 RX 数据，不能再挂异步读 */
        return ESP_ERR_INVALID_STATE;
    }

    u->ar.cb = cb;
    u->ar.ctx = ctx;
    u->ar.buf = buf;
    u->ar.len = len;
    u->ar.got = 0;
    u->ar.deadline_us = 0;
    u->ar.active = true;

    if (timeout_ms > 0) {
        u->ar.deadline_us = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
        esp_timer_start_once(u->rx_timeout_timer, (uint64_t)timeout_ms * 1000);
    }

    /* 唤醒 RX 任务去搬已经缓冲的数据，避免与任务并发写请求状态 */
    if (u->rx_task) {
        xTaskNotifyGive(u->rx_task);
    }
    return ESP_OK;
}

esp_err_t rmt_uart_register_callbacks(rmt_uart_handle_t u,
                                      const rmt_uart_callbacks_t *cbs, void *ctx)
{
    if (!u || !u->valid) {
        return ESP_ERR_INVALID_STATE;
    }
    if (cbs && cbs->on_rx_data && u->ar.active) {
        return ESP_ERR_INVALID_STATE;
    }
    if (cbs) {
        u->cbs = *cbs;
    } else {
        memset(&u->cbs, 0, sizeof(u->cbs));
    }
    u->cb_ctx = ctx;
    return ESP_OK;
}

esp_err_t rmt_uart_flush_tx(rmt_uart_handle_t u)
{
    if (!u || !u->valid || !u->tx_sb) {
        return ESP_ERR_INVALID_STATE;
    }
    xStreamBufferReset(u->tx_sb);
    if (u->aw.active) {
        ru_aw_finish(u, RMT_UART_ASYNC_CANCELED);
    }
    return ESP_OK;
}

bool rmt_uart_tx_idle(rmt_uart_handle_t u)
{
    if (!u || !u->valid || !u->tx_sb) {
        return false;
    }
    return xStreamBufferIsEmpty(u->tx_sb) && !u->tx_busy && !u->aw.active;
}

esp_err_t rmt_uart_flush_rx(rmt_uart_handle_t u)
{
    if (!u || !u->valid || !u->rx_sb) {
        return ESP_ERR_INVALID_STATE;
    }
    xStreamBufferReset(u->rx_sb);
    if (u->ar.active) {
        ru_ar_finish(u, RMT_UART_ASYNC_CANCELED);
    }
    return ESP_OK;
}

esp_err_t rmt_uart_get_stats(rmt_uart_handle_t u,
                                  rmt_uart_stats_t *out_stats)
{
    if (!u || !out_stats) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_stats = u->stats;
    return ESP_OK;
}
