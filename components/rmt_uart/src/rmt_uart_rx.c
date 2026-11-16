#include "rmt_uart_rx.h"

#include <string.h>

#include "esp_attr.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "rmt_uart_internal.h"

static const char *TAG = "ru_rx";

static void ru_flush_rx_batch(struct rmt_uart_s *u)
{
    if (u->rx_batch_n == 0) {
        return;
    }
    size_t n = u->rx_batch_n;
    u->rx_batch_n = 0;
    size_t sent = xStreamBufferSend(u->rx_sb, u->rx_batch, n, 0);
    u->stats.rx_bytes += (uint32_t)sent;
    if (sent < n) {
        __atomic_fetch_add(&u->stats.rx_overflow, 1, __ATOMIC_RELAXED);
        u->err_accum |= RMT_UART_ERROR_OVERRUN;
    }
}

static void ru_decoder_sink(void *ctx, uint8_t byte, ru_rx_flags_t flags)
{
    struct rmt_uart_s *u = ctx;
    if (flags.break_error) {
        u->stats.break_errors++;
        u->err_accum |= RMT_UART_ERROR_BREAK;
        return;
    }
    if (flags.framing_error) {
        u->stats.framing_errors++;
        u->err_accum |= RMT_UART_ERROR_FRAMING;
    }
    if (flags.parity_error) {
        u->stats.parity_errors++;
        u->err_accum |= RMT_UART_ERROR_PARITY;
    }
    if (u->rx_batch_n < sizeof(u->rx_batch)) {
        u->rx_batch[u->rx_batch_n++] = byte;
    } else {
        ru_flush_rx_batch(u);
        u->rx_batch[0] = byte;
        u->rx_batch_n = 1;
    }
}

static bool IRAM_ATTR ru_rx_done_cb(rmt_channel_handle_t channel,
                                    const rmt_rx_done_event_data_t *edata,
                                    void *user_ctx)
{
    (void)channel;
    struct rmt_uart_s *u = user_ctx;
    BaseType_t woken = pdFALSE;

    if (u->closing) {
        return false;
    }

    size_t pushed = ru_ring_push_from_isr(u->ring, edata->received_symbols,
                                                 edata->num_symbols);
    if (pushed < edata->num_symbols) {
        __atomic_fetch_add(&u->stats.rx_overflow, 1, __ATOMIC_RELAXED);
    }
    if (edata->flags.is_last) {
        /* 事务因空闲结束：紧跟着补一个空闲符号，保证解码顺序正确，
           任务可以在排空之前立刻重新布防，缩短未布防窗口 */
        rmt_symbol_word_t idle = {
            .level0 = 1,
            .duration0 = u->timing.frame_ticks,
            .level1 = 1,
            .duration1 = 0,
        };
        if (ru_ring_push_from_isr(u->ring, &idle, 1) != 1) {
            __atomic_fetch_add(&u->stats.rx_overflow, 1, __ATOMIC_RELAXED);
        }
        u->rx_need_rearm = true;
    }
    if (u->rx_task) {
        vTaskNotifyGiveFromISR(u->rx_task, &woken);
    }
    return woken == pdTRUE;
}

static void ru_rx_task(void *arg)
{
    struct rmt_uart_s *u = arg;
    rmt_symbol_word_t batch[32];

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (u->closing) {
            break;
        }

        /* 先重新布防，再解码；否则 3 路并发时排空 ring 的窗口会漏掉起始位 */
        if (u->rx_need_rearm) {
            u->rx_need_rearm = false;
            rmt_receive_config_t rcfg = {
                .signal_range_min_ns = u->rx_min_ns,
                .signal_range_max_ns = u->rx_max_ns,
                .flags = {
                    .en_partial_rx = true,
                },
            };
            esp_err_t err = rmt_receive(u->rx_ch, u->rx_buf,
                                        (u->cfg.use_dma ? RU_MEM_BLOCK_SYMS_DMA : RU_RX_USER_SYMBOLS) *
                                            sizeof(rmt_symbol_word_t),
                                        &rcfg);
            if (err == ESP_OK) {
                u->stats.rx_restarts++;
            } else {
                u->stats.rx_rearm_errors++;
                ESP_LOGW(TAG, "rmt_receive failed: %s", esp_err_to_name(err));
            }
        }

        size_t n;
        while (!u->closing && (n = ru_ring_pop(u->ring, batch, 32)) > 0) {
            ru_decoder_feed(u->dec, batch, n);
        }
        ru_flush_rx_batch(u);

        if (u->cbs.on_rx_data) {
            uint8_t chunk[64];
            size_t got;
            while ((got = xStreamBufferReceive(u->rx_sb, chunk, sizeof(chunk), 0)) > 0) {
                u->cbs.on_rx_data((rmt_uart_handle_t)u, chunk, got, u->cb_ctx);
            }
        } else if (u->ar.active) {
            ru_async_read_service(u);
        }

        uint32_t errs = u->err_accum;
        if (errs) {
            u->err_accum = 0;
            ru_notify_error(u, errs);
        }
    }

    xSemaphoreGive(u->rx_exited);
    vTaskDelete(NULL);
}

esp_err_t ru_rx_create(struct rmt_uart_s *u)
{
    rmt_rx_channel_config_t cfg = {
        .gpio_num = (gpio_num_t)u->cfg.rx_gpio,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = u->resolution_hz,
        .mem_block_symbols = u->cfg.use_dma ? RU_MEM_BLOCK_SYMS_DMA : RU_MEM_BLOCK_SYMS,
        .intr_priority = 3,
        .flags = {
            .invert_in = u->cfg.invert,
            .with_dma = u->cfg.use_dma,
            .io_loop_back = 0,
        },
    };
    ESP_RETURN_ON_ERROR(rmt_new_rx_channel(&cfg, &u->rx_ch), TAG, "new rx channel");

    size_t rx_symbols = u->cfg.use_dma ? RU_MEM_BLOCK_SYMS_DMA : RU_RX_USER_SYMBOLS;
    u->rx_buf = heap_caps_malloc(rx_symbols * sizeof(rmt_symbol_word_t),
                                 MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    u->ring = ru_ring_create(RU_RING_SYMBOLS);
    u->dec = ru_decoder_create(&u->timing, ru_decoder_sink, u);
    u->rx_sb = xStreamBufferCreate(u->cfg.rx_buffer_bytes, 1);
    if (!u->rx_buf || !u->ring || !u->dec || !u->rx_sb) {
        return ESP_ERR_NO_MEM;
    }

    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = ru_rx_done_cb,
    };
    ESP_RETURN_ON_ERROR(rmt_rx_register_event_callbacks(u->rx_ch, &cbs, u),
                        TAG, "register rx callback");
    ESP_RETURN_ON_ERROR(rmt_enable(u->rx_ch), TAG, "enable rx channel");

    if (xTaskCreate(ru_rx_task, "ru_rx", RU_RX_STACK, u, RU_RX_PRIO, &u->rx_task) != pdPASS) {
        u->rx_task = NULL;
        return ESP_ERR_NO_MEM;
    }

    rmt_receive_config_t rcfg = {
        .signal_range_min_ns = u->rx_min_ns,
        .signal_range_max_ns = u->rx_max_ns,
        .flags = {
            .en_partial_rx = true,
        },
    };
    ESP_RETURN_ON_ERROR(rmt_receive(u->rx_ch, u->rx_buf,
                                    rx_symbols * sizeof(rmt_symbol_word_t),
                                    &rcfg),
                        TAG, "arm rx");
    return ESP_OK;
}

void ru_rx_destroy(struct rmt_uart_s *u)
{
    if (u->rx_ch && !u->rx_stopped) {
        rmt_disable(u->rx_ch);
        u->rx_stopped = false;
        /* 关闭中断后等待可能仍在另一核执行的回调退出（回调只有 µs 级） */
        TickType_t dwell = pdMS_TO_TICKS(2);
        vTaskDelay(dwell ? dwell : 1);
        rmt_del_channel(u->rx_ch);
        u->rx_ch = NULL;
    } else if (u->rx_ch) {
        rmt_del_channel(u->rx_ch);
        u->rx_ch = NULL;
        u->rx_stopped = false;
    }
    if (u->rx_sb) {
        vStreamBufferDelete(u->rx_sb);
        u->rx_sb = NULL;
    }
    if (u->ring) {
        ru_ring_destroy(u->ring);
        u->ring = NULL;
    }
    if (u->dec) {
        ru_decoder_destroy(u->dec);
        u->dec = NULL;
    }
    heap_caps_free(u->rx_buf);
    u->rx_buf = NULL;
}
