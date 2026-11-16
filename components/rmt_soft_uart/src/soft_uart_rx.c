#include "soft_uart_rx.h"

#include <string.h>

#include "esp_attr.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "soft_uart_internal.h"

static const char *TAG = "su_rx";

static void su_decoder_sink(void *ctx, uint8_t byte, su_rx_flags_t flags)
{
    struct rmt_soft_uart_s *u = ctx;
    if (flags.break_error) {
        u->stats.break_errors++;
        return;
    }
    if (flags.framing_error) {
        u->stats.framing_errors++;
    }
    if (xStreamBufferSend(u->rx_sb, &byte, 1, 0) == 1) {
        u->stats.rx_bytes++;
    } else {
        u->stats.rx_overflow++;
    }
}

static bool IRAM_ATTR su_rx_done_cb(rmt_channel_handle_t channel,
                                    const rmt_rx_done_event_data_t *edata,
                                    void *user_ctx)
{
    (void)channel;
    struct rmt_soft_uart_s *u = user_ctx;
    BaseType_t woken = pdFALSE;

    if (u->closing) {
        return false;
    }

    size_t pushed = su_symbol_ring_push_from_isr(u->ring, edata->received_symbols,
                                                 edata->num_symbols);
    if (pushed < edata->num_symbols) {
        u->stats.rx_overflow++;
    }
    if (edata->flags.is_last) {
        u->rx_need_rearm = true;
    }
    if (u->rx_task) {
        vTaskNotifyGiveFromISR(u->rx_task, &woken);
    }
    return woken == pdTRUE;
}

static void su_rx_task(void *arg)
{
    struct rmt_soft_uart_s *u = arg;
    rmt_symbol_word_t batch[32];

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (u->closing) {
            break;
        }

        size_t n;
        while (!u->closing && (n = su_symbol_ring_pop(u->ring, batch, 32)) > 0) {
            su_decoder_feed(u->dec, batch, n);
        }
        if (u->closing) {
            break;
        }

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
                                        (size_t)SU_RX_USER_SYMBOLS * sizeof(rmt_symbol_word_t),
                                        &rcfg);
            if (err == ESP_OK) {
                u->stats.rx_restarts++;
            } else {
                u->stats.rx_rearm_errors++;
                ESP_LOGW(TAG, "rmt_receive failed: %s", esp_err_to_name(err));
            }
        }
    }

    xSemaphoreGive(u->rx_exited);
    vTaskDelete(NULL);
}

esp_err_t su_rx_create(struct rmt_soft_uart_s *u)
{
    rmt_rx_channel_config_t cfg = {
        .gpio_num = (gpio_num_t)u->cfg.rx_gpio,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = u->resolution_hz,
        .mem_block_symbols = SU_MEM_BLOCK_SYMS,
        .intr_priority = 0,
        .flags = {
            .invert_in = u->cfg.invert,
            .with_dma = 0,
            .io_loop_back = u->cfg.internal_loopback,
        },
    };
    ESP_RETURN_ON_ERROR(rmt_new_rx_channel(&cfg, &u->rx_ch), TAG, "new rx channel");

    u->rx_buf = heap_caps_malloc((size_t)SU_RX_USER_SYMBOLS * sizeof(rmt_symbol_word_t),
                                 MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    u->ring = su_symbol_ring_create(SU_RING_SYMBOLS);
    u->dec = su_decoder_create(&u->timing, su_decoder_sink, u);
    u->rx_sb = xStreamBufferCreate(u->cfg.rx_buffer_bytes, 1);
    if (!u->rx_buf || !u->ring || !u->dec || !u->rx_sb) {
        return ESP_ERR_NO_MEM;
    }

    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = su_rx_done_cb,
    };
    ESP_RETURN_ON_ERROR(rmt_rx_register_event_callbacks(u->rx_ch, &cbs, u),
                        TAG, "register rx callback");
    ESP_RETURN_ON_ERROR(rmt_enable(u->rx_ch), TAG, "enable rx channel");

    if (xTaskCreate(su_rx_task, "su_rx", SU_RX_STACK, u, SU_RX_PRIO, &u->rx_task) != pdPASS) {
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
                                    (size_t)SU_RX_USER_SYMBOLS * sizeof(rmt_symbol_word_t),
                                    &rcfg),
                        TAG, "arm rx");
    return ESP_OK;
}

void su_rx_destroy(struct rmt_soft_uart_s *u)
{
    if (u->rx_ch) {
        rmt_disable(u->rx_ch);
        /* 关闭中断后等待可能仍在另一核执行的回调退出（回调只有 µs 级） */
        vTaskDelay(2 / portTICK_PERIOD_MS);
        rmt_del_channel(u->rx_ch);
        u->rx_ch = NULL;
    }
    if (u->rx_sb) {
        vStreamBufferDelete(u->rx_sb);
        u->rx_sb = NULL;
    }
    if (u->ring) {
        su_symbol_ring_destroy(u->ring);
        u->ring = NULL;
    }
    if (u->dec) {
        su_decoder_destroy(u->dec);
        u->dec = NULL;
    }
    heap_caps_free(u->rx_buf);
    u->rx_buf = NULL;
}
