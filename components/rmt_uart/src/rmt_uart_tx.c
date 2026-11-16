#include "rmt_uart_tx.h"

#include <string.h>

#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "rmt_uart_internal.h"

static const char *TAG = "ru_tx";

static void ru_tx_task(void *arg)
{
    struct rmt_uart_s *u = arg;
    const rmt_transmit_config_t tcfg = {
        .loop_count = 0,
        .flags = {
            .eot_level = 1,
            .queue_nonblocking = 0,
        },
    };

    for (;;) {
        size_t got = xStreamBufferReceive(u->tx_sb, u->tx_bytes,
                                          RU_TX_CHUNK_BYTES, portMAX_DELAY);
        if (u->closing) {
            break;
        }
        if (got == 0) {
            continue;
        }

        size_t words = ru_encode(u->tx_bytes, got, &u->timing, u->tx_syms,
                                (size_t)RU_TX_CHUNK_BYTES * RU_WORDS_PER_BYTE);
        if (words == 0) {
            continue;
        }

        u->tx_busy = true;
        esp_err_t err = rmt_transmit(u->tx_ch, u->encoder, u->tx_syms,
                                     words * sizeof(rmt_symbol_word_t), &tcfg);
        if (err != ESP_OK) {
            u->tx_busy = false;
            u->stats.tx_errors++;
            ESP_LOGW(TAG, "rmt_transmit failed: %s", esp_err_to_name(err));
            continue;
        }

        /* 一个 chunk 的空中时间 = 64 字节 × 10 bit，按实际波特率放大 2 倍再加余量 */
        uint32_t chunk_ms = (uint32_t)((uint64_t)RU_TX_CHUNK_BYTES * 10ull * 1000ull /
                                       u->cfg.baud);
        int wait_ms = (int)(chunk_ms * 2u + 200u);
        if (wait_ms < 500) {
            wait_ms = 500;
        }

        err = rmt_tx_wait_all_done(u->tx_ch, wait_ms);
        u->tx_busy = false;
        if (err != ESP_OK) {
            u->stats.tx_errors++;
            ESP_LOGW(TAG, "tx wait failed: %s", esp_err_to_name(err));
            rmt_disable(u->tx_ch);
            rmt_enable(u->tx_ch);
            continue;
        }
        u->stats.tx_bytes += (uint32_t)got;
        ru_async_write_account(u, got);
    }

    xSemaphoreGive(u->tx_exited);
    vTaskDelete(NULL);
}

esp_err_t ru_tx_create(struct rmt_uart_s *u)
{
    rmt_tx_channel_config_t cfg = {
        .gpio_num = (gpio_num_t)u->cfg.tx_gpio,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = u->resolution_hz,
        .mem_block_symbols = u->cfg.use_dma ? RU_MEM_BLOCK_SYMS_DMA : RU_MEM_BLOCK_SYMS,
        .trans_queue_depth = 4,
        .intr_priority = 3,
        .flags = {
            .invert_out = u->cfg.invert,
            .with_dma = u->cfg.use_dma,
            .io_loop_back = 0,
            .io_od_mode = 0,
        },
    };
    ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&cfg, &u->tx_ch), TAG, "new tx channel");

    rmt_copy_encoder_config_t enc_cfg; /* 空结构体，驱动仅校验非空指针 */
    ESP_RETURN_ON_ERROR(rmt_new_copy_encoder(&enc_cfg, &u->encoder), TAG, "new encoder");

    u->tx_sb = xStreamBufferCreate(u->cfg.tx_buffer_bytes, 1);
    u->tx_bytes = heap_caps_malloc(RU_TX_CHUNK_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    u->tx_syms = heap_caps_malloc((size_t)RU_TX_CHUNK_BYTES * RU_WORDS_PER_BYTE * sizeof(rmt_symbol_word_t),
                                  MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!u->tx_sb || !u->tx_bytes || !u->tx_syms) {
        return ESP_ERR_NO_MEM;
    }

    ESP_RETURN_ON_ERROR(rmt_enable(u->tx_ch), TAG, "enable tx channel");

    /* RMT 通道使能但未发送时输出低电平，先把线拉到空闲高，否则 RX 会先看到一个长低电平 */
    rmt_symbol_word_t idle_symbol = {
        .level0 = 1,
        .duration0 = u->timing.bit_ticks * 10u,
        .level1 = 1,
        .duration1 = 0,
    };
    rmt_transmit_config_t idle_cfg = {
        .loop_count = 0,
        .flags = {
            .eot_level = 1,
        },
    };
    ESP_RETURN_ON_ERROR(rmt_transmit(u->tx_ch, u->encoder, &idle_symbol,
                                     sizeof(idle_symbol), &idle_cfg),
                        TAG, "idle prime");
    rmt_tx_wait_all_done(u->tx_ch, 100);

    if (xTaskCreate(ru_tx_task, "ru_tx", RU_TX_STACK, u, RU_TX_PRIO, &u->tx_task) != pdPASS) {
        u->tx_task = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void ru_tx_destroy(struct rmt_uart_s *u)
{
    if (u->tx_ch) {
        rmt_disable(u->tx_ch);
        rmt_del_channel(u->tx_ch);
        u->tx_ch = NULL;
    }
    if (u->encoder) {
        rmt_del_encoder(u->encoder);
        u->encoder = NULL;
    }
    if (u->tx_sb) {
        vStreamBufferDelete(u->tx_sb);
        u->tx_sb = NULL;
    }
    heap_caps_free(u->tx_bytes);
    u->tx_bytes = NULL;
    heap_caps_free(u->tx_syms);
    u->tx_syms = NULL;
}
