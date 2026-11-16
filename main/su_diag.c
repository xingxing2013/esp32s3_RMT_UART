#include "su_diag.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "driver/gptimer.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rmt_soft_uart.h"
#include "sdkconfig.h"

static const char *TAG = "su_diag";

#define DIAG_CHANNELS 3
#define DIAG_BLOCK    256

static const int s_tx_gpio[DIAG_CHANNELS] = {
    CONFIG_RMT_SOFT_UART_CH0_TX_GPIO,
    CONFIG_RMT_SOFT_UART_CH1_TX_GPIO,
    CONFIG_RMT_SOFT_UART_CH2_TX_GPIO,
};

static const int s_rx_gpio[DIAG_CHANNELS] = {
    CONFIG_RMT_SOFT_UART_CH0_RX_GPIO,
    CONFIG_RMT_SOFT_UART_CH1_RX_GPIO,
    CONFIG_RMT_SOFT_UART_CH2_RX_GPIO,
};

static volatile uint32_t s_irq_count;
static gptimer_handle_t s_timer;

static bool diag_alarm_cb(gptimer_handle_t timer,
                                   const gptimer_alarm_event_data_t *edata,
                                   void *user_ctx);

void su_diag_irq_start(uint32_t irq_khz)
{
    if (irq_khz == 0 || s_timer) {
        return;
    }
    gptimer_config_t tcfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&tcfg, &s_timer));
    gptimer_alarm_config_t acfg = {
        .alarm_count = 1000 / irq_khz, /* µs/period，分辨率 1 MHz */
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(s_timer, &acfg));
    gptimer_event_callbacks_t cbs = {
        .on_alarm = diag_alarm_cb,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(s_timer, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(s_timer));
    ESP_ERROR_CHECK(gptimer_start(s_timer));
    s_irq_count = 0;
}

void su_diag_irq_stop(void)
{
    if (!s_timer) {
        return;
    }
    gptimer_stop(s_timer);
    gptimer_disable(s_timer);
    gptimer_del_timer(s_timer);
    s_timer = NULL;
}

static bool IRAM_ATTR diag_alarm_cb(gptimer_handle_t timer,
                                   const gptimer_alarm_event_data_t *edata,
                                   void *user_ctx)
{
    (void)timer;
    (void)edata;
    (void)user_ctx;
    s_irq_count++;
    for (volatile int i = 0; i < 20; i++) {
        /* 模拟一段很短的中断内工作 */
    }
    return false;
}

typedef struct {
    rmt_soft_uart_handle_t handle;
    volatile bool stop;
    volatile uint32_t written;
} producer_t;

static void producer_task(void *arg)
{
    producer_t *p = arg;
    uint8_t block[DIAG_BLOCK];
    for (size_t i = 0; i < sizeof(block); i++) {
        block[i] = (uint8_t)i;
    }

    while (!p->stop) {
        size_t off = 0;
        while (off < sizeof(block) && !p->stop) {
            int n = rmt_soft_uart_write(p->handle, block + off, sizeof(block) - off,
                                        pdMS_TO_TICKS(50));
            if (n > 0) {
                p->written += (uint32_t)n;
                off += (size_t)n;
            }
        }
    }
    vTaskDelete(NULL);
}

typedef struct {
    volatile bool stop;
    uint32_t percent;
} hog_ctx_t;

static void hog_task(void *arg)
{
    hog_ctx_t *ctx = arg;
    const int64_t period_us = 10000;

    while (!ctx->stop) {
        int64_t start = esp_timer_get_time();
        int64_t busy_until = start + period_us * (int64_t)ctx->percent / 100;
        while (!ctx->stop && esp_timer_get_time() < busy_until) {
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelete(NULL);
}

static void read_idle(uint64_t *sum, uint32_t *count)
{
    static TaskStatus_t st[24];
    UBaseType_t n = uxTaskGetSystemState(st, 24, NULL);
    uint64_t s = 0;
    uint32_t c = 0;
    for (UBaseType_t i = 0; i < n; i++) {
        if (strncmp(st[i].pcTaskName, "IDLE", 4) == 0) {
            s += st[i].ulRunTimeCounter;
            c++;
        }
    }
    *sum = s;
    *count = c;
}

static void dump_task_stats(void)
{
    static char buf[1024];
    buf[0] = '\0';
    vTaskGetRunTimeStats(buf);
    ESP_LOGI(TAG, "task CPU (cumulative since boot):\n%s", buf);
}

void su_diag_run(uint32_t seconds, uint32_t baud, uint32_t irq_khz, uint32_t hog_percent)
{
    rmt_soft_uart_handle_t handles[DIAG_CHANNELS] = {0};
    producer_t prod[DIAG_CHANNELS] = {0};
    static uint8_t rx[DIAG_BLOCK];

    ESP_LOGI(TAG, "start: baud=%" PRIu32 ", %u s, irq=%" PRIu32 " kHz, hog=%" PRIu32 "%%",
             baud, (unsigned)seconds, irq_khz, hog_percent);

    for (int i = 0; i < DIAG_CHANNELS; i++) {
        rmt_soft_uart_config_t cfg = {
            .tx_gpio = s_tx_gpio[i],
            .rx_gpio = s_rx_gpio[i],
            .baud = baud,
            .rx_buffer_bytes = 4096,
            .tx_buffer_bytes = 2048,
        };
        esp_err_t err = rmt_soft_uart_new(&cfg, &handles[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ch%d new failed: %s", i, esp_err_to_name(err));
            return;
        }
    }

    gptimer_handle_t timer = NULL;
    if (irq_khz > 0) {
        su_diag_irq_start(irq_khz);
        timer = s_timer;
    }

    hog_ctx_t hog = {
        .stop = false,
        .percent = hog_percent,
    };
    if (hog_percent > 0) {
        xTaskCreate(hog_task, "hog", 2048, &hog, 5, NULL);
    }

    for (int i = 0; i < DIAG_CHANNELS; i++) {
        prod[i].handle = handles[i];
        xTaskCreate(producer_task, "producer", 3072, &prod[i], 9, NULL);
    }

    uint64_t idle0 = 0;
    uint32_t idle_cnt0 = 0;
    read_idle(&idle0, &idle_cnt0);

    int64_t t_start = esp_timer_get_time();
    uint32_t read_bytes = 0;
    int64_t t_end = t_start + (int64_t)seconds * 1000000;

    while (esp_timer_get_time() < t_end) {
        for (int i = 0; i < DIAG_CHANNELS; i++) {
            int n = rmt_soft_uart_read(handles[i], rx, sizeof(rx), pdMS_TO_TICKS(20));
            if (n > 0) {
                read_bytes += (uint32_t)n;
            }
        }
    }

    for (int i = 0; i < DIAG_CHANNELS; i++) {
        prod[i].stop = true;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    /* 把最后还没读走的数据收干净，避免把在途数据算成丢失 */
    for (int i = 0; i < DIAG_CHANNELS; i++) {
        int n;
        do {
            n = rmt_soft_uart_read(handles[i], rx, sizeof(rx), pdMS_TO_TICKS(50));
            if (n > 0) {
                read_bytes += (uint32_t)n;
            }
        } while (n > 0);
    }

    int64_t elapsed = esp_timer_get_time() - t_start;
    uint64_t idle1 = 0;
    uint32_t idle_cnt1 = 0;
    read_idle(&idle1, &idle_cnt1);

    if (timer) {
        su_diag_irq_stop();
    }
    if (hog_percent > 0) {
        hog.stop = true;
        vTaskDelay(pdMS_TO_TICKS(30));
    }

    uint32_t txb = 0, rxb = 0, fe = 0, ovf = 0, restarts = 0, txerr = 0, rearm_err = 0;
    for (int i = 0; i < DIAG_CHANNELS; i++) {
        txb += prod[i].written;
        rmt_soft_uart_stats_t st = {0};
        rmt_soft_uart_get_stats(handles[i], &st);
        rxb += st.rx_bytes;
        fe += st.framing_errors;
        ovf += st.rx_overflow;
        restarts += st.rx_restarts;
        txerr += st.tx_errors;
        rearm_err += st.rx_rearm_errors;
    }

    uint64_t idle_delta = idle1 - idle0;
    uint32_t cores = idle_cnt1 ? idle_cnt1 : 1;
    uint64_t capacity = (uint64_t)elapsed * cores;
    uint32_t busy_x100 = capacity ? (uint32_t)(10000u - idle_delta * 10000u / capacity) : 0;

    ESP_LOGI(TAG, "tx_bytes=%" PRIu32 ", rx_bytes=%" PRIu32 ", read=%" PRIu32
                  ", lost=%" PRIi32 ", fe=%" PRIu32 ", ovf=%" PRIu32
                  ", restarts=%" PRIu32 ", rearm_err=%" PRIu32 ", txerr=%" PRIu32,
             txb, rxb, read_bytes, (int32_t)(txb - rxb), fe, ovf, restarts, rearm_err, txerr);
    ESP_LOGI(TAG, "elapsed=%" PRIi64 " us, throughput=%.1f KB/s total, CPU busy=%" PRIu32
                  ".%02" PRIu32 "%% (idle tasks=%u, idle_delta=%" PRIu64 " us)",
             elapsed, (double)rxb / 1024.0 * 1000000.0 / (double)elapsed,
             busy_x100 / 100, busy_x100 % 100, (unsigned)cores, idle_delta);
    if (irq_khz > 0) {
        ESP_LOGI(TAG, "extra irq: %" PRIu32 " kHz, observed=%" PRIu32 " in %" PRIi64 " us",
                 irq_khz, s_irq_count, elapsed);
    }
    dump_task_stats();

    for (int i = 0; i < DIAG_CHANNELS; i++) {
        rmt_soft_uart_del(handles[i]);
    }
    ESP_LOGI(TAG, "done");
}
