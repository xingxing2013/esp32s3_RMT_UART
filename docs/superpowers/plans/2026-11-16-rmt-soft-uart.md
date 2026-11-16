# ESP32-S3 RMT 三路软件串口 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 ESP32-S3 上用 RMT 实现 3 路独立的全双工 8N1 软件串口，波特率运行时可配（300–1 000 000）。

**Architecture:** 每路 = 1 个 RMT TX 通道（copy encoder 发送字节展开的 symbol）+ 1 个 RMT RX 通道（`en_partial_rx` 乒乓，IRAM 回调只把 symbol 批量压入 SPSC 环形队列）。TX 由发送任务消费 StreamBuffer；RX 由解码任务按「绝对时间游标 + bit 中点采样」还原字节。纯逻辑（编码/解码/分辨率计算）与硬件完全分离。

**Tech Stack:** ESP-IDF v5.3.2（`esp_driver_rmt` 新驱动）、FreeRTOS StreamBuffer、Unity（板端测试）。

**Spec:** `docs/superpowers/specs/2026-11-16-rmt-soft-uart-design.md`

## Global Constraints

- ESP-IDF **v5.3.2**，目标 **esp32s3**，使用新 RMT 驱动（`driver/rmt_tx.h`、`driver/rmt_rx.h`），不使用 legacy `driver/rmt.h`。
- 帧格式固定 **8N1 + LSB first**；波特率 300–1 000 000，越界返回 `ESP_ERR_INVALID_ARG`。
- 最多 **3 路**；每路 RX `mem_block_symbols = 48`（1 块），**不得**给单路分配多块（3 路 RX 共需 3 块，全组只有 8 块）。
- RX `signal_range_max_ns = max(8 ms, 40 × bit 时间)`；`signal_range_min_ns = 0.2 × bit 时间`（不小于 1 个 tick）。
- 不使用 DMA；不在 `CONFIG_RMT_ISR_IRAM_SAFE` 下运行。
- 所有公共 API 返回 `esp_err_t` 或字节数；禁止静默丢弃数据，丢弃必须计数。
- 每个新终端先执行：`. D:\Development\Espressif\frameworks\esp-idf-v5.3.2\export.ps1`
- 没有硬件时只能验证「编译通过」；**任何行为性结论必须来自板端运行输出**，不得口头声称通过。
- 提交信息用中文或英文均可，但每个任务一次 commit。

## Review Focus

以下 5 类输入/场景在 spec 里只写了目标、没有现成测试，最容易在真实使用中出问题。每条都在对应任务里配了测试：

1. **起始位前有窄毛刺或线路噪声** —— 期望只产生可计数的帧错误并自动恢复，不能连续吐出垃圾字节。
2. **背靠背连续帧（帧间无空闲）** —— 期望一字节不丢、不粘连（解码器必须按期望时间点接续下一帧，而不是等空闲）。
3. **RX 溢出 / 重新装载间隙** —— 期望不崩溃、计数可见、后续数据继续可收。
4. **长低电平（break）与 0x00 的 9 bit 连续低电平** —— 期望 break 被识别且不投递伪字节，0x00 正常投递。
5. **`set_baud()` 在收发进行中调用** —— 期望允许丢弃在途数据，但句柄随后仍能正常收发，不崩溃、不泄漏。

---

### Task 1: 仓库与可编译工程骨架

**Files:**
- Create: `.gitignore`
- Create: `CMakeLists.txt`
- Create: `sdkconfig.defaults`
- Create: `main/CMakeLists.txt`
- Create: `main/main.c`
- Create: `components/rmt_soft_uart/CMakeLists.txt`
- Create: `components/rmt_soft_uart/include/rmt_soft_uart.h`
- Create: `components/rmt_soft_uart/src/soft_uart_core.c`

**Interfaces:**
- Consumes: 无
- Produces: 可 `idf.py build` 的工程；组件名 `rmt_soft_uart`；公共头文件路径 `rmt_soft_uart.h`

- [ ] **Step 1: 初始化仓库**

```powershell
git init
```

- [ ] **Step 2: 写 `.gitignore`**

```gitignore
build/
sdkconfig
sdkconfig.old
.vscode/
*.bin
*.elf
*.map
```

- [ ] **Step 3: 写根 `CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32s3_rmt_uart)
```

- [ ] **Step 4: 写 `sdkconfig.defaults`**

```
CONFIG_IDF_TARGET="esp32s3"
CONFIG_FREERTOS_HZ=1000
CONFIG_ESP_MAIN_TASK_STACK_SIZE=4096
CONFIG_COMPILER_WARN_WRITE_STRINGS=n
```

- [ ] **Step 5: 写组件 `CMakeLists.txt`（先只登记 core）**

`components/rmt_soft_uart/CMakeLists.txt`：

```cmake
idf_component_register(
    SRCS "src/soft_uart_core.c"
    INCLUDE_DIRS "include" "src"
    REQUIRES esp_driver_rmt esp_driver_gpio
    PRIV_REQUIRES unity
)
```

- [ ] **Step 6: 写公共头文件骨架**

`components/rmt_soft_uart/include/rmt_soft_uart.h`：

```c
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
esp_err_t rmt_soft_uart_set_baud(rmt_soft_uart_handle_t handle, uint32_t baud);
int       rmt_soft_uart_write(rmt_soft_uart_handle_t handle,
                              const void *data, size_t len, TickType_t timeout);
int       rmt_soft_uart_read(rmt_soft_uart_handle_t handle,
                             void *buf, size_t len, TickType_t timeout);
esp_err_t rmt_soft_uart_flush_rx(rmt_soft_uart_handle_t handle);
esp_err_t rmt_soft_uart_get_stats(rmt_soft_uart_handle_t handle,
                                  rmt_soft_uart_stats_t *out_stats);

#ifdef __cplusplus
}
#endif
```

- [ ] **Step 7: 写 core 占位实现（能链接，任务 6 会替换）**

`components/rmt_soft_uart/src/soft_uart_core.c`：

```c
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
```

- [ ] **Step 8: 写最小 `main`**

`main/CMakeLists.txt`：

```cmake
idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES rmt_soft_uart)
```

`main/main.c`：

```c
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "app";

void app_main(void)
{
    ESP_LOGI(TAG, "rmt soft uart scaffold");
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

- [ ] **Step 9: 配置目标并编译**

```powershell
. D:\Development\Espressif\frameworks\esp-idf-v5.3.2\export.ps1
idf.py set-target esp32s3
idf.py build
```

Expected: `Project build complete`。

- [ ] **Step 10: 提交**

```powershell
git add .gitignore CMakeLists.txt sdkconfig.defaults main components
git commit -m "chore: 初始化 ESP32-S3 RMT 软件串口工程骨架"
```

---

### Task 2: 时序计算与编码器（纯逻辑 + 板端单测）

**Files:**
- Create: `components/rmt_soft_uart/src/soft_uart_codec.h`
- Create: `components/rmt_soft_uart/src/soft_uart_codec.c`
- Create: `components/rmt_soft_uart/test/su_test_codec.c`
- Create: `components/rmt_soft_uart/test/su_tests.h`
- Modify: `components/rmt_soft_uart/CMakeLists.txt`

**Interfaces:**
- Consumes: 无
- Produces:
  - `esp_err_t su_timing_init(uint32_t baud, uint32_t *out_resolution_hz, su_timing_t *out_timing)`
  - `size_t su_encode(const uint8_t *data, size_t len, const su_timing_t *t, rmt_symbol_word_t *out, size_t out_words)`
  - 测试入口 `int su_test_codec_run(void)`（返回失败数）

- [ ] **Step 1: 写失败的测试**

`components/rmt_soft_uart/test/su_tests.h`：

```c
#pragma once

int su_test_codec_run(void);
```

`components/rmt_soft_uart/test/su_test_codec.c`：

```c
#include "su_tests.h"

#include "soft_uart_codec.h"
#include "unity.h"

static void test_timing_bounds(void)
{
    const uint32_t bauds[] = {300, 9600, 115200, 460800, 1000000};
    for (size_t i = 0; i < sizeof(bauds) / sizeof(bauds[0]); i++) {
        uint32_t res = 0;
        su_timing_t t = {0};
        TEST_ASSERT_EQUAL(ESP_OK, su_timing_init(bauds[i], &res, &t));
        TEST_ASSERT_TRUE(res > 0 && res <= 80000000u);
        TEST_ASSERT_TRUE(t.bit_ticks >= 1 && t.bit_ticks <= 32767u);
        TEST_ASSERT_EQUAL_UINT32(t.bit_ticks * 10u, t.frame_ticks);
        TEST_ASSERT_TRUE(t.sample_off[8] < t.frame_ticks);
        TEST_ASSERT_TRUE(t.sample_off[8] > t.sample_off[0]);
    }

    uint32_t res = 0;
    su_timing_t t = {0};
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, su_timing_init(299, &res, &t));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, su_timing_init(1000001, &res, &t));
}

static void test_encode_0x55(void)
{
    uint32_t res = 0;
    su_timing_t t = {0};
    TEST_ASSERT_EQUAL(ESP_OK, su_timing_init(115200, &res, &t));

    rmt_symbol_word_t words[5] = {0};
    const uint8_t src = 0x55;
    TEST_ASSERT_EQUAL_size_t(5, su_encode(&src, 1, &t, words, 5));

    /* 0x55 = start(0) 1 0 1 0 1 0 1 0 stop(1) -> 每对都是 (0,1) */
    for (int i = 0; i < 5; i++) {
        TEST_ASSERT_EQUAL_UINT32(0, words[i].level0);
        TEST_ASSERT_EQUAL_UINT32(1, words[i].level1);
        TEST_ASSERT_EQUAL_UINT32(t.bit_ticks, words[i].duration0);
        TEST_ASSERT_EQUAL_UINT32(t.bit_ticks, words[i].duration1);
    }
}

static void test_encode_0xaa(void)
{
    uint32_t res = 0;
    su_timing_t t = {0};
    TEST_ASSERT_EQUAL(ESP_OK, su_timing_init(9600, &res, &t));

    rmt_symbol_word_t words[5] = {0};
    const uint8_t src = 0xAA;
    TEST_ASSERT_EQUAL_size_t(5, su_encode(&src, 1, &t, words, 5));

    /* start(0) 0 1 0 1 0 1 0 1 stop(1) -> (0,0)(1,0)(1,0)(1,0)(1,1) */
    TEST_ASSERT_EQUAL_UINT32(0, words[0].level0);
    TEST_ASSERT_EQUAL_UINT32(0, words[0].level1);
    TEST_ASSERT_EQUAL_UINT32(1, words[4].level0);
    TEST_ASSERT_EQUAL_UINT32(1, words[4].level1);
}

static void test_encode_truncates_when_buffer_small(void)
{
    uint32_t res = 0;
    su_timing_t t = {0};
    TEST_ASSERT_EQUAL(ESP_OK, su_timing_init(115200, &res, &t));

    uint8_t src[4] = {1, 2, 3, 4};
    rmt_symbol_word_t words[10] = {0}; /* 只够 2 个字节 */
    TEST_ASSERT_EQUAL_size_t(10, su_encode(src, 4, &t, words, 10));
}

int su_test_codec_run(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_timing_bounds);
    RUN_TEST(test_encode_0x55);
    RUN_TEST(test_encode_0xaa);
    RUN_TEST(test_encode_truncates_when_buffer_small);
    return UNITY_END();
}
```

- [ ] **Step 2: 登记测试源并编译，确认失败**

修改 `components/rmt_soft_uart/CMakeLists.txt` 为：

```cmake
set(srcs "src/soft_uart_codec.c" "src/soft_uart_core.c")

if(CONFIG_RMT_SOFT_UART_RUN_TESTS)
    list(APPEND srcs "test/su_test_codec.c")
endif()

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "include" "src" "test"
    REQUIRES esp_driver_rmt esp_driver_gpio
    PRIV_REQUIRES unity
)
```

在 `components/rmt_soft_uart/Kconfig` 新建（本任务先建最小版本）：

```
menu "RMT Soft UART"

config RMT_SOFT_UART_RUN_TESTS
    bool "Run on-target unit tests at boot"
    default n

endmenu
```

`main/main.c` 改为：

```c
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "su_tests.h"

static const char *TAG = "app";

void app_main(void)
{
#if CONFIG_RMT_SOFT_UART_RUN_TESTS
    int failures = su_test_codec_run();
    ESP_LOGI(TAG, "codec tests: %d failure(s)", failures);
#endif
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

同时把 `main/CMakeLists.txt` 的 `REQUIRES` 改成 `rmt_soft_uart`（已经是）。

```powershell
. D:\Development\Espressif\frameworks\esp-idf-v5.3.2\export.ps1
idf.py menuconfig   # Component config -> RMT Soft UART -> Run on-target unit tests at boot -> y
idf.py build
```

Expected: 编译失败，报 `soft_uart_codec.h: No such file or directory`。

- [ ] **Step 3: 写 codec 头文件**

`components/rmt_soft_uart/src/soft_uart_codec.h`：

```c
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/rmt_types.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SU_MAX_BIT_TICKS 32767u

typedef struct {
    uint32_t bit_ticks;
    uint32_t frame_ticks;
    uint32_t sample_off[9];
} su_timing_t;

typedef struct {
    bool framing_error;
    bool break_error;
} su_rx_flags_t;

typedef void (*su_byte_cb_t)(void *ctx, uint8_t byte, su_rx_flags_t flags);

typedef struct su_decoder_s su_decoder_t;

esp_err_t su_timing_init(uint32_t baud, uint32_t *out_resolution_hz,
                         su_timing_t *out_timing);

size_t su_encode(const uint8_t *data, size_t len, const su_timing_t *t,
                 rmt_symbol_word_t *out, size_t out_words);

su_decoder_t *su_decoder_create(const su_timing_t *t, su_byte_cb_t cb, void *ctx);
void su_decoder_reset(su_decoder_t *d);
void su_decoder_feed(su_decoder_t *d, const rmt_symbol_word_t *syms, size_t n);
void su_decoder_destroy(su_decoder_t *d);

#ifdef __cplusplus
}
#endif
```

- [ ] **Step 4: 写 codec 实现（本任务只做时序 + 编码；解码留空壳，任务 3 填）**

`components/rmt_soft_uart/src/soft_uart_codec.c`：

```c
#include "soft_uart_codec.h"

#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"

static const char *TAG = "su_codec";

#define SU_RMT_SRC_HZ 80000000u

esp_err_t su_timing_init(uint32_t baud, uint32_t *out_resolution_hz,
                         su_timing_t *out_timing)
{
    if (!out_timing) {
        return ESP_ERR_INVALID_ARG;
    }
    if (baud < 300 || baud > 1000000) {
        return ESP_ERR_INVALID_ARG;
    }

    /* N = round(80MHz / (baud * 80))，限制在 1..255 */
    uint32_t denom = baud * 80u;
    uint32_t n = (SU_RMT_SRC_HZ + denom / 2u) / denom;
    if (n < 1) {
        n = 1;
    }
    if (n > 255) {
        n = 255;
    }

    uint32_t resolution = SU_RMT_SRC_HZ / n;
    uint32_t bit_ticks = (resolution + baud / 2u) / baud;
    if (bit_ticks == 0) {
        bit_ticks = 1;
    }
    if (bit_ticks > SU_MAX_BIT_TICKS) {
        return ESP_ERR_INVALID_ARG;
    }

    out_timing->bit_ticks = bit_ticks;
    out_timing->frame_ticks = bit_ticks * 10u;
    for (int k = 0; k < 9; k++) {
        out_timing->sample_off[k] = (bit_ticks * (uint32_t)(2 * k + 3) + 1u) / 2u;
    }
    if (out_resolution_hz) {
        *out_resolution_hz = resolution;
    }
    return ESP_OK;
}

size_t su_encode(const uint8_t *data, size_t len, const su_timing_t *t,
                 rmt_symbol_word_t *out, size_t out_words)
{
    if (!data || !t || !out || t->bit_ticks == 0) {
        return 0;
    }
    size_t capacity_bytes = out_words / 5u;
    if (len > capacity_bytes) {
        len = capacity_bytes;
    }

    size_t wi = 0;
    for (size_t i = 0; i < len; i++) {
        uint32_t level[10];
        level[0] = 0; /* start */
        for (int b = 0; b < 8; b++) {
            level[1 + b] = (data[i] >> b) & 1u; /* LSB first */
        }
        level[9] = 1; /* stop */
        for (int s = 0; s < 10; s += 2) {
            out[wi].level0 = level[s];
            out[wi].duration0 = t->bit_ticks;
            out[wi].level1 = level[s + 1];
            out[wi].duration1 = t->bit_ticks;
            wi++;
        }
    }
    return wi;
}

struct su_decoder_s {
    su_timing_t t;
    su_byte_cb_t cb;
    void *ctx;
};

su_decoder_t *su_decoder_create(const su_timing_t *t, su_byte_cb_t cb, void *ctx)
{
    if (!t) {
        return NULL;
    }
    su_decoder_t *d = heap_caps_calloc(1, sizeof(*d), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!d) {
        return NULL;
    }
    d->t = *t;
    d->cb = cb;
    d->ctx = ctx;
    return d;
}

void su_decoder_reset(su_decoder_t *d)
{
    (void)d; /* 任务 3 实现 */
}

void su_decoder_feed(su_decoder_t *d, const rmt_symbol_word_t *syms, size_t n)
{
    (void)d;
    (void)syms;
    (void)n; /* 任务 3 实现 */
}

void su_decoder_destroy(su_decoder_t *d)
{
    heap_caps_free(d);
}
```

- [ ] **Step 5: 编译（此时测试还跑不了，先确认能编译）**

```powershell
. D:\Development\Espressif\frameworks\esp-idf-v5.3.2\export.ps1
idf.py build
```

Expected: 编译成功；`su_test_codec.c` 里对 `su_decoder_*` 无引用，不会链接失败。

- [ ] **Step 6: 烧录并运行 codec 测试**

```powershell
idf.py -p COMx flash monitor
```

Expected 串口输出包含：

```
I (xxx) app: codec tests: 0 failure(s)
```

- [ ] **Step 7: 提交**

```powershell
git add components/rmt_soft_uart main
git commit -m "feat(codec): 分辨率选择与 8N1 编码器，含板端单测"
```

---

### Task 3: 解码器（状态机 + 板端单测）

**Files:**
- Modify: `components/rmt_soft_uart/src/soft_uart_codec.c`（替换 `su_decoder_*` 空实现）
- Modify: `components/rmt_soft_uart/test/su_test_codec.c`（新增解码测试）
- Modify: `components/rmt_soft_uart/src/soft_uart_codec.h`（`struct su_decoder_s` 保持私有，无需改）

**Interfaces:**
- Consumes: `su_timing_init`、`su_encode`
- Produces: `su_decoder_create/reset/feed/destroy` 的完整行为；回调签名 `void (*)(void *ctx, uint8_t byte, su_rx_flags_t flags)`

- [ ] **Step 1: 写失败的解码测试**

在 `test/su_test_codec.c` 顶部加入：

```c
typedef struct {
    uint8_t buf[600];
    size_t n;
    uint32_t fe;
    uint32_t br;
} rx_sink_t;

static void sink_cb(void *ctx, uint8_t byte, su_rx_flags_t flags)
{
    rx_sink_t *s = ctx;
    if (flags.break_error) {
        s->br++;
        return;
    }
    if (s->n < sizeof(s->buf)) {
        s->buf[s->n++] = byte;
    }
    if (flags.framing_error) {
        s->fe++;
    }
}
```

再追加这些测试函数，并在 `su_test_codec_run()` 里 `RUN_TEST` 它们：

```c
static void test_decode_roundtrip_chunked(void)
{
    uint32_t res = 0;
    su_timing_t t = {0};
    TEST_ASSERT_EQUAL(ESP_OK, su_timing_init(115200, &res, &t));

    uint8_t src[256];
    for (size_t i = 0; i < sizeof(src); i++) {
        src[i] = (uint8_t)(i * 37u + 11u);
    }
    rmt_symbol_word_t syms[256 * 5];
    size_t words = su_encode(src, sizeof(src), &t, syms, 256 * 5);
    TEST_ASSERT_EQUAL_size_t(256 * 5, words);

    rx_sink_t sink = {0};
    su_decoder_t *d = su_decoder_create(&t, sink_cb, &sink);
    TEST_ASSERT_NOT_NULL(d);

    const size_t chunks[] = {1, 2, 3, 5, 7, 11};
    size_t i = 0, ci = 0;
    while (i < words) {
        size_t n = chunks[ci++ % (sizeof(chunks) / sizeof(chunks[0]))];
        if (n > words - i) {
            n = words - i;
        }
        su_decoder_feed(d, &syms[i], n);
        i += n;
    }

    TEST_ASSERT_EQUAL_UINT32(256, sink.n);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(src, sink.buf, 256);
    TEST_ASSERT_EQUAL_UINT32(0, sink.fe);
    TEST_ASSERT_EQUAL_UINT32(0, sink.br);
    su_decoder_destroy(d);
}

static void test_decode_clock_skew(void)
{
    const uint32_t rates[] = {9600, 115200};
    const uint32_t num[] = {97, 103};
    for (size_t r = 0; r < 2; r++) {
        for (size_t s = 0; s < 2; s++) {
            uint32_t res = 0;
            su_timing_t t = {0};
            TEST_ASSERT_EQUAL(ESP_OK, su_timing_init(rates[r], &res, &t));

            uint8_t src[32];
            for (size_t i = 0; i < sizeof(src); i++) {
                src[i] = (uint8_t)(0xA5u ^ (i * 13u));
            }
            rmt_symbol_word_t syms[32 * 5];
            size_t words = su_encode(src, sizeof(src), &t, syms, 160);
            for (size_t i = 0; i < words; i++) {
                syms[i].duration0 = (uint16_t)((syms[i].duration0 * num[s] + 50u) / 100u);
                syms[i].duration1 = (uint16_t)((syms[i].duration1 * num[s] + 50u) / 100u);
            }

            rx_sink_t sink = {0};
            su_decoder_t *d = su_decoder_create(&t, sink_cb, &sink);
            TEST_ASSERT_NOT_NULL(d);
            su_decoder_feed(d, syms, words);
            TEST_ASSERT_EQUAL_UINT32(sizeof(src), sink.n);
            TEST_ASSERT_EQUAL_UINT8_ARRAY(src, sink.buf, sizeof(src));
            TEST_ASSERT_EQUAL_UINT32(0, sink.fe);
            su_decoder_destroy(d);
        }
    }
}

static size_t append_idle(rmt_symbol_word_t *out, size_t words, uint32_t ticks)
{
    while (ticks > 0) {
        uint32_t d = ticks > 32767u ? 32767u : ticks;
        out[words].level0 = 1;
        out[words].duration0 = d;
        out[words].level1 = 0;
        out[words].duration1 = 0;
        words++;
        ticks -= d;
    }
    return words;
}

static void test_decode_with_idle_gaps_and_split_symbols(void)
{
    uint32_t res = 0;
    su_timing_t t = {0};
    TEST_ASSERT_EQUAL(ESP_OK, su_timing_init(9600, &res, &t));

    uint8_t src[16];
    for (size_t i = 0; i < sizeof(src); i++) {
        src[i] = (uint8_t)(i + 1);
    }

    rmt_symbol_word_t stream[16 * 5 + 64];
    size_t words = 0;
    for (size_t i = 0; i < sizeof(src); i++) {
        rmt_symbol_word_t one[5];
        TEST_ASSERT_EQUAL_size_t(5, su_encode(&src[i], 1, &t, one, 5));
        words += append_idle(stream, words, 100000); /* 模拟 32767 拆分的长空闲 */
        memcpy(&stream[words], one, sizeof(one));
        words += 5;
    }
    words = append_idle(stream, words, 100000);

    rx_sink_t sink = {0};
    su_decoder_t *d = su_decoder_create(&t, sink_cb, &sink);
    TEST_ASSERT_NOT_NULL(d);
    su_decoder_feed(d, stream, words);
    TEST_ASSERT_EQUAL_UINT32(sizeof(src), sink.n);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(src, sink.buf, sizeof(src));
    TEST_ASSERT_EQUAL_UINT32(0, sink.fe);
    su_decoder_destroy(d);
}

static void test_decode_framing_error_still_delivers_byte(void)
{
    uint32_t res = 0;
    su_timing_t t = {0};
    TEST_ASSERT_EQUAL(ESP_OK, su_timing_init(115200, &res, &t));

    rmt_symbol_word_t syms[5] = {0};
    const uint8_t src = 0x3C;
    TEST_ASSERT_EQUAL_size_t(5, su_encode(&src, 1, &t, syms, 5));
    syms[4].level1 = 0; /* 停止位强制为低 */

    rmt_symbol_word_t stream[8];
    memcpy(stream, syms, sizeof(syms));
    size_t words = append_idle(stream, 5, 4 * t.bit_ticks); /* 拉高结束低电平 run */

    rx_sink_t sink = {0};
    su_decoder_t *d = su_decoder_create(&t, sink_cb, &sink);
    TEST_ASSERT_NOT_NULL(d);
    su_decoder_feed(d, stream, words);
    TEST_ASSERT_EQUAL_UINT32(1, sink.n);
    TEST_ASSERT_EQUAL_UINT8(src, sink.buf[0]);
    TEST_ASSERT_EQUAL_UINT32(1, sink.fe);
    su_decoder_destroy(d);
}

static void test_decode_break_is_not_a_byte(void)
{
    uint32_t res = 0;
    su_timing_t t = {0};
    TEST_ASSERT_EQUAL(ESP_OK, su_timing_init(9600, &res, &t));

    rmt_symbol_word_t stream[2];
    stream[0].level0 = 0;
    stream[0].duration0 = 12u * t.bit_ticks;
    stream[0].level1 = 1;
    stream[0].duration1 = t.bit_ticks;

    rx_sink_t sink = {0};
    su_decoder_t *d = su_decoder_create(&t, sink_cb, &sink);
    TEST_ASSERT_NOT_NULL(d);
    su_decoder_feed(d, stream, 2);
    TEST_ASSERT_EQUAL_UINT32(1, sink.br);
    TEST_ASSERT_EQUAL_UINT32(0, sink.n);
    su_decoder_destroy(d);
}

static void test_decode_0x00_is_not_break(void)
{
    uint32_t res = 0;
    su_timing_t t = {0};
    TEST_ASSERT_EQUAL(ESP_OK, su_timing_init(115200, &res, &t));

    rmt_symbol_word_t syms[5] = {0};
    const uint8_t src = 0x00;
    TEST_ASSERT_EQUAL_size_t(5, su_encode(&src, 1, &t, syms, 5));

    rx_sink_t sink = {0};
    su_decoder_t *d = su_decoder_create(&t, sink_cb, &sink);
    TEST_ASSERT_NOT_NULL(d);
    su_decoder_feed(d, syms, 5);
    TEST_ASSERT_EQUAL_UINT32(0, sink.br);
    TEST_ASSERT_EQUAL_UINT32(1, sink.n);
    TEST_ASSERT_EQUAL_UINT8(0x00, sink.buf[0]);
    su_decoder_destroy(d);
}

static void test_decode_recovers_after_glitch(void)
{
    uint32_t res = 0;
    su_timing_t t = {0};
    TEST_ASSERT_EQUAL(ESP_OK, su_timing_init(115200, &res, &t));

    rmt_symbol_word_t glitch[2];
    glitch[0].level0 = 0;
    glitch[0].duration0 = t.bit_ticks + t.bit_ticks / 2;
    glitch[0].level1 = 1;
    glitch[0].duration1 = 2 * t.bit_ticks;

    rmt_symbol_word_t syms[5] = {0};
    const uint8_t src = 0x7E;
    TEST_ASSERT_EQUAL_size_t(5, su_encode(&src, 1, &t, syms, 5));

    rmt_symbol_word_t stream[16];
    memcpy(stream, glitch, sizeof(glitch));
    memcpy(&stream[2], syms, sizeof(syms));

    rx_sink_t sink = {0};
    su_decoder_t *d = su_decoder_create(&t, sink_cb, &sink);
    TEST_ASSERT_NOT_NULL(d);
    su_decoder_feed(d, stream, 7);
    TEST_ASSERT_TRUE(sink.n >= 1);
    TEST_ASSERT_EQUAL_UINT8(src, sink.buf[sink.n - 1]);
    su_decoder_destroy(d);
}
```

`su_test_codec_run()` 里追加：

```c
    RUN_TEST(test_decode_roundtrip_chunked);
    RUN_TEST(test_decode_clock_skew);
    RUN_TEST(test_decode_with_idle_gaps_and_split_symbols);
    RUN_TEST(test_decode_framing_error_still_delivers_byte);
    RUN_TEST(test_decode_break_is_not_a_byte);
    RUN_TEST(test_decode_0x00_is_not_break);
    RUN_TEST(test_decode_recovers_after_glitch);
```

并在文件顶部加 `#include <string.h>`。

- [ ] **Step 2: 编译确认测试失败**

```powershell
. D:\Development\Espressif\frameworks\esp-idf-v5.3.2\export.ps1
idf.py build
```

Expected: 编译通过（解码器是空实现），烧录后测试报告大量失败（`sink.n == 0`）。

- [ ] **Step 3: 实现解码器**

用下面内容替换 `soft_uart_codec.c` 里的 `struct su_decoder_s` 到文件末尾：

```c
struct su_decoder_s {
    su_timing_t t;
    su_byte_cb_t cb;
    void *ctx;

    uint64_t now;
    bool prev_valid;
    int prev_level;

    int run_level;
    uint32_t run_ticks;

    bool in_frame;
    uint64_t frame_start;
    int next_sample;
    uint8_t data;

    bool pending_valid;
    uint8_t pending_byte;

    bool have_next;
    uint64_t next_start;
};

static void dec_emit(su_decoder_t *d, uint8_t byte, bool framing_error, bool break_error)
{
    su_rx_flags_t flags = {
        .framing_error = framing_error,
        .break_error = break_error,
    };
    if (d->cb) {
        d->cb(d->ctx, byte, flags);
    }
}

/* 低电平 run 结束时判定 break / 帧错误字节 */
static void dec_run_end(su_decoder_t *d)
{
    if (d->run_level != 0) {
        return;
    }
    if (d->run_ticks >= d->t.frame_ticks) {
        d->pending_valid = false;
        d->in_frame = false;
        d->have_next = false;
        dec_emit(d, 0, false, true);
    } else if (d->pending_valid) {
        d->pending_valid = false;
        dec_emit(d, d->pending_byte, true, false);
    }
}

static void dec_frame_begin(su_decoder_t *d, uint64_t t0)
{
    d->in_frame = true;
    d->frame_start = t0;
    d->next_sample = 0;
    d->data = 0;
    d->have_next = false;
}

static void dec_frame_consume(su_decoder_t *d, int level, uint64_t start, uint64_t end)
{
    (void)start;

    while (d->in_frame) {
        while (d->next_sample < 9) {
            uint64_t ts = d->frame_start + d->t.sample_off[d->next_sample];
            if (ts >= end) {
                return;
            }
            if (d->next_sample < 8) {
                if (level) {
                    d->data |= (uint8_t)(1u << d->next_sample);
                }
            } else if (level) {
                dec_emit(d, d->data, false, false);
            } else {
                d->pending_valid = true;
                d->pending_byte = d->data;
            }
            d->next_sample++;
        }

        d->in_frame = false;
        d->have_next = true;
        d->next_start = d->frame_start + d->t.frame_ticks;

        if (d->next_start < end) {
            if (level == 0) {
                d->have_next = false;
                dec_frame_begin(d, d->next_start);
                continue; /* 背靠背帧：同一段低电平里直接开始下一帧 */
            }
            d->have_next = false;
            return;
        }
        return;
    }
}

static void dec_edge(su_decoder_t *d, int level, uint32_t duration)
{
    if (duration == 0) {
        return;
    }
    uint64_t start = d->now;
    uint64_t end = start + duration;

    if (d->run_level != level) {
        dec_run_end(d);
        d->run_level = level;
        d->run_ticks = duration;
    } else if (d->run_ticks <= UINT32_MAX - duration) {
        d->run_ticks += duration;
    }

    if (!d->in_frame && d->have_next && d->next_start <= start) {
        if (level == 0 && d->next_start == start) {
            uint64_t t0 = d->next_start;
            d->have_next = false;
            dec_frame_begin(d, t0);
        } else {
            d->have_next = false;
        }
    }

    if (d->in_frame) {
        dec_frame_consume(d, level, start, end);
    }

    if (!d->in_frame && level == 0 && d->prev_valid && d->prev_level == 1) {
        dec_frame_begin(d, start);
        dec_frame_consume(d, level, start, end);
    }

    d->prev_valid = true;
    d->prev_level = level;
    d->now = end;
}

su_decoder_t *su_decoder_create(const su_timing_t *t, su_byte_cb_t cb, void *ctx)
{
    if (!t) {
        return NULL;
    }
    su_decoder_t *d = heap_caps_calloc(1, sizeof(*d), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!d) {
        return NULL;
    }
    d->t = *t;
    d->cb = cb;
    d->ctx = ctx;
    d->run_level = -1;
    return d;
}

void su_decoder_reset(su_decoder_t *d)
{
    if (!d) {
        return;
    }
    su_timing_t t = d->t;
    su_byte_cb_t cb = d->cb;
    void *ctx = d->ctx;
    memset(d, 0, sizeof(*d));
    d->t = t;
    d->cb = cb;
    d->ctx = ctx;
    d->run_level = -1;
}

void su_decoder_feed(su_decoder_t *d, const rmt_symbol_word_t *syms, size_t n)
{
    if (!d || !syms) {
        return;
    }
    for (size_t i = 0; i < n; i++) {
        dec_edge(d, syms[i].level0, syms[i].duration0);
        dec_edge(d, syms[i].level1, syms[i].duration1);
    }
}

void su_decoder_destroy(su_decoder_t *d)
{
    heap_caps_free(d);
}
```

- [ ] **Step 4: 编译并跑全部 codec 测试**

```powershell
. D:\Development\Espressif\frameworks\esp-idf-v5.3.2\export.ps1
idf.py build
idf.py -p COMx flash monitor
```

Expected: `codec tests: 0 failure(s)`。

若 `test_decode_recovers_after_glitch` 因毛刺产生额外字节而失败，先确认失败信息是 `sink.n` 数量不符还是最后一个字节不符——只允许「毛刺带来额外垃圾字节」，**最后一个字节必须是 0x7E**；不允许为了让测试通过而放宽这条断言。

- [ ] **Step 5: 提交**

```powershell
git add components/rmt_soft_uart
git commit -m "feat(codec): 增量解码状态机，支持跨 symbol/跨帧边界的 8N1 还原"
```

---

### Task 4: ISR 安全 symbol 环形队列

**Files:**
- Create: `components/rmt_soft_uart/src/symbol_ring.h`
- Create: `components/rmt_soft_uart/src/symbol_ring.c`
- Create: `components/rmt_soft_uart/test/su_test_ring.c`
- Modify: `components/rmt_soft_uart/test/su_tests.h`
- Modify: `components/rmt_soft_uart/CMakeLists.txt`
- Modify: `main/main.c`

**Interfaces:**
- Consumes: `rmt_symbol_word_t`
- Produces:
  - `su_symbol_ring_t *su_symbol_ring_create(uint32_t capacity)`
  - `void su_symbol_ring_destroy(su_symbol_ring_t *r)`
  - `size_t su_symbol_ring_push_from_isr(su_symbol_ring_t *r, const rmt_symbol_word_t *src, size_t n)`
  - `size_t su_symbol_ring_pop(su_symbol_ring_t *r, rmt_symbol_word_t *dst, size_t n)`
  - `size_t su_symbol_ring_count(const su_symbol_ring_t *r)`
  - `void su_symbol_ring_reset(su_symbol_ring_t *r)`
  - 测试入口 `int su_test_ring_run(void)`

- [ ] **Step 1: 写失败的测试**

`components/rmt_soft_uart/test/su_tests.h` 追加：

```c
int su_test_ring_run(void);
```

`components/rmt_soft_uart/test/su_test_ring.c`：

```c
#include "su_tests.h"

#include "symbol_ring.h"
#include "unity.h"

static rmt_symbol_word_t make_sym(uint32_t v)
{
    rmt_symbol_word_t s = {0};
    s.duration0 = (uint16_t)(v & 0x7FFFu);
    s.level0 = (uint16_t)((v >> 15) & 1u);
    return s;
}

static void test_ring_push_pop_and_wrap(void)
{
    su_symbol_ring_t *r = su_symbol_ring_create(8);
    TEST_ASSERT_NOT_NULL(r);
    TEST_ASSERT_EQUAL_UINT32(0, su_symbol_ring_count(r));

    rmt_symbol_word_t in[8];
    for (int i = 0; i < 8; i++) {
        in[i] = make_sym((uint32_t)i);
    }
    TEST_ASSERT_EQUAL_size_t(8, su_symbol_ring_push_from_isr(r, in, 8));
    TEST_ASSERT_EQUAL_UINT32(8, su_symbol_ring_count(r));
    TEST_ASSERT_EQUAL_size_t(0, su_symbol_ring_push_from_isr(r, in, 1));

    rmt_symbol_word_t out[8] = {0};
    TEST_ASSERT_EQUAL_size_t(5, su_symbol_ring_pop(r, out, 5));
    for (int i = 0; i < 5; i++) {
        TEST_ASSERT_EQUAL_UINT32((uint32_t)i, out[i].duration0);
    }

    rmt_symbol_word_t more[5];
    for (int i = 0; i < 5; i++) {
        more[i] = make_sym((uint32_t)(100 + i));
    }
    TEST_ASSERT_EQUAL_size_t(5, su_symbol_ring_push_from_isr(r, more, 5));
    TEST_ASSERT_EQUAL_UINT32(8, su_symbol_ring_count(r));

    TEST_ASSERT_EQUAL_size_t(8, su_symbol_ring_pop(r, out, 8));
    TEST_ASSERT_EQUAL_UINT32(5, out[0].duration0);
    TEST_ASSERT_EQUAL_UINT32(7, out[2].duration0);
    TEST_ASSERT_EQUAL_UINT32(100, out[3].duration0);
    TEST_ASSERT_EQUAL_UINT32(104, out[7].duration0);
    TEST_ASSERT_EQUAL_UINT32(0, su_symbol_ring_count(r));

    su_symbol_ring_destroy(r);
}

static void test_ring_reset(void)
{
    su_symbol_ring_t *r = su_symbol_ring_create(4);
    TEST_ASSERT_NOT_NULL(r);
    rmt_symbol_word_t s = make_sym(1);
    TEST_ASSERT_EQUAL_size_t(1, su_symbol_ring_push_from_isr(r, &s, 1));
    TEST_ASSERT_EQUAL_UINT32(1, su_symbol_ring_count(r));
    su_symbol_ring_reset(r);
    TEST_ASSERT_EQUAL_UINT32(0, su_symbol_ring_count(r));
    su_symbol_ring_destroy(r);
}

int su_test_ring_run(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_ring_push_pop_and_wrap);
    RUN_TEST(test_ring_reset);
    return UNITY_END();
}
```

- [ ] **Step 2: 编译确认失败**

`CMakeLists.txt` 里把 ring 源和测试加进去：

```cmake
set(srcs "src/soft_uart_codec.c" "src/symbol_ring.c" "src/soft_uart_core.c")

if(CONFIG_RMT_SOFT_UART_RUN_TESTS)
    list(APPEND srcs "test/su_test_codec.c" "test/su_test_ring.c")
endif()
```

```powershell
. D:\Development\Espressif\frameworks\esp-idf-v5.3.2\export.ps1
idf.py build
```

Expected: 编译失败，`symbol_ring.h: No such file or directory`。

- [ ] **Step 3: 实现环形队列**

`components/rmt_soft_uart/src/symbol_ring.h`：

```c
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/rmt_types.h"
#include "esp_attr.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    rmt_symbol_word_t *buf;
    uint32_t cap;
    uint32_t mask;
    volatile uint32_t head;
    volatile uint32_t tail;
} su_symbol_ring_t;

su_symbol_ring_t *su_symbol_ring_create(uint32_t capacity);

void su_symbol_ring_destroy(su_symbol_ring_t *r);

/* 单生产者（ISR）批量入队，返回实际写入数量 */
IRAM_ATTR size_t su_symbol_ring_push_from_isr(su_symbol_ring_t *r,
                                              const rmt_symbol_word_t *src, size_t n);

/* 单消费者（任务）批量出队，返回实际读出数量 */
size_t su_symbol_ring_pop(su_symbol_ring_t *r, rmt_symbol_word_t *dst, size_t n);

size_t su_symbol_ring_count(const su_symbol_ring_t *r);

void su_symbol_ring_reset(su_symbol_ring_t *r);

#ifdef __cplusplus
}
#endif
```

`components/rmt_soft_uart/src/symbol_ring.c`：

```c
#include "symbol_ring.h"

#include <string.h>

#include "esp_heap_caps.h"

static inline uint32_t ring_load(const volatile uint32_t *p)
{
    return __atomic_load_n(p, __ATOMIC_ACQUIRE);
}

static inline void ring_store(volatile uint32_t *p, uint32_t v)
{
    __atomic_store_n(p, v, __ATOMIC_RELEASE);
}

su_symbol_ring_t *su_symbol_ring_create(uint32_t capacity)
{
    if (capacity == 0) {
        return NULL;
    }
    uint32_t cap = 1;
    while (cap < capacity) {
        cap <<= 1;
    }

    su_symbol_ring_t *r = heap_caps_calloc(1, sizeof(*r), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!r) {
        return NULL;
    }
    r->buf = heap_caps_malloc(cap * sizeof(rmt_symbol_word_t),
                              MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!r->buf) {
        heap_caps_free(r);
        return NULL;
    }
    r->cap = cap;
    r->mask = cap - 1;
    return r;
}

void su_symbol_ring_destroy(su_symbol_ring_t *r)
{
    if (!r) {
        return;
    }
    heap_caps_free(r->buf);
    heap_caps_free(r);
}

IRAM_ATTR size_t su_symbol_ring_push_from_isr(su_symbol_ring_t *r,
                                              const rmt_symbol_word_t *src, size_t n)
{
    if (!r || !src || n == 0) {
        return 0;
    }
    uint32_t head = r->head;
    uint32_t tail = ring_load(&r->tail);
    uint32_t used = head - tail;
    uint32_t free_cnt = r->cap - used;
    if (n > free_cnt) {
        n = free_cnt;
    }
    if (n == 0) {
        return 0;
    }

    uint32_t idx = head & r->mask;
    uint32_t first = r->cap - idx;
    if (first > n) {
        first = (uint32_t)n;
    }
    memcpy(&r->buf[idx], src, first * sizeof(rmt_symbol_word_t));
    if (n > first) {
        memcpy(&r->buf[0], src + first, (n - first) * sizeof(rmt_symbol_word_t));
    }
    ring_store(&r->head, head + (uint32_t)n);
    return n;
}

size_t su_symbol_ring_pop(su_symbol_ring_t *r, rmt_symbol_word_t *dst, size_t n)
{
    if (!r || !dst || n == 0) {
        return 0;
    }
    uint32_t tail = r->tail;
    uint32_t head = ring_load(&r->head);
    uint32_t avail = head - tail;
    if (n > avail) {
        n = avail;
    }
    if (n == 0) {
        return 0;
    }

    uint32_t idx = tail & r->mask;
    uint32_t first = r->cap - idx;
    if (first > n) {
        first = (uint32_t)n;
    }
    memcpy(dst, &r->buf[idx], first * sizeof(rmt_symbol_word_t));
    if (n > first) {
        memcpy(dst + first, &r->buf[0], (n - first) * sizeof(rmt_symbol_word_t));
    }
    ring_store(&r->tail, tail + (uint32_t)n);
    return n;
}

size_t su_symbol_ring_count(const su_symbol_ring_t *r)
{
    if (!r) {
        return 0;
    }
    return ring_load(&r->head) - ring_load(&r->tail);
}

void su_symbol_ring_reset(su_symbol_ring_t *r)
{
    if (!r) {
        return;
    }
    ring_store(&r->tail, ring_load(&r->head));
}
```

- [ ] **Step 4: 接入测试入口并编译**

`main/main.c` 改为：

```c
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "su_tests.h"

static const char *TAG = "app";

void app_main(void)
{
#if CONFIG_RMT_SOFT_UART_RUN_TESTS
    int codec_failures = su_test_codec_run();
    int ring_failures = su_test_ring_run();
    ESP_LOGI(TAG, "codec tests: %d failure(s)", codec_failures);
    ESP_LOGI(TAG, "ring tests: %d failure(s)", ring_failures);
#endif
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

```powershell
. D:\Development\Espressif\frameworks\esp-idf-v5.3.2\export.ps1
idf.py build
idf.py -p COMx flash monitor
```

Expected:

```
I (xxx) app: codec tests: 0 failure(s)
I (xxx) app: ring tests: 0 failure(s)
```

- [ ] **Step 5: 提交**

```powershell
git add components/rmt_soft_uart main
git commit -m "feat(ring): ISR 安全 SPSC symbol 环形队列"
```

---

### Task 5: 内部结构、TX 通道与发送任务

**Files:**
- Create: `components/rmt_soft_uart/src/soft_uart_internal.h`
- Create: `components/rmt_soft_uart/src/soft_uart_tx.h`
- Create: `components/rmt_soft_uart/src/soft_uart_tx.c`
- Modify: `components/rmt_soft_uart/src/soft_uart_core.c`
- Modify: `components/rmt_soft_uart/CMakeLists.txt`
- Modify: `main/main.c`（临时验证 `new/write/del`）

**Interfaces:**
- Consumes: `su_timing_init`、`su_encode`
- Produces:
  - `struct rmt_soft_uart_s` 的字段布局（后续任务全部依赖）
  - `esp_err_t su_tx_create(struct rmt_soft_uart_s *u)`
  - `void su_tx_destroy(struct rmt_soft_uart_s *u)`
  - 可用的 `rmt_soft_uart_new` / `write` / `del`（RX 尚未接入）

- [ ] **Step 1: 写内部头**

`components/rmt_soft_uart/src/soft_uart_internal.h`：

```c
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include "rmt_soft_uart.h"
#include "soft_uart_codec.h"
#include "symbol_ring.h"

#define SU_TX_CHUNK_BYTES   64
#define SU_RX_USER_SYMBOLS  96
#define SU_RING_SYMBOLS     1024
#define SU_MEM_BLOCK_SYMS   48
#define SU_RX_STACK         4096
#define SU_TX_STACK         3584
#define SU_RX_PRIO          12
#define SU_TX_PRIO          10
#define SU_DEFAULT_RX_BYTES 1024
#define SU_DEFAULT_TX_BYTES 512

struct rmt_soft_uart_s {
    rmt_soft_uart_config_t cfg;
    uint32_t resolution_hz;
    su_timing_t timing;

    rmt_channel_handle_t tx_ch;
    rmt_channel_handle_t rx_ch;
    rmt_encoder_handle_t encoder;

    StreamBufferHandle_t tx_sb;
    StreamBufferHandle_t rx_sb;
    su_symbol_ring_t *ring;
    su_decoder_t *dec;

    TaskHandle_t tx_task;
    TaskHandle_t rx_task;
    SemaphoreHandle_t api_lock;
    SemaphoreHandle_t tx_exited;
    SemaphoreHandle_t rx_exited;

    rmt_symbol_word_t *rx_buf;
    rmt_symbol_word_t *tx_syms;
    uint8_t *tx_bytes;

    uint32_t rx_min_ns;
    uint32_t rx_max_ns;

    volatile bool closing;
    volatile bool rx_need_rearm;

    rmt_soft_uart_stats_t stats;
};
```

- [ ] **Step 2: 写 TX 接口头**

`components/rmt_soft_uart/src/soft_uart_tx.h`：

```c
#pragma once

#include "esp_err.h"

struct rmt_soft_uart_s;

esp_err_t su_tx_create(struct rmt_soft_uart_s *u);
void su_tx_destroy(struct rmt_soft_uart_s *u);
```

- [ ] **Step 3: 写 TX 实现**

`components/rmt_soft_uart/src/soft_uart_tx.c`：

```c
#include "soft_uart_tx.h"

#include <string.h>

#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "soft_uart_internal.h"

static const char *TAG = "su_tx";

static void su_tx_task(void *arg)
{
    struct rmt_soft_uart_s *u = arg;
    const rmt_transmit_config_t tcfg = {
        .loop_count = 0,
        .flags = {
            .eot_level = 1,
            .queue_nonblocking = 0,
        },
    };

    for (;;) {
        size_t got = xStreamBufferReceive(u->tx_sb, u->tx_bytes,
                                          SU_TX_CHUNK_BYTES, portMAX_DELAY);
        if (u->closing) {
            break;
        }
        if (got == 0) {
            continue;
        }

        size_t words = su_encode(u->tx_bytes, got, &u->timing, u->tx_syms,
                                (size_t)SU_TX_CHUNK_BYTES * 5u);
        if (words == 0) {
            continue;
        }

        esp_err_t err = rmt_transmit(u->tx_ch, u->encoder, u->tx_syms,
                                     words * sizeof(rmt_symbol_word_t), &tcfg);
        if (err != ESP_OK) {
            u->stats.tx_errors++;
            ESP_LOGW(TAG, "rmt_transmit failed: %s", esp_err_to_name(err));
            continue;
        }

        err = rmt_tx_wait_all_done(u->tx_ch, 500);
        if (err != ESP_OK) {
            u->stats.tx_errors++;
            ESP_LOGW(TAG, "tx wait failed: %s", esp_err_to_name(err));
            rmt_disable(u->tx_ch);
            rmt_enable(u->tx_ch);
            continue;
        }
        u->stats.tx_bytes += (uint32_t)got;
    }

    xSemaphoreGive(u->tx_exited);
    vTaskDelete(NULL);
}

esp_err_t su_tx_create(struct rmt_soft_uart_s *u)
{
    rmt_tx_channel_config_t cfg = {
        .gpio_num = (gpio_num_t)u->cfg.tx_gpio,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = u->resolution_hz,
        .mem_block_symbols = SU_MEM_BLOCK_SYMS,
        .trans_queue_depth = 4,
        .intr_priority = 0,
        .flags = {
            .invert_out = u->cfg.invert,
            .with_dma = 0,
            .io_loop_back = u->cfg.internal_loopback,
            .io_od_mode = 0,
        },
    };
    ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&cfg, &u->tx_ch), TAG, "new tx channel");

    rmt_copy_encoder_config_t enc_cfg = {0};
    ESP_RETURN_ON_ERROR(rmt_new_copy_encoder(&enc_cfg, &u->encoder), TAG, "new encoder");

    u->tx_sb = xStreamBufferCreate(u->cfg.tx_buffer_bytes, 1);
    u->tx_bytes = heap_caps_malloc(SU_TX_CHUNK_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    u->tx_syms = heap_caps_malloc((size_t)SU_TX_CHUNK_BYTES * 5u * sizeof(rmt_symbol_word_t),
                                  MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!u->tx_sb || !u->tx_bytes || !u->tx_syms) {
        return ESP_ERR_NO_MEM;
    }

    ESP_RETURN_ON_ERROR(rmt_enable(u->tx_ch), TAG, "enable tx channel");

    if (xTaskCreate(su_tx_task, "su_tx", SU_TX_STACK, u, SU_TX_PRIO, &u->tx_task) != pdPASS) {
        u->tx_task = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void su_tx_destroy(struct rmt_soft_uart_s *u)
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
```

- [ ] **Step 4: 在 core 里实现 `new/write/del`（RX 先留空）**

替换 `soft_uart_core.c` 全部内容：

```c
#include "rmt_soft_uart.h"

#include <string.h>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "soft_uart_internal.h"
#include "soft_uart_tx.h"

static const char *TAG = "su_core";

static void su_stop_workers(struct rmt_soft_uart_s *u)
{
    u->closing = true;

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

    return su_tx_create(u);
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
    (void)u;
    (void)buf;
    (void)len;
    (void)timeout;
    return -1; /* 任务 7 接入 RX */
}

esp_err_t rmt_soft_uart_flush_rx(rmt_soft_uart_handle_t u)
{
    (void)u;
    return ESP_ERR_NOT_SUPPORTED; /* 任务 8 实现 */
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
```

`CMakeLists.txt` 的 `srcs` 加 `src/soft_uart_tx.c`。

- [ ] **Step 5: 写临时验证 main 并编译**

`main/main.c` 追加到 `app_main`（测试代码之后）：

```c
    rmt_soft_uart_config_t cfg = {
        .tx_gpio = 4,
        .rx_gpio = 5,
        .baud = 115200,
    };
    rmt_soft_uart_handle_t u = NULL;
    ESP_ERROR_CHECK(rmt_soft_uart_new(&cfg, &u));
    const char msg[] = "U";
    int sent = rmt_soft_uart_write(u, msg, 1, pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "write returned %d", sent);
```

并在文件顶部加 `#include "rmt_soft_uart.h"`。

```powershell
. D:\Development\Espressif\frameworks\esp-idf-v5.3.2\export.ps1
idf.py build
```

Expected: 编译成功。

- [ ] **Step 6: 板上验证 TX 波形**

```powershell
idf.py -p COMx flash monitor
```

Expected: 串口打印 `write returned 1`。用逻辑分析仪或示波器看 GPIO4：115200 8N1、字节 `0x55`（'U'）的波形，起始位低、8 个数据位 LSB first、停止位高、空闲高。

如果没有逻辑分析仪，此步记为「未验证」，不要声称波形正确——任务 7 的回环测试会覆盖它。

- [ ] **Step 7: 提交**

```powershell
git add components/rmt_soft_uart main
git commit -m "feat(tx): RMT 发送通道、copy encoder 与发送任务"
```

---

### Task 6: RX 通道、IRAM 回调与解码任务

**Files:**
- Create: `components/rmt_soft_uart/src/soft_uart_rx.h`
- Create: `components/rmt_soft_uart/src/soft_uart_rx.c`
- Modify: `components/rmt_soft_uart/src/soft_uart_core.c`
- Modify: `components/rmt_soft_uart/CMakeLists.txt`

**Interfaces:**
- Consumes: `struct rmt_soft_uart_s`（Task 5）、`su_decoder_*`、`su_symbol_ring_*`
- Produces:
  - `esp_err_t su_rx_create(struct rmt_soft_uart_s *u)`
  - `void su_rx_destroy(struct rmt_soft_uart_s *u)`
  - 可用的 `rmt_soft_uart_read()`

- [ ] **Step 1: 写 RX 接口头**

`components/rmt_soft_uart/src/soft_uart_rx.h`：

```c
#pragma once

#include "esp_err.h"

struct rmt_soft_uart_s;

esp_err_t su_rx_create(struct rmt_soft_uart_s *u);
void su_rx_destroy(struct rmt_soft_uart_s *u);
```

- [ ] **Step 2: 写 RX 实现**

`components/rmt_soft_uart/src/soft_uart_rx.c`：

```c
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
```

- [ ] **Step 3: 把 RX 接进 core 的启动/停止/清理**

`soft_uart_core.c`：

1. 顶部加 `#include "soft_uart_rx.h"`。
2. `su_setup()` 末尾改为：

```c
    esp_err_t err = su_tx_create(u);
    if (err != ESP_OK) {
        return err;
    }
    return su_rx_create(u);
```

3. `su_teardown()` 改为：

```c
static void su_teardown(struct rmt_soft_uart_s *u)
{
    su_rx_destroy(u);
    su_tx_destroy(u);
}
```

4. `su_stop_workers()` 在 TX 等待之前加上 RX：

```c
    if (u->rx_task) {
        xTaskNotifyGive(u->rx_task);
        if (xSemaphoreTake(u->rx_exited, pdMS_TO_TICKS(2000)) != pdTRUE) {
            ESP_LOGW(TAG, "rx task did not exit in time");
        }
        u->rx_task = NULL;
    }
```

5. `rmt_soft_uart_read()` 改为：

```c
int rmt_soft_uart_read(rmt_soft_uart_handle_t u, void *buf, size_t len,
                       TickType_t timeout)
{
    if (!u || !buf || len == 0 || !u->rx_sb) {
        return -1;
    }
    return (int)xStreamBufferReceive(u->rx_sb, buf, len, timeout);
}
```

`CMakeLists.txt` 的 `srcs` 加 `src/soft_uart_rx.c`。

- [ ] **Step 4: 用内部回环验证收发**

`main/main.c` 的临时验证段改为：

```c
    rmt_soft_uart_config_t cfg = {
        .tx_gpio = 4,
        .rx_gpio = 4, /* 内部回环 */
        .baud = 115200,
        .internal_loopback = true,
    };
    rmt_soft_uart_handle_t u = NULL;
    ESP_ERROR_CHECK(rmt_soft_uart_new(&cfg, &u));

    const uint8_t txbuf[] = {0x55, 0x00, 0xFF, 0x7E, 0x31, 0x32, 0x33};
    int sent = rmt_soft_uart_write(u, txbuf, sizeof(txbuf), pdMS_TO_TICKS(100));
    uint8_t rxbuf[sizeof(txbuf)] = {0};
    int got = rmt_soft_uart_read(u, rxbuf, sizeof(rxbuf), pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "sent=%d got=%d match=%d", sent, got,
             (got == (int)sizeof(txbuf)) && (memcmp(txbuf, rxbuf, sizeof(txbuf)) == 0));
```

文件顶部加 `#include <string.h>`。

```powershell
. D:\Development\Espressif\frameworks\esp-idf-v5.3.2\export.ps1
idf.py build
idf.py -p COMx flash monitor
```

Expected:

```
I (xxx) app: sent=7 got=7 match=1
```

如果 `match=0`，先读 `rmt_soft_uart_get_stats()` 打印 `framing_errors / rx_overflow / rx_restarts / rx_rearm_errors`，再定位是解码错误还是丢数据；不要直接改采样点碰运气。

- [ ] **Step 5: 提交**

```powershell
git add components/rmt_soft_uart main
git commit -m "feat(rx): RMT RX 部分接收、IRAM 回调与解码任务，内部回环通过"
```

---

### Task 7: `set_baud`、`flush_rx` 与句柄生命周期加固

**Files:**
- Modify: `components/rmt_soft_uart/src/soft_uart_core.c`
- Modify: `main/main.c`（临时验证改波特率）

**Interfaces:**
- Consumes: 前 6 个任务的全部接口
- Produces: 完整可用的 `rmt_soft_uart_set_baud()` / `rmt_soft_uart_flush_rx()`

- [ ] **Step 1: 实现 `flush_rx`**

```c
esp_err_t rmt_soft_uart_flush_rx(rmt_soft_uart_handle_t u)
{
    if (!u || !u->rx_sb) {
        return ESP_ERR_INVALID_STATE;
    }
    xStreamBufferReset(u->rx_sb);
    return ESP_OK;
}
```

- [ ] **Step 2: 加固 `set_baud` 的失败路径**

```c
esp_err_t rmt_soft_uart_set_baud(rmt_soft_uart_handle_t u, uint32_t baud)
{
    if (!u || baud < RMT_SOFT_UART_BAUD_MIN || baud > RMT_SOFT_UART_BAUD_MAX) {
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
        u->cfg.baud = old_baud;
        if (su_setup(u) != ESP_OK) {
            ESP_LOGE(TAG, "restore baud %" PRIu32 " failed, handle unusable", old_baud);
        }
    }
    xSemaphoreGive(u->api_lock);
    return err;
}
```

并在文件顶部加 `#include <inttypes.h>`。

- [ ] **Step 3: 板上验证改波特率后仍可收发**

`main/main.c` 的临时段改为：

```c
    static const uint32_t seq[] = {115200, 9600, 115200};
    for (size_t i = 0; i < sizeof(seq) / sizeof(seq[0]); i++) {
        ESP_ERROR_CHECK(rmt_soft_uart_set_baud(u, seq[i]));
        vTaskDelay(pdMS_TO_TICKS(20));
        const uint8_t probe[] = {'O', 'K'};
        int sent = rmt_soft_uart_write(u, probe, sizeof(probe), pdMS_TO_TICKS(100));
        uint8_t rxbuf[2] = {0};
        int got = rmt_soft_uart_read(u, rxbuf, sizeof(rxbuf), pdMS_TO_TICKS(1000));
        ESP_LOGI(TAG, "baud=%" PRIu32 " sent=%d got=%d match=%d", seq[i], sent, got,
                 (got == 2) && (memcmp(probe, rxbuf, 2) == 0));
    }
```

```powershell
. D:\Development\Espressif\frameworks\esp-idf-v5.3.2\export.ps1
idf.py build
idf.py -p COMx flash monitor
```

Expected: 三行 `match=1`。

- [ ] **Step 4: 提交**

```powershell
git add components/rmt_soft_uart main
git commit -m "feat(core): 运行时改波特率与接收缓冲清空，失败可回滚"
```

---

### Task 8: 示例应用、Kconfig 与回环自测

**Files:**
- Create: `components/rmt_soft_uart/Kconfig`（补全）
- Create: `main/su_selftest.c`
- Create: `main/su_selftest.h`
- Rewrite: `main/main.c`
- Create: `README.md`
- Modify: `main/CMakeLists.txt`

**Interfaces:**
- Consumes: 全部公共 API
- Produces: 3 路示例（回声 + 状态打印）、`CONFIG_RMT_SOFT_UART_SELFTEST` 回环自测

- [ ] **Step 1: 补全 Kconfig**

`components/rmt_soft_uart/Kconfig`：

```
menu "RMT Soft UART"

config RMT_SOFT_UART_RUN_TESTS
    bool "Run on-target unit tests at boot"
    default n
    help
        上电先跑 codec / ring 的 Unity 用例。

config RMT_SOFT_UART_SELFTEST
    bool "Run 3-channel loopback selftest at boot"
    default n
    depends on !RMT_SOFT_UART_RUN_TESTS
    help
        上电跑 3 路回环自测，结果打印到控制台。

config RMT_SOFT_UART_CH0_TX_GPIO
    int "CH0 TX GPIO"
    default 4
config RMT_SOFT_UART_CH0_RX_GPIO
    int "CH0 RX GPIO"
    default 5
config RMT_SOFT_UART_CH1_TX_GPIO
    int "CH1 TX GPIO"
    default 6
config RMT_SOFT_UART_CH1_RX_GPIO
    int "CH1 RX GPIO"
    default 7
config RMT_SOFT_UART_CH2_TX_GPIO
    int "CH2 TX GPIO"
    default 8
config RMT_SOFT_UART_CH2_RX_GPIO
    int "CH2 RX GPIO"
    default 9
config RMT_SOFT_UART_DEMO_BAUD
    int "Demo baud rate"
    default 115200

endmenu
```

- [ ] **Step 2: 写自测模块**

`main/su_selftest.h`：

```c
#pragma once

void su_selftest_run(void);
```

`main/su_selftest.c`：

```c
#include "su_selftest.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rmt_soft_uart.h"
#include "sdkconfig.h"

static const char *TAG = "su_selftest";

typedef struct {
    const char *name;
    int tx_gpio;
    int rx_gpio;
    bool internal;
} ch_desc_t;

static uint32_t lcg_state = 0x12345678u;

static uint8_t lcg_next(void)
{
    lcg_state = lcg_state * 1664525u + 1013904223u;
    return (uint8_t)(lcg_state >> 24);
}

static bool run_case(const ch_desc_t *ch, uint32_t baud, size_t len)
{
    rmt_soft_uart_config_t cfg = {
        .tx_gpio = ch->tx_gpio,
        .rx_gpio = ch->rx_gpio,
        .baud = baud,
        .invert = false,
        .internal_loopback = ch->internal,
        .rx_buffer_bytes = 2048,
        .tx_buffer_bytes = 1024,
    };

    rmt_soft_uart_handle_t u = NULL;
    esp_err_t err = rmt_soft_uart_new(&cfg, &u);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s @%u: new failed: %s", ch->name, baud, esp_err_to_name(err));
        return false;
    }

    uint8_t *tx = malloc(len);
    uint8_t *rx = malloc(len);
    if (!tx || !rx) {
        free(tx);
        free(rx);
        rmt_soft_uart_del(u);
        return false;
    }
    for (size_t i = 0; i < len; i++) {
        tx[i] = lcg_next();
    }
    memset(rx, 0, len);

    size_t written = 0;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(2000);
    while (written < len) {
        int n = rmt_soft_uart_write(u, tx + written, len - written, pdMS_TO_TICKS(100));
        if (n <= 0) {
            break;
        }
        written += (size_t)n;
    }

    size_t got = 0;
    while (got < len && xTaskGetTickCount() < deadline) {
        int n = rmt_soft_uart_read(u, rx + got, len - got, pdMS_TO_TICKS(50));
        if (n <= 0) {
            break;
        }
        got += (size_t)n;
    }

    rmt_soft_uart_stats_t st = {0};
    rmt_soft_uart_get_stats(u, &st);
    bool ok = (written == len) && (got == len) && (memcmp(tx, rx, len) == 0) &&
              (st.framing_errors == 0) && (st.rx_overflow == 0);

    ESP_LOGI(TAG, "%s @%u %s: wrote=%u got=%u match=%d fe=%u ovf=%u restarts=%u",
             ch->name, (unsigned)baud, ok ? "PASS" : "FAIL",
             (unsigned)written, (unsigned)got,
             (got == len) && (memcmp(tx, rx, len) == 0),
             (unsigned)st.framing_errors, (unsigned)st.rx_overflow,
             (unsigned)st.rx_restarts);

    free(tx);
    free(rx);
    rmt_soft_uart_del(u);
    return ok;
}

void su_selftest_run(void)
{
    const ch_desc_t channels[] = {
        {"internal-ch0", CONFIG_RMT_SOFT_UART_CH0_TX_GPIO,
         CONFIG_RMT_SOFT_UART_CH0_TX_GPIO, true},
        {"internal-ch1", CONFIG_RMT_SOFT_UART_CH1_TX_GPIO,
         CONFIG_RMT_SOFT_UART_CH1_TX_GPIO, true},
        {"internal-ch2", CONFIG_RMT_SOFT_UART_CH2_TX_GPIO,
         CONFIG_RMT_SOFT_UART_CH2_TX_GPIO, true},
        {"wire-ch0", CONFIG_RMT_SOFT_UART_CH0_TX_GPIO, CONFIG_RMT_SOFT_UART_CH0_RX_GPIO, false},
        {"wire-ch1", CONFIG_RMT_SOFT_UART_CH1_TX_GPIO, CONFIG_RMT_SOFT_UART_CH1_RX_GPIO, false},
        {"wire-ch2", CONFIG_RMT_SOFT_UART_CH2_TX_GPIO, CONFIG_RMT_SOFT_UART_CH2_RX_GPIO, false},
    };

    int pass = 0, total = 0;
    for (size_t i = 0; i < sizeof(channels) / sizeof(channels[0]); i++) {
        total++;
        if (run_case(&channels[i], CONFIG_RMT_SOFT_UART_DEMO_BAUD, 1024)) {
            pass++;
        }
    }
    ESP_LOGI(TAG, "selftest summary: %d/%d passed", pass, total);
    ESP_LOGI(TAG, "external loopback cases require TX->RX jumper wires");
}
```

- [ ] **Step 3: 写正式示例 `main.c`**

`main/main.c`：

```c
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rmt_soft_uart.h"
#include "sdkconfig.h"
#include "su_selftest.h"
#include "su_tests.h"

static const char *TAG = "app";

typedef struct {
    const char *name;
    int tx_gpio;
    int rx_gpio;
} ch_cfg_t;

static const ch_cfg_t channels[] = {
    {"ch0", CONFIG_RMT_SOFT_UART_CH0_TX_GPIO, CONFIG_RMT_SOFT_UART_CH0_RX_GPIO},
    {"ch1", CONFIG_RMT_SOFT_UART_CH1_TX_GPIO, CONFIG_RMT_SOFT_UART_CH1_RX_GPIO},
    {"ch2", CONFIG_RMT_SOFT_UART_CH2_TX_GPIO, CONFIG_RMT_SOFT_UART_CH2_RX_GPIO},
};

static void echo_task(void *arg)
{
    rmt_soft_uart_handle_t u = arg;
    uint8_t buf[64];

    for (;;) {
        int n = rmt_soft_uart_read(u, buf, sizeof(buf), pdMS_TO_TICKS(1000));
        if (n > 0) {
            rmt_soft_uart_write(u, buf, (size_t)n, pdMS_TO_TICKS(1000));
        }
    }
}

static void stats_task(void *arg)
{
    rmt_soft_uart_handle_t *handles = arg;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        for (int i = 0; i < RMT_SOFT_UART_MAX_CHANNELS; i++) {
            rmt_soft_uart_stats_t st = {0};
            if (rmt_soft_uart_get_stats(handles[i], &st) == ESP_OK) {
                ESP_LOGI(TAG, "%s tx=%u rx=%u fe=%u brk=%u ovf=%u restarts=%u",
                         channels[i].name, (unsigned)st.tx_bytes,
                         (unsigned)st.rx_bytes, (unsigned)st.framing_errors,
                         (unsigned)st.break_errors, (unsigned)st.rx_overflow,
                         (unsigned)st.rx_restarts);
            }
        }
    }
}

void app_main(void)
{
#if CONFIG_RMT_SOFT_UART_RUN_TESTS
    int codec_failures = su_test_codec_run();
    int ring_failures = su_test_ring_run();
    ESP_LOGI(TAG, "codec tests: %d failure(s)", codec_failures);
    ESP_LOGI(TAG, "ring tests: %d failure(s)", ring_failures);
#endif

#if CONFIG_RMT_SOFT_UART_SELFTEST
    su_selftest_run();
#endif

    static rmt_soft_uart_handle_t handles[RMT_SOFT_UART_MAX_CHANNELS];

    for (int i = 0; i < RMT_SOFT_UART_MAX_CHANNELS; i++) {
        rmt_soft_uart_config_t cfg = {
            .tx_gpio = channels[i].tx_gpio,
            .rx_gpio = channels[i].rx_gpio,
            .baud = CONFIG_RMT_SOFT_UART_DEMO_BAUD,
            .rx_buffer_bytes = 2048,
            .tx_buffer_bytes = 1024,
        };
        ESP_ERROR_CHECK(rmt_soft_uart_new(&cfg, &handles[i]));
        xTaskCreate(echo_task, channels[i].name, 3072, handles[i], 8, NULL);
        ESP_LOGI(TAG, "%s started tx=%d rx=%d baud=%d", channels[i].name,
                 channels[i].tx_gpio, channels[i].rx_gpio,
                 CONFIG_RMT_SOFT_UART_DEMO_BAUD);
    }

    xTaskCreate(stats_task, "stats", 3072, handles, 4, NULL);
    ESP_LOGI(TAG, "3-channel soft UART running");
}
```

`main/CMakeLists.txt`：

```cmake
idf_component_register(SRCS "main.c" "su_selftest.c"
                       INCLUDE_DIRS "."
                       REQUIRES rmt_soft_uart)
```

`main.c` 需要 `su_tests.h`，该头在组件的 `test/` 目录，而 `INCLUDE_DIRS` 已包含 `test`，所以 `main` 能通过 `rmt_soft_uart` 的公共 include 路径找到它。若编译提示找不到，把 `su_tests.h` 移到 `components/rmt_soft_uart/include/`。

- [ ] **Step 4: 写 README**

`README.md`：

````markdown
# ESP32-S3 RMT 三路软件串口

用 RMT 外设实现 3 路独立全双工 8N1 软件串口，ESP-IDF v5.3+。

## 资源占用

- 3 × RMT TX + 3 × RMT RX（S3 共 4+4，剩 2 个）
- 每路 1 个发送任务 + 1 个接收任务
- 每路：RX symbol 环 4 KB、RX 字节缓冲 1–2 KB、TX 缓冲 0.5–1 KB

## 接线

| 通道 | TX | RX |
|---|---|---|
| ch0 | GPIO4 | GPIO5 |
| ch1 | GPIO6 | GPIO7 |
| ch2 | GPIO8 | GPIO9 |

引脚可在 `idf.py menuconfig -> Component config -> RMT Soft UART` 修改。

## 构建

```bash
. $IDF_PATH/export.sh     # Windows: export.ps1
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

## 自测

- `CONFIG_RMT_SOFT_UART_RUN_TESTS`：上电跑 codec/ring 单元测试（无需接线）
- `CONFIG_RMT_SOFT_UART_SELFTEST`：跑 3 路内部回环 + 外部回环
  - `internal-*` 用例不需要接线
  - `wire-*` 用例需要把每路 TX 与对应 RX 短接

## 限制

- 固定 8N1，无校验位/流控
- 无 DMA；建议可靠上限 460800，1 Mbaud 需实测
- 最后一字节的投递延迟上限 = `max(8 ms, 40 bit)`（115200 下约 8 ms）
- 空闲时每 8 ms 重新装载一次接收事务，`rx_restarts` 会持续增长，属正常现象
- `set_baud()` 会重建通道并丢弃在途数据；不要与其他线程的读写并发调用
````

- [ ] **Step 5: 编译**

```powershell
. D:\Development\Espressif\frameworks\esp-idf-v5.3.2\export.ps1
idf.py build
```

Expected: 编译成功，零警告（有警告必须修）。

- [ ] **Step 6: 跑内部回环自测**

```powershell
idf.py menuconfig   # 打开 SELFTEST，关闭 RUN_TESTS
idf.py -p COMx flash monitor
```

Expected:

```
I (xxx) su_selftest: internal-ch0 @115200 PASS: ... match=1 fe=0 ovf=0
I (xxx) su_selftest: internal-ch1 @115200 PASS: ...
I (xxx) su_selftest: internal-ch2 @115200 PASS: ...
I (xxx) su_selftest: wire-ch0 ... FAIL: ...   <-- 未接线时预期失败
I (xxx) su_selftest: selftest summary: 3/6 passed
```

- [ ] **Step 7: 提交**

```powershell
git add components/rmt_soft_uart main README.md
git commit -m "feat(app): 3 路回声示例、Kconfig 引脚配置与回环自测"
```

---

### Task 9: 板端验收（真实电气回环）

**Files:**
- 无新增；只改 `README.md` 的「实测结果」小节

**Interfaces:**
- Consumes: Task 8 的自测模式
- Produces: 可交付的验收记录

- [ ] **Step 1: 接好 3 组 TX→RX 短接线**

按 README 表格把 ch0/ch1/ch2 的 TX 与各自 RX 短接。

- [ ] **Step 2: 跑 115200 外部回环**

```powershell
idf.py menuconfig   # SELFTEST=1, RUN_TESTS=0, DEMO_BAUD=115200
idf.py -p COMx flash monitor
```

Expected: `selftest summary: 6/6 passed`，且每行 `fe=0 ovf=0 match=1`。

- [ ] **Step 3: 跑 9600 与 460800**

分别把 `DEMO_BAUD` 改为 9600、460800，重复 Step 2。Expected: 均为 `6/6 passed`。

- [ ] **Step 4: 反向验证（确认测试真的会失败）**

拔掉 ch0 的短接线，重跑 115200。Expected: `wire-ch0` 为 `FAIL`（`got=0` 或 `match=0`），其余 wire 用例仍 PASS。**这一步用来证明自测不是永远通过的空壳。**

- [ ] **Step 5: 记录实测结果**

在 `README.md` 末尾追加：

```markdown
## 实测结果

| 日期 | IDF | 板子 | 115200 | 9600 | 460800 | 备注 |
|---|---|---|---|---|---|---|
| 2026-xx-xx | v5.3.2 | ESP32-S3 | 6/6 | 6/6 | x/6 | 拔线可复现 FAIL |
```

- [ ] **Step 6: 提交**

```powershell
git add README.md
git commit -m "docs: 记录板端回环验收结果"
```

---

## 计划自检结果

- **Spec 覆盖**：目标/非目标（Task 1、8 的 README）、硬件约束（Task 5/6 的 `mem_block_symbols=48`、无 DMA）、时序参数（Task 2）、TX 数据流（Task 5）、RX 解码与事务生命周期（Task 3、6）、API（Task 1/5/6/7）、错误处理与释放顺序（Task 5/6/7）、验证方案（Task 2/3/4/8/9）、已知风险（README 限制）。
- **占位符**：无 TBD/TODO；每个代码步骤都有可直接粘贴的完整实现。
- **类型一致性**：`su_timing_t`、`su_rx_flags_t`、`su_decoder_*`、`su_symbol_ring_*`、`struct rmt_soft_uart_s` 字段在各任务中命名一致；`rmt_tx_wait_all_done()` 的第二个参数统一用毫秒整数。
- **Review Focus 对应测试**：① 毛刺 → `test_decode_recovers_after_glitch`；② 背靠背帧 → `test_decode_roundtrip_chunked`（连续 symbol 流，无空闲插入）；③ 溢出/重装载 → `rx_overflow`/`rx_rearm_errors` 计数 + Task 9 拔线用例；④ break 与 0x00 → `test_decode_break_is_not_a_byte`、`test_decode_0x00_is_not_break`；⑤ 运行中 set_baud → Task 7 Step 3 的三段波特率回环。
