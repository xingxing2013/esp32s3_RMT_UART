# RMT UART 组件化 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans (inline) to implement this plan task-by-task.

**Goal:** 把三路软件串口重构为可复用 IDF 组件 `rmt_uart`，支持两级初始化、5–8 数据位、校验位、停止位、同步/异步读写。

**Architecture:** 保留现有「RMT + symbol 环 + 解码状态机」内核，重命名为 `rmt_uart_*`；帧时序参数化（数据位/校验/停止位）；新增全局容量管理与异步请求状态机。

**Tech Stack:** ESP-IDF v5.3.2、`esp_driver_rmt`、FreeRTOS stream buffer、esp_timer、Unity。

**Spec:** `docs/superpowers/specs/2026-11-16-rmt-uart-component-design.md`

## Global Constraints

- ESP-IDF **v5.3.2**，目标 esp32s3；编译零警告
- 最多 **3 路**实例；每路 1 TX + 1 RX RMT 通道
- 波特率 300–1 000 000；数据位 5–8；校验 N/E/O；停止位 1/1.5/2
- 所有回调在任务上下文；每个实例最多 1 个未完成异步读 + 1 个未完成异步写
- 1 Mbaud 持续流存在 IDF 驱动断言限制，文档保留
- 硬件：GPIO42/41、40/39、38/37 已短接，COM35

## Task 1: 重命名为 rmt_uart 组件

**Files:**
- Move: `components/rmt_soft_uart/` → `components/rmt_uart/`
- Rename: `soft_uart_*` → `rmt_uart_*`，`su_*` → `ru_*`
- Modify: `main/*`, `README.md`

**接口：**
- 公共头 `rmt_uart.h`；类型 `rmt_uart_handle_t`；函数 `rmt_uart_init/deinit/write/read/...`

步骤：
1. `git mv components/rmt_soft_uart components/rmt_uart`
2. 重命名源文件：`soft_uart_codec.* → rmt_uart_frame.*`、`symbol_ring.* → rmt_uart_ring.*`、`soft_uart_tx/rx/core.* → rmt_uart_*`
3. 全局替换符号：`rmt_soft_uart_` → `rmt_uart_`、`su_` → `ru_`、`SU_` → `RU_`、`RMT_SOFT_UART_` → `RMT_UART_`
4. `idf_component_register` 源列表更新；`main` 的 REQUIRES 改为 `rmt_uart`
5. `idf.py build` 通过、零警告；板上自测仍 5/5

## Task 2: 帧格式参数化

**Files:**
- Modify: `src/rmt_uart_frame.h/.c`
- Modify: `test/su_test_codec.c` → `test/test_frame.c`

**接口：**
```c
typedef struct {
    uint8_t  data_bits;      /* 5..8 */
    uint8_t  parity_bits;    /* 0/1 */
    uint8_t  stop_bits_x2;   /* 2/3/4 */
    uint32_t bit_ticks;
    uint32_t frame_ticks;
    uint8_t  sample_count;
    uint32_t sample_off[10];
} ru_timing_t;

typedef struct {
    bool framing_error;
    bool parity_error;
    bool break_error;
} ru_rx_flags_t;

esp_err_t ru_timing_init(uint32_t baud, uint8_t data_bits, rmt_uart_parity_t parity,
                         rmt_uart_stop_bits_t stop, uint32_t *res_hz, ru_timing_t *out);
size_t ru_encode(const uint8_t *data, size_t len, const ru_timing_t *t,
                 rmt_symbol_word_t *out, size_t out_words);
```

实现要点：
- `frame_bits_x2 = 2*(1 + N + P) + S`
- `frame_ticks = (bit_ticks * frame_bits_x2 + 1) / 2`
- 采样点：数据 k → `bit*(2k+3)/2`；校验 → `bit*(2N+3)/2`；停止 → `bit*(2(N+P)+3)/2`
- 编码：level/duration 数组，起点 0、数据 LSB first、校验偶/奇、停止全 1；两两打包；末尾不足一对时 `duration1=0`
- 解码：按 `sample_count` 采样；数据 LSB 组装；校验比对；停止位采样；错误置位后仍投递
- break 判定改为 `run_ticks > frame_ticks`

测试（`test_frame.c`）：
```c
static const uint8_t  data_bits[] = {5, 6, 7, 8};
static const rmt_uart_parity_t parities[] = {NONE, EVEN, ODD};
static const rmt_uart_stop_bits_t stops[] = {1, 1.5, 2};
/* 36 组组合：随机 256 字节编码→解码，要求字节完全一致、无错误标志 */
```
另加：校验位翻转 → `parity_error` 且字节交付；停止位强制低 → `framing_error` 且字节交付。

## Task 3: 全局初始化与端口管理

**Files:**
- Modify: `src/rmt_uart_core.c`
- Modify: `include/rmt_uart.h`

**接口：**
```c
typedef struct { uint8_t max_ports; } rmt_uart_global_config_t;
esp_err_t rmt_uart_global_init(const rmt_uart_global_config_t *cfg);
esp_err_t rmt_uart_global_deinit(void);
uint8_t   rmt_uart_global_free_ports(void);
```

实现：
- 静态 `s_global = { inited, max_ports, used_mask, lock }`
- `rmt_uart_global_init(NULL)` → `max_ports=3`
- 端口位图分配；`rmt_uart_init` 先占位，失败回滚
- 引脚登记表保留（`tx_gpio == rx_gpio` 直接报错）
- `deinit` 释放位图与引脚；全部释放后允许 `global_deinit`

测试：全局 init 前 `rmt_uart_init` 返回 `ESP_ERR_INVALID_STATE`；4 路时第 4 路返回 `ESP_ERR_NOT_FOUND`。

## Task 4: 同步 API 补齐

**Files:** `src/rmt_uart_core.c`, `include/rmt_uart.h`

新增/调整：
```c
esp_err_t rmt_uart_flush_tx(h);   /* xStreamBufferReset(tx_sb) */
bool      rmt_uart_tx_idle(h);    /* tx_sb 空 且 无在途事务 */
esp_err_t rmt_uart_flush_rx(h);
```

`write`/`read` 语义不变（返回字节数，-1 错误，0 超时）。

## Task 5: 异步读写与回调

**Files:**
- Modify: `src/rmt_uart_internal.h`（请求结构）
- Modify: `src/rmt_uart_tx.c`、`src/rmt_uart_rx.c`、`src/rmt_uart_core.c`

结构：
```c
typedef struct {
    uint8_t *buf; size_t len, got;
    rmt_uart_done_cb_t cb; void *ctx;
    int64_t deadline_us;
    esp_timer_handle_t timer;
    bool active;
} ru_async_read_t;

typedef struct {
    size_t len, sent;
    rmt_uart_done_cb_t cb; void *ctx;
    bool active;
} ru_async_write_t;
```

- `write_async`：`xStreamBufferSend(tx_sb, data, len, 0)` 必须一次成功，否则 `ESP_ERR_INVALID_SIZE`；置 `async_write`
- TX 任务每发完一块 `async_write.sent += got`，达到 `len` 后调用回调（先清状态再回调）
- `read_async`：记录请求 + 启动一次性 `esp_timer`（回调里 `xTaskNotifyGive(rx_task)`）
- RX 任务每次唤醒：先处理超时，再从 `rx_sb` 搬数据到请求缓冲；读满完成
- 结束回调前清 `active`，防止重入
- 注册 `on_rx_data` 后，RX 任务把解码结果成批（≤64 B）交给回调，不写 `rx_sb`

测试：异步写 1 KB 完成回调、异步读 1 KB、异步读 50 ms 超时返回部分长度、`on_rx_data` 收到完整流。

## Task 6: 自测矩阵、示例与文档

**Files:**
- Modify: `main/su_selftest.c` → `main/rmt_uart_selftest.c`
- Create: `examples/rmt_uart_basic/`
- Create: `components/rmt_uart/idf_component.yml`
- Modify: `README.md`、`components/rmt_uart/Kconfig`

- 自测用例：8N1/8E1/8O1/8N2/7E1/5N1 单路 + 3 路并发 + 异步读写 + 波特率切换
- Kconfig 增加默认帧格式、默认缓冲、任务优先级
- `idf_component.yml`：`version: "1.0.0"`、`dependencies: idf: ">=5.3"`
- README 给出外部工程引用方式与 API 示例

## Task 7: 板上验证

1. `idf.py build` 零警告
2. 烧录，跑自测矩阵，要求全 PASS、`fe=0`、`parity_errors=0`、`ovf=0`
3. 跑 20 个 codec/ring 单测 + 异步用例
4. 记录到 README 实测表
