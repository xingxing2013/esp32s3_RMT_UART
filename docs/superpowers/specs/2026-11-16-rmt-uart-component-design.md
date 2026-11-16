# RMT 软件串口组件化 — 设计说明

- 日期：2026-11-16
- 目标芯片：ESP32-S3
- ESP-IDF：v5.3.2
- 状态：设计已确认（数据位 5–8 全支持），待实现

## 1. 目标

把现有的三路软件串口重构成一个**可被其他工程引用的 IDF 组件库**：

- 组件名 `rmt_uart`，公共头 `rmt_uart.h`，API 前缀 `rmt_uart_*`
- 两级初始化：全局初始化（容量/资源） + 每路实例初始化
- 帧格式可配：数据位 5–8、校验位 无/偶/奇、停止位 1/1.5/2
- 完整读写：同步读、同步写、异步读、异步写、流式接收回调、错误回调
- 用几路初始化几路，最多 3 路；每路一个句柄

### 非目标

- 硬件流控（RTS/CTS）、RS485 方向控制、9 位数据
- 1 Mbaud 持续流的驱动级修复（见 §9）

## 2. 组件结构

```
components/rmt_uart/
  CMakeLists.txt
  Kconfig
  idf_component.yml          # 供组件管理器引用
  README.md
  include/rmt_uart.h         # 唯一公共头
  src/rmt_uart_internal.h
  src/rmt_uart_frame.c/.h    # 帧时序、编码、解码（纯逻辑）
  src/rmt_uart_ring.c/.h     # ISR 安全 symbol 环
  src/rmt_uart_tx.c
  src/rmt_uart_rx.c
  src/rmt_uart_core.c        # 全局/实例生命周期、同步异步 API
  test/                      # 板端 Unity
examples/rmt_uart_basic/     # 外部工程可直接抄的示例
```

## 3. 公共 API

```c
/* ---------- 全局 ---------- */
typedef struct {
    uint8_t max_ports;   /* 1..3，默认 3 */
} rmt_uart_global_config_t;

esp_err_t rmt_uart_global_init(const rmt_uart_global_config_t *cfg); /* NULL = 默认 */
esp_err_t rmt_uart_global_deinit(void);
uint8_t   rmt_uart_global_free_ports(void);

/* ---------- 帧格式 ---------- */
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
    uint32_t baud;              /* 300..1000000 */
    uint8_t  data_bits;         /* 5..8，默认 8 */
    rmt_uart_parity_t    parity;      /* 默认 NONE */
    rmt_uart_stop_bits_t stop_bits;   /* 默认 1 */
    bool     invert;
    size_t   rx_buffer_bytes;   /* 0 = 默认 2048 */
    size_t   tx_buffer_bytes;   /* 0 = 默认 1024 */
} rmt_uart_config_t;

typedef struct rmt_uart_s *rmt_uart_handle_t;

esp_err_t rmt_uart_init(const rmt_uart_config_t *cfg, rmt_uart_handle_t *out);
esp_err_t rmt_uart_deinit(rmt_uart_handle_t h);

/* ---------- 同步 ---------- */
int  rmt_uart_write(rmt_uart_handle_t h, const void *data, size_t len, TickType_t timeout);
int  rmt_uart_read (rmt_uart_handle_t h, void *buf, size_t len, TickType_t timeout);
esp_err_t rmt_uart_flush_rx(rmt_uart_handle_t h);
esp_err_t rmt_uart_flush_tx(rmt_uart_handle_t h);
bool rmt_uart_tx_idle(rmt_uart_handle_t h);

/* ---------- 异步 ---------- */
typedef enum {
    RMT_UART_OK = 0,
    RMT_UART_ERR_TIMEOUT,
    RMT_UART_ERR_CANCELED,
    RMT_UART_ERR_NO_MEM,
    RMT_UART_ERR_INVALID_STATE,
} rmt_uart_async_result_t;

typedef void (*rmt_uart_done_cb_t)(rmt_uart_handle_t h, rmt_uart_async_result_t result,
                                   size_t transferred, void *ctx);

esp_err_t rmt_uart_write_async(rmt_uart_handle_t h, const void *data, size_t len,
                               rmt_uart_done_cb_t cb, void *ctx);
esp_err_t rmt_uart_read_async (rmt_uart_handle_t h, void *buf, size_t len,
                               uint32_t timeout_ms, rmt_uart_done_cb_t cb, void *ctx);

/* ---------- 流式事件 ---------- */
typedef enum {
    RMT_UART_ERROR_PARITY  = 1u << 0,
    RMT_UART_ERROR_FRAMING = 1u << 1,
    RMT_UART_ERROR_OVERRUN = 1u << 2,
    RMT_UART_ERROR_BREAK   = 1u << 3,
} rmt_uart_error_t;

typedef struct {
    void (*on_rx_data)(rmt_uart_handle_t h, const uint8_t *data, size_t len, void *ctx);
    void (*on_error)(rmt_uart_handle_t h, uint32_t errors, void *ctx);
} rmt_uart_callbacks_t;

esp_err_t rmt_uart_register_callbacks(rmt_uart_handle_t h,
                                      const rmt_uart_callbacks_t *cbs, void *ctx);

/* ---------- 统计 ---------- */
typedef struct {
    uint32_t tx_bytes, rx_bytes;
    uint32_t parity_errors, framing_errors, break_errors;
    uint32_t rx_overflow, rx_restarts, tx_errors, rx_rearm_errors;
} rmt_uart_stats_t;

esp_err_t rmt_uart_get_stats(rmt_uart_handle_t h, rmt_uart_stats_t *out);
```

### 约定

- 所有回调都在驱动任务上下文执行，**不是 ISR**；回调里禁止长阻塞
- 每个实例同一时刻最多 1 个未完成的异步读 + 1 个未完成的异步写，重复提交返回 `ESP_ERR_INVALID_STATE`
- `write_async` 要求 `len <= tx_buffer_bytes` 的剩余空间，否则返回 `ESP_ERR_INVALID_SIZE`；数据在返回前已拷入内部缓冲
- `read_async` 读满 `len` 或超时完成；`transferred` 为实际字节数
- 注册了 `on_rx_data` 后，收到的数据由回调消费，同步 `read` 不再返回数据
- 同一实例不要混用同步读与异步读

## 4. 两级初始化语义

```
rmt_uart_global_init()  →  容量账本 + 全局互斥 + 引脚登记表
rmt_uart_init() × N     →  每路创建 RMT TX/RX、任务、缓冲、解码器
rmt_uart_deinit() × N
rmt_uart_global_deinit()
```

- `max_ports` 默认 3；第 4 次 `rmt_uart_init` 返回 `ESP_ERR_NOT_FOUND`
- **不在全局初始化阶段预创建 RMT 通道**：通道分辨率必须随波特率确定（300 baud 需要 313 kHz，1 M 需要 80 MHz），预创建会把分辨率锁死。因此「保留 3 路」是容量约束，不是硬件锁
- 若 RMT 已被其他驱动占满，`rmt_uart_init` 返回 `ESP_ERR_NOT_FOUND`
- 引脚在实例初始化时登记，跨实例重复使用返回 `ESP_ERR_INVALID_STATE`

## 5. 帧格式

### 5.1 时序

设数据位 `N`（5–8）、校验位 `P`（0/1）、停止位 `S`（2/3/4 个半位）：

| 位 | 位置（bit） | 采样点（bit） |
|---|---|---|
| 起始 | [0,1) | — |
| 数据 k（0..N-1） | [1+k, 2+k) | 1.5+k |
| 校验 | [1+N, 2+N) | 1.5+N |
| 停止 | [1+N+P, 1+N+P+S/2) | 1.5+N+P |

- `frame_bits_x2 = 2*(1+N+P) + S`
- `frame_ticks = (bit_ticks * frame_bits_x2 + 1) / 2`
- `sample_count = N + P + 1`（最多 10）
- 停止位只校验第一个停止位的中点；1.5/2 位只影响帧长与下一帧的起点

### 5.2 编码

- 起始位 0；数据位 LSB first；校验位（偶校验 = 数据位 1 的个数 mod 2，奇校验取反）；停止位全 1
- 输出为 `(level, duration)` 序列，两两打包成 `rmt_symbol_word_t`；最多 12 段 → 6 个 symbol/字节

### 5.3 解码与错误

- 起始沿对齐，按 §5.1 采样
- 校验不符：`parity_errors++` + `on_error(PARITY)`，**字节仍投递**
- 停止位采样为 0：`framing_errors++` + `on_error(FRAMING)`，字节仍投递（与真实 UART 一致）
- 低电平持续 > `frame_ticks`：break，丢弃当前帧
- 解码器需要「空闲高 → 起始位」才能同步；布防时线路已拉低会先报一次 break

## 6. 同步 / 异步实现

### 6.1 数据通路

```
TX: write()/write_async() → tx stream buffer → TX 任务 → 编码 → RMT
RX: RMT → IRAM 回调 → symbol 环 → RX 任务 → 解码 → rx stream buffer / on_rx_data
                                                        └→ read() / read_async()
```

### 6.2 异步写

- 申请时把数据拷入 `tx_sb`（不额外分配），记录 `{len, sent, cb, ctx}`
- TX 任务每发完一个 chunk 累加 `sent`；`sent == len` 时在 TX 任务里调用回调
- 同一时刻只允许一个未完成写

### 6.3 异步读

- 记录 `{buf, len, got, deadline_us, cb, ctx}`
- RX 任务每次解码后把 `rx_sb` 里的数据搬进请求缓冲；读满即完成
- 超时用一次性 `esp_timer` 通知 RX 任务；到点后按实际字节数完成（`got==0` 返回 `TIMEOUT`）
- 同一时刻只允许一个未完成读

### 6.4 流式回调

- 注册 `on_rx_data` 后，RX 任务把解码出的字节成批交给回调（每批 ≤64 字节），不再写入 `rx_sb`
- 错误按位或聚合后通过 `on_error` 上报

## 7. 资源与并发

- 每实例：1 TX 通道 + 1 RX 通道 + 2 任务（TX/RX） + RX symbol 环 + 两个 stream buffer
- 全局：1 互斥锁、引脚登记表、端口位图
- 最多 3 实例 = 6 个 RMT 通道（S3 共 4+4） + 6 个任务
- 任务优先级：RX 12、TX 10，可在 Kconfig 调整
- 资源释放顺序：停任务 → `rmt_disable` → 删通道 → 删编码器/缓冲 → 释放引脚 → 释放实例

## 8. 测试

### 8.1 纯逻辑（板端 Unity）

- 组合遍历：数据位 {5,6,7,8} × 校验 {N,E,O} × 停止位 {1,1.5,2} = 36 组，随机 256 字节编解码往返
- 校验错注入：翻转校验位采样 → 期望 `parity_error` 且字节仍交付
- 停止位错注入：停止位强制为低 → 期望 `framing_error` 且字节仍交付
- 保留原有用例：分块喂入、±3% 时钟偏差、长空闲拆分、break、0x00、毛刺、丢 symbol 后重同步、冷启动、末字节收尾
- 时序合法性：所有组合下 `sample_off` 递增且 < `frame_ticks`，`frame_ticks` 不超过 15 bit 可表示范围

### 8.2 回环（板端）

- 单路：8N1 / 8E1 / 8O1 / 8N2 / 7E1 / 5N1 各 1 KB 往返
- 3 路并发：8N1 @115200/460800，1 KB
- 波特率切换：8N1 与 8E1 之间切换后仍能收发
- 同步与异步 API 各跑一遍
- 异步读超时、异步写完成回调、`on_rx_data` 流式回调

### 8.3 验收

- 所有组合回环 `match=1`、`fe=0`、`parity_errors=0`、`ovf=0`
- 板端 Unity 全绿
- 编译零警告（`-Wall`）

## 9. 已知限制

- 1 Mbaud 持续流会触发 IDF 5.3.2 RMT 驱动断言
  `rmt_isr_handle_rx_done rmt_rx.c:584 (offset >= rx_chan->mem_off)`；
  短突发正常，持续高吞吐建议 ≤460800
- 3 路 460800 持续满载时双核平均 CPU 约 85–90%
- 低波特率下毛刺过滤被硬件上限削弱到 ~3.19 µs
- 无 DMA
