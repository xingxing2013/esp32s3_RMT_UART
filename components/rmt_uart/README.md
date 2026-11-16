# rmt_uart — ESP32-S3 RMT 软件串口组件

用 RMT 外设把串口数量扩到 **最多 3 路**（S3 硬件只有 UART0/1/2，且 UART0 通常给 console）。
每路独立配置引脚、波特率、数据位、校验位、停止位，支持同步与异步读写。

- 数据位 **5–8**、校验 **无/偶/奇**、停止位 **1/1.5/2**
- 波特率 **300–1 000 000**，运行时可改
- 同步 `read/write`、异步 `read_async/write_async`、流式 `on_rx_data`、错误回调
- 最多 3 路，用几路初始化几路，每路一个句柄

## 依赖

| 项 | 要求 |
|---|---|
| ESP-IDF | **>= 5.3**（使用 `esp_driver_rmt` 新驱动） |
| 芯片 | ESP32-S3（其他芯片未验证） |
| 补丁驱动 | **持续收发必须**，见下方「驱动补丁」 |

## 引用方式

### 方式 A：整仓拷贝（最简单）

把 `components/rmt_uart` **和** `components/esp_driver_rmt` 一起拷到你的工程 `components/` 下：

```
your_project/
  components/
    rmt_uart/
    esp_driver_rmt/      <- 补丁驱动，覆盖 IDF 内置组件
  main/
```

`main/CMakeLists.txt`：

```cmake
idf_component_register(SRCS "main.c" INCLUDE_DIRS "." REQUIRES rmt_uart)
```

### 方式 B：EXTRA_COMPONENT_DIRS

组件放在工程外时：

```cmake
set(EXTRA_COMPONENT_DIRS "/path/to/components")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
```

### 方式 C：组件管理器

```yaml
# main/idf_component.yml
dependencies:
  rmt_uart:
    git: https://github.com/xingxing2013/esp32s3_RMT_UART.git
    path: components/rmt_uart
    version: main
```

> 组件管理器一个包只能提供一个组件。补丁驱动 `esp_driver_rmt` 必须另外作为
> 一个同名组件提供（`managed_components/esp_driver_rmt` 或工程 `components/esp_driver_rmt`），
> 不能塞进 `rmt_uart` 包内部。

### 驱动补丁（持续流必需）

IDF 5.3.2 的 `esp_driver_rmt` 在「partial RX + 乒乓回绕 + 空闲结束事务」时，
`rmt_isr_handle_rx_done` 会用 `offset - mem_off` 计算长度并单段拷贝；当硬件写指针已经
回绕（`offset < mem_off`）时触发断言 `offset >= rx_chan->mem_off`，设备重启。

`components/esp_driver_rmt` 是 IDF 该组件的副本，仅改了这一处：

- 长度按回绕计算：`total = wrapped ? (block - mem_off + offset) : (offset - mem_off)`
- 拷贝分两段：`[mem_off, block)` 与 `[0, offset)`

**验证方式**：编译日志里应看到
`-c .../components/esp_driver_rmt/src/rmt_rx.c`，说明本地组件已覆盖 IDF 内置组件。

不带补丁时：KB 级短突发正常，持续流会在任意波特率下断言重启。

## 快速开始

```c
#include "rmt_uart.h"

void app_main(void)
{
    ESP_ERROR_CHECK(rmt_uart_global_init(&(rmt_uart_global_config_t){.max_ports = 3}));

    rmt_uart_config_t cfg = {
        .tx_gpio = 42,
        .rx_gpio = 41,
        .baud = 115200,
        .data_bits = 8,
        .parity = RMT_UART_PARITY_NONE,
        .stop_bits = RMT_UART_STOP_BITS_1,
    };
    rmt_uart_handle_t uart = NULL;
    ESP_ERROR_CHECK(rmt_uart_init(&cfg, &uart));

    rmt_uart_write(uart, "hello", 5, pdMS_TO_TICKS(100));

    uint8_t buf[32];
    int n = rmt_uart_read(uart, buf, sizeof(buf), pdMS_TO_TICKS(100)); /* 0 = 超时 */
}
```

## API

### 初始化

```c
typedef struct { uint8_t max_ports; } rmt_uart_global_config_t;  /* 1..3，0 = 3 */

esp_err_t rmt_uart_global_init(const rmt_uart_global_config_t *cfg); /* NULL = 默认 */
esp_err_t rmt_uart_global_deinit(void);
uint8_t   rmt_uart_global_free_ports(void);

esp_err_t rmt_uart_init(const rmt_uart_config_t *cfg, rmt_uart_handle_t *out);
esp_err_t rmt_uart_deinit(rmt_uart_handle_t h);
esp_err_t rmt_uart_set_baud(rmt_uart_handle_t h, uint32_t baud);
```

| 配置字段 | 说明 |
|---|---|
| `tx_gpio` / `rx_gpio` | 必须不同；IDF 的 `gpio_config()` 会把同脚输出改回普通 GPIO，无法内部回环 |
| `baud` | 300–1 000 000 |
| `data_bits` | 5–8，0 = 8 |
| `parity` | `RMT_UART_PARITY_NONE/EVEN/ODD` |
| `stop_bits` | `RMT_UART_STOP_BITS_1/1_5/2` |
| `invert` | 收发同时反相 |
| `rx_buffer_bytes` / `tx_buffer_bytes` | 0 = 2000 / 1000 左右默认值 |
| `use_dma` | 仅 RMT ch3 支持，最多 1 路可用；收益有限 |

`max_ports` 是**容量约束**：第 4 次 `rmt_uart_init` 返回 `ESP_ERR_NOT_FOUND`。
全局初始化不预创建 RMT 通道（分辨率必须随波特率确定），所以它不是硬件级锁定。

### 同步

```c
int  rmt_uart_write(h, const void *data, size_t len, TickType_t timeout); /* 实际字节数 */
int  rmt_uart_read (h, void *buf, size_t len, TickType_t timeout);        /* 0 超时，-1 错误 */
esp_err_t rmt_uart_flush_rx(h);
esp_err_t rmt_uart_flush_tx(h);
bool      rmt_uart_tx_idle(h);
```

### 异步

```c
typedef void (*rmt_uart_done_cb_t)(rmt_uart_handle_t h,
                                   rmt_uart_async_result_t result,
                                   size_t transferred, void *ctx);

esp_err_t rmt_uart_write_async(h, const void *data, size_t len,
                               rmt_uart_done_cb_t cb, void *ctx);
esp_err_t rmt_uart_read_async (h, void *buf, size_t len, uint32_t timeout_ms,
                               rmt_uart_done_cb_t cb, void *ctx);
```

- 每个实例同一时刻最多 1 个未完成异步读 + 1 个未完成异步写，重复提交返回 `ESP_ERR_INVALID_STATE`
- `write_async` 在返回前把数据拷入内部缓冲，`len` 不能超过 TX 缓冲剩余空间
- `read_async` 读满或超时完成，`transferred` 是实际字节数
- 回调在驱动任务上下文执行，不是 ISR

### 流式回调与错误

```c
typedef struct {
    void (*on_rx_data)(h, const uint8_t *data, size_t len, void *ctx);
    void (*on_error)(h, uint32_t errors, void *ctx);
} rmt_uart_callbacks_t;

esp_err_t rmt_uart_register_callbacks(h, &cbs, ctx);
```

- `RMT_UART_ERROR_PARITY / FRAMING / OVERRUN / BREAK` 位或
- 注册 `on_rx_data` 后数据由回调消费，同步 `read` 不再返回数据，也不能再挂异步读

### 统计

```c
rmt_uart_stats_t st;
rmt_uart_get_stats(h, &st);
/* tx_bytes rx_bytes parity_errors framing_errors break_errors
   rx_overflow rx_restarts tx_errors rx_rearm_errors */
```

## 帧结构

```
空闲高 | 起始0 | D0 D1 ... D(N-1) | [校验] | 停止1 | [停止2/1.5]
```

- 数据 LSB first；偶校验 = 数据位 1 的个数 mod 2，奇校验取反
- 校验错 / 停止位错都会计数并回调，但**字节仍然交付**（与真实 UART 一致）
- 停止位只校验第一个停止位的中点

## 线程与缓冲模型

```
TX: write()/write_async() -> StreamBuffer -> TX 任务 -> 编码 -> RMT
RX: RMT -> IRAM 回调 -> symbol 环 -> RX 任务 -> 解码 -> StreamBuffer / on_rx_data
```

- 每路 2 个任务：RX 优先级 12、TX 优先级 10（Kconfig 可调）
- 每路资源：1 TX + 1 RX RMT 通道、symbol 环 4 KB、RX/TX StreamBuffer
- 同一实例不要混用同步读与异步读；`set_baud`/`deinit` 不要与其他线程的读写并发

## 性能

解码器基准（8N1 时序，160 MHz，`test_frame.c` 的 cycles/byte 用例）：

| 版本 | 逐 bit 段 | 合并 run（真实 RMT 流） |
|---|---|---|
| `-Og` | 1632.8 | — |
| `-O2` | 1451.6 | — |
| +32 位 tick +增量采样 +内联 | 943.7 | **670.6**（随机） / 471.8（0xFF） |
| +采样循环分相 + 恒定 1 bit 步进 | 914.3 | **619.2** / 387.0 |
| +`-O3`（组件内固定） | 822.9 | 554.3 / 373.8 |
| +长 run 位掩码批量置位 | **835.3** | **536.4** / **221.4** |

换算：1 Mbaud 约 34% 单核/路，460800 约 15% 单核/路。

3 路并发持续 10 s（补丁驱动）：

| 波特率 | 吞吐 | CPU | 帧错误 |
|---|---|---|---|
| 115200 | 31.8 KB/s | 16.9% | 0.008% |
| 460800 | 104.7 KB/s | 65.8% | 0.1% |
| 1 M | 132.9 KB/s | 88.0% | 0.014% |

## 例程

`examples/` 下三个可独立编译的工程（`idf.py set-target esp32s3 && idf.py build`）：

| 例程 | 内容 |
|---|---|
| `basic_echo` | 单路 8N1 同步回声，最小用法 |
| `three_ports` | 三路不同格式/波特率，同步 / 异步 / 流式回调三种写法 |
| `frame_formats` | 5–8 数据位、校验位、停止位配置，运行时 `set_baud` |

## 测试

- `CONFIG_RMT_UART_RUN_TESTS`：板上 Unity，19 个用例（36 组帧格式组合、校验/停止位错注入、时钟偏差、长空闲、break、毛刺、分块喂入、cycles/byte 基准）
- `CONFIG_RMT_UART_SELFTEST`：板上回环 15 项（三种格式矩阵、异步、回调、端口上限、3 路并发、混合波特率、切换）

## 开发说明

| 文件 | 职责 |
|---|---|
| `src/rmt_uart_frame.c` | 帧时序、编码、解码（纯逻辑，无硬件依赖） |
| `src/rmt_uart_ring.c` | ISR 安全 SPSC symbol 环 |
| `src/rmt_uart_tx.c` | RMT TX 通道、copy encoder、发送任务、异步写计数 |
| `src/rmt_uart_rx.c` | RMT RX 通道、IRAM 回调、解码任务、批量投递 |
| `src/rmt_uart_core.c` | 全局/实例生命周期、端口与引脚管理、同步异步 API |

扩展建议：

- 新增帧格式：只改 `rmt_uart_frame.c` 的时序/编码/解码，`test_frame.c` 的组合测试会自动覆盖
- 调优先级/缓冲：`Kconfig`
- 验证解码性能：`idf.py -T` 跑 Unity，看 `bench:` 日志的 cycles/byte

## 已知限制

- 持续流依赖补丁驱动；不带补丁只有短突发安全
- 1 M 三路持续流接近 CPU 上限（88%），约 44 KB/s/路
- 低波特率（≤ ~62.7 k）毛刺过滤被硬件上限削弱到约 3.19 µs
- 无硬件流控、无 9 位数据
- 解码器需要线路先出现空闲高电平才能同步；布防时线路已拉低会先报一次 break
