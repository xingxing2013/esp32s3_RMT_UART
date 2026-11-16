![build](https://github.com/xingxing2013/esp32s3_RMT_UART/actions/workflows/build.yml/badge.svg)

# ESP32-S3 RMT UART

用 RMT 外设实现的软件串口组件，最多 3 路，可被任意 ESP-IDF 工程引用。

## 特性

- 最多 **3 路**独立全双工串口，用几路初始化几路
- **每路独立配置波特率与帧格式**（已验证 9600 / 115200 / 460800 三路并发）
- 帧格式可配：数据位 **5–8**、校验 **无/偶/奇**、停止位 **1/1.5/2**
- 波特率 **300–1 000 000** 运行时可配
- **同步**（`write`/`read`）与**异步**（`write_async`/`read_async`）读写
- 流式接收回调 `on_rx_data` 与错误回调 `on_error`
- 两级初始化：全局容量管理 + 每路实例
- 完整的校验错/停止位错/break/溢出统计

## 引用方式

**方式一：拷贝组件**

把 `components/rmt_uart` 放进你的工程 `components/` 目录，然后在 `main/CMakeLists.txt`：

```cmake
idf_component_register(SRCS "main.c" REQUIRES rmt_uart)
```

**方式二：组件管理器**（已带 `idf_component.yml`）

```yaml
# main/idf_component.yml
dependencies:
  rmt_uart:
    git: https://github.com/xingxing2013/esp32s3_RMT_UART.git
    path: components/rmt_uart
    version: main
```

仓库地址：<https://github.com/xingxing2013/esp32s3_RMT_UART>

> 补丁驱动 `esp_driver_rmt` 需要另外作为同名组件提供（工程 `components/` 或 `managed_components/`），
> 组件管理器不会自动把它装进来。

完整示例见 `components/rmt_uart/examples/`（basic_echo / three_ports / frame_formats），
组件级详细说明见 `components/rmt_uart/README.md`。

## 快速开始

```c
#include "rmt_uart.h"

void app_main(void)
{
    /* 1. 全局初始化（进程内一次），默认最多 3 路 */
    ESP_ERROR_CHECK(rmt_uart_global_init(&(rmt_uart_global_config_t){.max_ports = 3}));

    /* 2. 每路一个句柄，用几路初始化几路 */
    rmt_uart_config_t cfg = {
        .tx_gpio = 42,
        .rx_gpio = 41,
        .baud = 115200,
        .data_bits = 8,
        .parity = RMT_UART_PARITY_NONE,
        .stop_bits = RMT_UART_STOP_BITS_1,
    };
    rmt_uart_handle_t uart0 = NULL;
    ESP_ERROR_CHECK(rmt_uart_init(&cfg, &uart0));

    rmt_uart_write(uart0, "hello", 5, pdMS_TO_TICKS(100));

    uint8_t buf[32];
    int n = rmt_uart_read(uart0, buf, sizeof(buf), pdMS_TO_TICKS(100)); /* 0 = 超时 */

    rmt_uart_deinit(uart0);
    rmt_uart_global_deinit();
}
```

> **关于「保留 3 路」**：RMT 通道分辨率必须随波特率确定（300 baud 需要 313 kHz，1 M 需要 80 MHz），
> 所以全局初始化不预创建通道。`max_ports` 是容量约束：第 4 次 `rmt_uart_init` 返回
> `ESP_ERR_NOT_FOUND`；如果 RMT 已被别的驱动占满，同样返回该错误。

## API

### 帧格式

```c
typedef enum {
    RMT_UART_PARITY_NONE = 0, RMT_UART_PARITY_EVEN, RMT_UART_PARITY_ODD,
} rmt_uart_parity_t;

typedef enum {
    RMT_UART_STOP_BITS_1 = 0, RMT_UART_STOP_BITS_1_5, RMT_UART_STOP_BITS_2,
} rmt_uart_stop_bits_t;
```

`data_bits` 取 5–8；校验错与停止位错都会计数并回调，但**字节仍然交付**（与真实 UART 一致）。

### 同步

```c
int  rmt_uart_write(h, const void *data, size_t len, TickType_t timeout); /* 实际写入字节数 */
int  rmt_uart_read (h, void *buf, size_t len, TickType_t timeout);        /* 0 = 超时，-1 = 错误 */
esp_err_t rmt_uart_flush_rx(h);
esp_err_t rmt_uart_flush_tx(h);
bool      rmt_uart_tx_idle(h);
```

### 异步

```c
typedef void (*rmt_uart_done_cb_t)(rmt_uart_handle_t h,
                                   rmt_uart_async_result_t result,
                                   size_t transferred, void *ctx);

rmt_uart_write_async(h, data, len, cb, ctx);   /* len ≤ TX 缓冲剩余空间 */
rmt_uart_read_async (h, buf, len, timeout_ms, cb, ctx);
```

- 每个实例同一时刻只允许 1 个未完成异步读 + 1 个未完成异步写
- `write_async` 在返回前把数据拷入内部缓冲
- `read_async` 读满或超时完成，`transferred` 是实际字节数
- 回调在驱动任务上下文执行（不是 ISR）

### 流式回调

```c
typedef struct {
    void (*on_rx_data)(h, const uint8_t *data, size_t len, void *ctx);
    void (*on_error)(h, uint32_t errors, void *ctx);   /* RMT_UART_ERROR_* 位或 */
} rmt_uart_callbacks_t;

rmt_uart_register_callbacks(h, &cbs, ctx);
```

注册 `on_rx_data` 后数据由回调消费，同步 `read` 不再返回数据，也不能再挂异步读。

### 统计

```c
rmt_uart_stats_t st;
rmt_uart_get_stats(h, &st);
/* tx_bytes / rx_bytes / parity_errors / framing_errors / break_errors
   rx_overflow / rx_restarts / tx_errors / rx_rearm_errors */
```

## 帧结构

```
空闲高 | 起始0 | D0 D1 ... D(N-1) | [校验] | 停止1 | [停止2/1.5]
```

- 数据 LSB first
- 偶校验 = 数据位中 1 的个数 mod 2；奇校验取反
- 停止位只校验第一个停止位的中点，1.5/2 位只影响帧长

## 自测

`idf.py menuconfig -> Component config -> RMT UART`：

- `Run on-target unit tests at boot`：19 个 Unity 用例（36 组帧格式组合、校验错、停止位错、时钟偏差、长空闲、break、毛刺等）
- `Run 3-channel loopback selftest at boot`：14 项板端回环

### 实测结果（ESP32-S3 rev v0.2，IDF v5.3.2，GPIO42/41、40/39、38/37 短接）

| 项目 | 结果 |
|---|---|
| 板上 Unity | **19/19 通过** |
| 板端自测矩阵 | **14/14 通过** |
| 8N1 / 8E1 / 8O1 / 8N2 / 7E1 / 5N1.5 / 6O2 | 全部 256 字节 `match=1, fe=0, pe=0` |
| 3 路并发 8N1 @115200 | PASS |
| 异步写 + 异步读 | PASS |
| `on_rx_data` 流式回调 | PASS |
| 端口上限（第 4 路） | 正确返回 `ESP_ERR_INVALID_STATE` |
| 9600 ↔ 115200 切换 | PASS |
| 1 Mbaud 短突发 1 KB | PASS |

## DMA 说明

`rmt_uart_config_t.use_dma` 可打开 RMT DMA。硬件限制（ESP32-S3 `rmt_ll.h`）：

- **只有 RMT 最后一个通道（ch3）具备 DMA 能力**（`HAL_ASSERT(channel == 3)`）
- DAC 通道在实例化时分配，所以 **3 路里最多 1 路能用 DMA**，且必须抢在别的实例之前
- 打开后 `mem_block_symbols` 从 48 提到 512（DMA 缓冲）

**对 CPU 的预期收益很小**：实测 1 Mbaud 每路 `su_rx` 约 18%、`su_tx` 约 5%，
开销主要在解码任务的逐 symbol 采样，而不是 RMT 搬运；DMA 只省掉 ISR 里的
72-symbol memcpy（约每 0.6 ms 一次）。持续负载下的 DMA 对比被上面的驱动断言打断，
未能给出可信数字——需要先修驱动。

## 性能优化（解码 CPU）

解码器基准（`test_frame.c` 里的 cycles/byte 用例，8N1 @115200 时序，ESP32-S3 160 MHz）：

| 版本 | 原始 symbol（10 段/字节） | 合并 run 后（真实 RMT 流） |
|---|---|---|
| `-Og` 基线 | 1632.8 cycles/byte | — |
| 仅开 `-O2` | 1451.6 | — |
| + 32 位 tick + 增量采样时间 + 内联 | 943.7 | **670.6**（随机数据） |
| 同上，`0xFF` 数据 | 959.9 | **471.8** |

累计降低 **约 59%**（真实流 1632 → 670）。对应估算：

- 1 Mbaud（100 KB/s/路）：约 **67 M cycles/s ≈ 42%** 单核/路（优化前会超过 100%）
- 460800（46 KB/s/路）：约 **19%** 单核/路

做法：

1. `-O2` 固化在组件内（`target_compile_options(... -O2)`），宿主工程用 `-Og` 也不影响
2. 解码器时间戳从 `uint64_t` 改成 `uint32_t` + 回绕安全比较，去掉 64 位加减
3. 采样点时间改为增量维护（`next_sample_ts`），不再每个采样点重算
4. 热路径 `always_inline`
5. **批量投递**：RX 侧每 ≤64 字节才调用一次 `xStreamBufferSend`，不再每字节一次

## 驱动补丁（持续流必需）

IDF 5.3.2 的 `esp_driver_rmt` 在「partial RX + 乒乓回绕 + 空闲结束事务」时，
`mem_off`（0/24）与硬件写指针（可能已回绕到 0~23）不一致，DONE 处理里直接断言。

本仓库在 `components/esp_driver_rmt/` 放了一份 IDF 该组件的副本并打了补丁：

- DONE 处理改成回绕感知：`offset < mem_off` 时按两段计算长度并分段拷贝
- 其余逻辑与 IDF 原版一致

**覆盖方式**：IDF 构建系统会用工程内同名组件覆盖内置组件（已实测，编译路径指向
`components/esp_driver_rmt/src/rmt_rx.c`），不需要改 IDF 安装目录。

**发布方式**：`rmt_uart` 组件包**不能**把补丁放进自己内部——一个组件包只能提供一个组件，
覆盖必须是一个名为 `esp_driver_rmt` 的兄弟组件。所以按以下任一种方式分发：

1. 随仓库一起提供两个组件：`components/rmt_uart` + `components/esp_driver_rmt`（当前做法）
2. 把补丁驱动单独打成组件包，安装后目录名必须仍是 `esp_driver_rmt`

### 补丁后的持续流实测（3 路并发，10 s）

| 波特率 | 崩溃 | 吞吐 | CPU | 帧错误 |
|---|---|---|---|---|
| 115200 | 无 | 31.8 KB/s | 16.9% | 26（0.008%） |
| 460800 | 无 | 104.7 KB/s | 65.8% | 1149（0.1%） |
| 1 M | 无 | 132.9 KB/s | 88.0% | 193（0.014%） |

## 限制

- 持续流依赖上面的补丁驱动；不带补丁时，IDF 原版驱动在任何波特率的持续流下都会断言重启，
  只有 KB 级短突发安全
- 1 M 三路持续流已接近 CPU 上限（88%），吞吐约 44 KB/s/路
- 低波特率（≤ ~62.7 k）毛刺过滤被硬件上限削弱到约 3.19 µs
- 无 DMA、无硬件流控、无 9 位数据
- 解码器需要线路先出现空闲高电平才能同步；布防时线路已拉低会先报一次 break

## 目录

```
components/rmt_uart/
  README.md              组件级详细说明（引用方式、API、开发说明）
  include/rmt_uart.h     公共 API
  src/                   帧编解码、ring、TX/RX、核心
  test/                  板端 Unity
  examples/              三个可独立编译的例程
    basic_echo/          单路 8N1 同步回声
    three_ports/         三路不同格式，同步/异步/回调
    frame_formats/       5-8 数据位、校验、停止位配置
components/esp_driver_rmt/  补丁版 IDF 驱动（持续流必需）
```
