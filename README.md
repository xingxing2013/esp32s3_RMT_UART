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
dependencies:
  rmt_uart:
    path: ../components/rmt_uart     # 或 registry 地址
```

完整示例见 `examples/rmt_uart_basic/`。

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

## 限制

- **任何波特率下的持续流**都可能触发 IDF 5.3.2 RMT 驱动断言
  `rmt_isr_handle_rx_done rmt_rx.c:584 (offset >= rx_chan->mem_off)` 并重启；
  实测 115200、460800、1 M 的持续压测都会命中，短突发（KB 级）稳定
- 该断言与波特率无关，是「partial RX + 乒乓回绕 + 空闲结束事务」的驱动缺陷；
  需要持续流时必须给 IDF 打补丁或 vendor 一份修过的 `esp_driver_rmt`
- 3 路 460800 持续满载时双核平均 CPU 约 85–90%
- 低波特率（≤ ~62.7 k）毛刺过滤被硬件上限削弱到约 3.19 µs
- 无 DMA、无硬件流控、无 9 位数据
- 解码器需要线路先出现空闲高电平才能同步；布防时线路已拉低会先报一次 break

## 目录

```
components/rmt_uart/
  include/rmt_uart.h     公共 API
  src/                   帧编解码、ring、TX/RX、核心
  test/                  板端 Unity
  examples/rmt_uart_basic 最小示例
```
