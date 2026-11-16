# ESP32-S3 RMT 三路软件串口 — 设计说明

- 日期：2026-11-16
- 目标芯片：ESP32-S3
- ESP-IDF：v5.3.2（新 RMT 驱动 `esp_driver_rmt`）
- 状态：设计已确认，待实现

## 1. 目标

用 ESP32-S3 的 RMT 外设实现 **3 路独立的全双工软件串口**，API 形态接近真实 UART：

- 每路独立 TX/RX 引脚，互不影响
- 波特率运行时可配（300 ~ 1 000 000），帧格式固定 **8N1、LSB first**
- 发送走缓冲队列（非阻塞写），接收走环形缓冲（阻塞读 + 超时）
- 提供错误统计，便于现场判断丢字节、帧错误

### 非目标（明确不做）

- 校验位、停止位、数据位可配
- 硬件流控（RTS/CTS）、RS485 方向控制
- RMT DMA（S3 只有 ch3 具备 DMA 能力，3 路无法对称使用）
- 与真实 UART 外设的引脚复用或中断共享

## 2. 硬件约束（已核实 soc_caps.h / rmt_ll.h）

| 项目 | ESP32-S3 | 影响 |
|---|---|---|
| RMT 组数 | 1 | 所有通道共享一个组 |
| TX / RX 通道 | 各 4 个（共 8 个物理通道） | 3 路全双工占 6 个，剩 2 个 |
| 每通道存储 | 48 个 symbol（192 字节） | 决定乒乓粒度 |
| symbol 时长位宽 | 15 bit（≤32767 tick） | 长电平会被硬件拆成多个 symbol，解码器必须能跨 symbol 续接 |
| DMA 能力 | 仅 channel 3 | 3 路不可能全 DMA；本设计不用 DMA |
| RX 乒乓 | 支持 | 用于减少中断次数 |

通道编号由驱动自动分配，**无法指定通道号**，因此「保留 ch3 给未来 DMA」只是容量上的余量，不是可强制的预留。

## 3. 总体架构

```
        ┌──────────── 每路一套 ────────────┐
TX GPIO ─ RMT TX ch ─ copy encoder ─ 发送任务
                                        ▲
                                  字节 StreamBuffer（用户 write）

RX GPIO ─ RMT RX ch ─ IRAM 回调（仅拷贝）
                          │
                    symbol 环形队列（SPSC）
                          │
                      解码任务 ── 字节 StreamBuffer ── 用户 read()
```

- 纯逻辑（编码/解码/分辨率计算）与硬件操作分离，便于板端单元测试。
- 每路 2 个任务（TX、RX），共 6 个任务；互不共享状态，除全局 RMT 资源分配外无锁竞争。

## 4. 组件结构与文件职责

```
components/rmt_soft_uart/
  CMakeLists.txt             依赖 esp_driver_rmt、freertos（StreamBuffer 自带）
  Kconfig                    默认波特率、缓冲大小、任务优先级、自测开关
  include/rmt_soft_uart.h    公共 API、配置结构、统计结构
  src/soft_uart_codec.h      纯逻辑接口：分辨率选择、编码、解码
  src/soft_uart_codec.c      上述实现（不直接操作硬件）
  src/symbol_ring.h/.c       ISR 安全的 SPSC symbol 环形队列
  src/soft_uart_tx.c         TX 通道/编码器生命周期、发送任务
  src/soft_uart_rx.c         RX 通道/回调/解码任务、重装载
  src/soft_uart_core.c       new/del/set_baud、参数校验、统计、失败清理
main/
  main.c                     3 路示例：回声 + 每秒状态打印
```

每个 `.c` 控制在单一职责内，估计 150~300 行。

## 5. 时序参数

### 5.1 分辨率选择（纯函数）

```
N          = clamp(round(1e6 / baud), 1, 255)
resolution = 80MHz / N
bit_ticks  = round(resolution / baud)
误差上界   = 0.5 / bit_ticks（每 bit 量化误差）
```

| 波特率 | N | 分辨率 | tick/bit | 每 bit 误差 |
|---|---|---|---|---|
| 300 | 255 | 313.7 kHz | 1046 | 0.05% |
| 9600 | 104 | 769.2 kHz | 80 | 0.6% |
| 115200 | 9 | 8.889 MHz | 77 | 0.6% |
| 460800 | 2 | 40 MHz | 87 | 0.6% |
| 1 000 000 | 1 | 80 MHz | 80 | 0.6% |

采样点在 bit 中点（1.5、2.5 … 9.5 bit），单帧最多累积 10 bit 的量化误差，合计 ≤6%，远小于 ±50% 的采样余量。不使用跨帧累积，每帧由起始沿重新对时。

### 5.2 缓冲与中断频率

| 参数 | 值 | 说明 |
|---|---|---|
| RX 硬件块 | `mem_block_symbols = 48`（1 块） | 3 路 RX 各占 1 块，不能给单路多块 |
| RMT RX 用户缓冲 | 96 symbol（384 B） | 开 `en_partial_rx`，回调约每 72 symbol 触发 |
| 回调频率 | ≈0.6 ms @115200 | 一次拿到约 72 个 symbol |
| symbol 环形队列 | 1024 symbol（4 KB）/路 | 115200 下约 9 ms 余量，300 波特下约 3.4 s |
| 字节 RX 缓冲 | 1024 B/路（可配） | |
| 字节 TX 缓冲 | 512 B/路（可配） | |
| TX 单次编码块 | ≤64 字节（≤640 symbol） | 发送任务堆上 2.5 KB 缓冲 |

## 6. TX 数据流

1. `rmt_soft_uart_write()` 校验句柄与长度后，`xStreamBufferSend()` 写入 TX 字节缓冲；返回实际写入字节数（可能小于请求值）。
2. 发送任务被唤醒，按 ≤64 字节取数据，调用 `su_encode()` 展开为 symbol：
   - 起始位：低电平 1 bit
   - 8 个数据位：LSB first，每 bit 1 个 symbol，电平按位值
   - 停止位：高电平 1 bit
3. `rmt_transmit(encoder=copy, payload=symbols, ...)`；`rmt_transmit_config_t.flags.eot_level = 1`，保证发送结束回到空闲高电平。
4. `rmt_tx_wait_all_done()` 后循环取下一块。
5. 通道 `mem_block_num = 1`，由驱动做 48 symbol 乒乓流式续接；115200 下每约 417 µs 填充一次。

发送任务优先级低于 RX 解码任务，保证接收优先。

## 7. RX 解码

### 7.1 中断侧（IRAM）

`on_recv_done` 回调只做一件事：把 `edata->received_symbols[0..num_symbols-1]` 整段压入该通道的 symbol 环形队列。

- 队列满：丢弃本次数据，`stats.rx_overflow++`，绝不阻塞。
- 回调返回 `need_yield` 标志，用于唤醒解码任务。
- 回调函数放 IRAM（驱动强制要求）。

### 7.2 任务侧（解码状态机）

对每个 symbol 维护「绝对 tick 游标 + 当前电平」，因此 32767 tick 拆分出的连续同电平 symbol 会被自然拼接。

状态机：

```
WAIT_START --(检测到高→低跳变，记 t0)--> FRAME
FRAME:
  在 t0 + (1.5 + k) * bit_ticks 处采样第 k 个数据位 (k=0..7)
  在 t0 + 9.5 * bit_ticks 处采样停止位
  stop == 1 -> 投递字节
  stop == 0 -> stats.framing_errors++，仍投递字节
  若低电平持续 > 10 bit -> break：丢弃当前帧，重新同步
  t0 += 10 * bit_ticks，回到 WAIT_START
```

- 采样实现为「按绝对时间查询电平」，不要求 symbol 与 bit 对齐；跨缓冲区边界时游标状态保留。
- 每帧用起始沿重新对时，不跨帧累积误差。
- 投递：写入该通道 RX 字节 StreamBuffer；满则 `stats.rx_overflow++` 并丢弃。

### 7.3 接收事务生命周期

- `signal_range_min_ns = 0.2 × bit 时间`：硬件滤除窄毛刺。
- `signal_range_max_ns = max(8 ms, 40 × bit 时间)`：线路持续同一电平超过该值即结束本次事务。
  - 40 bit 上限保证「起始位 + 8 个数据位」这种最长 9 bit 的连续低电平不会在中途截断事务。
  - 该值同时是**接收延迟上限**：最后一个字节之后最多再等这么久才会被投递。
- 事务结束（`edata.flags.is_last == true`）时，回调通知解码任务，由任务调用 `rmt_receive()` 重新装载，并 `stats.rx_restarts++`。
  - 重新装载间隙约数 µs；若起始位恰好落在该间隙会被漏掉。这是无 DMA 方案的固有代价，写入文档与统计。
- 空闲时约 125 次/秒重装载（8 ms 周期），CPU 开销可忽略；不像「按 32 bit 结束事务」的方案那样在高波特率下产生上万次中断。
- `set_baud` / `del` 前先 `rmt_disable()`，防止回调在资源释放期间触发。

## 8. 公共 API

```c
typedef struct {
    int      tx_gpio;
    int      rx_gpio;
    uint32_t baud;              // 300 ~ 1000000
    bool     invert;            // TX 输出、RX 输入同时反相
    bool     internal_loopback; // 仅调试/自测：TX 内部回环到 RX
    size_t   rx_buffer_bytes;   // 0 = 默认 1024
    size_t   tx_buffer_bytes;   // 0 = 默认 512
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
```

约定：

- `write/read` 返回实际字节数，`-1` 表示参数或状态错误；`0` 表示超时。
- `internal_loopback = true` 时，`tx_gpio` 必须有效，`rx_gpio` 可与之相同（驱动把输出内部回灌到输入路径），用于免接线冒烟测试。
- `stats.rx_restarts` 在每次重新装载接收事务时 +1。
- **`set_baud()` 会重建 RMT 通道（分辨率写在创建参数里），丢弃未发送和未读取的数据**，头文件注释明确写出。
- 同一句柄的 `write()` 可多任务并发（StreamBuffer 自身线程安全）；`set_baud()` / `del()` 不可与其他调用并发。

## 9. 错误处理与资源生命周期

| 场景 | 处理 |
|---|---|
| GPIO 非法 / TX 与 RX 同引脚且未开 loopback / 引脚重复占用 | `ESP_ERR_INVALID_ARG` + 明确日志 |
| 波特率越界 | `ESP_ERR_INVALID_ARG` |
| RMT 通道耗尽 | `ESP_ERR_NOT_FOUND`，已分配资源全部回滚 |
| TX 缓冲满 | 返回部分写入字节数，不静默丢弃 |
| RX symbol 队列 / 字节缓冲溢出 | 丢新数据 + 计数，ISR 不阻塞 |
| 帧错误 | 计数器 + 仍投递字节 |
| `new()` 中途失败 | 按「任务 → 流缓冲 → 编码器 → 通道」逆序释放 |

`del()` 顺序固定：停任务 → `rmt_disable` → 删除 RX/TX 通道 → 删除编码器 → 删除流缓冲与环形队列。RX 回调可能在 `rmt_disable` 前已进入，用「已停止」标志 + 临界区保证回调不再访问已释放内存。

## 10. 验证方案

本机没有 host C 编译器（gcc/clang/cl 均不可用），因此**不提供 PC 单测**，测试全部在板端 Unity 中进行。

1. **编译**：`idf.py -DIDF_TARGET=esp32s3 build`，使用本机 `esp-idf-v5.3.2`，要求零警告。
2. **纯逻辑测试（无需接线）**：把合成 symbol 序列喂给 `su_decoder_*`：
   - 固定图样 0x00 / 0xFF / 0x55 / 0xAA、随机 1 KB
   - 电平被 32767 tick 拆分的长 symbol
   - 帧间长空闲、>10 bit 的 break、注入窄毛刺
   - ±3% 时钟偏差
   - 跨 symbol 块边界的帧
3. **回环测试（需要板子）**：`CONFIG_RMT_SOFT_UART_SELFTEST`
   - 阶段一：3 路 `internal_loopback`，1 KB 随机数据往返比对
   - 阶段二：TX/RX 外部短接，3 路并发送随机数据，校验内容与统计计数
   - 打印 PASS/FAIL；115200 与 9600 各跑一遍

### 验收标准

- 3 路同时 115200 回环，连续 100 次 1 KB 随机数据，`framing_errors == 0`、`rx_overflow == 0`、内容完全一致
- 9600 下同样通过
- 运行中 `set_baud(115200 → 9600 → 115200)` 后仍能正常收发
- 断开 RX 输入时 `rx_restarts` 正常增长，不崩溃、不泄漏

## 11. 已知风险

| 风险 | 影响 | 缓解 |
|---|---|---|
| 1 Mbaud 下 ping-pong 回调间隔仅约 100 µs | 高优先级中断可能错过，丢字节 | 文档标注可靠上限为 460800；实测可提高 RX 缓冲块数 |
| 长空闲结束事务后重新装载有数 µs 间隙 | 极短时间内到达的起始位丢失 | `rx_restarts` 计数可见；必要时用 GPIO 边沿中断辅助重装载 |
| 3 路 RX 任务同时解码抢占 CPU | 其他任务延迟 | 解码任务优先级可配，默认 12；每路独立任务避免互相阻塞 |
| 无 host 单测 | 纯逻辑缺陷只能在板端暴露 | codec 设计为无硬件依赖，测试覆盖边界图样 |

## 12. 实现顺序

1. `soft_uart_codec`（纯逻辑）+ 板端合成 symbol 测试
2. `symbol_ring` + 其板端测试
3. `soft_uart_tx`（含 `internal_loopback` 冒烟）
4. `soft_uart_rx`（回调 + 解码任务）
5. `soft_uart_core`（资源管理与 `set_baud`）
6. `main.c` 示例与自测模式
7. 全量编译 + 板端回环验收
