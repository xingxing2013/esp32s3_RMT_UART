# ESP32-S3 RMT 三路软件串口

用 RMT 外设实现 3 路独立全双工 8N1 软件串口，ESP-IDF v5.3+。

## 资源占用

- 3 × RMT TX + 3 × RMT RX（S3 共 4+4，剩 2 个）
- 每路 1 个发送任务 + 1 个接收任务（共 6 个）
- 每路：RX symbol 环 4 KB、RX 字节缓冲 1–4 KB、TX 缓冲 0.5–2 KB
- RMT 中断优先级固定为 3（三路并发高波特率时需要）

## 接线

| 通道 | TX | RX |
|---|---|---|
| ch0 | GPIO42 | GPIO41 |
| ch1 | GPIO40 | GPIO39 |
| ch2 | GPIO38 | GPIO37 |

引脚可在 `idf.py menuconfig -> Component config -> RMT Soft UART` 修改。
**TX 与 RX 必须是不同引脚**：IDF 的 `gpio_config()` 会把同引脚的输出信号改回普通 GPIO，
RMT 无法在同一引脚上做内部回环，所以自测必须外部短接。

> GPIO37 在 8 线（Octal）PSRAM 模组（N8R8/N16R8）上被 PSRAM 占用，不能这样接；
> 4 线 PSRAM 模组（N8R2 等）可以。

## API

```c
rmt_soft_uart_config_t cfg = {
    .tx_gpio = 42,
    .rx_gpio = 41,
    .baud = 115200,
};
rmt_soft_uart_handle_t u;
ESP_ERROR_CHECK(rmt_soft_uart_new(&cfg, &u));

rmt_soft_uart_write(u, "hello", 5, pdMS_TO_TICKS(100));   // 返回实际写入字节数
uint8_t buf[16];
int n = rmt_soft_uart_read(u, buf, sizeof(buf), pdMS_TO_TICKS(100)); // 0 = 超时

rmt_soft_uart_set_baud(u, 9600);   // 会重建通道并丢弃在途数据
rmt_soft_uart_del(u);
```

## 构建

```bash
. $IDF_PATH/export.sh     # Windows: . $IDF_PATH\export.ps1
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

## 自测

- `CONFIG_RMT_SOFT_UART_RUN_TESTS`：上电跑 codec / ring 的 Unity 用例（20 个，无需接线）
- `CONFIG_RMT_SOFT_UART_SELFTEST`：跑 5 项回环自测（都需要 3 组 TX→RX 短接线）
  - 3 项 `wire-*`：单路 1 KB 随机数据往返
  - 1 项 `concurrent`：3 路同时收发 1 KB
  - 1 项 `baud-switch`：9600 ↔ 115200 连续 5 轮切换后仍能收发

预期输出（接好线）：

```
I (xxx) su_selftest: selftest summary: 5/5 passed
```

拔掉任意一路短接线后对应用例必须变成 FAIL —— 这是验证自测本身有效的反向检查。

## 限制与已知行为

- 固定 8N1，无校验位 / 流控 / 数据位可配
- RX 的 `signal_range_min_ns / max_ns` 会按硬件寄存器上限收敛：
  - 低波特率（≤ ~62.7 k）毛刺过滤窗口被限到约 3.19 µs，只能滤掉很窄的尖峰
  - 高波特率空闲窗口被限到 `32767 tick`：115200 下约 3.7 ms、1 M 下约 0.41 ms，
    因此 `rx_restarts` 会持续增长，属正常现象
- 接收延迟上限 = 96 symbol 的线上时间 + 空闲窗口（115200 下约 4.5 ms）
- TX 初始化时会先发 10 bit 空闲高电平，把线路拉到 UART 空闲态；
  否则 RMT 通道未发送时输出低，接收端会先看到一个长低电平
- 解码器需要「空闲高 → 起始位」的边沿关系才能同步；布防时线路已被拉低会先报一次 break
- 接收中出现窄毛刺时，毛刺会被当作起始位，最多损失紧随其后的 1 个字节；
  这个假字节可能不产生 `framing_errors` 计数
- `set_baud()` 会重建通道并丢弃在途数据；不要与其他线程的读写并发调用

## 实测结果

测试条件：ESP32-S3（chip rev v0.2），IDF v5.3.2，3 组 TX→RX 短接线，1 KB 随机数据。

| 日期 | 波特率 | 单路 ×3 | 3 路并发 | 切换测试 | 结果 |
|---|---|---|---|---|---|
| 2026-11-16 | 9600 | PASS | PASS | PASS | 5/5 |
| 2026-11-16 | 115200 | PASS | PASS | PASS | 5/5 |
| 2026-11-16 | 460800 | PASS | PASS | PASS | 5/5 |
| 2026-11-16 | 1000000 | PASS | PASS | PASS | 5/5 |

- 每次均 `fe=0`、`ovf=0`、`match=1`
- 板上 Unity 用例：17 个 codec + 3 个 ring = **20/20 通过**
- 1 M 下 3 路并发 `restarts=16`（空闲窗口只有 0.41 ms，事务会频繁重装载），仍无丢字节
