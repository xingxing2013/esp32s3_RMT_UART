# ESP32-S3 RMT 三路软件串口

用 RMT 外设实现 3 路独立全双工 8N1 软件串口，ESP-IDF v5.3+。

## 资源占用

- 3 × RMT TX + 3 × RMT RX（S3 共 4+4，剩 2 个）
- 每路 1 个发送任务 + 1 个接收任务（共 6 个）
- 每路：RX symbol 环 4 KB、RX 字节缓冲 1–2 KB、TX 缓冲 0.5–1 KB

## 接线

| 通道 | TX | RX |
|---|---|---|
| ch0 | GPIO42 | GPIO41 |
| ch1 | GPIO40 | GPIO39 |
| ch2 | GPIO38 | GPIO37 |

引脚可在 `idf.py menuconfig -> Component config -> RMT Soft UART` 修改。
`internal_loopback = true` 时 TX 与 RX 用同一个引脚，不需要外部接线。

## API

```c
rmt_soft_uart_config_t cfg = {
    .tx_gpio = 4,
    .rx_gpio = 5,
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

- `CONFIG_RMT_SOFT_UART_RUN_TESTS`：上电跑 codec / ring 的 Unity 用例（无需接线）
- `CONFIG_RMT_SOFT_UART_SELFTEST`：跑 8 项回环自测
  - 3 项 `internal-*`：单路内部回环，不需要接线
  - 3 项 `wire-*`：需要把每路 TX 与对应 RX 短接
  - 1 项 `concurrent`：3 路同时收发（内部回环），不需要接线
  - 1 项 `baud-switch`：9600 ↔ 115200 反复切换后仍能收发（内部回环）

预期输出（接好线，115200）：

```
I (xxx) app: selftest summary: 8/8 passed
```

未接 `wire-*` 短接线时是 `5/8 passed`；拔掉任意一路短接线后对应用例必须变成 FAIL —— 这是验证自测本身有效的反向检查。

## 限制与已知行为

- 固定 8N1，无校验位 / 流控 / 数据位可配
- 无 DMA；建议可靠上限 460800，1 Mbaud 需实测
- RMT RX 的 `signal_range_min_ns / max_ns` 会按硬件寄存器上限收敛：
  - 低波特率（≤ ~62.7 k）毛刺过滤窗口被限到约 3.19 µs，只能滤掉很窄的尖峰
  - 高波特率空闲窗口被限到 `32767 tick`，115200 下约 3.7 ms，因此 `rx_restarts` 会持续增长，属正常现象
- 接收延迟上限 = 96 symbol 的线上时间 + 空闲窗口；115200 下约 4.5 ms，9600 下约 9 ms
- 解码器需要线路先出现一段空闲高电平才能同步；若恰好在某个字节中途开始接收，首字节不可用（真实 UART 同理）
- 接收过程中出现窄毛刺时，毛刺会被当作起始位，最多损失紧随其后的 1 个字节；这个假字节有可能不产生 `framing_errors` 计数
- `set_baud()` 会重建通道并丢弃在途数据；不要与其他线程的读写并发调用

## 实测结果

| 日期 | IDF | 板子 | 115200 | 9600 | 460800 | 备注 |
|---|---|---|---|---|---|---|
| 待填 | v5.3.2 | ESP32-S3 | 待测 | 待测 | 待测 | 需接 TX→RX 短接线 |
