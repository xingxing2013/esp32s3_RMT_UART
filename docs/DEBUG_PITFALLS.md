# 调试踩坑记录（RMT 软件串口）

本文记录本项目在 ESP32-S3 + IDF 5.3.2 上开发、验证过程中真实踩过的坑。
每条包含：**症状 → 根因 → 修复 → 防坑检查**。后续开发（包括换芯片、升级 IDF、加功能）先扫一遍对应章节。

> 维护约定：新增坑请给出可复现的症状和最小验证方法；不要只写“注意 XX”。

---

## 1. RMT 外设与时序

### 1.1 RX 窗口参数有硬件寄存器上限，naive 公式在所有波特率下都会失败

**症状**：`rmt_uart_init()` 永远返回 `ESP_ERR_INVALID_ARG`，`rmt_receive` 报 `signal_range_min_ns too big` / `signal_range_max_ns too big`。编译、单测都发现不了。

**根因**：驱动在 `rmt_rx.c` 里校验：

- `filter_reg = 80MHz × min_ns / 1e9 ≤ 255`（约 3.19 µs）
- `idle_reg = channel_res × max_ns / 1e9 ≤ 32767`

“0.2 bit 过滤 + max(8ms, 40bit) 空闲” 这两条约束的可用区间**不重叠**：
过滤要求 baud ≳ 62.7k，空闲要求 baud ≲ 51.2k。

**修复**：`ru_rx_window_ns()` 把两个值收敛到寄存器上限内，并保证 `max_ns > 11 bit`；
`test_rx_window_limits` 直接按寄存器公式断言 `filter_ticks ≤ 255 && idle_ticks ≤ 32767`。

**防坑**：任何 RMT 时序参数，先按寄存器位宽反算上限；单元测试直接断言“寄存器换算值”，不要断言“我算出来的 ns”。

### 1.2 同一 GPIO 做内部回环不可用

**症状**：`tx_gpio == rx_gpio` + `io_loop_back` 时 RX 一个边沿都收不到（`restarts=0`）。

**根因**：`rmt_rx_initialize` 对 RX 引脚调用 `gpio_config()`，而 `gpio_config()` 内部会
`gpio_output_enable()` → `esp_rom_gpio_connect_out_signal(pin, SIG_GPIO_OUT_IDX)`
把该脚的输出信号从 RMT 改回普通 GPIO 寄存器。

**修复**：API 直接拒绝 `tx_gpio == rx_gpio`；自测一律用外部短接线。

**防坑**：IDF 里任何“同一引脚既做外设输出又做输入”的方案都要先查 `gpio_config()` 的副作用。

### 1.3 TX 通道使能后、首次发送前输出是低电平

**症状**：RX 先收到一段长低电平，首字节解码错位（实测首字节从起始位后第 2 位开始采样）。

**根因**：RMT TX 通道 enable 但未 transmit 时，引脚保持低；UART 空闲应为高。

**修复**：`su_tx_create()` 在 enable 后先发一个 10 bit 的空闲高电平 symbol 并等待完成，把线拉到 idle。

**防坑**：软件串口初始化后必须确认“发第一帧之前线是高”。

### 1.4 RMT 不会为“布防前”的空闲期产生 symbol

**症状**：首字节仍然错位，且反推 bit 流显示从数据位中间开始采样。

**根因**：`rmt_receive()` 布防时线路已经是高，但硬件只从**第一个边沿**开始产生 symbol，
所以流的第一个 symbol 可能就是起始位；解码器要求“前一电平为高”才认起始位 → 漏掉。

**修复**：`first_low_is_start = !prev_valid && level == 0 && duration < frame_ticks`，
即“首个低电平且短于一帧”按起始位处理；被拉低的长电平仍按 break 处理。

**防坑**：不要假设 RMT 会给出布防瞬间的电平 symbol；冷启动逻辑必须显式处理。

### 1.5 最后一个字节丢失

**症状**：发送 1024 字节，只收到 1023；`fe=0`、`ovf=0`，`first_diff` 在末尾。

**根因**：事务因空闲结束时，驱动不保证交付最后一段高电平 symbol；停止位采样点落空，帧永远不完成。

**修复**：`is_last` 时在 ISR 里往 symbol 环里**补一个 `frame_ticks` 长的空闲高电平 symbol**，
解码器自然完成最后一帧。

**防坑**：任何“按空闲结束事务”的接收设计，都要显式收尾最后一帧。

### 1.6 先排空 ring 再重新布防 → 高波特率并发丢起始位

**症状**：3 路并发 460800 时偶发丢 1–2 字节，`fe` 少量增加，单路正常。

**根因**：`is_last` 后任务先排空 ring（3 路时可能几百 µs）再 `rmt_receive`，这段“未布防”窗口漏掉下一个起始位。

**修复**：任务**先 `rmt_receive()` 再解码**；同时把空闲 symbol 随 `is_last` 一起入队，保证解码顺序不变。

**防坑**：RX 重装载必须紧跟事务结束，不能排在耗时处理之后。

### 1.7 IDF 5.3.2 驱动断言：partial RX + 乒乓回绕 + 空闲结束

**症状**：持续收发几毫秒后
`assert failed: rmt_isr_handle_rx_done rmt_rx.c:584 (offset >= rx_chan->mem_off)`，设备重启。
115200 / 460800 / 1M 都会触发；KB 级短突发正常。

**根因**：DONE 处理用 `offset - mem_off` 算长度并单段拷贝；空闲结束时硬件写指针可能已回绕
（`offset < mem_off`），断言失败，且单段拷贝会读错数据。

**修复**：`components/esp_driver_rmt/`（IDF 组件副本 + 补丁）：

- 长度按回绕计算：`total = wrapped ? (block - mem_off + offset) : (offset - mem_off)`
- 拷贝分两段：`[mem_off, block)` 与 `[0, offset)`

**防坑**：本地同名组件会覆盖 IDF 内置组件（看编译日志的 `-c .../components/esp_driver_rmt/src/rmt_rx.c`）；
升级 IDF 后要重新比对这份副本。

### 1.8 `duration = 0` 的 symbol 是 EOF，不是空操作

**症状**：8E1 / 8O1 / 5N1.5 / 6O2 等“每字节段数为奇数”的格式，一次只发出 1 个字节，
RX 只收到 3–4 字节；软件回环单测全绿。

**根因**：编码器按字节两两打包，奇数段时给最后一个 symbol 补 `duration1 = 0`；
**RMT 硬件把 0 时长当作传输结束**，后续数据全部不发。解码器把 0 时长当无操作，所以软件测试发现不了。

**修复**：跨字节保留“半个 symbol”（`pending`），只在整段末尾补零。

**防坑**：RMT symbol 的 duration 不能为 0（除末尾 EOF）；编码器的单测必须同时覆盖硬件传输路径（回环）。

### 1.9 break 与 10 bit 低电平的边界

**症状**：`0x00` 且停止位为低（正好 10 bit 低）被误判为 break、字节被丢弃。

**根因**：break 判定用了 `run_ticks >= frame_ticks`。

**修复**：改成严格 `>`；并补 10 bit（帧错误，字节仍交付）与 11 bit（break）两个用例。

**防坑**：边界条件写成用例，不要靠“应该没问题”。

---

## 2. 解码器实现与性能

### 2.1 64 位时间戳在 Xtensa 上很贵

`now/frame_start/next_start` 用 `uint64_t` 时每次加减比较都是多条指令。
改为 `uint32_t` + 回绕安全比较 `(int32_t)(a - b) < 0`（8.9 MHz 下约 8 分钟回绕一次，比较方式天然正确）。

### 2.2 采样点时间要增量维护

采样点间隔恒为 1 bit，`next_sample_ts += bit_ticks` 即可，不要每次 `frame_start + sample_off[i]`。

### 2.3 检查不要放在最内层

“长 run 位掩码批量置位”的判定如果放在每个采样点里，随机数据反而变慢（554 → 593）。
正确做法是**每个 edge 判一次 span**，≥3 bit 才走批量路径。

### 2.4 回调不是瓶颈，别乱改接口

实测回调只占 ~13–21 cycles/byte（536 vs 523），为它改 API 不划算。
优化前先用 `cycles/byte` 基准定位，不要凭感觉。

### 2.5 IRAM 对这个负载没帮助

`ru_decoder_feed` 放 IRAM：670 → 670。热点是分支与逐位工作，不是取指。
放 IRAM 只浪费 1.7 KB，已撤回。

---

## 3. 测试与基准的坑

### 3.1 `words += append_idle(...)` 写错，制造未初始化 symbol

`append_idle()` 返回的是新的总长度，不是增量。写成 `+=` 会让后续 memcpy 写到错误位置，
留下未初始化的 symbol，解码出 600 字节垃圾。正确写法：`words = append_idle(...)`。

### 3.2 测试数组越界

`rmt_symbol_word_t in[8]` 却从 `&in[6]` 取 3 个元素。栈上的越界值让断言“Expected 9 Was 1”。

### 3.3 毛刺测试必须给“恢复空闲”

毛刺会被当成起始位，10 bit 内的真实帧会被吞掉。测试要构造成“毛刺 → 空闲 → 有效帧”，
断言“最后解出正确字节、总字节数 ≤2”，而不是“毛刺后立刻跟帧”。

### 3.4 冷启动/断帧测试需要前导空闲

流的第一个 symbol 就是低电平时没有“高→低”的起始沿。测试要么加前导空闲，
要么专门构造“首个低电平”用例（现在两种都有）。

### 3.5 数据位 <8 时要掩码比较

发 `0xEE` 收 `0x6E`（7 位）是正常的。回环比对必须 `rx[i] == (tx[i] & mask)`。

### 3.6 基准里用 32 KB 栈数组 → LoadProhibited

在 4 KB 任务栈里声明 `uint32_t lv[4096]` 直接栈溢出。
基准/测试里的大数组用 `static` 或流式处理。

### 3.7 基准必须模拟 RMT 的真实 symbol 流

编码器输出是“每 bit 一段”，RMT 硬件会把连续同电平合并成 run。
用未合并的流测出的 cycles/byte 会高估（943 vs 670）。基准里要 merge 同电平段再喂。

---

## 4. 压测工具的坑

### 4.1 闭环“写一轮-读一轮”在丢 1 字节后失步

一旦丢字节，后续比对整体错位，读循环超时、吞吐雪崩，指标失去意义。
正确做法：**独立 producer 持续灌数据 + 独立 consumer 收**，用 `tx_bytes - rx_bytes` 算真实丢失。

### 4.2 定时器单位错 1000 倍

`alarm_count = 1000000 / irq_khz` 想生成 20 kHz，实际是 20 Hz（`observed=160 in 8s`）。
1 MHz 分辨率下应为 `1000 / irq_khz`。

### 4.3 用统计值算丢失，不要用逐字节比对

UART 丢 1 字节会让后面全部错位；逐字节 `memcmp` 只能告诉你“不一致”，
`stats.tx_bytes - stats.rx_bytes` 才能给出丢失量。

---

## 5. FreeRTOS 与资源生命周期

- **任务 join 用 `portMAX_DELAY`**：原实现等 500 ms 超时后仍释放资源 → use-after-free。超时后**绝不释放**。
- **先停 RX 硬件再 join**：否则 ISR 可能在任务句柄被删除后仍 `vTaskNotifyGiveFromISR`。
- **`rmt_disable` 后再等 2 ms** 才删通道/释放缓冲，覆盖另一核上仍在执行的回调。
- **不要重复 `rmt_disable`**：会打印 `channel not in enable or run state` 噪音。用 `rx_stopped` 标志。
- **`IRAM_ATTR` 不要同时写在声明和定义**：`-Wattributes` 段冲突。
- **`rmt_copy_encoder_config_t` 是空结构体**：`= {0}` 会 `excess elements` 警告，直接声明不初始化。
- **`CONFIG_RMT_ISR_IRAM_SAFE` 必须关闭**：回调依赖 flash 里的 `memcpy`；已加 `#error` 拦截。

---

## 6. 工具链与脚本

### 6.1 sdkconfig 布尔值语法与重复行

- 关闭必须写 `# CONFIG_X is not set`，写 `CONFIG_X=n` 不生效。
- **同一符号出现两次时最后一行生效**。用 `Add-Content` 追加 `=y` 会被前面的 `is not set` 覆盖。
  脚本里先删除旧行再追加。

### 6.2 PowerShell `-replace` 默认大小写不敏感

`-replace 'rmt_soft_uart','rmt_uart'` 会把 `RMT_SOFT_UART` 也换成小写，
导致 `CONFIG_rmt_uart_...` 这种坏宏。批量改名前确认大小写，或用 `-creplace`。

### 6.3 `$` 锚点在 CRLF 文件上不匹配

`-replace '^# CONFIG_X is not set$'` 匹配不到 `\r\n` 行尾。用 `\r?$` 或按行处理。

### 6.4 PowerShell 5.1 没有 `Set-Content -NoNewline`

用 `[System.IO.File]::WriteAllText()` / `ReadAllText()` 做全文替换，避免换行被改写。

### 6.5 `gh api --jq` 里的引号会被 PowerShell 吃掉

`--jq '.[] | "\(.a) \(.b)"'` 在 PowerShell 里容易变成多个参数。
用 `--json` 输出后 `ConvertFrom-Json`，或 `-f` 拼接格式串。

### 6.6 `gh api` 的 commit 端点不接受短 SHA

`/git/commits/5ac3d3b` 会 404；先取 `git/ref/heads/main` 的完整 40 位 SHA。

### 6.7 github.com 的 git 传输被重置，但 API 正常

症状：`git push` 报 `Recv failure: Connection was reset` / `Failed to connect ... 443`，
而 `gh api` 一直可用。处理：

1. 用 Git Data API 推：blob → tree（带 `base_tree`）→ commit → PATCH ref
2. 空仓库不支持建 blob，先用 Contents API 建一个初始提交
3. 打完补丁/推送后**校验 tree**：对比每个文件的 blob SHA 与本地 `git hash-object`，并确认没有丢文件
4. 网络恢复后 `git fetch origin main && git reset --hard origin/main` 对齐

### 6.8 串口被占用 / 重新枚举

- `PermissionError(13, 设备没有发挥作用)`：上一个 capture/esptool 进程没退，`Stop-Process` 掉再试。
- 板子重新上电后 COM 号会变（COM35 → COM99），先枚举 `USB-SERIAL` 设备确认。

---

## 7. CI

- `espressif/esp-idf-ci-action@v1` 输入是 `esp_idf_version`、`target`、`path`、`command`；
  `esp_idf_version` 直接当 Docker tag（用 `v5.3.2`）。
- 三个例程用 `EXTRA_COMPONENT_DIRS "../../.."`，CI 里同样成立。
- 单测编译用 `ci/tests.defaults` + `-DSDKCONFIG_DEFAULTS="sdkconfig.defaults;ci/tests.defaults"`。
- `actions/checkout@v4` 会有 Node20 弃用提示，不影响结果。

---

## 8. 新功能检查清单

新增帧格式 / 换芯片 / 升级 IDF 时逐项确认：

- [ ] RMT 窗口参数按寄存器上限收敛，并有按寄存器公式断言的单测
- [ ] `duration = 0` 只允许出现在 payload 末尾
- [ ] 事务结束能收尾最后一帧（最后一字节不丢）
- [ ] `is_last` 后立即重装载，再解码
- [ ] 同引脚/同外设的 `gpio_config()` 副作用已确认
- [ ] 冷启动（首个 symbol 是低）与长低电平（break）都有用例
- [ ] 回环比对按数据位掩码
- [ ] 基准输入模拟真实 RMT run-length 流
- [ ] 持续流压测用 producer/consumer + 统计值算丢失
- [ ] 本地覆盖的 IDF 组件在升级后重新比对
- [ ] 板上跑一遍 Unity + 回环自测，CI 只保证编译
