# rmt_uart 例程

每个子目录都是一个**独立可编译**的 IDF 工程，直接：

```bash
cd components/rmt_uart/examples/<name>
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

它们通过 `EXTRA_COMPONENT_DIRS "../../.."` 引用 `components/` 下的
`rmt_uart` 和补丁版 `esp_driver_rmt`，不需要额外拷贝。

| 例程 | 说明 | 接线 |
|---|---|---|
| `basic_echo` | 单路 8N1 @115200，同步 read/write 回声 | TX 42 ↔ RX 41 |
| `three_ports` | 三路：8N1 同步、7E1 异步、8N2 流式回调 | 42/41、40/39、38/37 |
| `frame_formats` | 改宏切换 5–8 数据位/校验/停止位，并演示 `set_baud` | TX 42 ↔ RX 41 |

> 三组默认引脚对应常见 S3 开发板；换板时改文件顶部的宏即可。
> GPIO37 在 8 线 PSRAM 模组（N8R8/N16R8）上被 PSRAM 占用，注意避开。
