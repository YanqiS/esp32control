# STM32 ESP32 CAN/DBC Protocol

This project uses two standard CAN frames to control the STM32 relay board and read back command status.

## CAN bit rate

`main.c` currently configures CAN1 for about 125 kbit/s when the STM32F103 APB1 CAN clock is 36 MHz: Prescaler 18, BS1 13TQ, BS2 2TQ.


## 板卡使用流程（精简版）

1. **板卡上电**：外部电源生成 5V/3.3V，STM32 常供电启动；默认 ESP32 电源关闭、所有继电器释放。
2. **STM32 初始化**：初始化系统时钟、GPIO、CAN、MOSFET 控制脚、继电器控制脚；CAN 工具/上位机设置为标准帧 125 kbit/s。
3. **通过 CAN 打开 ESP32 电源**：上位机发送 `0x321` 控制帧，例如打开 ESP32-1：`01 01 00 00 00 00 00 00`；STM32 拉低对应 P-MOS gate，使 `U1_VCC` 输出 3.3V。
4. **等待 ESP32 就绪**：ESP32 上电后建议等待 3~5 秒，用于系统启动、蓝牙和 Wi-Fi 初始化；期间可监听 STM32 回复的 `0x322` 状态帧确认命令已被接收。
5. **建立蓝牙连接**：车机搜索并连接 ESP32 模拟设备，连接完成后 ESP32 进入蓝牙电话模拟待命状态。
6. **通过 CAN 触发按键继电器**：上位机发送 `0x321` 按键命令，STM32 输出 GPIO，经 ULN2003 驱动继电器吸合；继电器触点把 ESP32 对应 GPIO 短接到 GND，ESP32 识别为按键按下。
7. **ESP32 执行动作**：`INCOME` 模拟来电，`ANSWER` 接听，`HANGUP` 挂断，`DIAL1`/`DIAL2` 外拨预设号码；`DurationMs` 只控制继电器按下时间，推荐普通短按使用 200 ms。
8. **测试结束**：发送 `0x321` 的 `AllRelaysOff` 释放全部继电器，需要断电时再发送 `PowerOff`；每条命令后 STM32 会用 `0x322` 返回 OK 或错误状态。

常用测试报文：

| 操作 | `0x321` 数据 | 预期 `0x322` 回复 |
| --- | --- | --- |
| ESP32-1 上电 | `01 01 00 00 00 00 00 00` | `00 01 XX` |
| ESP32-2 上电 | `02 01 00 00 00 00 00 00` | `00 01 XX` |
| ESP32-1 接听 200 ms | `01 11 C8 00 00 00 00 00` | `00 11 XX` |
| ESP32-2 外拨号码 1，300 ms | `02 13 2C 01 00 00 00 00` | `00 13 XX` |
| 释放全部继电器 | `00 20 00 00 00 00 00 00` | `00 20 XX` |
| ESP32-1 下电 | `01 00 00 00 00 00 00 00` | `00 00 XX` |

`0x322` 中 `XX` 是 STM32 `HAL_GetTick()` 的低 8 位，只用于简单判断回复是新的。

## DBC file

Use [`esp32_control.dbc`](esp32_control.dbc) in your CAN tool. If you create the DBC manually, create these two standard-frame messages.

### Message: `ESP32_Control`

| Setting | Value |
| --- | --- |
| CAN ID | `0x321` / decimal `801` |
| Frame type | Standard data frame |
| DLC | `8` |
| Transmitter | `CAN_TOOL` / upper controller |
| Receiver | `STM32` |

| Signal | Start bit | Length | Byte order | Signed | Factor | Offset | Unit | Meaning |
| --- | ---: | ---: | --- | --- | ---: | ---: | --- | --- |
| `Target` | 0 | 8 | Intel/little-endian | Unsigned | 1 | 0 | - | Which ESP32 to control |
| `Command` | 8 | 8 | Intel/little-endian | Unsigned | 1 | 0 | - | Power/key command |
| `DurationMs` | 16 | 16 | Intel/little-endian | Unsigned | 1 | 0 | ms | Relay press duration |

`Target` values:

| Value | Meaning |
| ---: | --- |
| `0` | All ESP32 devices |
| `1` | ESP32-1 |
| `2` | ESP32-2 |

`Command` values:

| Value | Hex | Meaning |
| ---: | --- | --- |
| `0` | `0x00` | Power off selected ESP32 |
| `1` | `0x01` | Power on selected ESP32 |
| `16` | `0x10` | Press `INCOME` relay |
| `17` | `0x11` | Press `ANSWER` relay |
| `18` | `0x12` | Press `HANGUP` relay |
| `19` | `0x13` | Press `DIAL1` relay |
| `20` | `0x14` | Press `DIAL2` relay |
| `32` | `0x20` | Release all relays |

`DurationMs` is used by the relay/key commands. The firmware clamps it to 50 ms through 5000 ms. Use 200 ms for a normal short press.

Example payloads for CAN ID `0x321`:

| Goal | Data bytes |
| --- | --- |
| Power on ESP32-1 | `01 01 00 00 00 00 00 00` |
| Power off ESP32-2 | `02 00 00 00 00 00 00 00` |
| Press ESP32-1 answer for 200 ms | `01 11 C8 00 00 00 00 00` |
| Press ESP32-2 dial 1 for 300 ms | `02 13 2C 01 00 00 00 00` |
| Release all relays | `00 20 00 00 00 00 00 00` |

### Message: `ESP32_Status`

| Setting | Value |
| --- | --- |
| CAN ID | `0x322` / decimal `802` |
| Frame type | Standard data frame |
| DLC | `3` |
| Transmitter | `STM32` |
| Receiver | `CAN_TOOL` / upper controller |

| Signal | Start bit | Length | Byte order | Signed | Factor | Offset | Unit | Meaning |
| --- | ---: | ---: | --- | --- | ---: | ---: | --- | --- |
| `StatusCode` | 0 | 8 | Intel/little-endian | Unsigned | 1 | 0 | - | Command result |
| `Detail` | 8 | 8 | Intel/little-endian | Unsigned | 1 | 0 | - | Echo or error detail |
| `TickLow` | 16 | 8 | Intel/little-endian | Unsigned | 1 | 0 | ms | Low byte of `HAL_GetTick()` |

`StatusCode` values:

| Value | Hex | Meaning |
| ---: | --- | --- |
| `0` | `0x00` | OK; `Detail` echoes the command |
| `225` | `0xE1` | CAN DLC too short; `Detail` is received DLC |
| `226` | `0xE2` | Invalid target; `Detail` is target value |
| `227` | `0xE3` | Invalid command; `Detail` is command value |
