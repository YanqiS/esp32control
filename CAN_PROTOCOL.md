# STM32 ESP32 CAN/DBC Protocol

This project uses two standard CAN frames to control the STM32 relay board and read back command status.

## CAN bit rate

`main.c` currently configures CAN1 for about 125 kbit/s when the STM32F103 APB1 CAN clock is 36 MHz: Prescaler 18, BS1 13TQ, BS2 2TQ.

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
