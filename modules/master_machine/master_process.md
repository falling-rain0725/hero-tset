# 视觉通信模块

## 模块职责

本模块负责视觉数据帧的封装、发送、接收缓存、帧校验和数据解析。上层通过 `Vision_Send_s` 提供发送字段，通过 `Vision_Recv_s` 获取最近一帧有效的接收数据。协议字节流的处理由 `seasky_protocol.c` 实现。

## 发送接口

```c
Vision_Recv_s *VisionInit(UART_HandleTypeDef *handle);
void VisionSend(const Vision_Send_s *data);
uint8_t VisionGetLatest(Vision_Recv_s *out);
```

`VisionInit()` 初始化所选 UART 或 USB VCP 通道，并返回接收数据结构的地址。`VisionSend()` 接收一个发送结构体指针，完成标志字段打包、浮点数序列化、帧校验和底层发送。`VisionGetLatest()` 将最近一帧有效数据复制到 `out`；返回 1 表示当前在线且数据有效，返回 0 表示尚未收到有效帧或视觉模块已离线。

## 发送帧布局

| 字节偏移 | 内容 | 说明 |
| --- | --- | --- |
| 0 | 帧头 `0xA5` | 固定值 |
| 1–2 | `data_len` | 小端序，`11 * 4 + 2 = 46` |
| 3 | CRC8 | 覆盖偏移 0–2 |
| 4–5 | 命令号 `0x0002` | 小端序 |
| 6 | 标志低字节 | bit 0–3 为 `self_color`，bit 4–7 为 `work_mode` |
| 7 | 标志高字节 | bit 0–7 为 `bullet_speed`，对应 16 位标志值 bit 8–15 |
| 8–51 | 11 个小端序 `float32` | 顺序见下表 |
| 52–53 | CRC16 | 覆盖偏移 0–51，小端序写入 |

浮点负载顺序如下。协议保留位固定写入 `0.0f`。

| 序号 | 字段 |
| --- | --- |
| 0 | `yaw` |
| 1 | `pitch` |
| 2 | `roll` |
| 3–4 | 保留，固定为零 |
| 5 | `yaw_vel` |
| 6 | `pitch_vel` |
| 7 | `game_progress` |
| 8 | `current_HP` |
| 9 | `current_base_hp` |
| 10 | `current_enemy_base_hp` |

总帧长为 54 字节。

## 接收帧布局

接收命令号为 `0x0001`。帧总长为 42 字节，`data_len` 为 `8 * 4 + 2 = 34`。帧头和 CRC 字段位置、计算范围与发送帧相同。标志低字节的低四位保存 `target_state`；接收浮点负载包含 8 个小端序 `float32`，顺序为：

| 序号 | 字段 |
| --- | --- |
| 0 | `pitch` |
| 1 | `yaw` |
| 2 | `delta_pitch` |
| 3 | `delta_yaw` |
| 4 | `target_distance` |
| 5 | `vel_x` |
| 6 | `vel_y` |
| 7 | `angle_yaw` |

解析器支持分段接收和连续多帧输入；只有帧头、长度、CRC、命令号和负载均有效的数据才会更新接收结构体。
