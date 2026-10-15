# Fsi6x

FS-i6X 遥控接收机 SBUS 解析模块：从 UART 接收 iA6B 的 SBUS 帧，解析通道与拨杆并转换为 CMD 控制数据 / Module that parses FS-i6X plus iA6B SBUS frames from a UART, extracts channels and switches and converts them to CMD control data

## 1. 模块作用 / Purpose

构造时，Fsi6x 把 UART 配置为 100000 bit/s、偶校验、8 数据位、**2 停止位**，并创建线程 `uart_fsi6x`（栈深与优先级见 `Param`）。线程每 5 ms 读取一帧 25 字节的 SBUS 数据，读超时 4 ms。帧头不是 `0x0F` 或帧尾不是 `0x00` 时该帧被丢弃；接收机报告 `frame_lost` 或 `failsafe` 时同样丢弃，此后超过 100 ms 没有有效帧即判定为离线，每个周期向 CMD 提交控制量全零、`chassis_online` 与 `gimbal_online` 为 `false` 的数据。

有效帧转换为 `CMD::Data`，通过 `cmd.FeedRC(CMD::RCInputSource::RC_INPUT_FSI6X, ...)` 提交给 CMD，占用 CMD 的 `RC_INPUT_FSI6X` 输入槽。

四个摇杆通道按实测行程 240 至 1807、中位 1024 归一化到 [-1, 1]，两侧分别缩放。映射到 `CMD::Data`：

- 底盘：`x` 为左摇杆左右（右为正），`y` 为左摇杆上下（上为正），`z` 为右摇杆左右（左为正，即左转）。
- 云台：`yaw` 为右摇杆左右（左为正），`pit` 为右摇杆上下（上为正）。均限幅到 [-1, 1]。
- 底盘模式与发射：SWB、SWD 保留给应用层，本模块置 `self_define` 为 `NONE`、`isfire` 为 `false`。

`GetEvent()` 返回 Fsi6x 的 `LibXR::Event`，EventBinder 用它把下列事件 ID 绑定到别的模块。编号沿用 DR16 的布局，便于与既有配置对照：

- SWA（两档）→ 左拨杆：`FSI6X_SW_L_POS_TOP`（0，高位）、`FSI6X_SW_L_POS_BOT`（1，低位）、`FSI6X_SW_L_POS_MID`（2，中位，两档时不会出现）。
- SWC（三档）→ 右拨杆：`FSI6X_SW_R_POS_TOP`（3）、`FSI6X_SW_R_POS_BOT`（4）、`FSI6X_SW_R_POS_MID`（5）。

事件只在档位**发生变化**时触发，且首帧只记录状态不触发，避免上电瞬间误切模式。

`GetSBUSData()` 返回最近一帧的原始数据（`SBUSData`），供应用层读取 SWB、SWD 与两个旋钮。

Upon construction, Fsi6x configures the UART to 100000 bit/s, even parity, 8 data bits and **2 stop bits**, and creates the thread `uart_fsi6x` (stack depth and priority from `Param`). Every 5 ms the thread reads one 25-byte SBUS frame with a 4 ms read timeout. A frame is discarded when its header is not `0x0F` or its footer is not `0x00`, and also when the receiver reports `frame_lost` or `failsafe`; when no valid frame has arrived for more than 100 ms, Fsi6x is considered offline and in every following cycle submits data to CMD with all control values zero and `chassis_online` and `gimbal_online` set to `false`.

A valid frame is converted to `CMD::Data` and submitted to CMD through `cmd.FeedRC(CMD::RCInputSource::RC_INPUT_FSI6X, ...)`, occupying the `RC_INPUT_FSI6X` input slot of CMD.

The four stick channels are normalized to [-1, 1] using the measured travel of 240 to 1807 with a middle of 1024, scaling each side separately. The mapping to `CMD::Data` is:

- Chassis: `x` is the left stick horizontal (right positive), `y` is the left stick vertical (up positive) and `z` is the right stick horizontal (left positive, that is counter-clockwise).
- Gimbal: `yaw` is the right stick horizontal (left positive) and `pit` is the right stick vertical (up positive). Both are clamped to [-1, 1].
- Chassis mode and firing: SWB and SWD are reserved for the application layer, so this Module sets `self_define` to `NONE` and `isfire` to `false`.

`GetEvent()` returns the `LibXR::Event` of Fsi6x, with which EventBinder binds the following event IDs to other Modules. The numbers follow the DR16 layout so that existing configurations stay comparable:

- SWA (two positions) to the left switch: `FSI6X_SW_L_POS_TOP` (0, high), `FSI6X_SW_L_POS_BOT` (1, low), `FSI6X_SW_L_POS_MID` (2, middle, which a two-position switch never reaches).
- SWC (three positions) to the right switch: `FSI6X_SW_R_POS_TOP` (3), `FSI6X_SW_R_POS_BOT` (4), `FSI6X_SW_R_POS_MID` (5).

An event is activated only when the position **changes**, and the first frame only records state without activating anything, so power-up cannot switch modes spuriously.

`GetSBUSData()` returns the raw data of the latest frame (`SBUSData`), for the application layer to read SWB, SWD and the two knobs.

## 2. 通道映射 / Channel Mapping

辅助通道按遥控器的下述配置，SBUS 的位打包与通道一一对应。

| 下标 | 通道 | 功能 | 数值 |
| --- | --- | --- | --- |
| `ch[0]` | CH1 | 右摇杆左右 | 左为高位，右为低位 |
| `ch[1]` | CH2 | 右摇杆上下 | 上为高位，下为低位 |
| `ch[2]` | CH3 | 左摇杆上下 | 上为高位，下为低位 |
| `ch[3]` | CH4 | 左摇杆左右 | 右为高位，左为低位 |
| `ch[4]` | CH5 | SWC，三档 | 240 / 1024 / 1807 |
| `ch[5]` | CH6 | SWA，两档 | 240 / 1807 |
| `ch[6]` | CH7 | SWB，两档 | 240 / 1807 |
| `ch[7]` | CH8 | SWD，两档 | 240 / 1807 |
| `ch[8]` | CH9 | VRA 旋钮 | 240 至 1807，顺时针增加 |
| `ch[9]` | CH10 | VRB 旋钮 | 240 至 1807，顺时针增加 |

`SBUSData::ch` 保存上表的**原始值**，`SBUSData::sw[0..3]` 依次为 SWA、SWB、SWC、SWD 的档位（`-1` 低位、`0` 中位、`1` 高位，判定死区为 ±200）。

需要用不同的辅助通道配置时，改 `Fsi6x.hpp` 顶部的 `FSI6X_CH_*` 常量与 `ChannelIndex` 枚举即可，位解析部分不用动。

With the auxiliary channels configured as above, the SBUS bit packing lines up with the channel list.

`SBUSData::ch` holds the **raw** values of the table above, and `SBUSData::sw[0..3]` holds the positions of SWA, SWB, SWC and SWD in that order (`-1` low, `0` middle, `1` high, with a dead band of ±200).

To use a different auxiliary channel assignment, change the `FSI6X_CH_*` constants and the `ChannelIndex` enumerator at the top of `Fsi6x.hpp`; the bit unpacking needs no change.

## 3. 构造接口 / Constructor

```cpp
Fsi6x(LibXR::UART& uart,
      CMD& cmd,
      const Param& param = {.task_stack_depth_uart = 2048,
                            .thread_priority_uart = LibXR::Thread::Priority::HIGH});
```

依赖 / Dependencies：

- `uart`：连接 iA6B 的 `LibXR::UART`，取自 BSP 的硬件注册（`XR_REGISTER`）。C 板的 DBUS 接口对应 `usart3`。
- `cmd`：`CMD` 实例，接收解析后的控制数据。

配置参数（`Param`）：

- `task_stack_depth_uart`：接收线程栈深，默认 2048。
- `thread_priority_uart`：接收线程优先级，默认 `LibXR::Thread::Priority::HIGH`。

- `uart`: the `LibXR::UART` connected to the iA6B, taken from the BSP's registration (`XR_REGISTER`). The DBUS connector of board C maps to `usart3`.
- `cmd`: the `CMD` instance that receives the parsed control data.

Configuration parameters (`Param`):

- `task_stack_depth_uart`: receive thread stack depth, default 2048.
- `thread_priority_uart`: receive thread priority, default `LibXR::Thread::Priority::HIGH`.

## 4. Topic

无 / None

## 5. 配置示例 / Configuration Example

`xrobot instance add` 写入的实例，`uart` 填写为 BSP 中注册的 UART 名称，`cmd` 填写为 `starpang888/CMD` 实例的 id，该实例在 `modules:` 中排在 Fsi6x 之前：

An instance written by `xrobot instance add`, with `uart` set to a UART name registered by the BSP and `cmd` set to the id of the `starpang888/CMD` instance, which is listed before Fsi6x in `modules:`:

```yaml
modules:
  - module: starpang888/CMD
    id: cmd
  - module: starpang888/Fsi6x
    id: fsi6x
    args:
      - uart: usart3
      - cmd: cmd
      - param:
          task_stack_depth_uart: 1536
          thread_priority_uart: LibXR::Thread::Priority::HIGH
```

拨杆联动通过 EventBinder 写在配置里：

```yaml
  - module: starpang888/EventBinder
    id: event_binder
    args:
      - modules: '{{"fsi6x", fsi6x}, {"cmd", cmd}}'
      - event_binding_groups: '{{{{"fsi6x", Fsi6x::SwitchPos::FSI6X_SW_L_POS_TOP, "cmd", CMD::Mode::CMD_OP_CTRL}, {"fsi6x", Fsi6x::SwitchPos::FSI6X_SW_L_POS_BOT, "cmd", CMD::Mode::CMD_AUTO_CTRL}}}}'
```

## 6. 依赖与硬件 / Dependencies and Hardware

依赖 / Dependencies：

- `starpang888/CMD`：接收解析后的控制数据（`CMD::FeedRC`）。Fsi6x 占用其 `RC_INPUT_FSI6X` 输入槽。
- LibXR。

硬件 / Hardware：

- 连接 iA6B 的 SBUS 输出的 UART，并通过 `XR_REGISTER` 注册。波特率、校验与停止位由 Fsi6x 在构造时设置。
- **SBUS 是反相信号**，必须在硬件侧反相（反相电路或接收机的反相输出）；LibXR 的 `UART::Configuration` 没有反相选项，模块本身无法处理。

- `starpang888/CMD`: receives the parsed control data (`CMD::FeedRC`). Fsi6x occupies its `RC_INPUT_FSI6X` input slot.
- LibXR.

- A UART connected to the SBUS output of the iA6B and registered with `XR_REGISTER`; Fsi6x sets the baud rate, parity and stop bits at construction.
- **SBUS is an inverted signal** and must be inverted in hardware (an inverter circuit or an inverted output of the receiver). `LibXR::UART::Configuration` has no inversion option, so the Module cannot handle it.
