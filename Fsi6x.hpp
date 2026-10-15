#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: FS-i6X 遥控接收机 SBUS 解析模块：从 UART 接收 iA6B 的 SBUS 帧，解析通道与拨杆，转换为 CMD 控制数据并发出拨杆事件 / Module that parses FS-i6X plus iA6B SBUS frames from a UART, extracts channels and switches, converts them to CMD control data and emits switch events
depends:
- id: QDU-Robomaster/CMD
  ref: same-or-dev
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <cstddef>
#include <cstdint>

#include "CMD.hpp"
#include "event.hpp"
#include "libxr.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "uart.hpp"

/// SBUS 帧长度 / SBUS frame length
constexpr std::size_t FSI6X_FRAME_LENGTH = 25;
/// SBUS 帧头 / SBUS frame header
constexpr uint8_t FSI6X_FRAME_HEADER = 0x0F;
/// SBUS 帧尾 / SBUS frame footer
constexpr uint8_t FSI6X_FRAME_FOOTER = 0x00;
/// 通道低位实测值 / Measured low channel value
constexpr int16_t FSI6X_CH_VALUE_MIN = 240;
/// 通道中位值 / Middle channel value
constexpr int16_t FSI6X_CH_VALUE_MID = 1024;
/// 通道高位实测值 / Measured high channel value
constexpr int16_t FSI6X_CH_VALUE_MAX = 1807;
/// 拨杆中位判定死区 / Dead band for detecting the middle switch position
constexpr int16_t FSI6X_SW_DEADBAND = 200;
/// SBUS 波特率 / SBUS baud rate
constexpr uint32_t FSI6X_BAUDRATE = 100000;
/// 接收周期 / Receive period, in milliseconds
constexpr uint32_t FSI6X_RX_PERIOD_MS = 5;
/// 单次读取超时 / Single read timeout, in milliseconds
constexpr uint32_t FSI6X_RX_TIMEOUT_MS = 4;
/// 离线判定时间 / Offline timeout, in milliseconds
constexpr uint32_t FSI6X_OFFLINE_TIMEOUT_MS = 100;

/**
 * @brief FS-i6X 遥控接收机 SBUS 解析模块：从 UART 接收 iA6B 的 SBUS 帧，解析通道与
 *        拨杆，转换为 CMD 控制数据并发出拨杆事件。
 *        FS-i6X receiver parsing Module that receives SBUS frames from an iA6B over a
 *        UART, extracts channels and switches, converts them to CMD control data and
 *        emits switch events.
 */
class Fsi6x
{
 public:
  /**
   * @brief 拨杆位置事件 ID。编号与原 DR16 模块一致，便于事件绑定配置迁移。
   *        Switch position event IDs, numbered like the original DR16 Module so that
   *        event binding configurations stay portable.
   */
  enum class SwitchPos : uint8_t
  {
    FSI6X_SW_L_POS_TOP = 0x00,  ///< SWA 高位 SWA high position
    FSI6X_SW_L_POS_BOT = 0x01,  ///< SWA 低位 SWA low position
    FSI6X_SW_L_POS_MID = 0x02,  ///< SWA 中位 SWA middle position
    FSI6X_SW_R_POS_TOP = 0x03,  ///< SWC 高位 SWC high position
    FSI6X_SW_R_POS_BOT = 0x04,  ///< SWC 低位 SWC low position
    FSI6X_SW_R_POS_MID = 0x05,  ///< SWC 中位 SWC middle position
    FSI6X_SW_POS_NUM = 6        ///< 拨杆事件数量 Number of switch events
  };

  /**
   * @brief SBUS 通道下标，次序与 `SBUSData::ch` 一致。
   *        SBUS channel indices, matching the order of `SBUSData::ch`.
   */
  enum ChannelIndex : uint8_t
  {
    FSI6X_CH_R_X = 0,  ///< CH1 右摇杆左右 Right stick horizontal
    FSI6X_CH_R_Y = 1,  ///< CH2 右摇杆上下 Right stick vertical
    FSI6X_CH_L_Y = 2,  ///< CH3 左摇杆上下 Left stick vertical
    FSI6X_CH_L_X = 3,  ///< CH4 左摇杆左右 Left stick horizontal
    FSI6X_CH_SWC = 4,  ///< CH5 SWC，三档 SWC, three positions
    FSI6X_CH_SWA = 5,  ///< CH6 SWA，两档 SWA, two positions
    FSI6X_CH_SWB = 6,  ///< CH7 SWB，两档 SWB, two positions
    FSI6X_CH_SWD = 7,  ///< CH8 SWD，两档 SWD, two positions
    FSI6X_CH_VRA = 8,  ///< CH9 VRA 旋钮 VRA knob
    FSI6X_CH_VRB = 9,  ///< CH10 VRB 旋钮 VRB knob
    FSI6X_CH_NUM = 10  ///< 通道数量 Number of channels
  };

  /**
   * @brief 拨杆下标，次序与 `SBUSData::sw` 一致。
   *        Switch indices, matching the order of `SBUSData::sw`.
   */
  enum SwitchIndex : uint8_t
  {
    FSI6X_SW_A = 0,   ///< SWA SWA
    FSI6X_SW_B = 1,   ///< SWB SWB
    FSI6X_SW_C = 2,   ///< SWC SWC
    FSI6X_SW_D = 3,   ///< SWD SWD
    FSI6X_SW_NUM = 4  ///< 拨杆数量 Number of switches
  };

  /**
   * @brief 拨杆档位。
   *        Switch positions.
   */
  enum SwitchLevel : int8_t
  {
    FSI6X_SW_LEVEL_LOW = -1,  ///< 低位 Low position
    FSI6X_SW_LEVEL_MID = 0,   ///< 中位 Middle position
    FSI6X_SW_LEVEL_HIGH = 1   ///< 高位 High position
  };

  /**
   * @brief 一帧 SBUS 的解析结果，通道保留原始值。
   *        Parsed result of one SBUS frame, with raw channel values kept.
   */
  struct SBUSData
  {
    int16_t ch[FSI6X_CH_NUM];  ///< CH1 至 CH10 原始值，中位约 1024
                               ///< Raw CH1 to CH10 values, middle around 1024
    int8_t sw[FSI6X_SW_NUM];   ///< SWA、SWB、SWC、SWD 档位，取 -1、0 或 1
                               ///< SWA, SWB, SWC and SWD positions, -1, 0 or 1
    uint8_t frame_lost;        ///< SBUS 丢帧标志 SBUS frame lost flag
    uint8_t failsafe;          ///< SBUS 失控标志 SBUS failsafe flag
  };

  /**
   * @brief Fsi6x 配置参数。
   *        Fsi6x configuration parameters.
   */
  struct Param
  {
    uint32_t task_stack_depth_uart;  ///< 接收线程栈深
                                     ///< Receive thread stack depth
    LibXR::Thread::Priority thread_priority_uart;  ///< 接收线程优先级
                                                   ///< Receive thread priority
  };

  /**
   * @brief 构造 Fsi6x，配置 UART 并创建接收线程。
   *        Construct Fsi6x, configure the UART and create the receive thread.
   *
   * @param uart 连接 iA6B 的 UART，被设置为 100000 bit/s、偶校验、8 数据位、
   *             2 停止位。SBUS 的反相需要在硬件侧完成。
   *             UART connected to the iA6B, set to 100000 bit/s, even parity, 8 data
   *             bits and 2 stop bits. SBUS inversion must be done in hardware.
   * @param cmd CMD 实例，接收解析后的控制数据。
   *            CMD instance that receives the parsed control data.
   * @param param 配置参数。
   *             Configuration parameters.
   */
  Fsi6x(LibXR::UART& uart, CMD& cmd,
        const Param& param = {.task_stack_depth_uart = 2048,
                              .thread_priority_uart = LibXR::Thread::Priority::HIGH})
      : cmd_(&cmd), uart_(&uart), sem_(0), op_(sem_, FSI6X_RX_TIMEOUT_MS)
  {
    uart_->SetConfig({FSI6X_BAUDRATE, LibXR::UART::Parity::EVEN, 8, 2});
    thread_uart_.Create(this, ThreadFsi6x, "uart_fsi6x", param.task_stack_depth_uart,
                        param.thread_priority_uart);
  }

  /**
   * @brief 获取 Fsi6x 的事件对象，SWA 与 SWC 的档位变化在其上触发。
   *        Get the event object of Fsi6x, on which the SWA and SWC position changes are
   *        activated.
   *
   * @return 事件对象的引用。
   *         Reference to the event object.
   */
  LibXR::Event& GetEvent() { return fsi6x_event_; }

  /**
   * @brief 获取最近一帧 SBUS 的原始数据，供应用层使用 SWB、SWD 与旋钮。
   *        Get the raw data of the latest SBUS frame, for the application layer to use
   *        SWB, SWD and the knobs.
   *
   * @return 最近一帧的解析结果，尚未收到有效帧时各字段为 0。
   *         Parsed result of the latest frame; all fields are 0 before the first valid
   *         frame.
   */
  SBUSData GetSBUSData() const { return sbus_data_; }

  /**
   * @brief 接收线程函数：每 5 ms 读取并解析一帧 SBUS 数据，超过 100 ms 无有效帧时
   *        判定为离线。
   *        Receive thread function that reads and parses one SBUS frame every 5 ms and
   *        treats Fsi6x as offline when no valid frame arrives for more than 100 ms.
   *
   * @param fsi6x Fsi6x 实例指针。
   *             Pointer to the Fsi6x instance.
   */
  static void ThreadFsi6x(Fsi6x* fsi6x)
  {
    uint8_t rx_buffer[FSI6X_FRAME_LENGTH] = {0};
    CMD::Data rc_data;

    auto last_tick = LibXR::Timebase::GetMilliseconds();
    while (1)
    {
      if (fsi6x->uart_->Read({rx_buffer, FSI6X_FRAME_LENGTH}, fsi6x->op_) ==
          LibXR::ErrorCode::OK)
      {
        if (fsi6x->ParseSBUS(rx_buffer, rc_data) == LibXR::ErrorCode::OK)
        {
          fsi6x->last_time_ = LibXR::Timebase::GetMilliseconds();
          fsi6x->cmd_->FeedRC(CMD::RCInputSource::RC_INPUT_DR16, rc_data);
        }
        else
        {
          LibXR::Memory::FastSet(rx_buffer, 0, FSI6X_FRAME_LENGTH);
        }
      }
      fsi6x->CheckoutOffline();
      LibXR::Thread::SleepUntil(last_tick, FSI6X_RX_PERIOD_MS);
    }
  }

  /**
   * @brief 解析 SBUS 原始数据，生成 CMD 控制数据并触发拨杆事件。
   *        Parse raw SBUS data, generate the CMD control data and activate the switch
   *        events.
   *
   * @param raw_data 25 字节的 SBUS 原始数据。
   *                 25 bytes of raw SBUS data.
   * @param output_data 输出的 CMD 控制数据。
   *                    Output CMD control data.
   * @return `OK` 表示解析成功；`PTR_NULL` 表示 `raw_data` 为空；`CHECK_ERR` 表示
   *         帧头帧尾不符，或接收机报告丢帧、失控。
   *         `OK` on success; `PTR_NULL` when `raw_data` is null; `CHECK_ERR` when the
   *         header or footer does not match, or the receiver reports frame loss or
   *         failsafe.
   */
  LibXR::ErrorCode ParseSBUS(const uint8_t* raw_data, CMD::Data& output_data)
  {
    if (!raw_data)
    {
      return LibXR::ErrorCode::PTR_NULL;
    }

    if (raw_data[0] != FSI6X_FRAME_HEADER ||
        raw_data[FSI6X_FRAME_LENGTH - 1] != FSI6X_FRAME_FOOTER)
    {
      return LibXR::ErrorCode::CHECK_ERR;
    }

    SBUSData curr_sbus{};

    curr_sbus.ch[FSI6X_CH_R_X] =
        static_cast<int16_t>((raw_data[1] | raw_data[2] << 8) & 0x07FF);
    curr_sbus.ch[FSI6X_CH_R_Y] =
        static_cast<int16_t>(((raw_data[2] >> 3) | (raw_data[3] << 5)) & 0x07FF);
    curr_sbus.ch[FSI6X_CH_L_Y] = static_cast<int16_t>(
        (((raw_data[3] >> 6) | (raw_data[4] << 2) | (raw_data[5] << 10)) & 0x07FF));
    curr_sbus.ch[FSI6X_CH_L_X] =
        static_cast<int16_t>(((raw_data[5] >> 1) | (raw_data[6] << 7)) & 0x07FF);
    curr_sbus.ch[FSI6X_CH_SWC] =
        static_cast<int16_t>(((raw_data[6] >> 4) | (raw_data[7] << 4)) & 0x07FF);
    curr_sbus.ch[FSI6X_CH_SWA] = static_cast<int16_t>(
        (((raw_data[7] >> 7) | (raw_data[8] << 1) | (raw_data[9] << 9)) & 0x07FF));
    curr_sbus.ch[FSI6X_CH_SWB] =
        static_cast<int16_t>(((raw_data[9] >> 2) | (raw_data[10] << 6)) & 0x07FF);
    curr_sbus.ch[FSI6X_CH_SWD] =
        static_cast<int16_t>(((raw_data[10] >> 5) | (raw_data[11] << 3)) & 0x07FF);
    curr_sbus.ch[FSI6X_CH_VRA] =
        static_cast<int16_t>((raw_data[12] | raw_data[13] << 8) & 0x07FF);
    curr_sbus.ch[FSI6X_CH_VRB] =
        static_cast<int16_t>(((raw_data[13] >> 3) | (raw_data[14] << 5)) & 0x07FF);

    static constexpr ChannelIndex SWITCH_CHANNEL[FSI6X_SW_NUM] = {
        FSI6X_CH_SWA, FSI6X_CH_SWB, FSI6X_CH_SWC, FSI6X_CH_SWD};
    for (uint8_t i = 0; i < FSI6X_SW_NUM; i++)
    {
      curr_sbus.sw[i] = ToSwitchLevel(curr_sbus.ch[SWITCH_CHANNEL[i]]);
    }

    curr_sbus.frame_lost = static_cast<uint8_t>((raw_data[23] >> 2) & 0x01);
    curr_sbus.failsafe = static_cast<uint8_t>((raw_data[23] >> 3) & 0x01);

    this->sbus_data_ = curr_sbus;

    if (curr_sbus.failsafe != 0 || curr_sbus.frame_lost != 0)
    {
      return LibXR::ErrorCode::CHECK_ERR;
    }

    /* 首帧只记录状态，不产生事件，避免上电瞬间误触发模式切换 */
    if (this->sbus_valid_)
    {
      if (curr_sbus.sw[FSI6X_SW_A] != this->last_sbus_.sw[FSI6X_SW_A])
      {
        this->fsi6x_event_.Active(
            SwitchEventId(SwitchPos::FSI6X_SW_L_POS_TOP, curr_sbus.sw[FSI6X_SW_A]));
      }
      if (curr_sbus.sw[FSI6X_SW_C] != this->last_sbus_.sw[FSI6X_SW_C])
      {
        this->fsi6x_event_.Active(
            SwitchEventId(SwitchPos::FSI6X_SW_R_POS_TOP, curr_sbus.sw[FSI6X_SW_C]));
      }
    }
    this->last_sbus_ = curr_sbus;
    this->sbus_valid_ = true;

    const float right_x = NormalizeChannel(curr_sbus.ch[FSI6X_CH_R_X]);
    const float right_y = NormalizeChannel(curr_sbus.ch[FSI6X_CH_R_Y]);
    const float left_y = NormalizeChannel(curr_sbus.ch[FSI6X_CH_L_Y]);
    const float left_x = NormalizeChannel(curr_sbus.ch[FSI6X_CH_L_X]);

    output_data = CMD::Data();

    output_data.chassis.x = left_x;    // 左摇杆左右，右为正
    output_data.chassis.y = left_y;    // 左摇杆上下，上为正
    output_data.chassis.z = right_x;   // 右摇杆左右，左为正（左转）
    output_data.gimbal.yaw = right_x;  // 右摇杆左右，左为正（左转）
    output_data.gimbal.pit = right_y;  // 右摇杆上下，上为正

    /* SWB 与 SWD 保留给应用层：这里不产生控制量，需要开火或加减速时在应用层读取
       GetSBUSData()，或改由事件绑定驱动其他模块。 */
    output_data.chassis.self_define = CMD::ChasStat::NONE;
    output_data.launcher.isfire = false;

    output_data.chassis.x = std::clamp(output_data.chassis.x, -1.0f, 1.0f);
    output_data.chassis.y = std::clamp(output_data.chassis.y, -1.0f, 1.0f);
    output_data.chassis.z = std::clamp(output_data.chassis.z, -1.0f, 1.0f);
    output_data.gimbal.yaw = std::clamp(output_data.gimbal.yaw, -1.0f, 1.0f);
    output_data.gimbal.pit = std::clamp(output_data.gimbal.pit, -1.0f, 1.0f);

    output_data.chassis_online = true;
    output_data.gimbal_online = true;
    output_data.ctrl_source = CMD::ControlSource::CTRL_SOURCE_RC;

    return LibXR::ErrorCode::OK;
  }

  /**
   * @brief 向 CMD 提交离线数据：控制量全零，底盘与云台在线标志为 false。
   *        Submit offline data to CMD: all control values zero and the chassis and gimbal
   *        online flags false.
   */
  void Offline()
  {
    cmd_data_ = CMD::Data();

    cmd_data_.chassis_online = false;
    cmd_data_.gimbal_online = false;
    cmd_data_.ctrl_source = CMD::ControlSource::CTRL_SOURCE_RC;

    cmd_->FeedRC(CMD::RCInputSource::RC_INPUT_DR16, cmd_data_);
  }

 private:
  CMD* cmd_;                                 /* CMD 模块指针 */
  LibXR::UART* uart_;                        /* UART 接口指针 */
  SBUSData last_sbus_{};                     /* 上一帧 SBUS 数据 */
  SBUSData sbus_data_{};                     /* 最近一帧 SBUS 数据 */
  CMD::Data cmd_data_{};                     /* 命令数据 */
  LibXR::Event fsi6x_event_;                 /* 事件处理器 */
  LibXR::Thread thread_uart_;                /* UART 线程 */
  LibXR::Semaphore sem_;                     /* 读操作信号量 */
  LibXR::ReadOperation op_;                  /* 读操作（阻塞型） */
  LibXR::MillisecondTimestamp last_time_{};  /* 上次接收时间 */
  bool sbus_valid_ = false;                  /* 是否已收到过有效帧 */

  /**
   * @brief 把通道原始值归一化到 [-1, 1]，两侧分别按实测行程缩放。
   *        Normalize a raw channel value to [-1, 1], scaling each side by its measured
   *        travel.
   *
   * @param value 通道原始值。
   *              Raw channel value.
   * @return 归一化后的控制量。
   *         Normalized control value.
   */
  static float NormalizeChannel(int16_t value)
  {
    constexpr float POSITIVE_SPAN =
        static_cast<float>(FSI6X_CH_VALUE_MAX - FSI6X_CH_VALUE_MID);
    constexpr float NEGATIVE_SPAN =
        static_cast<float>(FSI6X_CH_VALUE_MID - FSI6X_CH_VALUE_MIN);

    const float offset = static_cast<float>(value - FSI6X_CH_VALUE_MID);
    return offset >= 0.0f ? offset / POSITIVE_SPAN : offset / NEGATIVE_SPAN;
  }

  /**
   * @brief 按死区把通道原始值判定为拨杆档位。
   *        Turn a raw channel value into a switch position, using a dead band.
   *
   * @param value 通道原始值。
   *              Raw channel value.
   * @return `-1` 低位、`0` 中位或 `1` 高位。
   *         `-1` low, `0` middle or `1` high.
   */
  static int8_t ToSwitchLevel(int16_t value)
  {
    if (value > FSI6X_CH_VALUE_MID + FSI6X_SW_DEADBAND)
    {
      return FSI6X_SW_LEVEL_HIGH;
    }
    if (value < FSI6X_CH_VALUE_MID - FSI6X_SW_DEADBAND)
    {
      return FSI6X_SW_LEVEL_LOW;
    }
    return FSI6X_SW_LEVEL_MID;
  }

  /**
   * @brief 由拨杆档位计算事件 ID：高位、低位、中位依次对应基准 ID 加 0、1、2。
   *        Compute the event ID from a switch position: high, low and middle map to the
   *        base ID plus 0, 1 and 2.
   *
   * @param base 该拨杆高位的基准事件 ID。
   *             Base event ID of the switch high position.
   * @param level 拨杆档位。
   *              Switch position.
   * @return 事件 ID。
   *         Event ID.
   */
  static uint32_t SwitchEventId(SwitchPos base, int8_t level)
  {
    if (level > FSI6X_SW_LEVEL_MID)
    {
      return static_cast<uint32_t>(base);
    }
    if (level < FSI6X_SW_LEVEL_MID)
    {
      return static_cast<uint32_t>(base) + 1;
    }
    return static_cast<uint32_t>(base) + 2;
  }

  /**
   * @brief 超过 100 ms 没有有效帧时向 CMD 提交离线数据。
   *        Submit offline data to CMD when no valid frame has arrived for more than
   *        100 ms.
   */
  void CheckoutOffline()
  {
    auto current_time = LibXR::Timebase::GetMilliseconds();
    if ((current_time - last_time_).ToMillisecond() > FSI6X_OFFLINE_TIMEOUT_MS)
    {
      Offline();
    }
  }
};
