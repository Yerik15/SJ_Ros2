/***
 * @Author: _yerik
 * @Date: 2025-07-21 11:08:24
 * @LastEditTime: 2025-07-21 11:08:50
 * @LastEditors: _yerik
 * @Description:
 * @FilePath: /Simple_Joint/SJ_ws/SJ_Ros2/SJ_hw/include/SJ_hw/damiao.h
 * @Code. Run. No errors.
 */
#ifndef DAMIAO_H
#define DAMIAO_H

#include <array>
#include <cmath>
#include <cstdint>
#include <libserial/SerialPort.h>
#include <thread>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>
#include <atomic>
#include <cstring>

// 控制模式相关宏定义
#define POS_MODE 0x100       // 位置模式
#define SPEED_MODE 0x200     // 速度模式
#define POSI_MODE 0x300      // 增量位置模式
#define max_retries 20       // 最大重试次数
#define retry_interval 50000 // 重试间隔（微秒）

namespace damiao
{
#pragma pack(1)
#define Motor_id uint32_t // 电机ID类型定义

  // 电机类型
  enum DM_Motor_Type
  {
    DM4310,
    DM4310_48V,
    DM4340,
    DM4340_48V,
    DM6006,
    DM8006,
    DM8009,
    DM10010L,
    DM10010,
    DMH3510,
    DMH6215,
    DMG6220,
    Num_Of_Motor
  }; // 电机类型数量

  // 电机控制模式
  enum Control_Mode
  {
    MIT_MODE = 1,       // MIT模式
    POS_VEL_MODE = 2,   // 位置速度模式
    VEL_MODE = 3,       // 速度模式
    POS_FORCE_MODE = 4, // 位置力矩模式
  };

  // 寄存器列表 具体参考达妙手册
  enum DM_REG
  {
    UV_Value = 0,
    KT_Value = 1,
    OT_Value = 2,
    OC_Value = 3,
    ACC = 4,
    DEC = 5,
    MAX_SPD = 6,
    MST_ID = 7,
    ESC_ID = 8,
    TIMEOUT = 9,
    CTRL_MODE = 10,
    Damp = 11,
    Inertia = 12,
    hw_ver = 13,
    sw_ver = 14,
    SN = 15,
    NPP = 16,
    Rs = 17,
    LS = 18,
    Flux = 19,
    Gr = 20,
    PMAX = 21,
    VMAX = 22,
    TMAX = 23,
    I_BW = 24,
    KP_ASR = 25,
    KI_ASR = 26,
    KP_APR = 27,
    KI_APR = 28,
    OV_Value = 29,
    GREF = 30,
    Deta = 31,
    V_BW = 32,
    IQ_c1 = 33,
    VL_c1 = 34,
    can_br = 35,
    sub_ver = 36,
    u_off = 50,
    v_off = 51,
    k1 = 52,
    k2 = 53,
    m_off = 54,
    dir = 55,
    p_m = 80,
    xout = 81,
  };

  // CAN通信接收帧结构体
  typedef struct
  {
    uint8_t FrameHeader;    // 帧头
    uint8_t CMD;            // 命令 0x00: 心跳
                            //     0x01: receive fail 0x11: receive success
                            //     0x02: send fail 0x12: send success
                            //     0x03: set baudrate fail 0x13: set baudrate success
                            //     0xEE: communication error 此时格式段为错误码
                            //     8: 超压 9: 欠压 A: 过流 B: MOS过温 C: 电机线圈过温 D: 通讯丢失 E: 过载
    uint8_t canDataLen : 6; // 数据长度（6位）
    uint8_t canIde : 1;     // 帧类型（0标准帧，1扩展帧）
    uint8_t canRtr : 1;     // 远程帧标志（0数据帧，1远程帧）
    uint32_t canId;         // CAN ID（电机反馈ID）
    uint8_t canData[8];     // 数据区
    uint8_t frameEnd;       // 帧尾
  } CAN_Receive_Frame;

  // CAN发送帧结构体
  typedef struct can_send_frame
  {
    uint8_t FrameHeader[2] = {0x55, 0xAA}; // 帧头
    uint8_t FrameLen = 0x1e;               // 帧长
    uint8_t CMD = 0x03;                    // 命令 1：转发CAN数据帧 2：PC与设备握手，设备反馈OK 3:
                                           // 非反馈CAN转发，不反馈发送状态
    uint32_t sendTimes = 1;                // 发送次数
    uint32_t timeInterval = 10;            // 发送间隔
    uint8_t IDType = 0;                    // ID类型（0标准帧，1扩展帧）
    uint32_t canId = 0x01;                 // CAN ID
    uint8_t frameType = 0;                 // 帧类型（0数据帧，1远程帧）
    uint8_t len = 0x08;                    // 数据长度
    uint8_t idAcc = 0;
    uint8_t dataAcc = 0;
    uint8_t data[8] = {0}; // 数据区
    uint8_t crc = 0;       // CRC校验（未解析）

    // 修改CAN发送帧内容
    void modify(const Motor_id id, const uint8_t *send_data)
    {
      canId = id;
      std::copy(send_data, send_data + 8, data);
    }

  } can_send_frame;

#pragma pack() // 恢复默认字节对齐

  // 电机限制参数结构体
  typedef struct
  {
    float Q_MAX;   // 最大速度
    float DQ_MAX;  // 最大加速度
    float TAU_MAX; // 最大力矩
  } Limit_param;

  // 电机PMAX DQMAX TAUMAX参数
  extern Limit_param limit_param[Num_Of_Motor];

  // 关节数据结构体
  struct DmActData
  {
    std::string name;                    // 关节名称
    DM_Motor_Type motorType;             // 电机类型
    int can_id;                          // CAN ID
    int mst_id;                          // 主机ID
    double pos, vel, effort;             // 实时位置、速度、力矩
    double cmd_pos, cmd_vel, cmd_effort; // 控制命令：目标位置、速度、力矩
    double kp, kd;                       // 控制参数
  };

  // 电机对象类
  class Motor
  {
  private:
    Motor_id Master_id;        // 主机ID
    Motor_id Slave_id;         // 从机ID（电机ID）
    float state_q = 0;         // 当前位置
    float state_dq = 0;        // 当前速度
    float state_tau = 0;       // 当前力矩
    Limit_param limit_param{}; // 限制参数
    DM_Motor_Type Motor_Type;  // 电机类型

    // 参数联合体（float/uint32）
    union ValueUnion
    {
      float floatValue;
      uint32_t uint32Value;
    };

    // 参数类型结构体
    struct ValueType
    {
      ValueUnion value;
      bool isFloat;
    };

    // 参数映射表
    std::unordered_map<uint32_t, ValueType> param_map;

  public:
    /**
     * @brief 构造函数
     *
     * @param Motor_Type 电机类型
     * @param Slave_id CAN ID
     * @param Master_id 主机ID
     */
    Motor(DM_Motor_Type Motor_Type, Motor_id Slave_id, Motor_id Master_id);

    // 接收电机数据
    void receive_data(float q, float dq, float tau);

    // 获取电机类型
    DM_Motor_Type GetMotorType() const { return this->Motor_Type; }

    // 获取主机ID
    Motor_id GetMasterId() const { return this->Master_id; }

    // 获取从机ID
    Motor_id GetSlaveId() const { return this->Slave_id; }

    // 获取当前状态
    float Get_Position() const { return this->state_q; }
    float Get_Velocity() const { return this->state_dq; }
    float Get_tau() const { return this->state_tau; }

    // 获取限制参数
    Limit_param get_limit_param() { return limit_param; }

    // 设置/获取参数
    void set_param(int key, float value);
    void set_param(int key, uint32_t value);
    float get_param_as_float(int key) const;
    uint32_t get_param_as_uint32(int key) const;
    bool is_have_param(int key) const;
  };

  /**
   * @brief 电机控制类
   *
   * 使用USB转CAN进行通信，linux做虚拟串口
   */
  class Motor_Control
  {
  public:
    /*
     * @brief 构造函数
     * @param serial_port 串口名
     * @param baud_rate 波特率
     * @param data_ptr 关节数据指针
     */
    Motor_Control(std::string serial_port, int baud_rate,
                  std::unordered_map<int, DmActData> *data_ptr);

    ~Motor_Control();

    // 读取串口电机数据线程
    void get_motor_data_thread();

    // 使能电机
    void enable();

    // 写入命令到电机
    void write();

    // 读取电机数据
    void read();

    // 刷新电机状态
    void refresh_motor_status(const Motor &motor);

    // 失能电机
    void disable();

    // 设置零点
    void set_zero_position();

    // 控制命令（MIT/位置速度/速度）
    void control_mit(Motor &DM_Motor, float kp, float kd, float q, float dq, float tau);
    void control_pos_vel(Motor &DM_Motor, float pos, float vel);
    void control_vel(Motor &DM_Motor, float vel);

    // 接收数据
    void receive();

    // 接收参数
    void receive_param();

    // 添加电机对象
    void addMotor(Motor *DM_Motor);

    // 读取电机寄存器参数
    float read_motor_param(Motor &DM_Motor, uint8_t RID);

    // 切换电机控制模式
    bool switchControlMode(Motor &DM_Motor, Control_Mode mode);

    // 修改电机寄存器参数
    bool change_motor_param(Motor &DM_Motor, uint8_t RID, float data);

    // 保存所有参数到电机flash
    void save_motor_param(Motor &DM_Motor);

    // 修改电机限制参数（非寄存器参数）
    static void changeMotorLimit(Motor &DM_Motor, float P_MAX, float Q_MAX, float T_MAX);

  private:
    // 写数据到串口
    void WriteData(const can_send_frame &frame);

    // 读数据（带超时）
    bool ReadData(CAN_Receive_Frame &frame, const size_t timeout_ms = 100);

    // 控制命令
    void control_cmd(Motor_id id, uint8_t cmd);

    // 写寄存器参数
    void write_motor_param(Motor &DM_Motor, uint8_t RID, const uint8_t data[4]);

    // 判断寄存器ID是否在范围内
    static bool is_in_ranges(int number)
    {
      return (7 <= number && number <= 10) || (13 <= number && number <= 16) ||
             (35 <= number && number <= 36);
    }

    // 类型转换辅助函数
    static uint32_t float_to_uint32(float value)
    {
      return static_cast<uint32_t>(value);
    }
    static float uint32_to_float(uint32_t value)
    {
      return static_cast<float>(value);
    }
    static float uint8_to_float(const uint8_t data[4])
    {
      uint32_t combined = (static_cast<uint32_t>(data[3]) << 24) |
                          (static_cast<uint32_t>(data[2]) << 16) |
                          (static_cast<uint32_t>(data[1]) << 8) |
                          static_cast<uint32_t>(data[0]);
      float result;
      memcpy(&result, &combined, sizeof(result));
      return result;
    }

    // 线程与数据成员
    std::thread rec_thread; // 接收线程
    // std::unique_ptr<std::thread> rec_thread;
    std::unordered_map<Motor_id, Motor *> motors;  // 电机对象映射表
    LibSerial::SerialPort serial_;                 // 串口对象
    std::unordered_map<int, DmActData> *data_ptr_; // 关节数据指针
    // bool stop_thread_ = false;
    std::atomic<bool> stop_thread_;   // 线程停止标志
    can_send_frame send_data;         // 发送数据帧
    CAN_Receive_Frame receive_data{}; // 接收数据帧
  };

}; // namespace damiao

#endif