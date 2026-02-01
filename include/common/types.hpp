/**
 * @file types.hpp
 * @brief 通用数据类型定义 - 与STM32保持一致
 * @author Zomnk
 * @date 2026-02-01
 */

#ifndef ODROID_COMMON_TYPES_HPP
#define ODROID_COMMON_TYPES_HPP

#include <cstdint>
#include <array>
#include <string>

namespace odroid {

//==============================================================================
// 电机相关常量 (与STM32 ita_robot.h 保持一致)
//==============================================================================

// 电机类型
enum class MotorType : uint8_t {
    DM6006 = 0,     // Yaw, Knee, Ankle
    DM4340 = 1,     // Roll
    DM8006 = 2,     // Pitch
};

// 关节索引
enum class JointIndex : uint8_t {
    Yaw   = 0,
    Roll  = 1,
    Pitch = 2,
    Knee  = 3,
    Ankle = 4,
};

// 每条腿电机数量
constexpr size_t MOTORS_PER_LEG = 5;
// 总电机数量
constexpr size_t TOTAL_MOTORS = 10;

// 电机参数范围 (与STM32保持一致)
struct MotorParams {
    float pos_max;      // 位置最大值 (rad)
    float vel_max;      // 速度最大值 (rad/s)
    float torque_max;   // 扭矩最大值 (Nm)
};

// DM6006参数 (Yaw, Knee, Ankle)
constexpr MotorParams DM6006_PARAMS = {12.5f, 45.0f, 12.0f};
// DM4340参数 (Roll)
constexpr MotorParams DM4340_PARAMS = {12.5f, 10.0f, 27.0f};
// DM8006参数 (Pitch)
constexpr MotorParams DM8006_PARAMS = {12.5f, 45.0f, 20.0f};

// 各关节对应的电机参数
constexpr MotorParams JOINT_PARAMS[MOTORS_PER_LEG] = {
    DM6006_PARAMS,  // Yaw
    DM4340_PARAMS,  // Roll
    DM8006_PARAMS,  // Pitch
    DM6006_PARAMS,  // Knee
    DM6006_PARAMS,  // Ankle
};

// IMU参数范围
constexpr float GYRO_MAX = 2000.0f;     // deg/s
constexpr float GYRO_MIN = -2000.0f;
constexpr float ACCEL_MAX = 16.0f;      // g
constexpr float ACCEL_MIN = -16.0f;
constexpr float RPY_MAX = 3.14159265f;  // rad (PI)
constexpr float RPY_MIN = -3.14159265f;
constexpr float TEMP_MAX = 100.0f;      // °C
constexpr float TEMP_MIN = 0.0f;

// Kp/Kd范围
constexpr float KP_MAX = 500.0f;
constexpr float KP_MIN = 0.0f;
constexpr float KD_MAX = 5.0f;
constexpr float KD_MIN = 0.0f;

//==============================================================================
// SPI通信数据布局 (与STM32保持一致)
//==============================================================================

// 控制指令: 10电机 × 4参数 = 40个uint16 = 80字节
constexpr size_t CONTROL_DATA_NUM = 40;
// 反馈数据: 10电机 × 4参数 + 2IMU × 10参数 = 60个uint16 = 120字节
constexpr size_t FEEDBACK_DATA_NUM = 60;
// SPI传输长度 (取最大值)
constexpr size_t SPI_TRANSFER_SIZE = FEEDBACK_DATA_NUM;

//==============================================================================
// 数据结构定义
//==============================================================================

/**
 * @brief 单电机控制指令
 */
struct MotorCommand {
    float position = 0.0f;      // 目标位置 (rad)
    float velocity = 0.0f;      // 目标速度 (rad/s)
    float kp = 0.0f;            // 位置刚度
    float kd = 0.0f;            // 阻尼系数
    
    // 默认构造
    MotorCommand() = default;
    
    // 便捷构造
    MotorCommand(float pos, float vel, float kp_, float kd_)
        : position(pos), velocity(vel), kp(kp_), kd(kd_) {}
};

/**
 * @brief 单电机反馈数据
 */
struct MotorFeedback {
    float position = 0.0f;      // 当前位置 (rad)
    float velocity = 0.0f;      // 当前速度 (rad/s)
    float torque = 0.0f;        // 当前扭矩 (Nm)
    float temperature = 0.0f;   // 转子温度 (°C)
};

/**
 * @brief 单腿控制指令
 */
struct LegCommand {
    MotorCommand yaw;
    MotorCommand roll;
    MotorCommand pitch;
    MotorCommand knee;
    MotorCommand ankle;
    
    // 数组访问
    MotorCommand& operator[](size_t idx) {
        return reinterpret_cast<MotorCommand*>(this)[idx];
    }
    const MotorCommand& operator[](size_t idx) const {
        return reinterpret_cast<const MotorCommand*>(this)[idx];
    }
};

/**
 * @brief 单腿反馈数据
 */
struct LegFeedback {
    MotorFeedback yaw;
    MotorFeedback roll;
    MotorFeedback pitch;
    MotorFeedback knee;
    MotorFeedback ankle;
    
    // 数组访问
    MotorFeedback& operator[](size_t idx) {
        return reinterpret_cast<MotorFeedback*>(this)[idx];
    }
    const MotorFeedback& operator[](size_t idx) const {
        return reinterpret_cast<const MotorFeedback*>(this)[idx];
    }
};

/**
 * @brief IMU反馈数据
 */
struct IMUFeedback {
    float gyro_x = 0.0f;        // 角速度X (deg/s)
    float gyro_y = 0.0f;        // 角速度Y (deg/s)
    float gyro_z = 0.0f;        // 角速度Z (deg/s)
    float accel_x = 0.0f;       // 加速度X (g)
    float accel_y = 0.0f;       // 加速度Y (g)
    float accel_z = 0.0f;       // 加速度Z (g)
    float roll = 0.0f;          // 横滚角 (rad)
    float pitch = 0.0f;         // 俯仰角 (rad)
    float yaw = 0.0f;           // 偏航角 (rad)
    float temperature = 0.0f;   // 温度 (°C)
};

/**
 * @brief 机器人控制指令 (发送给STM32)
 */
struct RobotCommand {
    LegCommand left;            // 左腿 (20个uint16)
    LegCommand right;           // 右腿 (20个uint16)
    uint64_t timestamp_us = 0;  // 时间戳 (微秒)
    uint32_t sequence = 0;      // 序列号
};

/**
 * @brief 机器人反馈数据 (从STM32接收)
 */
struct RobotFeedback {
    LegFeedback left;           // 左腿 (20个uint16)
    LegFeedback right;          // 右腿 (20个uint16)
    IMUFeedback icm20602;       // ICM20602 IMU (10个uint16)
    IMUFeedback waveshare;      // 微雪IMU (10个uint16)
    uint64_t timestamp_us = 0;  // 接收时间戳
    uint32_t sequence = 0;      // 序列号
};

/**
 * @brief SPI原始缓冲区
 */
struct SPIBuffer {
    std::array<uint16_t, SPI_TRANSFER_SIZE> tx;
    std::array<uint16_t, SPI_TRANSFER_SIZE> rx;
    
    SPIBuffer() {
        tx.fill(32768);  // 零点 (中间值)
        rx.fill(0);
    }
};

//==============================================================================
// 状态枚举
//==============================================================================

/**
 * @brief 通信状态
 */
enum class CommStatus {
    Disconnected = 0,
    Connected,
    Error,
    Timeout,
};

/**
 * @brief 系统状态
 */
enum class SystemStatus {
    Idle = 0,
    Running,
    EmergencyStop,
    Error,
};

} // namespace odroid

#endif // ODROID_COMMON_TYPES_HPP
