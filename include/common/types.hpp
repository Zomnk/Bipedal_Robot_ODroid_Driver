/**
 * @file types.hpp
 * @brief 数据类型定义
 * @author Zomnk
 * @date 2026-02-01
 */

#ifndef ODROID_COMMON_TYPES_HPP
#define ODROID_COMMON_TYPES_HPP

#include <cstdint>
#include "constants.hpp"

namespace odroid {

//==============================================================================
// 电机相关类型
//==============================================================================

/**
 * @brief 单个电机控制指令
 */
struct MotorCommand {
    float position = 0.0f;    // 目标位置 (rad)
    float velocity = 0.0f;    // 目标速度 (rad/s)
    float kp = 0.0f;          // 位置增益
    float kd = 0.0f;          // 速度增益
    float feedforward = 0.0f; // 前馈力矩 (Nm) - 暂未使用
};

/**
 * @brief 单个电机反馈数据
 */
struct MotorFeedback {
    float position = 0.0f;    // 当前位置 (rad)
    float velocity = 0.0f;    // 当前速度 (rad/s)
    float torque = 0.0f;      // 当前力矩 (Nm)
    float temperature = 0.0f; // 温度 (C)
};

//==============================================================================
// 腿部相关类型 (每条腿5个关节)
//==============================================================================

/**
 * @brief 单条腿控制指令 (5个关节)
 */
struct LegCommand {
    MotorCommand hip;         // 髋关节 (DM6006)
    MotorCommand thigh;       // 大腿 (DM4340)
    MotorCommand calf;        // 小腿 (DM4340)
    MotorCommand knee;        // 膝关节 (DM4340)
    MotorCommand ankle;       // 踝关节 (DM8006)
};

/**
 * @brief 单条腿反馈数据 (5个关节)
 */
struct LegFeedback {
    MotorFeedback hip;        // 髋关节 (DM6006)
    MotorFeedback thigh;      // 大腿 (DM4340)
    MotorFeedback calf;       // 小腿 (DM4340)
    MotorFeedback knee;       // 膝关节 (DM4340)
    MotorFeedback ankle;      // 踝关节 (DM8006)
};

//==============================================================================
// IMU相关类型
//==============================================================================

/**
 * @brief IMU反馈数据 (10个uint16_t = 20字节)
 */
struct IMUFeedback {
    float accel[3] = {0};     // 加速度 x,y,z (m/s^2)
    float gyro[3] = {0};      // 角速度 x,y,z (deg/s)
    float euler[3] = {0};     // 欧拉角 roll,pitch,yaw (deg)
    float temperature = 0.0f; // 温度 (C)
};

//==============================================================================
// 机器人整体类型
//==============================================================================

/**
 * @brief 机器人控制指令 (10个电机 = 40 words)
 */
struct RobotCommand {
    LegCommand left_leg;      // 左腿 (5个关节)
    LegCommand right_leg;     // 右腿 (5个关节)
    uint64_t timestamp_us = 0;// 时间戳
};

/**
 * @brief 机器人反馈数据 (10个电机 + 2个IMU = 60 words)
 */
struct RobotFeedback {
    LegFeedback left_leg;     // 左腿 (5个关节)
    LegFeedback right_leg;    // 右腿 (5个关节)
    IMUFeedback imu[2];       // 两个IMU
    uint64_t timestamp_us = 0;// 时间戳
};

//==============================================================================
// SPI缓冲区类型
//==============================================================================

/**
 * @brief SPI发送缓冲区 (控制指令)
 * 布局: [左腿5电机*4参数=20] [右腿5电机*4参数=20] = 40 words
 */
struct SPITxBuffer {
    uint16_t data[SPI_TX_WORDS] = {0};
};

/**
 * @brief SPI接收缓冲区 (反馈数据)
 * 布局: [左腿5电机*4参数=20] [右腿5电机*4参数=20] [IMU0*10] [IMU1*10] = 60 words
 */
struct SPIRxBuffer {
    uint16_t data[SPI_RX_WORDS] = {0};
};

} // namespace odroid

#endif // ODROID_COMMON_TYPES_HPP
