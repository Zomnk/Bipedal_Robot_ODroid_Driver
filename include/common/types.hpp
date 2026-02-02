/**
 * @file types.hpp
 * @brief 数据类型定义 (与STM32 F405保持一致)
 * @author Zomnk
 * @date 2026-02-01
 */

#ifndef ODROID_COMMON_TYPES_HPP
#define ODROID_COMMON_TYPES_HPP

#include <cstdint>
#include <array>
#include "constants.hpp"

namespace odroid {

/**
 * @brief 电机控制指令
 */
struct MotorCommand {
    float position = 0.0f;    // 目标位置 (rad)
    float velocity = 0.0f;    // 目标速度 (rad/s)
    float kp = 0.0f;          // 位置增益
    float kd = 0.0f;          // 速度增益
};

/**
 * @brief 电机反馈数据
 */
struct MotorFeedback {
    float position = 0.0f;      // 当前位置 (rad)
    float velocity = 0.0f;      // 当前速度 (rad/s)
    float torque = 0.0f;        // 当前力矩 (Nm)
    float temperature = 0.0f;   // 电机温度 (C)
};

/**
 * @brief 单腿控制指令 (5个关节，与STM32保持一致)
 * @note 关节顺序: Yaw, Roll, Pitch, Knee, Ankle
 *       电机型号: DM6006, DM4340, DM8006, DM6006, DM6006
 */
struct LegCommand {
    MotorCommand yaw;     // 髋关节Yaw (DM6006)
    MotorCommand roll;    // 髋关节Roll (DM4340)
    MotorCommand pitch;   // 髋关节Pitch (DM8006)
    MotorCommand knee;    // 膝关节 (DM6006)
    MotorCommand ankle;   // 踝关节 (DM6006)
};

/**
 * @brief 单腿反馈数据 (5个关节，与STM32保持一致)
 */
struct LegFeedback {
    MotorFeedback yaw;     // 髋关节Yaw (DM6006)
    MotorFeedback roll;    // 髋关节Roll (DM4340)
    MotorFeedback pitch;   // 髋关节Pitch (DM8006)
    MotorFeedback knee;    // 膝关节 (DM6006)
    MotorFeedback ankle;   // 踝关节 (DM6006)
};

/**
 * @brief IMU反馈数据 (与STM32保持一致)
 * @note 数据顺序: gyro3, accel3, euler3, temp
 */
struct IMUFeedback {
    float gyro[3] = {0};      // 角速度 (deg/s) [X, Y, Z]
    float accel[3] = {0};     // 加速度 (g) [X, Y, Z]
    float euler[3] = {0};     // 欧拉角 (rad) [roll, pitch, yaw]
    float temperature = 0.0f; // 温度 (C)
};

/**
 * @brief 机器人控制指令
 */
struct RobotCommand {
    LegCommand left_leg;   // 左腿
    LegCommand right_leg;  // 右腿
    uint64_t timestamp_us = 0;
};

/**
 * @brief 机器人反馈数据
 */
struct RobotFeedback {
    LegFeedback left_leg;
    LegFeedback right_leg;
    IMUFeedback imu[2];      // 2个IMU: [0]=ICM20602, [1]=Waveshare
    uint64_t timestamp_us = 0;
};

/**
 * @brief SPI配置
 */
struct SPIConfig {
    const char* device = SPI_DEVICE_DEFAULT;
    uint32_t speed_hz = SPI_SPEED_HZ;
    uint8_t bits_per_word = SPI_BITS_PER_WORD;
    uint8_t mode = SPI_MODE;
};

/**
 * @brief SPI发送缓冲区
 */
struct SPITxBuffer {
    uint16_t data[SPI_TX_WORDS] = {0};
};

/**
 * @brief SPI接收缓冲区
 */
struct SPIRxBuffer {
    uint16_t data[SPI_RX_WORDS] = {0};
};

} // namespace odroid

#endif // ODROID_COMMON_TYPES_HPP