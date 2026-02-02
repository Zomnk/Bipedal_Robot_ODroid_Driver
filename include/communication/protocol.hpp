/**
 * @file protocol.hpp
 * @brief SPI通信协议编解码 (与STM32 F405完全匹配)
 * @author Zomnk
 * @date 2026-02-01
 *
 * 数据布局 (与STM32一致):
 * TX (40 words): [左腿5电机*4参数=20] [右腿5电机*4参数=20]
 * RX (60 words): [左腿5电机*4参数=20] [右腿5电机*4参数=20] [ICM20602*10] [Waveshare*10]
 * 
 * 关节顺序: Yaw(DM6006), Roll(DM4340), Pitch(DM8006), Knee(DM6006), Ankle(DM6006)
 * IMU数据顺序: gyro*3, accel*3, euler*3, temp
 */

#ifndef ODROID_COMMUNICATION_PROTOCOL_HPP
#define ODROID_COMMUNICATION_PROTOCOL_HPP

#include <cstdint>
#include <cmath>
#include <algorithm>

#include "common/types.hpp"
#include "common/constants.hpp"
#include "common/logger.hpp"

namespace odroid {

//==============================================================================
// 编解码工具函数
//==============================================================================

/**
 * @brief 对称范围有符号数编码 [-max, +max] -> [0, 65535]
 * @note 与STM32的Math_Float_To_Int保持一致
 */
inline uint16_t encode_symmetric(float value, float max_val) {
    value = std::clamp(value, -max_val, max_val);
    return static_cast<uint16_t>((value / max_val + 1.0f) * 32767.5f);
}

/**
 * @brief 对称范围有符号数解码 [0, 65535] -> [-max, +max]
 * @note 与STM32的Math_Int_To_Float保持一致
 */
inline float decode_symmetric(uint16_t raw, float max_val) {
    return (static_cast<float>(raw) / 32767.5f - 1.0f) * max_val;
}

/**
 * @brief 非负值编码 [0, max] -> [0, 65535]
 */
inline uint16_t encode_unsigned(float value, float max_val) {
    value = std::clamp(value, 0.0f, max_val);
    return static_cast<uint16_t>(value / max_val * 65535.0f);
}

/**
 * @brief 非负值解码 [0, 65535] -> [0, max]
 */
inline float decode_unsigned(uint16_t raw, float max_val) {
    return static_cast<float>(raw) / 65535.0f * max_val;
}

/**
 * @brief 带最小值的范围编码 [min, max] -> [0, 65535]
 * @note 用于温度等有最小值的参数
 */
inline uint16_t encode_range(float value, float min_val, float max_val) {
    value = std::clamp(value, min_val, max_val);
    return static_cast<uint16_t>((value - min_val) / (max_val - min_val) * 65535.0f);
}

/**
 * @brief 带最小值的范围解码 [0, 65535] -> [min, max]
 */
inline float decode_range(uint16_t raw, float min_val, float max_val) {
    return static_cast<float>(raw) / 65535.0f * (max_val - min_val) + min_val;
}

//==============================================================================
// 协议编解码类
//==============================================================================

class Protocol {
public:
    /**
     * @brief 编码机器人控制指令到SPI发送缓冲区
     * @param cmd 机器人控制指令
     * @param tx_buffer SPI发送缓冲区
     */
    static void encode_robot_cmd(const RobotCommand& cmd, SPITxBuffer& tx_buffer) {
        uint16_t* data = tx_buffer.data;
        size_t idx = 0;

        // 编码左腿 (5个电机, 每个4参数)
        encode_leg_cmd(cmd.left_leg, data, idx);

        // 编码右腿 (5个电机, 每个4参数)
        encode_leg_cmd(cmd.right_leg, data, idx);
    }

    /**
     * @brief 解码SPI接收缓冲区到机器人反馈数据
     * @param rx_buffer SPI接收缓冲区
     * @param feedback 机器人反馈数据
     */
    static void decode_robot_feedback(const SPIRxBuffer& rx_buffer, RobotFeedback& feedback) {
        const uint16_t* data = rx_buffer.data;
        size_t idx = 0;

        // 解码左腿反馈 (20 words)
        decode_leg_feedback(data, idx, feedback.left_leg);

        // 解码右腿反馈 (20 words)
        decode_leg_feedback(data, idx, feedback.right_leg);

        // 解码ICM20602 IMU (10 words)
        decode_imu_feedback(data, idx, feedback.imu[0]);

        // 解码Waveshare IMU (10 words)
        decode_imu_feedback(data, idx, feedback.imu[1]);

        feedback.timestamp_us = get_time_us();
    }

private:
    /**
     * @brief 编码单腿控制指令
     * @note 关节顺序与电机类型 (与STM32保持一致):
     *       Yaw(DM6006), Roll(DM4340), Pitch(DM8006), Knee(DM6006), Ankle(DM6006)
     */
    static void encode_leg_cmd(const LegCommand& leg, uint16_t* data, size_t& idx) {
        // Yaw - DM6006
        encode_motor_cmd(leg.yaw, data, idx, DM6006_POS_MAX, DM6006_VEL_MAX,
                        DM6006_KP_MAX, DM6006_KD_MAX);
        // Roll - DM4340
        encode_motor_cmd(leg.roll, data, idx, DM4340_POS_MAX, DM4340_VEL_MAX,
                        DM4340_KP_MAX, DM4340_KD_MAX);
        // Pitch - DM8006
        encode_motor_cmd(leg.pitch, data, idx, DM8006_POS_MAX, DM8006_VEL_MAX,
                        DM8006_KP_MAX, DM8006_KD_MAX);
        // Knee - DM6006
        encode_motor_cmd(leg.knee, data, idx, DM6006_POS_MAX, DM6006_VEL_MAX,
                        DM6006_KP_MAX, DM6006_KD_MAX);
        // Ankle - DM6006
        encode_motor_cmd(leg.ankle, data, idx, DM6006_POS_MAX, DM6006_VEL_MAX,
                        DM6006_KP_MAX, DM6006_KD_MAX);
    }

    /**
     * @brief 编码单个电机控制指令
     * @note 参数顺序: pos, vel, kp, kd
     */
    static void encode_motor_cmd(const MotorCommand& cmd, uint16_t* data, size_t& idx,
                                 float pos_max, float vel_max, float kp_max, float kd_max) {
        data[idx++] = encode_symmetric(cmd.position, pos_max);
        data[idx++] = encode_symmetric(cmd.velocity, vel_max);
        data[idx++] = encode_unsigned(cmd.kp, kp_max);
        data[idx++] = encode_unsigned(cmd.kd, kd_max);
    }

    /**
     * @brief 解码单腿反馈数据
     * @note 关节顺序与电机类型 (与STM32保持一致):
     *       Yaw(DM6006), Roll(DM4340), Pitch(DM8006), Knee(DM6006), Ankle(DM6006)
     */
    static void decode_leg_feedback(const uint16_t* data, size_t& idx, LegFeedback& leg) {
        // Yaw - DM6006
        decode_motor_feedback(data, idx, leg.yaw, DM6006_POS_MAX, DM6006_VEL_MAX,
                             DM6006_TORQUE_MAX);
        // Roll - DM4340
        decode_motor_feedback(data, idx, leg.roll, DM4340_POS_MAX, DM4340_VEL_MAX,
                             DM4340_TORQUE_MAX);
        // Pitch - DM8006
        decode_motor_feedback(data, idx, leg.pitch, DM8006_POS_MAX, DM8006_VEL_MAX,
                             DM8006_TORQUE_MAX);
        // Knee - DM6006
        decode_motor_feedback(data, idx, leg.knee, DM6006_POS_MAX, DM6006_VEL_MAX,
                             DM6006_TORQUE_MAX);
        // Ankle - DM6006
        decode_motor_feedback(data, idx, leg.ankle, DM6006_POS_MAX, DM6006_VEL_MAX,
                             DM6006_TORQUE_MAX);
    }

    /**
     * @brief 解码单个电机反馈数据
     * @note 参数顺序: angle, omega, torque, temperature
     */
    static void decode_motor_feedback(const uint16_t* data, size_t& idx, MotorFeedback& motor,
                                      float pos_max, float vel_max, float torque_max) {
        motor.position = decode_symmetric(data[idx++], pos_max);
        motor.velocity = decode_symmetric(data[idx++], vel_max);
        motor.torque = decode_symmetric(data[idx++], torque_max);
        motor.temperature = decode_range(data[idx++], IMU_TEMP_MIN, IMU_TEMP_MAX);
    }

    /**
     * @brief 解码IMU反馈数据
     * @note 数据顺序 (与STM32 Pack_Feedback_Data一致):
     *       gyro[3], accel[3], euler[3], temperature
     */
    static void decode_imu_feedback(const uint16_t* data, size_t& idx, IMUFeedback& imu) {
        // 角速度 (deg/s, 范围+-2000)
        imu.gyro[0] = decode_symmetric(data[idx++], IMU_GYRO_MAX);
        imu.gyro[1] = decode_symmetric(data[idx++], IMU_GYRO_MAX);
        imu.gyro[2] = decode_symmetric(data[idx++], IMU_GYRO_MAX);
        
        // 加速度 (g, 范围+-16)
        imu.accel[0] = decode_symmetric(data[idx++], IMU_ACCEL_MAX);
        imu.accel[1] = decode_symmetric(data[idx++], IMU_ACCEL_MAX);
        imu.accel[2] = decode_symmetric(data[idx++], IMU_ACCEL_MAX);
        
        // 欧拉角 (rad, 范围+-pi)
        imu.euler[0] = decode_symmetric(data[idx++], IMU_RPY_MAX);
        imu.euler[1] = decode_symmetric(data[idx++], IMU_RPY_MAX);
        imu.euler[2] = decode_symmetric(data[idx++], IMU_RPY_MAX);
        
        // 温度 (-20 ~ 100度)
        imu.temperature = decode_range(data[idx++], IMU_TEMP_MIN, IMU_TEMP_MAX);
    }
};

} // namespace odroid

#endif // ODROID_COMMUNICATION_PROTOCOL_HPP