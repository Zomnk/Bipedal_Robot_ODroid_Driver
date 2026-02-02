/**
 * @file protocol.hpp
 * @brief SPI通信协议编解码 (与STM32 F405匹配)
 * @author Zomnk
 * @date 2026-02-01
 *
 * 数据布局:
 * TX (40 words): [左腿5电机*4参数=20] [右腿5电机*4参数=20]
 * RX (60 words): [左腿5电机*4参数=20] [右腿5电机*4参数=20] [IMU0*10] [IMU1*10]
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
 */
inline uint16_t encode_symmetric(float value, float max_val) {
    value = std::clamp(value, -max_val, max_val);
    return static_cast<uint16_t>((value / max_val + 1.0f) * 32767.5f);
}

/**
 * @brief 对称范围有符号数解码 [0, 65535] -> [-max, +max]
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
        
        // 解码IMU0 (10 words)
        decode_imu_feedback(data, idx, feedback.imu[0]);
        
        // 解码IMU1 (10 words)
        decode_imu_feedback(data, idx, feedback.imu[1]);
        
        feedback.timestamp_us = get_time_us();
    }
    
private:
    /**
     * @brief 编码单腿控制指令
     */
    static void encode_leg_cmd(const LegCommand& leg, uint16_t* data, size_t& idx) {
        // 髋关节 DM6006
        encode_motor_cmd(leg.hip, data, idx, DM6006_POS_MAX, DM6006_VEL_MAX, 
                        DM6006_KP_MAX, DM6006_KD_MAX);
        // 大腿 DM4340
        encode_motor_cmd(leg.thigh, data, idx, DM4340_POS_MAX, DM4340_VEL_MAX,
                        DM4340_KP_MAX, DM4340_KD_MAX);
        // 小腿 DM4340
        encode_motor_cmd(leg.calf, data, idx, DM4340_POS_MAX, DM4340_VEL_MAX,
                        DM4340_KP_MAX, DM4340_KD_MAX);
        // 膝关节 DM4340
        encode_motor_cmd(leg.knee, data, idx, DM4340_POS_MAX, DM4340_VEL_MAX,
                        DM4340_KP_MAX, DM4340_KD_MAX);
        // 踝关节 DM8006
        encode_motor_cmd(leg.ankle, data, idx, DM8006_POS_MAX, DM8006_VEL_MAX,
                        DM8006_KP_MAX, DM8006_KD_MAX);
    }
    
    /**
     * @brief 编码单个电机控制指令
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
     */
    static void decode_leg_feedback(const uint16_t* data, size_t& idx, LegFeedback& leg) {
        // 髋关节 DM6006
        decode_motor_feedback(data, idx, leg.hip, DM6006_POS_MAX, DM6006_VEL_MAX, 
                             DM6006_TORQUE_MAX);
        // 大腿 DM4340
        decode_motor_feedback(data, idx, leg.thigh, DM4340_POS_MAX, DM4340_VEL_MAX,
                             DM4340_TORQUE_MAX);
        // 小腿 DM4340
        decode_motor_feedback(data, idx, leg.calf, DM4340_POS_MAX, DM4340_VEL_MAX,
                             DM4340_TORQUE_MAX);
        // 膝关节 DM4340
        decode_motor_feedback(data, idx, leg.knee, DM4340_POS_MAX, DM4340_VEL_MAX,
                             DM4340_TORQUE_MAX);
        // 踝关节 DM8006
        decode_motor_feedback(data, idx, leg.ankle, DM8006_POS_MAX, DM8006_VEL_MAX,
                             DM8006_TORQUE_MAX);
    }
    
    /**
     * @brief 解码单个电机反馈数据
     */
    static void decode_motor_feedback(const uint16_t* data, size_t& idx, MotorFeedback& motor,
                                      float pos_max, float vel_max, float torque_max) {
        motor.position = decode_symmetric(data[idx++], pos_max);
        motor.velocity = decode_symmetric(data[idx++], vel_max);
        motor.torque = decode_symmetric(data[idx++], torque_max);
        motor.temperature = decode_unsigned(data[idx++], 100.0f);  // 温度0-100度
    }
    
    /**
     * @brief 解码IMU反馈数据
     */
    static void decode_imu_feedback(const uint16_t* data, size_t& idx, IMUFeedback& imu) {
        // 加速度 (m/s^2, 范围+-16g)
        imu.accel[0] = decode_symmetric(data[idx++], 16.0f * 9.81f);
        imu.accel[1] = decode_symmetric(data[idx++], 16.0f * 9.81f);
        imu.accel[2] = decode_symmetric(data[idx++], 16.0f * 9.81f);
        
        // 角速度 (deg/s, 范围+-2000)
        imu.gyro[0] = decode_symmetric(data[idx++], 2000.0f);
        imu.gyro[1] = decode_symmetric(data[idx++], 2000.0f);
        imu.gyro[2] = decode_symmetric(data[idx++], 2000.0f);
        
        // 欧拉角 (度, roll/pitch +-180, yaw +-180)
        imu.euler[0] = decode_symmetric(data[idx++], 180.0f);
        imu.euler[1] = decode_symmetric(data[idx++], 180.0f);
        imu.euler[2] = decode_symmetric(data[idx++], 180.0f);
        
        // 温度 (0-100度)
        imu.temperature = decode_unsigned(data[idx++], 100.0f);
    }
};

} // namespace odroid

#endif // ODROID_COMMUNICATION_PROTOCOL_HPP