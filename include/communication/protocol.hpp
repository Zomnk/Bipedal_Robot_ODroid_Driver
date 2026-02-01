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
 * @brief 对称范围浮点数编码 [-max, +max] -> [0, 65535]
 */
inline uint16_t encode_float_symmetric(float value, float max) {
    value = std::clamp(value, -max, max);
    float normalized = (value + max) / (2.0f * max);
    return static_cast<uint16_t>(normalized * 65535.0f + 0.5f);
}

/**
 * @brief 对称范围浮点数解码 [0, 65535] -> [-max, +max]
 */
inline float decode_float_symmetric(uint16_t value, float max) {
    float normalized = static_cast<float>(value) / 65535.0f;
    return normalized * 2.0f * max - max;
}

/**
 * @brief 指定范围浮点数编码 [min, max] -> [0, 65535]
 */
inline uint16_t encode_float_range(float value, float min, float max) {
    value = std::clamp(value, min, max);
    float normalized = (value - min) / (max - min);
    return static_cast<uint16_t>(normalized * 65535.0f + 0.5f);
}

/**
 * @brief 指定范围浮点数解码 [0, 65535] -> [min, max]
 */
inline float decode_float_range(uint16_t value, float min, float max) {
    float normalized = static_cast<float>(value) / 65535.0f;
    return normalized * (max - min) + min;
}

//==============================================================================
// 协议编解码器
//==============================================================================

class Protocol {
public:
    Protocol() = default;

    /**
     * @brief 编码单个电机指令 (4 words)
     * 布局: [position, velocity, kp, kd]
     */
    static void encode_motor_cmd(const MotorCommand& cmd, uint16_t* buf,
                                  float pos_max, float vel_max, 
                                  float kp_max, float kd_max) {
        buf[0] = encode_float_symmetric(cmd.position, pos_max);
        buf[1] = encode_float_symmetric(cmd.velocity, vel_max);
        buf[2] = encode_float_range(cmd.kp, 0, kp_max);
        buf[3] = encode_float_range(cmd.kd, 0, kd_max);
    }

    /**
     * @brief 编码完整机器人指令 (40 words)
     * 布局: 
     *   左腿: [hip 0-3] [thigh 4-7] [calf 8-11] [knee 12-15] [ankle 16-19]
     *   右腿: [hip 20-23] [thigh 24-27] [calf 28-31] [knee 32-35] [ankle 36-39]
     */
    static void encode_robot_cmd(const RobotCommand& cmd, SPITxBuffer& buf) {
        uint16_t* data = buf.data;
        
        // 左腿 (索引 0-19)
        encode_motor_cmd(cmd.left_leg.hip,   &data[0],  DM6006_POS_MAX, DM6006_VEL_MAX, DM6006_KP_MAX, DM6006_KD_MAX);
        encode_motor_cmd(cmd.left_leg.thigh, &data[4],  DM4340_POS_MAX, DM4340_VEL_MAX, DM4340_KP_MAX, DM4340_KD_MAX);
        encode_motor_cmd(cmd.left_leg.calf,  &data[8],  DM4340_POS_MAX, DM4340_VEL_MAX, DM4340_KP_MAX, DM4340_KD_MAX);
        encode_motor_cmd(cmd.left_leg.knee,  &data[12], DM4340_POS_MAX, DM4340_VEL_MAX, DM4340_KP_MAX, DM4340_KD_MAX);
        encode_motor_cmd(cmd.left_leg.ankle, &data[16], DM8006_POS_MAX, DM8006_VEL_MAX, DM8006_KP_MAX, DM8006_KD_MAX);
        
        // 右腿 (索引 20-39)
        encode_motor_cmd(cmd.right_leg.hip,   &data[20], DM6006_POS_MAX, DM6006_VEL_MAX, DM6006_KP_MAX, DM6006_KD_MAX);
        encode_motor_cmd(cmd.right_leg.thigh, &data[24], DM4340_POS_MAX, DM4340_VEL_MAX, DM4340_KP_MAX, DM4340_KD_MAX);
        encode_motor_cmd(cmd.right_leg.calf,  &data[28], DM4340_POS_MAX, DM4340_VEL_MAX, DM4340_KP_MAX, DM4340_KD_MAX);
        encode_motor_cmd(cmd.right_leg.knee,  &data[32], DM4340_POS_MAX, DM4340_VEL_MAX, DM4340_KP_MAX, DM4340_KD_MAX);
        encode_motor_cmd(cmd.right_leg.ankle, &data[36], DM8006_POS_MAX, DM8006_VEL_MAX, DM8006_KP_MAX, DM8006_KD_MAX);
    }

    /**
     * @brief 解码单个电机反馈 (4 words)
     * 布局: [position, velocity, torque, temperature]
     */
    static void decode_motor_fb(const uint16_t* buf, MotorFeedback& fb,
                                 float pos_max, float vel_max, float torque_max) {
        fb.position    = decode_float_symmetric(buf[0], pos_max);
        fb.velocity    = decode_float_symmetric(buf[1], vel_max);
        fb.torque      = decode_float_symmetric(buf[2], torque_max);
        fb.temperature = decode_float_range(buf[3], 0, 100);
    }

    /**
     * @brief 解码IMU反馈 (10 words)
     * 布局: [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, 
     *        euler_x, euler_y, euler_z, temperature]
     */
    static void decode_imu_fb(const uint16_t* buf, IMUFeedback& fb) {
        fb.accel[0] = decode_float_symmetric(buf[0], IMU_ACCEL_MAX);
        fb.accel[1] = decode_float_symmetric(buf[1], IMU_ACCEL_MAX);
        fb.accel[2] = decode_float_symmetric(buf[2], IMU_ACCEL_MAX);
        fb.gyro[0]  = decode_float_symmetric(buf[3], IMU_GYRO_MAX);
        fb.gyro[1]  = decode_float_symmetric(buf[4], IMU_GYRO_MAX);
        fb.gyro[2]  = decode_float_symmetric(buf[5], IMU_GYRO_MAX);
        fb.euler[0] = decode_float_symmetric(buf[6], IMU_EULER_MAX);
        fb.euler[1] = decode_float_symmetric(buf[7], IMU_EULER_MAX);
        fb.euler[2] = decode_float_symmetric(buf[8], IMU_EULER_MAX);
        fb.temperature = decode_float_range(buf[9], 0, 100);
    }

    /**
     * @brief 解码完整机器人反馈 (60 words)
     * 布局:
     *   左腿: [hip 0-3] [thigh 4-7] [calf 8-11] [knee 12-15] [ankle 16-19]
     *   右腿: [hip 20-23] [thigh 24-27] [calf 28-31] [knee 32-35] [ankle 36-39]
     *   IMU0: [40-49]
     *   IMU1: [50-59]
     */
    static void decode_robot_fb(const SPIRxBuffer& buf, RobotFeedback& fb) {
        const uint16_t* data = buf.data;
        
        // 左腿 (索引 0-19)
        decode_motor_fb(&data[0],  fb.left_leg.hip,   DM6006_POS_MAX, DM6006_VEL_MAX, DM6006_TORQUE_MAX);
        decode_motor_fb(&data[4],  fb.left_leg.thigh, DM4340_POS_MAX, DM4340_VEL_MAX, DM4340_TORQUE_MAX);
        decode_motor_fb(&data[8],  fb.left_leg.calf,  DM4340_POS_MAX, DM4340_VEL_MAX, DM4340_TORQUE_MAX);
        decode_motor_fb(&data[12], fb.left_leg.knee,  DM4340_POS_MAX, DM4340_VEL_MAX, DM4340_TORQUE_MAX);
        decode_motor_fb(&data[16], fb.left_leg.ankle, DM8006_POS_MAX, DM8006_VEL_MAX, DM8006_TORQUE_MAX);
        
        // 右腿 (索引 20-39)
        decode_motor_fb(&data[20], fb.right_leg.hip,   DM6006_POS_MAX, DM6006_VEL_MAX, DM6006_TORQUE_MAX);
        decode_motor_fb(&data[24], fb.right_leg.thigh, DM4340_POS_MAX, DM4340_VEL_MAX, DM4340_TORQUE_MAX);
        decode_motor_fb(&data[28], fb.right_leg.calf,  DM4340_POS_MAX, DM4340_VEL_MAX, DM4340_TORQUE_MAX);
        decode_motor_fb(&data[32], fb.right_leg.knee,  DM4340_POS_MAX, DM4340_VEL_MAX, DM4340_TORQUE_MAX);
        decode_motor_fb(&data[36], fb.right_leg.ankle, DM8006_POS_MAX, DM8006_VEL_MAX, DM8006_TORQUE_MAX);
        
        // IMU (索引 40-59)
        decode_imu_fb(&data[40], fb.imu[0]);
        decode_imu_fb(&data[50], fb.imu[1]);
        
        fb.timestamp_us = get_time_us();
    }

    /**
     * @brief 调试打印机器人指令
     */
    static void print_robot_cmd(const RobotCommand& cmd) {
        LOG_DEBUG("=== Robot Command ===");
        LOG_DEBUG("Left  Hip  : pos=%7.3f vel=%7.3f kp=%6.1f kd=%5.2f",
                  cmd.left_leg.hip.position, cmd.left_leg.hip.velocity,
                  cmd.left_leg.hip.kp, cmd.left_leg.hip.kd);
        LOG_DEBUG("Left  Thigh: pos=%7.3f vel=%7.3f kp=%6.1f kd=%5.2f",
                  cmd.left_leg.thigh.position, cmd.left_leg.thigh.velocity,
                  cmd.left_leg.thigh.kp, cmd.left_leg.thigh.kd);
        LOG_DEBUG("Left  Calf : pos=%7.3f vel=%7.3f kp=%6.1f kd=%5.2f",
                  cmd.left_leg.calf.position, cmd.left_leg.calf.velocity,
                  cmd.left_leg.calf.kp, cmd.left_leg.calf.kd);
        LOG_DEBUG("Left  Knee : pos=%7.3f vel=%7.3f kp=%6.1f kd=%5.2f",
                  cmd.left_leg.knee.position, cmd.left_leg.knee.velocity,
                  cmd.left_leg.knee.kp, cmd.left_leg.knee.kd);
        LOG_DEBUG("Left  Ankle: pos=%7.3f vel=%7.3f kp=%6.1f kd=%5.2f",
                  cmd.left_leg.ankle.position, cmd.left_leg.ankle.velocity,
                  cmd.left_leg.ankle.kp, cmd.left_leg.ankle.kd);
    }

    /**
     * @brief 调试打印机器人反馈
     */
    static void print_robot_fb(const RobotFeedback& fb) {
        LOG_DEBUG("=== Robot Feedback ===");
        LOG_DEBUG("Left  Hip  : pos=%7.3f vel=%7.3f torque=%6.2f temp=%5.1f",
                  fb.left_leg.hip.position, fb.left_leg.hip.velocity,
                  fb.left_leg.hip.torque, fb.left_leg.hip.temperature);
        LOG_DEBUG("Left  Ankle: pos=%7.3f vel=%7.3f torque=%6.2f temp=%5.1f",
                  fb.left_leg.ankle.position, fb.left_leg.ankle.velocity,
                  fb.left_leg.ankle.torque, fb.left_leg.ankle.temperature);
        LOG_DEBUG("IMU0: accel=[%6.2f,%6.2f,%6.2f] gyro=[%7.1f,%7.1f,%7.1f]",
                  fb.imu[0].accel[0], fb.imu[0].accel[1], fb.imu[0].accel[2],
                  fb.imu[0].gyro[0], fb.imu[0].gyro[1], fb.imu[0].gyro[2]);
        LOG_DEBUG("IMU0: euler=[%6.1f,%6.1f,%6.1f] temp=%5.1f",
                  fb.imu[0].euler[0], fb.imu[0].euler[1], fb.imu[0].euler[2],
                  fb.imu[0].temperature);
    }
};

} // namespace odroid

#endif // ODROID_COMMUNICATION_PROTOCOL_HPP
