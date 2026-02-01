/**
 * @file protocol.hpp
 * @brief SPI通信协议编解码 (与STM32 ita_robot对应)
 * @author Zomnk
 * @date 2026-02-01
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

inline uint16_t encode_float_symmetric(float value, float max) {
    value = std::clamp(value, -max, max);
    float normalized = (value + max) / (2.0f * max);
    return static_cast<uint16_t>(normalized * 65535.0f + 0.5f);
}

inline float decode_float_symmetric(uint16_t value, float max) {
    float normalized = static_cast<float>(value) / 65535.0f;
    return normalized * 2.0f * max - max;
}

inline uint16_t encode_float_range(float value, float min, float max) {
    value = std::clamp(value, min, max);
    float normalized = (value - min) / (max - min);
    return static_cast<uint16_t>(normalized * 65535.0f + 0.5f);
}

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

    static void encode_motor_cmd(const MotorCommand& cmd, uint16_t* buf,
                                  float pos_max, float vel_max, float kp_max, float kd_max) {
        buf[0] = encode_float_symmetric(cmd.position, pos_max);
        buf[1] = encode_float_symmetric(cmd.velocity, vel_max);
        buf[2] = encode_float_range(cmd.kp, 0, kp_max);
        buf[3] = encode_float_range(cmd.kd, 0, kd_max);
    }

    static void encode_robot_cmd(const RobotCommand& cmd, SPITxBuffer& buf) {
        uint16_t* data = buf.data;
        encode_motor_cmd(cmd.left_leg.hip,   &data[0],  DM6006_POS_MAX, DM6006_VEL_MAX, DM6006_KP_MAX, DM6006_KD_MAX);
        encode_motor_cmd(cmd.left_leg.thigh, &data[4],  DM4340_POS_MAX, DM4340_VEL_MAX, DM4340_KP_MAX, DM4340_KD_MAX);
        encode_motor_cmd(cmd.left_leg.calf,  &data[8],  DM4340_POS_MAX, DM4340_VEL_MAX, DM4340_KP_MAX, DM4340_KD_MAX);
        encode_motor_cmd(cmd.left_leg.knee,  &data[12], DM4340_POS_MAX, DM4340_VEL_MAX, DM4340_KP_MAX, DM4340_KD_MAX);
        encode_motor_cmd(cmd.left_leg.ankle, &data[16], DM8006_POS_MAX, DM8006_VEL_MAX, DM8006_KP_MAX, DM8006_KD_MAX);
        encode_motor_cmd(cmd.right_leg.hip,   &data[20], DM6006_POS_MAX, DM6006_VEL_MAX, DM6006_KP_MAX, DM6006_KD_MAX);
        encode_motor_cmd(cmd.right_leg.thigh, &data[24], DM4340_POS_MAX, DM4340_VEL_MAX, DM4340_KP_MAX, DM4340_KD_MAX);
        encode_motor_cmd(cmd.right_leg.calf,  &data[28], DM4340_POS_MAX, DM4340_VEL_MAX, DM4340_KP_MAX, DM4340_KD_MAX);
        encode_motor_cmd(cmd.right_leg.knee,  &data[32], DM4340_POS_MAX, DM4340_VEL_MAX, DM4340_KP_MAX, DM4340_KD_MAX);
        encode_motor_cmd(cmd.right_leg.ankle, &data[36], DM8006_POS_MAX, DM8006_VEL_MAX, DM8006_KP_MAX, DM8006_KD_MAX);
    }

    static void decode_motor_fb(const uint16_t* buf, MotorFeedback& fb,
                                 float pos_max, float vel_max, float torque_max) {
        fb.position = decode_float_symmetric(buf[0], pos_max);
        fb.velocity = decode_float_symmetric(buf[1], vel_max);
        fb.torque   = decode_float_symmetric(buf[2], torque_max);
        fb.temperature = decode_float_range(buf[3], 0, 100);
    }

    static void decode_imu_fb(const uint16_t* buf, IMUFeedback& fb) {
        fb.accel[0] = decode_float_symmetric(buf[0], IMU_ACCEL_MAX);
        fb.accel[1] = decode_float_symmetric(buf[1], IMU_ACCEL_MAX);
        fb.accel[2] = decode_float_symmetric(buf[2], IMU_ACCEL_MAX);
        fb.gyro[0] = decode_float_symmetric(buf[3], IMU_GYRO_MAX);
        fb.gyro[1] = decode_float_symmetric(buf[4], IMU_GYRO_MAX);
        fb.gyro[2] = decode_float_symmetric(buf[5], IMU_GYRO_MAX);
        fb.euler[0] = decode_float_symmetric(buf[6], IMU_EULER_MAX);
        fb.euler[1] = decode_float_symmetric(buf[7], IMU_EULER_MAX);
        fb.euler[2] = decode_float_symmetric(buf[8], IMU_EULER_MAX);
        fb.temperature = decode_float_range(buf[9], 0, 100);
    }

    static void decode_robot_fb(const SPIRxBuffer& buf, RobotFeedback& fb) {
        const uint16_t* data = buf.data;
        decode_motor_fb(&data[0],  fb.left_leg.hip,   DM6006_POS_MAX, DM6006_VEL_MAX, DM6006_TORQUE_MAX);
        decode_motor_fb(&data[4],  fb.left_leg.thigh, DM4340_POS_MAX, DM4340_VEL_MAX, DM4340_TORQUE_MAX);
        decode_motor_fb(&data[8],  fb.left_leg.calf,  DM4340_POS_MAX, DM4340_VEL_MAX, DM4340_TORQUE_MAX);
        decode_motor_fb(&data[12], fb.left_leg.knee,  DM4340_POS_MAX, DM4340_VEL_MAX, DM4340_TORQUE_MAX);
        decode_motor_fb(&data[16], fb.right_leg.hip,   DM6006_POS_MAX, DM6006_VEL_MAX, DM6006_TORQUE_MAX);
        decode_motor_fb(&data[20], fb.right_leg.thigh, DM4340_POS_MAX, DM4340_VEL_MAX, DM4340_TORQUE_MAX);
        decode_motor_fb(&data[24], fb.right_leg.calf,  DM4340_POS_MAX, DM4340_VEL_MAX, DM4340_TORQUE_MAX);
        decode_motor_fb(&data[28], fb.right_leg.knee,  DM4340_POS_MAX, DM4340_VEL_MAX, DM4340_TORQUE_MAX);
        decode_imu_fb(&data[32], fb.imu[0]);
        decode_imu_fb(&data[42], fb.imu[1]);
        fb.timestamp_us = get_time_us();
    }

    static void print_robot_cmd(const RobotCommand& cmd) {
        LOG_DEBUG("=== Robot Command ===");
        LOG_DEBUG("Left Hip: pos=%.3f vel=%.3f kp=%.1f kd=%.2f",
                  cmd.left_leg.hip.position, cmd.left_leg.hip.velocity,
                  cmd.left_leg.hip.kp, cmd.left_leg.hip.kd);
    }

    static void print_robot_fb(const RobotFeedback& fb) {
        LOG_DEBUG("=== Robot Feedback ===");
        LOG_DEBUG("Left Hip: pos=%.3f vel=%.3f torque=%.3f temp=%.1f",
                  fb.left_leg.hip.position, fb.left_leg.hip.velocity,
                  fb.left_leg.hip.torque, fb.left_leg.hip.temperature);
        LOG_DEBUG("IMU0: accel=[%.2f,%.2f,%.2f] gyro=[%.1f,%.1f,%.1f]",
                  fb.imu[0].accel[0], fb.imu[0].accel[1], fb.imu[0].accel[2],
                  fb.imu[0].gyro[0], fb.imu[0].gyro[1], fb.imu[0].gyro[2]);
    }
};

} // namespace odroid

#endif // ODROID_COMMUNICATION_PROTOCOL_HPP
