/**
 * @file test_motor_control.cpp
 * @brief 电机控制测试
 */

#include <iostream>
#include "common/logger.hpp"
#include "common/types.hpp"
#include "common/constants.hpp"
#include "communication/protocol.hpp"

using namespace odroid;

int main() {
    Logger::instance().set_level(LogLevel::DEBUG);
    LOG_INFO("电机控制测试开始");

    // 测试电机编码
    MotorCommand cmd;
    cmd.position = 1.0f;
    cmd.velocity = 5.0f;
    cmd.kp = 150.0f;
    cmd.kd = 3.0f;

    uint16_t buf[4];
    Protocol::encode_motor_cmd(cmd, buf, DM6006_POS_MAX, DM6006_VEL_MAX, DM6006_KP_MAX, DM6006_KD_MAX);
    LOG_INFO("DM6006编码: pos=%u, vel=%u, kp=%u, kd=%u", buf[0], buf[1], buf[2], buf[3]);

    // 解码验证
    MotorFeedback fb;
    Protocol::decode_motor_fb(buf, fb, DM6006_POS_MAX, DM6006_VEL_MAX, DM6006_TORQUE_MAX);
    LOG_INFO("解码结果: pos=%.3f, vel=%.3f, torque=%.3f", fb.position, fb.velocity, fb.torque);

    LOG_INFO("电机控制测试完成");
    return 0;
}
