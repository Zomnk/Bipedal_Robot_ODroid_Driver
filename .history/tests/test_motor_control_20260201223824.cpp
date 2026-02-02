/**
 * @file test_motor_control.cpp
 * @brief 电机控制测试
 * @author Zomnk
 * @date 2026-02-01
 */

#include <iostream>
#include <cmath>
#include "common/logger.hpp"
#include "common/types.hpp"
#include "common/constants.hpp"
#include "communication/protocol.hpp"

using namespace odroid;

void test_motor_encoding(const char* name, float pos, float vel, float kp, float kd,
                          float pos_max, float vel_max, float kp_max, float kd_max) {
    MotorCommand cmd;
    cmd.position = pos;
    cmd.velocity = vel;
    cmd.kp = kp;
    cmd.kd = kd;

    uint16_t buf[4];
    Protocol::encode_motor_cmd(cmd, buf, pos_max, vel_max, kp_max, kd_max);
    
    LOG_INFO("%s: pos=%.2f->%u, vel=%.2f->%u, kp=%.1f->%u, kd=%.2f->%u",
             name, pos, buf[0], vel, buf[1], kp, buf[2], kd, buf[3]);
}

void test_encoding_accuracy() {
    LOG_INFO("--- 编码精度测试 ---");
    
    // 测试对称编码
    float test_values[] = {-12.5f, -1.0f, 0.0f, 1.0f, 12.5f};
    for (float val : test_values) {
        uint16_t encoded = encode_float_symmetric(val, 12.5f);
        float decoded = decode_float_symmetric(encoded, 12.5f);
        float error = std::abs(val - decoded);
        LOG_INFO("对称编码: %.3f -> %u -> %.6f (误差: %.6f)", val, encoded, decoded, error);
    }
}

int main() {
    Logger::instance().set_level(LogLevel::DEBUG);
    
    LOG_INFO("=== 电机控制测试开始 ===");

    // 测试不同电机类型的编码
    LOG_INFO("--- DM6006 (髋关节) 编码测试 ---");
    test_motor_encoding("DM6006", 1.0f, 5.0f, 150.0f, 3.0f,
                         DM6006_POS_MAX, DM6006_VEL_MAX, DM6006_KP_MAX, DM6006_KD_MAX);
    
    LOG_INFO("--- DM4340 (大腿/膝关节) 编码测试 ---");
    test_motor_encoding("DM4340", 0.5f, 10.0f, 200.0f, 2.0f,
                         DM4340_POS_MAX, DM4340_VEL_MAX, DM4340_KP_MAX, DM4340_KD_MAX);
    
    LOG_INFO("--- DM8006 (踝关节) 编码测试 ---");
    test_motor_encoding("DM8006", -0.3f, 3.0f, 100.0f, 4.0f,
                         DM8006_POS_MAX, DM8006_VEL_MAX, DM8006_KP_MAX, DM8006_KD_MAX);

    // 编码精度测试
    test_encoding_accuracy();

    // 完整机器人编码/解码测试
    LOG_INFO("--- 完整机器人编解码测试 ---");
    RobotCommand cmd;
    cmd.left_leg.hip.position = 1.0f;
    cmd.left_leg.hip.velocity = 2.0f;
    cmd.left_leg.hip.kp = 100.0f;
    cmd.left_leg.hip.kd = 2.0f;
    
    cmd.right_leg.ankle.position = -0.5f;
    cmd.right_leg.ankle.velocity = 1.0f;
    cmd.right_leg.ankle.kp = 150.0f;
    cmd.right_leg.ankle.kd = 3.0f;

    SPITxBuffer tx_buf;
    Protocol::encode_robot_cmd(cmd, tx_buf);
    
    LOG_INFO("TX缓冲区内容 (部分):");
    LOG_INFO("  [0-3] 左髋: %u %u %u %u", 
             tx_buf.data[0], tx_buf.data[1], tx_buf.data[2], tx_buf.data[3]);
    LOG_INFO("  [36-39] 右踝: %u %u %u %u", 
             tx_buf.data[36], tx_buf.data[37], tx_buf.data[38], tx_buf.data[39]);

    LOG_INFO("=== 电机控制测试完成 ===");
    return 0;
}
