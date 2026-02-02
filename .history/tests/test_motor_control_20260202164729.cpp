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

void test_encoding_accuracy() {
    LOG_INFO("--- 编码精度测试 ---");

    // 测试对称编码
    float test_values[] = {-12.5f, -1.0f, 0.0f, 1.0f, 12.5f};
    for (float val : test_values) {
        uint16_t encoded = encode_symmetric(val, 12.5f);
        float decoded = decode_symmetric(encoded, 12.5f);
        float error = std::abs(val - decoded);
        LOG_INFO("对称编码: %.3f -> %u -> %.6f (误差: %.6f)", val, encoded, decoded, error);
    }
    
    // 测试非负编码
    float kp_values[] = {0.0f, 100.0f, 250.0f, 500.0f};
    for (float val : kp_values) {
        uint16_t encoded = encode_unsigned(val, 500.0f);
        float decoded = decode_unsigned(encoded, 500.0f);
        float error = std::abs(val - decoded);
        LOG_INFO("非负编码: %.1f -> %u -> %.6f (误差: %.6f)", val, encoded, decoded, error);
    }
}

int main() {
    Logger::instance().set_level(LogLevel::DEBUG);

    LOG_INFO("=== 电机控制测试开始 ===");

    // 测试完整机器人编码/解码链路
    LOG_INFO("--- 完整机器人编码测试 ---");
    RobotCommand cmd;
    
    // 左腿各关节设置 (与STM32关节顺序一致)
    cmd.left_leg.yaw.position = 1.0f;      // DM6006
    cmd.left_leg.yaw.velocity = 2.0f;
    cmd.left_leg.yaw.kp = 100.0f;
    cmd.left_leg.yaw.kd = 0.2f;

    cmd.left_leg.roll.position = 0.5f;     // DM4340
    cmd.left_leg.roll.velocity = 1.0f;
    cmd.left_leg.roll.kp = 150.0f;
    cmd.left_leg.roll.kd = 2.5f;

    cmd.left_leg.pitch.position = -0.3f;   // DM8006
    cmd.left_leg.pitch.velocity = 3.0f;
    cmd.left_leg.pitch.kp = 200.0f;
    cmd.left_leg.pitch.kd = 3.0f;

    cmd.left_leg.knee.position = 0.8f;     // DM6006
    cmd.left_leg.knee.velocity = 4.0f;
    cmd.left_leg.knee.kp = 120.0f;
    cmd.left_leg.knee.kd = 2.2f;

    cmd.left_leg.ankle.position = -0.5f;   // DM6006
    cmd.left_leg.ankle.velocity = 1.5f;
    cmd.left_leg.ankle.kp = 180.0f;
    cmd.left_leg.ankle.kd = 3.5f;

    SPITxBuffer tx_buf;
    Protocol::encode_robot_cmd(cmd, tx_buf);

    LOG_INFO("TX缓冲区数据 (采样):");
    LOG_INFO("  左腿Yaw [0-3]: %u %u %u %u",
             tx_buf.data[0], tx_buf.data[1], tx_buf.data[2], tx_buf.data[3]);
    LOG_INFO("  左腿Roll [4-7]: %u %u %u %u",
             tx_buf.data[4], tx_buf.data[5], tx_buf.data[6], tx_buf.data[7]);
    LOG_INFO("  左腿Pitch [8-11]: %u %u %u %u",
             tx_buf.data[8], tx_buf.data[9], tx_buf.data[10], tx_buf.data[11]);
    LOG_INFO("  左腿Knee [12-15]: %u %u %u %u",
             tx_buf.data[12], tx_buf.data[13], tx_buf.data[14], tx_buf.data[15]);
    LOG_INFO("  左腿Ankle [16-19]: %u %u %u %u",
             tx_buf.data[16], tx_buf.data[17], tx_buf.data[18], tx_buf.data[19]);
    LOG_INFO("  右腿Ankle [36-39]: %u %u %u %u",
             tx_buf.data[36], tx_buf.data[37], tx_buf.data[38], tx_buf.data[39]);

    // 编码精度测试
    test_encoding_accuracy();

    LOG_INFO("=== 电机控制测试完成 ===");
    return 0;
}