/**
 * @file test_spi_comm.cpp
 * @brief SPI通信测试
 * @author Zomnk
 * @date 2026-02-01
 */

#include <iostream>
#include <cstring>
#include "common/logger.hpp"
#include "common/types.hpp"
#include "communication/protocol.hpp"

using namespace odroid;

int main() {
    Logger::instance().set_level(LogLevel::DEBUG);
    
    LOG_INFO("=== SPI通信测试开始 ===");

    // 测试协议编解码
    LOG_INFO("1. 创建机器人控制指令...");
    RobotCommand cmd;
    cmd.left_leg.hip.position = 0.5f;
    cmd.left_leg.hip.velocity = 1.0f;
    cmd.left_leg.hip.kp = 100.0f;
    cmd.left_leg.hip.kd = 2.0f;
    
    cmd.left_leg.ankle.position = -0.3f;
    cmd.left_leg.ankle.velocity = 0.5f;
    cmd.left_leg.ankle.kp = 150.0f;
    cmd.left_leg.ankle.kd = 3.0f;

    // 编码
    LOG_INFO("2. 编码控制指令...");
    SPITxBuffer tx_buf;
    Protocol::encode_robot_cmd(cmd, tx_buf);
    LOG_INFO("   TX缓冲区大小: %zu words", SPI_TX_WORDS);
    LOG_INFO("   Left Hip: [%u, %u, %u, %u]", 
             tx_buf.data[0], tx_buf.data[1], tx_buf.data[2], tx_buf.data[3]);
    LOG_INFO("   Left Ankle: [%u, %u, %u, %u]", 
             tx_buf.data[16], tx_buf.data[17], tx_buf.data[18], tx_buf.data[19]);

    // 模拟解码
    LOG_INFO("3. 模拟接收并解码反馈数据...");
    SPIRxBuffer rx_buf;
    memset(&rx_buf, 0, sizeof(rx_buf));
    
    // 模拟回传电机数据
    rx_buf.data[0] = tx_buf.data[0];  // left hip pos
    rx_buf.data[1] = tx_buf.data[1];  // left hip vel
    rx_buf.data[2] = 32768;           // torque = 0
    rx_buf.data[3] = 16384;           // temp = 25C
    
    // 模拟IMU数据 (索引40-49)
    rx_buf.data[40] = 32768;  // accel_x = 0
    rx_buf.data[41] = 32768;  // accel_y = 0
    rx_buf.data[42] = 38000;  // accel_z ~= 9.8 (重力)
    rx_buf.data[43] = 32768;  // gyro_x = 0
    rx_buf.data[44] = 32768;  // gyro_y = 0
    rx_buf.data[45] = 32768;  // gyro_z = 0
    rx_buf.data[46] = 32768;  // euler_roll = 0
    rx_buf.data[47] = 32768;  // euler_pitch = 0
    rx_buf.data[48] = 32768;  // euler_yaw = 0
    rx_buf.data[49] = 16384;  // temp = 25C

    RobotFeedback fb;
    Protocol::decode_robot_fb(rx_buf, fb);
    
    LOG_INFO("4. 解码结果:");
    LOG_INFO("   Left Hip: pos=%.3f, vel=%.3f, torque=%.3f, temp=%.1f",
             fb.left_leg.hip.position, fb.left_leg.hip.velocity,
             fb.left_leg.hip.torque, fb.left_leg.hip.temperature);
    LOG_INFO("   IMU0: accel=[%.2f, %.2f, %.2f]",
             fb.imu[0].accel[0], fb.imu[0].accel[1], fb.imu[0].accel[2]);
    LOG_INFO("   IMU0: gyro=[%.1f, %.1f, %.1f]",
             fb.imu[0].gyro[0], fb.imu[0].gyro[1], fb.imu[0].gyro[2]);

    LOG_INFO("=== SPI通信测试完成 ===");
    return 0;
}
