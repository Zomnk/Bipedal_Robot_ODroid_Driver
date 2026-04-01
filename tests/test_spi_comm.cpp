/**
 * @file test_spi_comm.cpp
 * @brief SPI通信测试
 * @author Zomnk
 * @date 2026-02-01
 */

#include <iostream>
#include "common/logger.hpp"
#include "common/types.hpp"
#include "communication/protocol.hpp"

using namespace odroid;

int main() {
    Logger::instance().set_level(LogLevel::DEBUG);

    LOG_INFO("=== SPI通信测试开始 ===");

    // 测试协议编解码
    LOG_INFO("1. 创建机器人控制指令...");
    RobotCommand cmd{};  // 值初始化
    
    // 使用与STM32一致的关节命名
    cmd.left_leg.yaw.position = 0.5f;    // DM6006
    cmd.left_leg.yaw.velocity = 1.0f;
    cmd.left_leg.yaw.kp = 100.0f;
    cmd.left_leg.yaw.kd = 2.0f;

    cmd.left_leg.ankle.position = -0.3f;  // DM6006
    cmd.left_leg.ankle.velocity = 0.5f;
    cmd.left_leg.ankle.kp = 150.0f;
    cmd.left_leg.ankle.kd = 3.0f;

    // 编码
    LOG_INFO("2. 编码控制指令...");
    SPITxBuffer tx_buf{};
    Protocol::encode_robot_cmd(cmd, tx_buf);
    LOG_INFO("   TX缓冲区大小: %zu words", SPI_TX_WORDS);
    LOG_INFO("   Left Yaw [0-3]: [%u, %u, %u, %u]",
             tx_buf.data[0], tx_buf.data[1], tx_buf.data[2], tx_buf.data[3]);
    LOG_INFO("   Left Ankle [16-19]: [%u, %u, %u, %u]",
             tx_buf.data[16], tx_buf.data[17], tx_buf.data[18], tx_buf.data[19]);

    // 模拟解码
    LOG_INFO("3. 模拟接收并解码反馈数据...");
    SPIRxBuffer rx_buf{};  // 值初始化

    // 模拟电机回传数据 (Left Yaw)
    rx_buf.data[0] = tx_buf.data[0];  // pos
    rx_buf.data[1] = tx_buf.data[1];  // vel
    rx_buf.data[2] = 32768;           // torque = 0
    rx_buf.data[3] = 32768;           // temp = 40C (中间值)

    // 模拟ICM20602 IMU数据 (索引40-49)
    // 数据顺序: gyro*3, accel*3, euler*3, temp
    rx_buf.data[40] = 32768;  // gyro_x = 0
    rx_buf.data[41] = 32768;  // gyro_y = 0
    rx_buf.data[42] = 32768;  // gyro_z = 0
    rx_buf.data[43] = 32768;  // accel_x = 0
    rx_buf.data[44] = 32768;  // accel_y = 0
    rx_buf.data[45] = 38000;  // accel_z ~= 1g (重力方向)
    rx_buf.data[46] = 32768;  // euler_roll = 0
    rx_buf.data[47] = 32768;  // euler_pitch = 0
    rx_buf.data[48] = 32768;  // euler_yaw = 0
    rx_buf.data[49] = 32768;  // temp = 40C (中间值)

    // 模拟Waveshare IMU数据 (索引50-63)
    // 数据顺序: gyro*3, accel*3, euler*3, temp, quat_w, quat_x, quat_y, quat_z
    rx_buf.data[50] = 32768;  // gyro_x = 0
    rx_buf.data[51] = 32768;  // gyro_y = 0
    rx_buf.data[52] = 32768;  // gyro_z = 0
    rx_buf.data[53] = 32768;  // accel_x = 0
    rx_buf.data[54] = 32768;  // accel_y = 0
    rx_buf.data[55] = 38000;  // accel_z ~= 1g
    rx_buf.data[56] = 32768;  // euler_roll = 0
    rx_buf.data[57] = 32768;  // euler_pitch = 0
    rx_buf.data[58] = 32768;  // euler_yaw = 0
    rx_buf.data[59] = 32768;  // temp = 40C
    rx_buf.data[60] = 65535;  // quat_w = 1.0
    rx_buf.data[61] = 32768;  // quat_x = 0.0
    rx_buf.data[62] = 32768;  // quat_y = 0.0
    rx_buf.data[63] = 32768;  // quat_z = 0.0

    RobotFeedback fb{};
    Protocol::decode_robot_feedback(rx_buf, fb);

    LOG_INFO("4. 解码结果:");
    LOG_INFO("   Left Yaw: pos=%.3f, vel=%.3f, torque=%.3f, temp=%.1f",
             fb.left_leg.yaw.position, fb.left_leg.yaw.velocity,
             fb.left_leg.yaw.torque, fb.left_leg.yaw.temperature);
    LOG_INFO("   IMU0 gyro: [%.1f, %.1f, %.1f] deg/s",
             fb.imu[0].gyro[0], fb.imu[0].gyro[1], fb.imu[0].gyro[2]);
    LOG_INFO("   IMU0 accel: [%.2f, %.2f, %.2f] g",
             fb.imu[0].accel[0], fb.imu[0].accel[1], fb.imu[0].accel[2]);
    LOG_INFO("   IMU0 euler: [%.3f, %.3f, %.3f] rad",
             fb.imu[0].euler[0], fb.imu[0].euler[1], fb.imu[0].euler[2]);
    LOG_INFO("   IMU1 (Waveshare) quat: [%.3f, %.3f, %.3f, %.3f]",
             fb.imu[1].quat[0], fb.imu[1].quat[1],
             fb.imu[1].quat[2], fb.imu[1].quat[3]);
    LOG_INFO("   RX缓冲区大小: %zu words", SPI_RX_WORDS);

    LOG_INFO("=== SPI通信测试完成 ===");
    return 0;
}