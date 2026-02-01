/**
 * @file test_spi_comm.cpp
 * @brief SPI通信测试
 */

#include <iostream>
#include <cstring>
#include "common/logger.hpp"
#include "common/types.hpp"
#include "communication/protocol.hpp"

using namespace odroid;

int main() {
    Logger::instance().set_level(LogLevel::DEBUG);
    LOG_INFO("SPI通信测试开始");

    // 测试协议编解码
    RobotCommand cmd;
    cmd.left_leg.hip.position = 0.5f;
    cmd.left_leg.hip.velocity = 1.0f;
    cmd.left_leg.hip.kp = 100.0f;
    cmd.left_leg.hip.kd = 2.0f;

    SPITxBuffer tx_buf;
    Protocol::encode_robot_cmd(cmd, tx_buf);
    LOG_INFO("编码完成: tx_buf[0]=%u, tx_buf[1]=%u", tx_buf.data[0], tx_buf.data[1]);

    // 模拟解码
    SPIRxBuffer rx_buf;
    memset(&rx_buf, 0, sizeof(rx_buf));
    rx_buf.data[0] = tx_buf.data[0];  // 模拟回传
    rx_buf.data[1] = tx_buf.data[1];
    rx_buf.data[2] = 32768;  // 中间值表示0力矩
    rx_buf.data[3] = 32768;  // 25度温度

    RobotFeedback fb;
    Protocol::decode_robot_fb(rx_buf, fb);
    LOG_INFO("解码完成: pos=%.3f, vel=%.3f", fb.left_leg.hip.position, fb.left_leg.hip.velocity);

    LOG_INFO("SPI通信测试完成");
    return 0;
}
