/**
 * @file test_spi_comm.cpp
 * @brief SPI通信测试 - 通过SPI2从STM32读取真实数据并打印
 * @author Zomnk
 * @date 2026-02-01
 *
 * @note 使用方法:
 *   sudo ./test_spi_comm
 *   程序将启动SPI通信线程，每0.5秒打印一次完整数据
 *   按 Ctrl+C 退出
 */

#include <iostream>
#include <csignal>
#include <unistd.h>
#include <cmath>

#include "common/logger.hpp"
#include "common/constants.hpp"
#include "common/types.hpp"
#include "core/robot_interface.hpp"

using namespace odroid;

// 全局运行标志
volatile bool g_running = true;

void signal_handler(int sig) {
    LOG_INFO("收到信号 %d, 准备退出...", sig);
    g_running = false;
}

int main() {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    Logger::instance().set_level(LogLevel::INFO);

    LOG_INFO("=== SPI通信测试 (真实数据) ===");
    LOG_INFO("RX缓冲区大小: %zu words (%zu bytes)", SPI_RX_WORDS, SPI_RX_WORDS * 2);
    LOG_INFO("TX缓冲区大小: %zu words (%zu bytes)", SPI_TX_WORDS, SPI_TX_WORDS * 2);
    LOG_INFO("按 Ctrl+C 退出");
    LOG_INFO("");

    // 初始化SPI驱动
    RobotInterface robot;
    if (!robot.init()) {
        LOG_ERROR("SPI初始化失败! 请检查:");
        LOG_ERROR("  1. SPI设备是否正常 (/dev/spidev0.0)");
        LOG_ERROR("  2. 是否使用sudo权限运行");
        LOG_ERROR("  3. STM32是否正常供电");
        return -1;
    }
    LOG_INFO("✓ SPI初始化成功");

    // 启动通信线程 (1kHz)
    if (!robot.start()) {
        LOG_ERROR("SPI通信线程启动失败!");
        return -1;
    }
    LOG_INFO("✓ SPI通信线程已启动 (1kHz)");
    LOG_INFO("");

    // 发送默认零指令
    RobotCommand cmd{};
    robot.send_command(cmd);

    uint64_t last_print_time = 0;
    const uint64_t print_interval_us = 500000;  // 0.5s
    uint64_t loop_count = 0;

    while (g_running) {
        RobotFeedback feedback;
        if (robot.get_feedback(feedback)) {
            uint64_t now = get_time_us();
            if (now - last_print_time >= print_interval_us) {
                LOG_INFO("========================================");
                LOG_INFO("[SPI通信 #%lu] 传输计数: %lu", loop_count, robot.get_transfer_count());
                LOG_INFO("========================================");

                // 打印TX原始数据 (40 words)
                LOG_INFO("【TX原始数据】(ODroid → STM32, %zu words)", SPI_TX_WORDS);
                for (size_t i = 0; i < SPI_TX_WORDS; i += 8) {
                    auto& tx = robot.get_tx_buffer();
                    size_t end = (i + 8 <= SPI_TX_WORDS) ? i + 8 : SPI_TX_WORDS;
                    LOG_INFO("  [%2zu-%2zu]:", i, end - 1);
                    for (size_t j = i; j < end; j++) {
                        LOG_INFO(" %04X", tx.data[j]);
                    }
                }

                // 打印RX原始数据 (64 words)
                LOG_INFO("【RX原始数据】(STM32 → ODroid, %zu words)", SPI_RX_WORDS);
                auto& rx = robot.get_rx_buffer();
                for (size_t i = 0; i < SPI_RX_WORDS; i += 8) {
                    size_t end = (i + 8 <= SPI_RX_WORDS) ? i + 8 : SPI_RX_WORDS;
                    LOG_INFO("  [%2zu-%2zu]:", i, end - 1);
                    for (size_t j = i; j < end; j++) {
                        LOG_INFO(" %04X", rx.data[j]);
                    }
                }

                // 打印解码后的反馈数据
                LOG_INFO("【解码数据】");
                LOG_INFO("");

                // 左腿电机
                LOG_INFO("--- 左腿电机反馈 ---");
                LOG_INFO("  Yaw(DM6006):   pos=%+7.3f rad, vel=%+7.3f rad/s, torque=%+6.3f Nm, temp=%.1f C",
                         feedback.left_leg.yaw.position, feedback.left_leg.yaw.velocity,
                         feedback.left_leg.yaw.torque, feedback.left_leg.yaw.temperature);
                LOG_INFO("  Roll(DM4340):  pos=%+7.3f rad, vel=%+7.3f rad/s, torque=%+6.3f Nm, temp=%.1f C",
                         feedback.left_leg.roll.position, feedback.left_leg.roll.velocity,
                         feedback.left_leg.roll.torque, feedback.left_leg.roll.temperature);
                LOG_INFO("  Pitch(DM8006): pos=%+7.3f rad, vel=%+7.3f rad/s, torque=%+6.3f Nm, temp=%.1f C",
                         feedback.left_leg.pitch.position, feedback.left_leg.pitch.velocity,
                         feedback.left_leg.pitch.torque, feedback.left_leg.pitch.temperature);
                LOG_INFO("  Knee(DM6006):  pos=%+7.3f rad, vel=%+7.3f rad/s, torque=%+6.3f Nm, temp=%.1f C",
                         feedback.left_leg.knee.position, feedback.left_leg.knee.velocity,
                         feedback.left_leg.knee.torque, feedback.left_leg.knee.temperature);
                LOG_INFO("  Ankle(DM6006): pos=%+7.3f rad, vel=%+7.3f rad/s, torque=%+6.3f Nm, temp=%.1f C",
                         feedback.left_leg.ankle.position, feedback.left_leg.ankle.velocity,
                         feedback.left_leg.ankle.torque, feedback.left_leg.ankle.temperature);
                LOG_INFO("");

                // 右腿电机
                LOG_INFO("--- 右腿电机反馈 ---");
                LOG_INFO("  Yaw(DM6006):   pos=%+7.3f rad, vel=%+7.3f rad/s, torque=%+6.3f Nm, temp=%.1f C",
                         feedback.right_leg.yaw.position, feedback.right_leg.yaw.velocity,
                         feedback.right_leg.yaw.torque, feedback.right_leg.yaw.temperature);
                LOG_INFO("  Roll(DM4340):  pos=%+7.3f rad, vel=%+7.3f rad/s, torque=%+6.3f Nm, temp=%.1f C",
                         feedback.right_leg.roll.position, feedback.right_leg.roll.velocity,
                         feedback.right_leg.roll.torque, feedback.right_leg.roll.temperature);
                LOG_INFO("  Pitch(DM8006): pos=%+7.3f rad, vel=%+7.3f rad/s, torque=%+6.3f Nm, temp=%.1f C",
                         feedback.right_leg.pitch.position, feedback.right_leg.pitch.velocity,
                         feedback.right_leg.pitch.torque, feedback.right_leg.pitch.temperature);
                LOG_INFO("  Knee(DM6006):  pos=%+7.3f rad, vel=%+7.3f rad/s, torque=%+6.3f Nm, temp=%.1f C",
                         feedback.right_leg.knee.position, feedback.right_leg.knee.velocity,
                         feedback.right_leg.knee.torque, feedback.right_leg.knee.temperature);
                LOG_INFO("  Ankle(DM6006): pos=%+7.3f rad, vel=%+7.3f rad/s, torque=%+6.3f Nm, temp=%.1f C",
                         feedback.right_leg.ankle.position, feedback.right_leg.ankle.velocity,
                         feedback.right_leg.ankle.torque, feedback.right_leg.ankle.temperature);
                LOG_INFO("");

                // ICM20602 IMU
                LOG_INFO("--- ICM20602 IMU反馈 ---");
                LOG_INFO("  Gyro:  [%+8.2f, %+8.2f, %+8.2f] rad/s",
                         feedback.imu[0].gyro[0], feedback.imu[0].gyro[1], feedback.imu[0].gyro[2]);
                LOG_INFO("  Accel: [%+6.3f, %+6.3f, %+6.3f] g",
                         feedback.imu[0].accel[0], feedback.imu[0].accel[1], feedback.imu[0].accel[2]);
                LOG_INFO("  Euler: [%+7.4f, %+7.4f, %+7.4f] rad",
                         feedback.imu[0].euler[0], feedback.imu[0].euler[1], feedback.imu[0].euler[2]);
                LOG_INFO("  Temp:  %.1f C", feedback.imu[0].temperature);
                LOG_INFO("");

                // Waveshare IMU (含四元数)
                LOG_INFO("--- Waveshare IMU反馈 (14 words) ---");
                LOG_INFO("  Gyro:  [%+8.2f, %+8.2f, %+8.2f] rad/s",
                         feedback.imu[1].gyro[0], feedback.imu[1].gyro[1], feedback.imu[1].gyro[2]);
                LOG_INFO("  Accel: [%+6.3f, %+6.3f, %+6.3f] g",
                         feedback.imu[1].accel[0], feedback.imu[1].accel[1], feedback.imu[1].accel[2]);
                LOG_INFO("  Euler: [%+7.4f, %+7.4f, %+7.4f] rad",
                         feedback.imu[1].euler[0], feedback.imu[1].euler[1], feedback.imu[1].euler[2]);
                LOG_INFO("  Temp:  %.1f C", feedback.imu[1].temperature);
                LOG_INFO("  Quat:  [%+7.4f, %+7.4f, %+7.4f, %+7.4f] (w, x, y, z)",
                         feedback.imu[1].quat[0], feedback.imu[1].quat[1],
                         feedback.imu[1].quat[2], feedback.imu[1].quat[3]);
                LOG_INFO("========================================");
                LOG_INFO("");

                last_print_time = now;
            }
        }

        loop_count++;
        usleep(1000);  // 1ms 循环
    }

    // 发送停止指令
    LOG_INFO("发送停止指令...");
    RobotCommand stop_cmd{};
    robot.send_command(stop_cmd);
    usleep(10000);

    robot.stop();
    robot.print_stats();

    LOG_INFO("=== SPI通信测试结束 ===");
    return 0;
}
