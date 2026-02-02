/**
 * @file test_motor_control.cpp
 * @brief 电机控制测试 - 左腿五电机位置切换测试
 * @author Zomnk
 * @date 2026-02-01
 */

#include <iostream>
#include <cmath>
#include <csignal>
#include <unistd.h>
#include "common/logger.hpp"
#include "common/types.hpp"
#include "common/constants.hpp"
#include "core/robot_interface.hpp"
#include "core/data_hub.hpp"

using namespace odroid;

// 全局运行标志
volatile bool g_running = true;

// 信号处理
void signal_handler(int sig) {
    LOG_INFO("收到信号 %d, 准备退出...", sig);
    g_running = false;
}

// 打印左腿电机反馈信息
void print_left_leg_feedback(const LegFeedback& leg) {
    LOG_INFO("--- 左腿电机反馈 ---");
    LOG_INFO("  Yaw:   pos=%+7.3f rad, vel=%+7.3f rad/s, torque=%+6.3f Nm, temp=%.1f C",
             leg.yaw.position, leg.yaw.velocity, leg.yaw.torque, leg.yaw.temperature);
    LOG_INFO("  Roll:  pos=%+7.3f rad, vel=%+7.3f rad/s, torque=%+6.3f Nm, temp=%.1f C",
             leg.roll.position, leg.roll.velocity, leg.roll.torque, leg.roll.temperature);
    LOG_INFO("  Pitch: pos=%+7.3f rad, vel=%+7.3f rad/s, torque=%+6.3f Nm, temp=%.1f C",
             leg.pitch.position, leg.pitch.velocity, leg.pitch.torque, leg.pitch.temperature);
    LOG_INFO("  Knee:  pos=%+7.3f rad, vel=%+7.3f rad/s, torque=%+6.3f Nm, temp=%.1f C",
             leg.knee.position, leg.knee.velocity, leg.knee.torque, leg.knee.temperature);
    LOG_INFO("  Ankle: pos=%+7.3f rad, vel=%+7.3f rad/s, torque=%+6.3f Nm, temp=%.1f C",
             leg.ankle.position, leg.ankle.velocity, leg.ankle.torque, leg.ankle.temperature);
}

// 打印IMU反馈信息
void print_imu_feedback(const IMUFeedback& imu, const char* name) {
    LOG_INFO("--- %s IMU反馈 ---", name);
    LOG_INFO("  Gyro:  [%+8.2f, %+8.2f, %+8.2f] deg/s",
             imu.gyro[0], imu.gyro[1], imu.gyro[2]);
    LOG_INFO("  Accel: [%+6.3f, %+6.3f, %+6.3f] g",
             imu.accel[0], imu.accel[1], imu.accel[2]);
    LOG_INFO("  Euler: [%+7.4f, %+7.4f, %+7.4f] rad",
             imu.euler[0], imu.euler[1], imu.euler[2]);
    LOG_INFO("  Temp:  %.1f C", imu.temperature);
}

// 设置左腿所有电机的控制参数
void set_left_leg_command(LegCommand& leg, float position, float velocity, float kp, float kd) {
    // Yaw - DM6006
    leg.yaw.position = position;
    leg.yaw.velocity = velocity;
    leg.yaw.kp = kp;
    leg.yaw.kd = kd;
    
    // Roll - DM4340
    leg.roll.position = position;
    leg.roll.velocity = velocity;
    leg.roll.kp = kp;
    leg.roll.kd = kd;
    
    // Pitch - DM8006
    leg.pitch.position = position;
    leg.pitch.velocity = velocity;
    leg.pitch.kp = kp;
    leg.pitch.kd = kd;
    
    // Knee - DM6006
    leg.knee.position = position;
    leg.knee.velocity = velocity;
    leg.knee.kp = kp;
    leg.knee.kd = kd;
    
    // Ankle - DM6006
    leg.ankle.position = position;
    leg.ankle.velocity = velocity;
    leg.ankle.kp = kp;
    leg.ankle.kd = kd;
}

int main() {
    // 设置信号处理
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    Logger::instance().set_level(LogLevel::INFO);

    LOG_INFO("=== 左腿电机控制测试 ===");
    LOG_INFO("测试参数: vel=0, kp=3, kd=0.4");
    LOG_INFO("位置目标: 每3秒在 -1.57 和 +1.57 之间切换");
    LOG_INFO("按 Ctrl+C 退出测试");
    LOG_INFO("");

    // 初始化机器人接口
    RobotInterface robot;
    if (!robot.init()) {
        LOG_ERROR("机器人接口初始化失败!");
        return -1;
    }

    // 启动通信线程
    if (!robot.start()) {
        LOG_ERROR("通信线程启动失败!");
        return -1;
    }

    // 控制参数
    const float kp = 3.0f;
    const float kd = 0.4f;
    const float velocity = 0.0f;
    const float pos_high = 1.57f;   // +90度
    const float pos_low = -1.57f;   // -90度
    const uint64_t switch_interval_us = 3000000;  // 3秒切换

    // 状态变量
    float target_position = pos_high;
    uint64_t last_switch_time = get_time_us();
    uint64_t last_print_time = get_time_us();
    const uint64_t print_interval_us = 500000;  // 每500ms打印一次
    int print_count = 0;  // 计数器，用于间隔打印原始数据

    LOG_INFO("开始控制循环...");
    LOG_INFO("");

    while (g_running) {
        uint64_t now = get_time_us();

        // 每3秒切换位置目标
        if (now - last_switch_time >= switch_interval_us) {
            target_position = (target_position > 0) ? pos_low : pos_high;
            last_switch_time = now;
            LOG_INFO(">>> 切换目标位置: %.2f rad <<<", target_position);
        }

        // 发送左腿控制指令
        RobotCommand cmd{};
        set_left_leg_command(cmd.left_leg, target_position, velocity, kp, kd);
        cmd.timestamp_us = now;
        robot.send_command(cmd);

        // 每500ms打印反馈信息
        if (now - last_print_time >= print_interval_us) {
            RobotFeedback feedback;
            if (robot.get_feedback(feedback)) {
                LOG_INFO("========== 反馈数据 [目标位置: %+.2f rad] ==========", target_position);
                print_left_leg_feedback(feedback.left_leg);
                print_imu_feedback(feedback.imu[0], "ICM20602");
                print_imu_feedback(feedback.imu[1], "Waveshare");
                
                // 每5次打印一次原始数据(约2.5秒一次)
                if (++print_count % 5 == 0) {
                    robot.print_raw_data();
                }
                LOG_INFO("");
            }
            last_print_time = now;
        }

        // 休眠1ms
        usleep(1000);
    }

    // 停止前发送零位置零增益指令
    LOG_INFO("发送停止指令...");
    RobotCommand stop_cmd{};
    set_left_leg_command(stop_cmd.left_leg, 0.0f, 0.0f, 0.0f, 0.0f);
    robot.send_command(stop_cmd);
    usleep(10000);  // 等待发送完成

    robot.stop();
    robot.print_stats();

    LOG_INFO("=== 电机控制测试结束 ===");
    return 0;
}