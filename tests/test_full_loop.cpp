/**
 * @file test_full_loop.cpp
 * @brief Jetson-ODroid-STM32完整数据流测试
 * @author Zomnk
 * @date 2026-02-03
 * 
 * @note 测试数据流：
 *       Jetson -> ODroid -> STM32 (控制指令)
 *       STM32 -> ODroid -> Jetson (观测反馈)
 * 
 * 使用方法：
 *   1. 先在Jetson上运行 jetson_full_test
 *   2. 再在ODroid上运行此程序
 *   3. 观察双向数据传输和电机响应
 */

#include <iostream>
#include <csignal>
#include <unistd.h>
#include <cmath>

#include "common/logger.hpp"
#include "common/time_utils.hpp"
#include "common/types.hpp"
#include "communication/udp_driver.hpp"
#include "communication/jetson_protocol.hpp"
#include "core/robot_interface.hpp"

using namespace odroid;

// 全局运行标志
volatile bool g_running = true;

void signal_handler(int sig) {
    LOG_INFO("收到信号 %d, 准备退出...", sig);
    g_running = false;
}

int main(int argc, char** argv) {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    Logger::instance().set_level(LogLevel::INFO);
    
    LOG_INFO("=== Jetson-ODroid-STM32 完整数据流测试 ===");
    LOG_INFO("数据流: Jetson(正弦action) -> ODroid -> STM32(电机)");
    LOG_INFO("       STM32(反馈) -> ODroid -> Jetson(打印)");
    LOG_INFO("");
    
    // 解析命令行参数
    const char* jetson_ip = "192.168.5.141";
    int port = 10000;
    
    if (argc >= 2) {
        jetson_ip = argv[1];
    }
    if (argc >= 3) {
        port = atoi(argv[2]);
    }
    
    LOG_INFO("Jetson IP: %s, Port: %d", jetson_ip, port);
    LOG_INFO("");
    
    // 初始化机器人接口（与STM32通信）
    RobotInterface robot;
    if (!robot.init()) {
        LOG_ERROR("机器人接口初始化失败!");
        return -1;
    }
    
    if (!robot.start()) {
        LOG_ERROR("通信线程启动失败!");
        return -1;
    }
    
    LOG_INFO("机器人接口已启动");
    
    // 初始化UDP（与Jetson通信）
    UDPConfig udp_config;
    udp_config.jetson_ip = jetson_ip;
    udp_config.jetson_port = port;
    udp_config.recv_timeout_ms = 50;
    
    UDPDriver udp;
    if (!udp.init(udp_config)) {
        LOG_ERROR("UDP初始化失败!");
        robot.stop();
        return -1;
    }
    
    LOG_INFO("UDP通信已初始化");
    LOG_INFO("开始完整数据流测试，按Ctrl+C退出");
    LOG_INFO("");
    
    uint64_t last_print_time = 0;
    uint64_t loop_count = 0;
    uint64_t action_recv_count = 0;
    uint64_t obs_send_count = 0;
    
    // 上次动作缓存（用于观测量）
    float last_action[NUM_JOINTS] = {0};
    
    while (g_running) {
        // ===== 1. 从Jetson接收Action (Response消息) =====
        JetsonResponse action_from_jetson;
        int recv_bytes = udp.receive(action_from_jetson);
        
        if (recv_bytes > 0) {
            action_recv_count++;
            
            // ===== 2. 将Action发送给STM32控制电机 =====
            RobotCommand cmd{};
            cmd.timestamp_us = get_time_us();
            
            // 检测标定模式：dq_exp[0] == -999.0 表示进入标定模式
            // 使用预留字段dq_exp作为标志，避免影响实际控制变量
            bool calibration_mode = (action_from_jetson.dq_exp[0] == -999.0f);
            
            // 控制参数：标定模式下kp=kd=0实现完全卸力
            const float kp = calibration_mode ? 0.0f : 1.0f;
            const float kd = calibration_mode ? 0.0f : 0.4f;
            const float velocity = 0.0f;
            
            if (calibration_mode && action_recv_count % 100 == 1) {
                LOG_INFO("[标定模式] 所有电机扭矩已卸载 (kp=0, kd=0)");
            }
            
            // 左腿5个电机
            cmd.left_leg.yaw.position = action_from_jetson.q_exp[0];
            cmd.left_leg.yaw.velocity = velocity;
            cmd.left_leg.yaw.kp = kp;
            cmd.left_leg.yaw.kd = kd;
            
            cmd.left_leg.roll.position = action_from_jetson.q_exp[1];
            cmd.left_leg.roll.velocity = velocity;
            cmd.left_leg.roll.kp = kp;
            cmd.left_leg.roll.kd = kd;
            
            cmd.left_leg.pitch.position = action_from_jetson.q_exp[2];
            cmd.left_leg.pitch.velocity = velocity;
            cmd.left_leg.pitch.kp = kp;
            cmd.left_leg.pitch.kd = kd;
            
            cmd.left_leg.knee.position = action_from_jetson.q_exp[3];
            cmd.left_leg.knee.velocity = velocity;
            cmd.left_leg.knee.kp = kp;
            cmd.left_leg.knee.kd = kd;
            
            cmd.left_leg.ankle.position = action_from_jetson.q_exp[4];
            cmd.left_leg.ankle.velocity = velocity;
            cmd.left_leg.ankle.kp = kp;
            cmd.left_leg.ankle.kd = kd;
            
            // 右腿5个电机
            cmd.right_leg.yaw.position = action_from_jetson.q_exp[5];
            cmd.right_leg.yaw.velocity = velocity;
            cmd.right_leg.yaw.kp = kp;
            cmd.right_leg.yaw.kd = kd;
            
            cmd.right_leg.roll.position = action_from_jetson.q_exp[6];
            cmd.right_leg.roll.velocity = velocity;
            cmd.right_leg.roll.kp = kp;
            cmd.right_leg.roll.kd = kd;
            
            cmd.right_leg.pitch.position = action_from_jetson.q_exp[7];
            cmd.right_leg.pitch.velocity = velocity;
            cmd.right_leg.pitch.kp = kp;
            cmd.right_leg.pitch.kd = kd;
            
            cmd.right_leg.knee.position = action_from_jetson.q_exp[8];
            cmd.right_leg.knee.velocity = velocity;
            cmd.right_leg.knee.kp = kp;
            cmd.right_leg.knee.kd = kd;
            
            cmd.right_leg.ankle.position = action_from_jetson.q_exp[9];
            cmd.right_leg.ankle.velocity = velocity;
            cmd.right_leg.ankle.kp = kp;
            cmd.right_leg.ankle.kd = kd;
            
            robot.send_command(cmd);
        }
        
        // ===== 3. 从STM32读取反馈 =====
        RobotFeedback feedback;
        if (robot.get_feedback(feedback)) {
            // ===== 4. 构造观测量发送给Jetson (Request消息) =====
            JetsonRequest obs_to_jetson;
            
            // 角速度 - 微雪IMU (imu[1])
            // 注意：STM32已经输出rad/s，无需再次转换
            obs_to_jetson.omega[0] = feedback.imu[1].gyro[0];  // rad/s (已转换)
            obs_to_jetson.omega[1] = feedback.imu[1].gyro[1];  // rad/s (已转换)
            obs_to_jetson.omega[2] = feedback.imu[1].gyro[2];  // rad/s (已转换)
            
            // 欧拉角姿态 - 微雪IMU (imu[1])
            // 注意：STM32已经输出rad，无需再次转换
            obs_to_jetson.eu_ang[0] = feedback.imu[1].euler[0];  // Roll (rad)
            obs_to_jetson.eu_ang[1] = feedback.imu[1].euler[1];  // Pitch (rad)
            obs_to_jetson.eu_ang[2] = feedback.imu[1].euler[2];  // Yaw (rad)
            
            // 关节位置 (10维)
            obs_to_jetson.q[0] = feedback.left_leg.yaw.position;
            obs_to_jetson.q[1] = feedback.left_leg.roll.position;
            obs_to_jetson.q[2] = feedback.left_leg.pitch.position;
            obs_to_jetson.q[3] = feedback.left_leg.knee.position;
            obs_to_jetson.q[4] = feedback.left_leg.ankle.position;
            obs_to_jetson.q[5] = feedback.right_leg.yaw.position;
            obs_to_jetson.q[6] = feedback.right_leg.roll.position;
            obs_to_jetson.q[7] = feedback.right_leg.pitch.position;
            obs_to_jetson.q[8] = feedback.right_leg.knee.position;
            obs_to_jetson.q[9] = feedback.right_leg.ankle.position;
            
            // 关节速度 (10维)
            obs_to_jetson.dq[0] = feedback.left_leg.yaw.velocity;
            obs_to_jetson.dq[1] = feedback.left_leg.roll.velocity;
            obs_to_jetson.dq[2] = feedback.left_leg.pitch.velocity;
            obs_to_jetson.dq[3] = feedback.left_leg.knee.velocity;
            obs_to_jetson.dq[4] = feedback.left_leg.ankle.velocity;
            obs_to_jetson.dq[5] = feedback.right_leg.yaw.velocity;
            obs_to_jetson.dq[6] = feedback.right_leg.roll.velocity;
            obs_to_jetson.dq[7] = feedback.right_leg.pitch.velocity;
            obs_to_jetson.dq[8] = feedback.right_leg.knee.velocity;
            obs_to_jetson.dq[9] = feedback.right_leg.ankle.velocity;
            
            // 上次动作 (从上次接收的Jetson action)
            for (int i = 0; i < NUM_JOINTS; i++) {
                obs_to_jetson.init_pos[i] = last_action[i];
            }
            
            // 其他字段（测试用）
            obs_to_jetson.trigger = 1.0f;
            obs_to_jetson.command[0] = 0.0f;  // vx
            obs_to_jetson.command[1] = 0.0f;  // vy
            obs_to_jetson.command[2] = 0.0f;  // yaw_rate
            obs_to_jetson.command[3] = 0.0f;
            
            // 加速度 - 微雪IMU
            obs_to_jetson.acc[0] = feedback.imu[1].accel[0];
            obs_to_jetson.acc[1] = feedback.imu[1].accel[1];
            obs_to_jetson.acc[2] = feedback.imu[1].accel[2];
            
            // 力矩反馈
            obs_to_jetson.tau[0] = feedback.left_leg.yaw.torque;
            obs_to_jetson.tau[1] = feedback.left_leg.roll.torque;
            obs_to_jetson.tau[2] = feedback.left_leg.pitch.torque;
            obs_to_jetson.tau[3] = feedback.left_leg.knee.torque;
            obs_to_jetson.tau[4] = feedback.left_leg.ankle.torque;
            obs_to_jetson.tau[5] = feedback.right_leg.yaw.torque;
            obs_to_jetson.tau[6] = feedback.right_leg.roll.torque;
            obs_to_jetson.tau[7] = feedback.right_leg.pitch.torque;
            obs_to_jetson.tau[8] = feedback.right_leg.knee.torque;
            obs_to_jetson.tau[9] = feedback.right_leg.ankle.torque;
            
            // 发送给Jetson
            int sent_bytes = udp.send(obs_to_jetson);
            if (sent_bytes > 0) {
                obs_send_count++;
                
                // 更新上次动作缓存
                if (recv_bytes > 0) {
                    for (int i = 0; i < NUM_JOINTS; i++) {
                        last_action[i] = action_from_jetson.q_exp[i];
                    }
                }
            }
        }
        
        loop_count++;
        
        // 每500ms打印统计信息
        uint64_t now = get_time_us();
        if (now - last_print_time >= 500000) {
            LOG_INFO("[统计 #%lu] 接收Action: %lu, 发送Obs: %lu, UDP连接: %s",
                     loop_count, action_recv_count, obs_send_count,
                     udp.is_connected() ? "是" : "否");
            
            if (action_recv_count > 0) {
                LOG_INFO("  最后接收的Action[0-4]: [%.3f, %.3f, %.3f, %.3f, %.3f]",
                         last_action[0], last_action[1], last_action[2], 
                         last_action[3], last_action[4]);
            }
            
            // 打印电机反馈示例（左腿）
            if (robot.get_feedback(feedback)) {
                LOG_INFO("  左腿电机位置: [%.3f, %.3f, %.3f, %.3f, %.3f]",
                         feedback.left_leg.yaw.position,
                         feedback.left_leg.roll.position,
                         feedback.left_leg.pitch.position,
                         feedback.left_leg.knee.position,
                         feedback.left_leg.ankle.position);
                LOG_INFO("  微雪IMU: 欧拉角=[%.3f, %.3f, %.3f], 角速度=[%.2f, %.2f, %.2f] deg/s",
                         feedback.imu[1].euler[0], feedback.imu[1].euler[1], feedback.imu[1].euler[2],
                         feedback.imu[1].gyro[0], feedback.imu[1].gyro[1], feedback.imu[1].gyro[2]);
            }
            LOG_INFO("");
            last_print_time = now;
        }
        
        usleep(2000);  // 2ms, 500Hz
    }
    
    // 停止前发送零位置零增益指令
    LOG_INFO("发送停止指令...");
    RobotCommand stop_cmd{};
    robot.send_command(stop_cmd);
    usleep(10000);
    
    robot.stop();
    robot.print_stats();
    udp.print_stats();
    udp.close();
    
    LOG_INFO("=== 完整数据流测试结束 ===");
    return 0;
}
