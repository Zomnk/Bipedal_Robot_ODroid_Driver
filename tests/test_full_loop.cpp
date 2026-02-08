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
            // tau_exp[0] 存储当前正在标定的关节ID (0-9)
            bool calibration_mode = (action_from_jetson.dq_exp[0] == -999.0f);
            int calibrating_joint_id = calibration_mode ? static_cast<int>(action_from_jetson.tau_exp[0]) : -1;
            
            // 控制参数：每个关节独立的Kp和Kd增益
            // 索引顺序: [0-4]左腿 (Yaw, Roll, Pitch, Knee, Ankle)
            //          [5-9]右腿 (Yaw, Roll, Pitch, Knee, Ankle)
            const float kp_gains[NUM_JOINTS] = {
                10.0f, 20.0f, 10.0f, 10.0f, 10.0f,  // 左腿
                10.0f, 20.0f, 10.0f, 10.0f, 10.0f   // 右腿
            };
            const float kd_gains[NUM_JOINTS] = {
                0.4f, 0.4f, 0.4f, 0.4f, 0.4f,  // 左腿
                0.4f, 0.4f, 0.4f, 0.4f, 0.4f   // 右腿
            };
            const float velocity = 0.0f;

            // 使用独立的Kp/Kd值，标定模式时卸载对应关节扭矩
            cmd.left_leg.yaw.position = action_from_jetson.q_exp[0];
            cmd.left_leg.yaw.velocity = velocity;
            cmd.left_leg.yaw.kp = (calibrating_joint_id == 0) ? 0.0f : kp_gains[0];
            cmd.left_leg.yaw.kd = (calibrating_joint_id == 0) ? 0.0f : kd_gains[0];
            
            cmd.left_leg.roll.position = action_from_jetson.q_exp[1];
            cmd.left_leg.roll.velocity = velocity;
            cmd.left_leg.roll.kp = (calibrating_joint_id == 1) ? 0.0f : kp_gains[1];
            cmd.left_leg.roll.kd = (calibrating_joint_id == 1) ? 0.0f : kd_gains[1];
            
            cmd.left_leg.pitch.position = action_from_jetson.q_exp[2];
            cmd.left_leg.pitch.velocity = velocity;
            cmd.left_leg.pitch.kp = (calibrating_joint_id == 2) ? 0.0f : kp_gains[2];
            cmd.left_leg.pitch.kd = (calibrating_joint_id == 2) ? 0.0f : kd_gains[2];
            
            cmd.left_leg.knee.position = action_from_jetson.q_exp[3];
            cmd.left_leg.knee.velocity = velocity;
            cmd.left_leg.knee.kp = (calibrating_joint_id == 3) ? 0.0f : kp_gains[3];
            cmd.left_leg.knee.kd = (calibrating_joint_id == 3) ? 0.0f : kd_gains[3];
            
            cmd.left_leg.ankle.position = action_from_jetson.q_exp[4];
            cmd.left_leg.ankle.velocity = velocity;
            cmd.left_leg.ankle.kp = (calibrating_joint_id == 4) ? 0.0f : kp_gains[4];
            cmd.left_leg.ankle.kd = (calibrating_joint_id == 4) ? 0.0f : kd_gains[4];
            
            // 右腿5个电机
            cmd.right_leg.yaw.position = action_from_jetson.q_exp[5];
            cmd.right_leg.yaw.velocity = velocity;
            cmd.right_leg.yaw.kp = (calibrating_joint_id == 5) ? 0.0f : kp_gains[5];
            cmd.right_leg.yaw.kd = (calibrating_joint_id == 5) ? 0.0f : kd_gains[5];
            
            cmd.right_leg.roll.position = action_from_jetson.q_exp[6];
            cmd.right_leg.roll.velocity = velocity;
            cmd.right_leg.roll.kp = (calibrating_joint_id == 6) ? 0.0f : kp_gains[6];
            cmd.right_leg.roll.kd = (calibrating_joint_id == 6) ? 0.0f : kd_gains[6];
            
            cmd.right_leg.pitch.position = action_from_jetson.q_exp[7];
            cmd.right_leg.pitch.velocity = velocity;
            cmd.right_leg.pitch.kp = (calibrating_joint_id == 7) ? 0.0f : kp_gains[7];
            cmd.right_leg.pitch.kd = (calibrating_joint_id == 7) ? 0.0f : kd_gains[7];
            
            cmd.right_leg.knee.position = action_from_jetson.q_exp[8];
            cmd.right_leg.knee.velocity = velocity;
            cmd.right_leg.knee.kp = (calibrating_joint_id == 8) ? 0.0f : kp_gains[8];
            cmd.right_leg.knee.kd = (calibrating_joint_id == 8) ? 0.0f : kd_gains[8];
            
            cmd.right_leg.ankle.position = action_from_jetson.q_exp[9];
            cmd.right_leg.ankle.velocity = velocity;
            cmd.right_leg.ankle.kp = (calibrating_joint_id == 9) ? 0.0f : kp_gains[9];
            cmd.right_leg.ankle.kd = (calibrating_joint_id == 9) ? 0.0f : kd_gains[9];
            
            robot.send_command(cmd);
        }
        
        // ===== 3. 从STM32读取反馈 =====
        RobotFeedback feedback;
        if (robot.get_feedback(feedback)) {
            // ===== 4. 构造观测量发送给Jetson (Request消息) =====
            // 必须严格按照Jetson端MsgRequest的字段顺序赋值！
            // 顺序: trigger, command[4], eu_ang[3], omega[3], acc[3], 
            //       q[10], dq[10], tau[10], init_pos[10]
            JetsonRequest obs_to_jetson;
            
            // 1. 触发标志 (1个float)
            obs_to_jetson.trigger = 1.0f;
            
            // 2. 控制指令 (4个float: vx, vy, yaw_rate, reserved)
            // 测试模式下使用零速度指令
            obs_to_jetson.command[0] = 0.0f;  // vx
            obs_to_jetson.command[1] = 0.0f;  // vy
            obs_to_jetson.command[2] = 0.0f;  // yaw_rate
            obs_to_jetson.command[3] = 0.0f;  // reserved
            
            // 3. 欧拉角姿态 (3个float: roll, pitch, yaw) - 微雪IMU (imu[1])
            // 注意：STM32已经输出rad，无需再次转换
            obs_to_jetson.eu_ang[0] = feedback.imu[1].euler[0];  // Roll (rad)
            obs_to_jetson.eu_ang[1] = feedback.imu[1].euler[1];  // Pitch (rad)
            obs_to_jetson.eu_ang[2] = feedback.imu[1].euler[2];  // Yaw (rad)
            
            // 4. 角速度 (3个float: wx, wy, wz) - 微雪IMU (imu[1])
            // 注意：STM32已经输出rad/s，无需再次转换
            obs_to_jetson.omega[0] = feedback.imu[1].gyro[0];  // rad/s (已转换)
            obs_to_jetson.omega[1] = feedback.imu[1].gyro[1];  // rad/s (已转换)
            obs_to_jetson.omega[2] = feedback.imu[1].gyro[2];  // rad/s (已转换)
            
            // 5. 加速度 (3个float: ax, ay, az) - 微雪IMU
            obs_to_jetson.acc[0] = feedback.imu[1].accel[0];
            obs_to_jetson.acc[1] = feedback.imu[1].accel[1];
            obs_to_jetson.acc[2] = feedback.imu[1].accel[2];
            
            // 6. 关节位置 (10个float)
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
            
            // 7. 关节速度 (10个float)
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
            
            // 8. 力矩反馈 (10个float)
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
            
            // 9. 上次动作 (10个float) - 从上次接收的Jetson action填充
            for (int i = 0; i < NUM_JOINTS; i++) {
                obs_to_jetson.init_pos[i] = last_action[i];
            }
            
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
            LOG_INFO("========================================");
            LOG_INFO("[统计 #%lu] 接收Action: %lu, 发送Obs: %lu, UDP连接: %s",
                     loop_count, action_recv_count, obs_send_count,
                     udp.is_connected() ? "是" : "否");
            LOG_INFO("========================================");
            
            // ===== 打印Jetson发送的Action =====
            if (action_recv_count > 0) {
                LOG_INFO("【Jetson动作指令】");
                LOG_INFO("  左腿动作: Yaw=%.3f, Roll=%.3f, Pitch=%.3f, Knee=%.3f, Ankle=%.3f",
                         last_action[0], last_action[1], last_action[2], last_action[3], last_action[4]);
                LOG_INFO("  右腿动作: Yaw=%.3f, Roll=%.3f, Pitch=%.3f, Knee=%.3f, Ankle=%.3f",
                         last_action[5], last_action[6], last_action[7], last_action[8], last_action[9]);
                LOG_INFO("");
            }
            
            // ===== 打印电机反馈（位置、速度、力矩）=====
            if (robot.get_feedback(feedback)) {
                LOG_INFO("【左腿电机反馈】");
                LOG_INFO("  Yaw   - 位置:% 7.3f rad, 速度:% 7.3f rad/s, 力矩:% 7.3f Nm",
                         feedback.left_leg.yaw.position, feedback.left_leg.yaw.velocity, feedback.left_leg.yaw.torque);
                LOG_INFO("  Roll  - 位置:% 7.3f rad, 速度:% 7.3f rad/s, 力矩:% 7.3f Nm",
                         feedback.left_leg.roll.position, feedback.left_leg.roll.velocity, feedback.left_leg.roll.torque);
                LOG_INFO("  Pitch - 位置:% 7.3f rad, 速度:% 7.3f rad/s, 力矩:% 7.3f Nm",
                         feedback.left_leg.pitch.position, feedback.left_leg.pitch.velocity, feedback.left_leg.pitch.torque);
                LOG_INFO("  Knee  - 位置:% 7.3f rad, 速度:% 7.3f rad/s, 力矩:% 7.3f Nm",
                         feedback.left_leg.knee.position, feedback.left_leg.knee.velocity, feedback.left_leg.knee.torque);
                LOG_INFO("  Ankle - 位置:% 7.3f rad, 速度:% 7.3f rad/s, 力矩:% 7.3f Nm",
                         feedback.left_leg.ankle.position, feedback.left_leg.ankle.velocity, feedback.left_leg.ankle.torque);
                LOG_INFO("");
                
                LOG_INFO("【右腿电机反馈】");
                LOG_INFO("  Yaw   - 位置:% 7.3f rad, 速度:% 7.3f rad/s, 力矩:% 7.3f Nm",
                         feedback.right_leg.yaw.position, feedback.right_leg.yaw.velocity, feedback.right_leg.yaw.torque);
                LOG_INFO("  Roll  - 位置:% 7.3f rad, 速度:% 7.3f rad/s, 力矩:% 7.3f Nm",
                         feedback.right_leg.roll.position, feedback.right_leg.roll.velocity, feedback.right_leg.roll.torque);
                LOG_INFO("  Pitch - 位置:% 7.3f rad, 速度:% 7.3f rad/s, 力矩:% 7.3f Nm",
                         feedback.right_leg.pitch.position, feedback.right_leg.pitch.velocity, feedback.right_leg.pitch.torque);
                LOG_INFO("  Knee  - 位置:% 7.3f rad, 速度:% 7.3f rad/s, 力矩:% 7.3f Nm",
                         feedback.right_leg.knee.position, feedback.right_leg.knee.velocity, feedback.right_leg.knee.torque);
                LOG_INFO("  Ankle - 位置:% 7.3f rad, 速度:% 7.3f rad/s, 力矩:% 7.3f Nm",
                         feedback.right_leg.ankle.position, feedback.right_leg.ankle.velocity, feedback.right_leg.ankle.torque);
                LOG_INFO("");
                
                LOG_INFO("【IMU反馈数据】(微雪10轴)");
                LOG_INFO("  欧拉角 - Roll:% 7.3f°, Pitch:% 7.3f°, Yaw:% 7.3f°",
                         feedback.imu[1].euler[0] * 180.0f / M_PI,
                         feedback.imu[1].euler[1] * 180.0f / M_PI,
                         feedback.imu[1].euler[2] * 180.0f / M_PI);
                LOG_INFO("  角速度 - ωx:% 7.3f rad/s, ωy:% 7.3f rad/s, ωz:% 7.3f rad/s",
                         feedback.imu[1].gyro[0], feedback.imu[1].gyro[1], feedback.imu[1].gyro[2]);
                LOG_INFO("  加速度 - ax:% 7.3f g, ay:% 7.3f g, az:% 7.3f g",
                         feedback.imu[1].accel[0], feedback.imu[1].accel[1], feedback.imu[1].accel[2]);
            }
            LOG_INFO("========================================");
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
