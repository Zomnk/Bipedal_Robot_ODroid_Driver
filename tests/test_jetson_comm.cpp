/**
 * @file test_jetson_comm.cpp
 * @brief Jetson通信测试程序
 * @author Zomnk
 * @date 2026-02-02
 *
 * @note 测试ODroid与Jetson之间的UDP通信
 * 使用方法:
 *   1. 在Jetson上运行sim2sim_lcm的udp_publisher_tinker
 *   2. 在ODroid上运行此测试程序
 *   3. 观察双向数据传输是否正常
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
#include "core/jetson_interface.hpp"

using namespace odroid;

// 全局运行标志
volatile bool g_running = true;

void signal_handler(int sig) {
    LOG_INFO("收到信号 %d, 准备退出...", sig);
    g_running = false;
}

// 打印JetsonRequest内容
void print_request(const JetsonRequest& req) {
    LOG_INFO("--- JetsonRequest ---");
    LOG_INFO("  trigger: %.1f", req.trigger);
    LOG_INFO("  command: [%.2f, %.2f, %.2f, %.2f]", 
             req.command[0], req.command[1], req.command[2], req.command[3]);
    LOG_INFO("  eu_ang: [%.3f, %.3f, %.3f] rad", 
             req.eu_ang[0], req.eu_ang[1], req.eu_ang[2]);
    LOG_INFO("  omega: [%.2f, %.2f, %.2f] rad/s", 
             req.omega[0], req.omega[1], req.omega[2]);
    LOG_INFO("  q: [%.2f, %.2f, %.2f, %.2f, %.2f | %.2f, %.2f, %.2f, %.2f, %.2f]",
             req.q[0], req.q[1], req.q[2], req.q[3], req.q[4],
             req.q[5], req.q[6], req.q[7], req.q[8], req.q[9]);
}

// 打印JetsonResponse内容
void print_response(const JetsonResponse& resp) {
    LOG_INFO("--- JetsonResponse ---");
    LOG_INFO("  q_exp: [%.3f, %.3f, %.3f, %.3f, %.3f | %.3f, %.3f, %.3f, %.3f, %.3f]",
             resp.q_exp[0], resp.q_exp[1], resp.q_exp[2], resp.q_exp[3], resp.q_exp[4],
             resp.q_exp[5], resp.q_exp[6], resp.q_exp[7], resp.q_exp[8], resp.q_exp[9]);
}

// ============================================================================
// 测试1: 基础UDP收发测试（简化版，仅验证消息传输）
// ============================================================================
void test_basic_udp(const char* jetson_ip, int port) {
    LOG_INFO("=== 测试1: 简化UDP消息传输测试 ===");
    LOG_INFO("目标: 验证UDP消息是否能正确收发，不涉及推理");
    
    UDPConfig config;
    config.jetson_ip = jetson_ip;
    config.jetson_port = port;
    config.recv_timeout_ms = 50;
    
    UDPDriver udp;
    if (!udp.init(config)) {
        LOG_ERROR("UDP初始化失败!");
        return;
    }
    
    LOG_INFO("开始UDP通信测试, 按Ctrl+C退出");
    LOG_INFO("Request size: %zu bytes", sizeof(JetsonRequest));
    LOG_INFO("Response size: %zu bytes", sizeof(JetsonResponse));
    
    uint64_t last_print_time = 0;
    uint64_t send_count = 0;
    uint64_t recv_count = 0;
    uint64_t loop_count = 0;
    
    while (g_running) {
        // 构造简单测试数据（使用递增序列便于验证）
        JetsonRequest request;
        request.trigger = 1.0f;
        
        // 使用固定测试值
        request.command[0] = 1.0f + loop_count * 0.01f;  // vx (递增)
        request.command[1] = 2.0f;  // vy
        request.command[2] = 3.0f;  // yaw_rate
        request.command[3] = 4.0f;  // reserved
        
        // 姿态数据（固定值）
        request.eu_ang[0] = 0.1f;  // roll
        request.eu_ang[1] = 0.2f;  // pitch
        request.eu_ang[2] = 0.3f;  // yaw
        
        // 角速度（固定值）
        request.omega[0] = 1.1f;
        request.omega[1] = 1.2f;
        request.omega[2] = 1.3f;
        
        // 加速度（固定值）
        request.acc[0] = 2.1f;
        request.acc[1] = 2.2f;
        request.acc[2] = 2.3f;
        
        // 关节数据（使用简单递增序列）
        for (int i = 0; i < NUM_JOINTS; i++) {
            request.q[i] = (float)(i + 1) * 0.1f;
            request.dq[i] = (float)(i + 1) * 0.2f;
            request.tau[i] = (float)(i + 1) * 0.3f;
            request.init_pos[i] = (float)(i + 1) * 0.05f;
        }
        
        // 发送
        int sent = udp.send(request);
        if (sent > 0) {
            send_count++;
        }
        
        // 接收
        JetsonResponse response;
        int recvd = udp.receive(response);
        if (recvd > 0) {
            recv_count++;
        }
        
        loop_count++;
        
        // 每500ms打印一次状态
        uint64_t now = get_time_us();
        if (now - last_print_time >= 500000) {
            LOG_INFO("[状态] Loop: %lu, TX: %lu, RX: %lu, 连接: %s", 
                     loop_count, send_count, recv_count, 
                     udp.is_connected() ? "是" : "否");
            
            // 打印最后发送的Request
            LOG_INFO("[发送] trigger=%.1f, cmd=[%.2f,%.2f,%.2f,%.2f]",
                     request.trigger,
                     request.command[0], request.command[1], 
                     request.command[2], request.command[3]);
            LOG_INFO("       姿态=[%.3f,%.3f,%.3f], 角速度=[%.2f,%.2f,%.2f]",
                     request.eu_ang[0], request.eu_ang[1], request.eu_ang[2],
                     request.omega[0], request.omega[1], request.omega[2]);
            LOG_INFO("       q[0-4]=[%.2f,%.2f,%.2f,%.2f,%.2f]",
                     request.q[0], request.q[1], request.q[2], request.q[3], request.q[4]);
            
            // 打印最后接收的Response
            if (recv_count > 0) {
                JetsonResponse last_resp = udp.get_last_response();
                LOG_INFO("[接收] q_exp[0-4]=[%.3f,%.3f,%.3f,%.3f,%.3f]",
                         last_resp.q_exp[0], last_resp.q_exp[1], 
                         last_resp.q_exp[2], last_resp.q_exp[3], last_resp.q_exp[4]);
                LOG_INFO("       q_exp[5-9]=[%.3f,%.3f,%.3f,%.3f,%.3f]",
                         last_resp.q_exp[5], last_resp.q_exp[6], 
                         last_resp.q_exp[7], last_resp.q_exp[8], last_resp.q_exp[9]);
            } else {
                LOG_WARN("       尚未接收到Jetson响应");
            }
            LOG_INFO("");
            last_print_time = now;
        }
        
        usleep(2000);  // 500Hz
    }
    
    LOG_INFO("\n=== 测试统计 ===");
    udp.print_stats();
    udp.close();
}

// ============================================================================
// 测试2: JetsonInterface完整测试
// ============================================================================
void test_jetson_interface(const char* jetson_ip, int port) {
    LOG_INFO("=== 测试2: JetsonInterface完整测试 ===");
    
    JetsonInterfaceConfig config;
    config.udp_config.jetson_ip = jetson_ip;
    config.udp_config.jetson_port = port;
    config.kp = 3.0f;
    config.kd = 0.15f;
    
    JetsonInterface jetson;
    if (!jetson.init(config)) {
        LOG_ERROR("Jetson接口初始化失败!");
        return;
    }
    
    if (!jetson.start()) {
        LOG_ERROR("Jetson接口启动失败!");
        return;
    }
    
    LOG_INFO("Jetson接口已启动, 按Ctrl+C退出");
    
    uint64_t last_print_time = 0;
    
    while (g_running) {
        // 模拟机器人反馈数据
        RobotFeedback feedback;
        float t = get_time_us() / 1000000.0f;
        
        // IMU数据
        feedback.imu[1].euler[0] = 0.1f * std::sin(t);
        feedback.imu[1].euler[1] = 0.05f * std::cos(t);
        feedback.imu[1].euler[2] = 0.0f;
        feedback.imu[1].gyro[0] = 5.0f * std::cos(t);  // deg/s
        feedback.imu[1].gyro[1] = -2.5f * std::sin(t);
        feedback.imu[1].gyro[2] = 0.0f;
        
        // 左腿关节
        feedback.left_leg.yaw.position = 0.1f * std::sin(t);
        feedback.left_leg.roll.position = 0.1f * std::sin(t + 0.3f);
        feedback.left_leg.pitch.position = 0.1f * std::sin(t + 0.6f);
        feedback.left_leg.knee.position = 0.1f * std::sin(t + 0.9f);
        feedback.left_leg.ankle.position = 0.1f * std::sin(t + 1.2f);
        
        // 右腿关节
        feedback.right_leg.yaw.position = 0.1f * std::sin(t + 1.5f);
        feedback.right_leg.roll.position = 0.1f * std::sin(t + 1.8f);
        feedback.right_leg.pitch.position = 0.1f * std::sin(t + 2.1f);
        feedback.right_leg.knee.position = 0.1f * std::sin(t + 2.4f);
        feedback.right_leg.ankle.position = 0.1f * std::sin(t + 2.7f);
        
        // 控制指令
        float command[4] = {0.5f, 0.0f, 0.0f, 0.0f};
        
        // 发送观测数据
        jetson.send_observation(feedback, command);
        
        // 获取动作
        RobotCommand cmd;
        bool has_action = jetson.get_action(cmd);
        
        // 每500ms打印
        uint64_t now = get_time_us();
        if (now - last_print_time >= 500000) {
            LOG_INFO("Connected: %s", jetson.is_connected() ? "Yes" : "No");
            if (has_action) {
                LOG_INFO("Action received:");
                LOG_INFO("  Left:  [%.3f, %.3f, %.3f, %.3f, %.3f]",
                         cmd.left_leg.yaw.position,
                         cmd.left_leg.roll.position,
                         cmd.left_leg.pitch.position,
                         cmd.left_leg.knee.position,
                         cmd.left_leg.ankle.position);
                LOG_INFO("  Right: [%.3f, %.3f, %.3f, %.3f, %.3f]",
                         cmd.right_leg.yaw.position,
                         cmd.right_leg.roll.position,
                         cmd.right_leg.pitch.position,
                         cmd.right_leg.knee.position,
                         cmd.right_leg.ankle.position);
            }
            last_print_time = now;
        }
        
        usleep(2000);  // 500Hz
    }
    
    jetson.print_stats();
    jetson.stop();
}

// ============================================================================
// 测试3: 回环测试 (无需Jetson, 自发自收)
// ============================================================================
void test_loopback() {
    LOG_INFO("=== 测试3: UDP回环测试 ===");
    LOG_INFO("此测试需要在本机同时运行发送和接收");
    LOG_INFO("请在另一个终端运行: nc -u -l 10000 (或其他UDP回显程序)");
    
    UDPConfig config;
    config.jetson_ip = "127.0.0.1";
    config.jetson_port = 10000;
    config.recv_timeout_ms = 100;
    
    UDPDriver udp;
    if (!udp.init(config)) {
        LOG_ERROR("UDP初始化失败!");
        return;
    }
    
    LOG_INFO("发送测试数据...");
    
    JetsonRequest request;
    request.trigger = 1.0f;
    request.command[0] = 1.23f;
    request.command[1] = 4.56f;
    
    for (int i = 0; i < 10 && g_running; i++) {
        int sent = udp.send(request);
        LOG_INFO("Sent %d bytes", sent);
        
        JetsonResponse response;
        int recv = udp.receive(response);
        if (recv > 0) {
            LOG_INFO("Received %d bytes", recv);
        } else {
            LOG_INFO("No response (timeout)");
        }
        
        sleep(1);
    }
    
    udp.close();
}

// ============================================================================
// Main
// ============================================================================
int main(int argc, char** argv) {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    Logger::instance().set_level(LogLevel::INFO);
    
    LOG_INFO("=== ODroid-Jetson 通信测试 ===");
    LOG_INFO("协议版本: sim2sim_lcm compatible");
    LOG_INFO("Request size: %zu bytes", sizeof(JetsonRequest));
    LOG_INFO("Response size: %zu bytes", sizeof(JetsonResponse));
    LOG_INFO("");
    
    // 解析命令行参数
    const char* odroid_ip = "192.168.5.159";
    const char* jetson_ip = "192.168.5.141";
    int port = 10000;
    int test_mode = 1;
    
    if (argc >= 2) {
        jetson_ip = argv[1];
    }
    if (argc >= 3) {
        port = atoi(argv[2]);
    }
    if (argc >= 4) {
        test_mode = atoi(argv[3]);
    }
    
    LOG_INFO("ODroid IP: %s", odroid_ip);
    LOG_INFO("Jetson IP: %s", jetson_ip);
    LOG_INFO("Port: %d", port);
    LOG_INFO("Test mode: %d", test_mode);
    LOG_INFO("");
    
    switch (test_mode) {
        case 1:
            test_basic_udp(jetson_ip, port);
            break;
        case 2:
            test_jetson_interface(jetson_ip, port);
            break;
        case 3:
            test_loopback();
            break;
        default:
            LOG_ERROR("未知测试模式: %d", test_mode);
            LOG_INFO("用法: %s [jetson_ip] [port] [test_mode]", argv[0]);
            LOG_INFO("  test_mode 1: 基础UDP收发测试");
            LOG_INFO("  test_mode 2: JetsonInterface完整测试");
            LOG_INFO("  test_mode 3: 本地回环测试");
            return 1;
    }
    
    LOG_INFO("=== 测试结束 ===");
    return 0;
}
