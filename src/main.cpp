/**
 * @file main.cpp
 * @brief ODroid机器人驱动主程序
 * @author Zomnk
 * @date 2026-02-04
 *
 * @details 本文件是ODroid端的主入口程序，负责：
 *          1. 管理SPI通信（ODroid ↔ STM32）
 *          2. 管理UDP通信（ODroid ↔ Jetson）
 *          3. 数据桥接（将Jetson的动作指令转发给STM32，将STM32的反馈转发给Jetson）
 *          4. 实时性能监控和统计
 *
 * @note 编译: mkdir build && cd build && cmake .. && make
 * @note 运行: sudo ./robot_driver <jetson_ip> [port]
 *       示例: sudo ./robot_driver 192.168.5.141 10000
 */

#include <cstdio>
#include <csignal>
#include <cstring>
#include <cmath>
#include <unistd.h>

#include "common/logger.hpp"
#include "common/time_utils.hpp"
#include "realtime/rt_utils.hpp"
#include "core/robot_interface.hpp"
#include "core/jetson_interface.hpp"

using namespace odroid;

/*
 * ============================================================
 * 全局变量
 * ============================================================
 */

/// 运行标志
static volatile bool g_running = true;

/**
 * @brief 信号处理函数
 * @param sig 信号编号
 */
void signal_handler(int sig) {
    (void)sig;
    g_running = false;
    LOG_INFO("收到退出信号，准备关闭...");
}

/**
 * @brief 打印使用说明
 * @param prog 程序名称
 */
void print_usage(const char* prog) {
    printf("用法: %s <jetson_ip> [port]\n", prog);
    printf("  jetson_ip: Jetson设备的IP地址\n");
    printf("  port:      UDP端口号 (默认: 10000)\n");
    printf("\n示例:\n");
    printf("  %s 192.168.5.141 10000\n", prog);
}

/**
 * @brief 主函数
 *
 * @details 程序执行流程：
 *          1. 解析命令行参数
 *          2. 初始化日志系统
 *          3. 初始化SPI通信（与STM32）
 *          4. 初始化UDP通信（与Jetson）
 *          5. 启动实时通信线程
 *          6. 进入主循环，监控状态并打印统计信息
 *          7. 收到退出信号后清理退出
 *
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return 正常退出返回0，错误返回1
 */
int main(int argc, char* argv[]) {
    // ========== 检查命令行参数 ==========
    if (argc < 2) {
        print_usage(argv[0]);
        return 1;
    }

    // ========== 解析命令行参数 ==========
    std::string jetson_ip = argv[1];
    int port = (argc >= 3) ? std::atoi(argv[2]) : 10000;

    // ========== 初始化日志系统 ==========
    Logger::instance().set_level(LogLevel::INFO);

    // ========== 打印启动信息 ==========
    LOG_INFO("============================================");
    LOG_INFO("  ODroid 机器人驱动程序");
    LOG_INFO("  版本: v1.0.0");
    LOG_INFO("  日期: 2026-02-04");
    LOG_INFO("============================================");
    LOG_INFO("通信配置:");
    LOG_INFO("  - SPI通信: ODroid <-> STM32 (1kHz)");
    LOG_INFO("  - UDP通信: ODroid <-> Jetson (%s:%d)", jetson_ip.c_str(), port);
    LOG_INFO("============================================");

    // ========== 注册信号处理函数 ==========
    signal(SIGINT, signal_handler);   // Ctrl+C
    signal(SIGTERM, signal_handler);  // kill命令

    // ========== 初始化SPI通信（STM32） ==========
    LOG_INFO("正在初始化SPI通信...");
    SPIConfig spi_config;
    RobotInterface robot;
    
    if (!robot.init(spi_config)) {
        LOG_FATAL("SPI通信初始化失败！");
        LOG_FATAL("请检查：");
        LOG_FATAL("  1. SPI设备是否正常 (/dev/spidev0.0)");
        LOG_FATAL("  2. 是否使用sudo权限运行");
        LOG_FATAL("  3. STM32是否正常供电");
        return 1;
    }
    LOG_INFO("✓ SPI通信初始化成功");

    // ========== 初始化UDP通信（Jetson） ==========
    LOG_INFO("正在初始化UDP通信...");
    JetsonInterface jetson;
    
    // 配置Jetson接口
    JetsonInterfaceConfig jetson_config;
    jetson_config.udp_config.jetson_ip = jetson_ip;
    jetson_config.udp_config.jetson_port = port;
    jetson_config.comm_rate_hz = 500;  // 500Hz
    
    if (!jetson.init(jetson_config)) {
        LOG_FATAL("UDP通信初始化失败！");
        LOG_FATAL("请检查：");
        LOG_FATAL("  1. 网络连接是否正常");
        LOG_FATAL("  2. IP地址是否正确: %s", jetson_ip.c_str());
        LOG_FATAL("  3. 端口是否被占用: %d", port);
        robot.stop();
        return 1;
    }
    LOG_INFO("✓ UDP通信初始化成功");

    // ========== 启动实时通信线程 ==========
    LOG_INFO("============================================");
    LOG_INFO("正在启动实时通信线程...");
    
    // 启动SPI线程（1kHz，与STM32通信）
    if (!robot.start()) {  // 默认周期已在constants.hpp中定义
        LOG_FATAL("SPI通信线程启动失败！");
        return 1;
    }
    LOG_INFO("✓ SPI线程已启动 (1kHz)");
    
    // 启动UDP线程（500Hz，与Jetson通信）
    if (!jetson.start()) {
        LOG_FATAL("UDP通信线程启动失败！");
        robot.stop();
        return 1;
    }
    LOG_INFO("✓ UDP线程已启动 (500Hz)");

    // ========== 开始主循环 ==========
    LOG_INFO("============================================");
    LOG_INFO("开始数据交换，等待Jetson连接...");
    LOG_INFO("提示: 在Jetson端运行相应程序");
    LOG_INFO("系统运行中... 按 Ctrl+C 退出");
    LOG_INFO("============================================");

    Timer stats_timer;
    Timer total_timer;  // 用于记录总运行时间
    uint64_t loop_count = 0;
    uint64_t bridge_count = 0;  // 数据桥接次数
    uint64_t send_count = 0;    // 发送计数
    uint64_t recv_count = 0;    // 接收计数
    bool first_connection_logged = false;  // 首次连接标志
    float user_command[4] = {0.0f, 0.0f, 0.0f, 0.0f};  // vx, vy, yaw_rate, reserved

    // ========== 主循环 - 数据桥接和监控 ==========
    while (g_running && robot.is_running()) {
        // ===== 数据桥接：STM32 <-> Jetson =====
        RobotFeedback feedback;
        bool got_feedback = robot.get_feedback(feedback);
        
        if (got_feedback) {
            // 1. 无条件发送观测数据给 Jetson（主动建立连接）
            jetson.send_observation(feedback, user_command);
            send_count++;
            
            // 2. 尝试获取 Jetson 的动作指令
            RobotCommand cmd;
            if (jetson.get_action(cmd)) {
                // 首次连接成功时打印日志
                if (!first_connection_logged) {
                    LOG_INFO("✓ Jetson已连接！数据交换已建立");
                    first_connection_logged = true;
                }
                
                // 3. 转发给 STM32 控制电机
                robot.send_command(cmd);
                bridge_count++;
                recv_count++;
            }
        } else {
            // 即使没有反馈，也发送一个空的观测数据保持连接
            RobotFeedback empty_feedback{};
            jetson.send_observation(empty_feedback, user_command);
            send_count++;
        }
        
        // 每1秒打印一次详细状态信息
        if (stats_timer.elapsed_sec() >= 1.0) {
            LOG_INFO("========================================");
            LOG_INFO("运行时间: %.1f 秒 | 主循环: %lu", 
                     total_timer.elapsed_sec(), loop_count);
            LOG_INFO("通信计数: 发送=%lu, 接收=%lu, 桥接=%lu", 
                     send_count, recv_count, bridge_count);
            
            // 获取机器人反馈数据
            RobotFeedback feedback;
            if (robot.get_feedback(feedback)) {
                LOG_INFO("----------------------------------------");
                LOG_INFO("左腿关节状态:");
                LOG_INFO("  Yaw:   位置=% 7.3f rad, 速度=% 7.3f rad/s, 力矩=% 7.3f Nm",
                         feedback.left_leg.yaw.position, feedback.left_leg.yaw.velocity, feedback.left_leg.yaw.torque);
                LOG_INFO("  Roll:  位置=% 7.3f rad, 速度=% 7.3f rad/s, 力矩=% 7.3f Nm",
                         feedback.left_leg.roll.position, feedback.left_leg.roll.velocity, feedback.left_leg.roll.torque);
                LOG_INFO("  Pitch: 位置=% 7.3f rad, 速度=% 7.3f rad/s, 力矩=% 7.3f Nm",
                         feedback.left_leg.pitch.position, feedback.left_leg.pitch.velocity, feedback.left_leg.pitch.torque);
                LOG_INFO("  Knee:  位置=% 7.3f rad, 速度=% 7.3f rad/s, 力矩=% 7.3f Nm",
                         feedback.left_leg.knee.position, feedback.left_leg.knee.velocity, feedback.left_leg.knee.torque);
                LOG_INFO("  Ankle: 位置=% 7.3f rad, 速度=% 7.3f rad/s, 力矩=% 7.3f Nm",
                         feedback.left_leg.ankle.position, feedback.left_leg.ankle.velocity, feedback.left_leg.ankle.torque);
                
                LOG_INFO("----------------------------------------");
                LOG_INFO("右腿关节状态:");
                LOG_INFO("  Yaw:   位置=% 7.3f rad, 速度=% 7.3f rad/s, 力矩=% 7.3f Nm",
                         feedback.right_leg.yaw.position, feedback.right_leg.yaw.velocity, feedback.right_leg.yaw.torque);
                LOG_INFO("  Roll:  位置=% 7.3f rad, 速度=% 7.3f rad/s, 力矩=% 7.3f Nm",
                         feedback.right_leg.roll.position, feedback.right_leg.roll.velocity, feedback.right_leg.roll.torque);
                LOG_INFO("  Pitch: 位置=% 7.3f rad, 速度=% 7.3f rad/s, 力矩=% 7.3f Nm",
                         feedback.right_leg.pitch.position, feedback.right_leg.pitch.velocity, feedback.right_leg.pitch.torque);
                LOG_INFO("  Knee:  位置=% 7.3f rad, 速度=% 7.3f rad/s, 力矩=% 7.3f Nm",
                         feedback.right_leg.knee.position, feedback.right_leg.knee.velocity, feedback.right_leg.knee.torque);
                LOG_INFO("  Ankle: 位置=% 7.3f rad, 速度=% 7.3f rad/s, 力矩=% 7.3f Nm",
                         feedback.right_leg.ankle.position, feedback.right_leg.ankle.velocity, feedback.right_leg.ankle.torque);
                
                LOG_INFO("----------------------------------------");
                LOG_INFO("IMU数据 (Waveshare 10轴):");
                LOG_INFO("  欧拉角: Roll=% 7.3f°, Pitch=% 7.3f°, Yaw=% 7.3f°",
                         feedback.imu[1].euler[0] * 180.0f / M_PI,
                         feedback.imu[1].euler[1] * 180.0f / M_PI,
                         feedback.imu[1].euler[2] * 180.0f / M_PI);
                LOG_INFO("  角速度: ωx=% 7.2f rad/s, ωy=% 7.2f rad/s, ωz=% 7.2f rad/s",
                         feedback.imu[1].gyro[0], feedback.imu[1].gyro[1], feedback.imu[1].gyro[2]);
                LOG_INFO("  加速度: ax=% 7.3f g, ay=% 7.3f g, az=% 7.3f g",
                         feedback.imu[1].accel[0], feedback.imu[1].accel[1], feedback.imu[1].accel[2]);
            } else {
                LOG_WARN("未能获取机器人反馈数据");
            }
            
            LOG_INFO("----------------------------------------");
            // 连接状态
            if (jetson.is_connected()) {
                LOG_INFO("Jetson状态: 已连接 ✓");
            } else {
                LOG_WARN("Jetson状态: 断开连接！");
            }
            
            LOG_INFO("----------------------------------------");
            LOG_INFO("通信统计:");
            LOG_INFO("  SPI周期数: %lu, 平均延迟: %.2f us, 最大延迟: %lu us",
                     robot.get_cycle_count(), robot.get_avg_latency_us(), robot.get_max_latency_us());
            
            LOG_INFO("========================================");
            
            stats_timer.reset();
        }

        loop_count++;
        usleep(2000);  // 2ms (500Hz) - 与 test_full_loop 保持一致
    }

    // ========== 清理退出 ==========
    LOG_INFO("============================================");
    LOG_INFO("正在停止系统...");
    
    // 先停止UDP通信（可能会卡在线程join）
    LOG_INFO("正在停止UDP通信...");
    jetson.stop();
    LOG_INFO("✓ UDP通信已停止");
    
    // 再停止SPI通信
    LOG_INFO("正在停止SPI通信...");
    robot.stop();
    LOG_INFO("✓ SPI通信已停止");
    
    // 打印最终统计
    LOG_INFO("============================================");
    LOG_INFO("最终统计信息:");
    LOG_INFO("  总运行时间: %.2f 秒", total_timer.elapsed_sec());
    LOG_INFO("  主循环总数: %lu", loop_count);
    LOG_INFO("----------------------------------------");
    LOG_INFO("SPI通信最终统计:");
    robot.print_stats();
    LOG_INFO("----------------------------------------");
    LOG_INFO("UDP通信最终统计:");
    jetson.print_stats();
    
    LOG_INFO("============================================");
    LOG_INFO("系统已正常退出");
    LOG_INFO("============================================");

    return 0;
}
