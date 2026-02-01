/**
 * @file main.cpp
 * @brief ODroid机器人驱动主程序
 * @author Zomnk
 * @date 2026-02-01
 * 
 * 编译: mkdir build && cd build && cmake .. && make
 * 运行: sudo ./robot_driver
 */

#include <cstdio>
#include <csignal>

#include "common/logger.hpp"
#include "common/time_utils.hpp"
#include "realtime/rt_utils.hpp"
#include "core/robot_interface.hpp"

using namespace odroid;

static volatile bool g_running = true;

void signal_handler(int sig) {
    (void)sig;
    g_running = false;
    LOG_INFO("Received signal, shutting down...");
}

int main(int argc, char* argv[]) {
    // 解析参数
    SPIConfig spi_config;
    if (argc > 1) {
        spi_config.device = argv[1];
    }

    Logger::instance().set_level(LogLevel::DEBUG);

    LOG_INFO("===========================================");
    LOG_INFO("    ODroid Robot Driver");
    LOG_INFO("    SPI Communication with STM32");
    LOG_INFO("===========================================");

    // 信号处理
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // 创建机器人接口
    RobotInterface robot;

    // 初始化
    if (!robot.init(spi_config)) {
        LOG_FATAL("Failed to initialize robot interface");
        return 1;
    }

    // 启动
    if (!robot.start()) {
        LOG_FATAL("Failed to start robot interface");
        return 1;
    }

    LOG_INFO("Robot driver running. Press Ctrl+C to stop.");

    // 主循环 - 监控和状态打印
    Timer stats_timer;
    while (g_running && robot.is_running()) {
        // 每5秒打印一次统计信息
        if (stats_timer.elapsed_sec() >= 5.0) {
            robot.print_stats();
            stats_timer.reset();
        }

        // 获取反馈数据 (示例)
        RobotFeedback fb;
        if (DataHub::instance().has_new_feedback()) {
            DataHub::instance().get_feedback(fb);
            // 这里可以添加数据处理逻辑
        }

        sleep_ms(100);  // 主循环不需要高频率
    }

    // 停止
    robot.stop();

    LOG_INFO("Robot driver stopped");
    return 0;
}
