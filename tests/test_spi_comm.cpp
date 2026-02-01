/**
 * @file test_spi_comm.cpp
 * @brief SPI通信测试程序
 * @author Zomnk
 * @date 2026-02-01
 * 
 * 用于测试ODroid与STM32之间的SPI通信
 * 
 * 编译: mkdir build && cd build && cmake .. && make test_spi_comm
 * 运行: sudo ./test_spi_comm [选项]
 * 
 * 选项:
 *   -d <device>  SPI设备 (默认: /dev/spidev0.0)
 *   -s <speed>   SPI速度Hz (默认: 10000000)
 *   -n <count>   传输次数 (默认: 1000)
 *   -t <period>  传输周期us (默认: 1000)
 *   -v           详细输出
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <getopt.h>

#include "common/types.hpp"
#include "common/constants.hpp"
#include "common/logger.hpp"
#include "common/time_utils.hpp"
#include "realtime/rt_utils.hpp"
#include "communication/spi_driver.hpp"
#include "communication/protocol.hpp"

using namespace odroid;

// 全局退出标志
static volatile bool g_running = true;

void signal_handler(int sig) {
    (void)sig;
    g_running = false;
    LOG_INFO("Received signal, stopping...");
}

void print_usage(const char* prog) {
    printf("Usage: %s [options]\n", prog);
    printf("Options:\n");
    printf("  -d <device>  SPI device (default: /dev/spidev0.0)\n");
    printf("  -s <speed>   SPI speed Hz (default: 10000000)\n");
    printf("  -n <count>   Transfer count (default: 1000, 0=infinite)\n");
    printf("  -t <period>  Transfer period us (default: 1000)\n");
    printf("  -v           Verbose output\n");
    printf("  -h           Show this help\n");
}

int main(int argc, char* argv[]) {
    // 默认参数
    SPIConfig config;
    uint64_t transfer_count = 1000;
    uint64_t period_us = 1000;
    bool verbose = false;

    // 解析命令行参数
    int opt;
    while ((opt = getopt(argc, argv, "d:s:n:t:vh")) != -1) {
        switch (opt) {
            case 'd':
                config.device = optarg;
                break;
            case 's':
                config.speed_hz = atoi(optarg);
                break;
            case 'n':
                transfer_count = atoi(optarg);
                break;
            case 't':
                period_us = atoi(optarg);
                break;
            case 'v':
                verbose = true;
                break;
            case 'h':
                print_usage(argv[0]);
                return 0;
            default:
                print_usage(argv[0]);
                return 1;
        }
    }

    // 设置日志级别
    Logger::instance().set_level(verbose ? LogLevel::DEBUG : LogLevel::INFO);

    LOG_INFO("=== SPI Communication Test ===");
    LOG_INFO("Device: %s", config.device.c_str());
    LOG_INFO("Speed: %u Hz", config.speed_hz);
    LOG_INFO("Count: %lu (0=infinite)", transfer_count);
    LOG_INFO("Period: %lu us", period_us);

    // 信号处理
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // 检查权限
    if (!check_spi_permission(config.device.c_str())) {
        LOG_WARN("Try running with sudo or add user to spi group");
    }

    // 打开SPI设备
    SPIDriver spi;
    if (!spi.open(config)) {
        LOG_ERROR("Failed to open SPI device");
        return 1;
    }

    // 统计
    RuntimeStats latency_stats;
    uint64_t success_count = 0;
    uint64_t error_count = 0;

    // 测试循环
    LOG_INFO("Starting SPI communication test...");
    LOG_INFO("Press Ctrl+C to stop");

    PeriodicTimer timer(period_us);
    uint64_t loop = 0;

    while (g_running && (transfer_count == 0 || loop < transfer_count)) {
        Timer loop_timer;

        // 准备测试数据
        RobotCommand cmd;
        cmd.left_leg.hip.position = 0.5f * sin(loop * 0.01);
        cmd.left_leg.hip.velocity = 0.0f;
        cmd.left_leg.hip.kp = 50.0f;
        cmd.left_leg.hip.kd = 1.0f;
        // 其他关节类似...

        // 编码
        SPITxBuffer tx_buf;
        Protocol::encode_robot_cmd(cmd, tx_buf);

        // 传输
        SPIRxBuffer rx_buf;
        Timer transfer_timer;
        bool ok = spi.transfer(tx_buf, rx_buf);
        uint64_t transfer_time = transfer_timer.elapsed_us();

        if (ok) {
            success_count++;
            latency_stats.update(transfer_time);

            // 解码反馈
            RobotFeedback fb;
            Protocol::decode_robot_fb(rx_buf, fb);

            // 详细输出
            if (verbose && (loop % 100 == 0)) {
                LOG_DEBUG("Loop %lu: transfer=%lu us", loop, transfer_time);
                LOG_DEBUG("  TX[0-3]: %04X %04X %04X %04X",
                         tx_buf.data[0], tx_buf.data[1], tx_buf.data[2], tx_buf.data[3]);
                LOG_DEBUG("  RX[0-3]: %04X %04X %04X %04X",
                         rx_buf.data[0], rx_buf.data[1], rx_buf.data[2], rx_buf.data[3]);
                LOG_DEBUG("  Hip: pos=%.3f, vel=%.3f, torque=%.3f",
                         fb.left_leg.hip.position, fb.left_leg.hip.velocity,
                         fb.left_leg.hip.torque);
            }
        } else {
            error_count++;
            LOG_ERROR("SPI transfer failed at loop %lu", loop);
        }

        // 进度显示
        if (transfer_count > 0 && loop % 100 == 0) {
            printf("\rProgress: %lu/%lu (%.1f%%)", 
                   loop, transfer_count, 100.0 * loop / transfer_count);
            fflush(stdout);
        }

        loop++;

        // 等待下一个周期
        if (timer.wait()) {
            // 错过了deadline
            if (verbose) {
                LOG_WARN("Missed deadline at loop %lu", loop);
            }
        }
    }

    printf("\n");

    // 打印统计
    LOG_INFO("=== Test Results ===");
    LOG_INFO("Total loops: %lu", loop);
    LOG_INFO("Success: %lu, Errors: %lu", success_count, error_count);
    LOG_INFO("Success rate: %.2f%%", 100.0 * success_count / loop);
    LOG_INFO("Transfer latency:");
    LOG_INFO("  Mean: %.1f us", latency_stats.mean_us());
    LOG_INFO("  Min:  %lu us", latency_stats.min_us());
    LOG_INFO("  Max:  %lu us", latency_stats.max_us());
    LOG_INFO("  Std:  %.1f us", latency_stats.std_dev_us());

    spi.close();
    LOG_INFO("Test completed");

    return (error_count == 0) ? 0 : 1;
}
