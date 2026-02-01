/**
 * @file test_rt_timing.cpp
 * @brief 实时性测试程序
 * @author Zomnk
 * @date 2026-02-01
 * 
 * 测试RT-PREEMPT内核的实时性能
 * 
 * 编译: mkdir build && cd build && cmake .. && make test_rt_timing
 * 运行: sudo ./test_rt_timing [周期us] [次数]
 */

#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <vector>
#include <algorithm>
#include <numeric>

#include "common/logger.hpp"
#include "common/time_utils.hpp"
#include "realtime/rt_utils.hpp"

using namespace odroid;

static volatile bool g_running = true;

void signal_handler(int sig) {
    (void)sig;
    g_running = false;
}

int main(int argc, char* argv[]) {
    // 参数
    uint64_t period_us = 1000;      // 默认1ms
    uint64_t test_count = 10000;    // 默认10000次
    
    if (argc > 1) period_us = atoi(argv[1]);
    if (argc > 2) test_count = atoi(argv[2]);

    Logger::instance().set_level(LogLevel::INFO);
    
    LOG_INFO("=== RT Timing Test ===");
    LOG_INFO("Period: %lu us", period_us);
    LOG_INFO("Count: %lu", test_count);

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // 打印系统信息
    print_system_info();

    // 检查RT权限
    if (!check_rt_capabilities()) {
        LOG_WARN("No RT capabilities, results may not be optimal");
    }

    // 初始化RT环境
    init_rt_environment();

    // 设置RT优先级
    if (!set_thread_priority(RT_PRIORITY_MAX)) {
        LOG_WARN("Failed to set RT priority, continuing anyway");
    }

    // 设置CPU亲和性
    if (!set_thread_affinity(CPU_CORE_SPI)) {
        LOG_WARN("Failed to set CPU affinity, continuing anyway");
    }

    // 收集延迟数据
    std::vector<int64_t> latencies;
    latencies.reserve(test_count);

    LOG_INFO("Starting timing test...");

    PeriodicTimer timer(period_us);
    uint64_t target_time = get_time_ns() + period_us * 1000;
    uint64_t loop = 0;
    uint64_t missed = 0;

    while (g_running && loop < test_count) {
        // 等待下一个周期
        if (timer.wait()) {
            missed++;
        }

        // 计算延迟
        int64_t now = get_time_ns();
        int64_t latency = (now - target_time) / 1000;  // 转换为us
        latencies.push_back(latency);

        // 更新目标时间
        target_time += period_us * 1000;

        // 进度显示
        if (loop % 1000 == 0) {
            printf("\rProgress: %lu/%lu", loop, test_count);
            fflush(stdout);
        }

        loop++;
    }

    printf("\n");

    if (latencies.empty()) {
        LOG_ERROR("No data collected");
        return 1;
    }

    // 统计分析
    std::sort(latencies.begin(), latencies.end());
    
    int64_t min_lat = latencies.front();
    int64_t max_lat = latencies.back();
    int64_t median = latencies[latencies.size() / 2];
    int64_t p99 = latencies[latencies.size() * 99 / 100];
    int64_t p999 = latencies[latencies.size() * 999 / 1000];
    
    double mean = std::accumulate(latencies.begin(), latencies.end(), 0.0) / latencies.size();
    
    double variance = 0;
    for (int64_t lat : latencies) {
        variance += (lat - mean) * (lat - mean);
    }
    variance /= latencies.size();
    double std_dev = std::sqrt(variance);

    // 统计超出范围的次数
    uint64_t over_10us = 0, over_50us = 0, over_100us = 0, over_1ms = 0;
    for (int64_t lat : latencies) {
        int64_t abs_lat = std::abs(lat);
        if (abs_lat > 10) over_10us++;
        if (abs_lat > 50) over_50us++;
        if (abs_lat > 100) over_100us++;
        if (abs_lat > 1000) over_1ms++;
    }

    // 打印结果
    LOG_INFO("=== Test Results ===");
    LOG_INFO("Total loops: %lu", loop);
    LOG_INFO("Missed deadlines: %lu (%.2f%%)", missed, 100.0 * missed / loop);
    LOG_INFO("");
    LOG_INFO("Latency Statistics (us):");
    LOG_INFO("  Min:    %ld", min_lat);
    LOG_INFO("  Max:    %ld", max_lat);
    LOG_INFO("  Mean:   %.1f", mean);
    LOG_INFO("  Median: %ld", median);
    LOG_INFO("  StdDev: %.1f", std_dev);
    LOG_INFO("  P99:    %ld", p99);
    LOG_INFO("  P99.9:  %ld", p999);
    LOG_INFO("");
    LOG_INFO("Jitter Distribution:");
    LOG_INFO("  >10us:  %lu (%.2f%%)", over_10us, 100.0 * over_10us / loop);
    LOG_INFO("  >50us:  %lu (%.2f%%)", over_50us, 100.0 * over_50us / loop);
    LOG_INFO("  >100us: %lu (%.2f%%)", over_100us, 100.0 * over_100us / loop);
    LOG_INFO("  >1ms:   %lu (%.2f%%)", over_1ms, 100.0 * over_1ms / loop);

    // 评估结果
    LOG_INFO("");
    if (p99 < 50 && max_lat < 500) {
        LOG_INFO("Result: EXCELLENT - Suitable for 1kHz control");
    } else if (p99 < 100 && max_lat < 1000) {
        LOG_INFO("Result: GOOD - Suitable for 500Hz control");
    } else if (p99 < 500) {
        LOG_INFO("Result: FAIR - May need tuning for real-time control");
    } else {
        LOG_WARN("Result: POOR - RT performance needs improvement");
        LOG_WARN("Suggestions:");
        LOG_WARN("  1. Ensure RT-PREEMPT kernel is installed");
        LOG_WARN("  2. Set CPU isolation: isolcpus=2,3");
        LOG_WARN("  3. Disable CPU frequency scaling");
        LOG_WARN("  4. Check /etc/security/limits.conf for rtprio");
    }

    return 0;
}
