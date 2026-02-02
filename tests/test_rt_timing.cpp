/**
 * @file test_rt_timing.cpp
 * @brief 实时定时测试
 * @author Zomnk
 * @date 2026-02-01
 */

#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include "common/logger.hpp"
#include "common/constants.hpp"
#include "common/time_utils.hpp"

using namespace odroid;

int main() {
    Logger::instance().set_level(LogLevel::DEBUG);
    
    LOG_INFO("=== 实时定时测试开始 ===");
    LOG_INFO("控制周期: %u us (%u Hz)", CONTROL_PERIOD_US, CONTROL_FREQ_HZ);

    constexpr int NUM_ITERATIONS = 1000;
    uint64_t times[NUM_ITERATIONS];

    // 使用std::this_thread::sleep_for测试
    LOG_INFO("--- 测试 std::this_thread::sleep_for ---");
    for (int i = 0; i < NUM_ITERATIONS; ++i) {
        times[i] = get_time_us();
        std::this_thread::sleep_for(std::chrono::microseconds(CONTROL_PERIOD_US));
    }

    // 统计周期误差
    int64_t max_jitter = 0;
    int64_t min_jitter = INT64_MAX;
    int64_t sum_jitter = 0;

    for (int i = 1; i < NUM_ITERATIONS; ++i) {
        int64_t period = static_cast<int64_t>(times[i] - times[i-1]);
        int64_t jitter = std::abs(period - static_cast<int64_t>(CONTROL_PERIOD_US));
        if (jitter > max_jitter) max_jitter = jitter;
        if (jitter < min_jitter) min_jitter = jitter;
        sum_jitter += jitter;
    }

    double avg_jitter = static_cast<double>(sum_jitter) / (NUM_ITERATIONS - 1);
    LOG_INFO("sleep_for结果: 平均抖动=%.1fus, 最小=%ldus, 最大=%ldus",
             avg_jitter, (long)min_jitter, (long)max_jitter);

    // 使用PeriodicTimer测试
    LOG_INFO("--- 测试 PeriodicTimer ---");
    PeriodicTimer timer(CONTROL_PERIOD_US);
    int missed = 0;

    for (int i = 0; i < NUM_ITERATIONS; ++i) {
        times[i] = get_time_us();
        if (timer.wait()) {
            missed++;
        }
    }

    // 重新统计
    max_jitter = 0;
    min_jitter = INT64_MAX;
    sum_jitter = 0;

    for (int i = 1; i < NUM_ITERATIONS; ++i) {
        int64_t period = static_cast<int64_t>(times[i] - times[i-1]);
        int64_t jitter = std::abs(period - static_cast<int64_t>(CONTROL_PERIOD_US));
        if (jitter > max_jitter) max_jitter = jitter;
        if (jitter < min_jitter) min_jitter = jitter;
        sum_jitter += jitter;
    }

    avg_jitter = static_cast<double>(sum_jitter) / (NUM_ITERATIONS - 1);
    LOG_INFO("PeriodicTimer结果: 平均抖动=%.1fus, 最小=%ldus, 最大=%ldus, 错过=%d",
             avg_jitter, (long)min_jitter, (long)max_jitter, missed);

    // 测试Timer类
    LOG_INFO("--- 测试 Timer 类 ---");
    Timer t;
    sleep_ms(100);
    LOG_INFO("100ms后: elapsed_ms=%lu, elapsed_us=%lu",
             (unsigned long)t.elapsed_ms(), (unsigned long)t.elapsed_us());

    LOG_INFO("=== 实时定时测试完成 ===");
    return 0;
}
