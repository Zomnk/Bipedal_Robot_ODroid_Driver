/**
 * @file test_rt_timing.cpp
 * @brief 实时定时测试
 */

#include <iostream>
#include <thread>
#include <chrono>
#include "common/logger.hpp"
#include "common/constants.hpp"

using namespace odroid;

int main() {
    Logger::instance().set_level(LogLevel::DEBUG);
    LOG_INFO("实时定时测试开始");

    constexpr int NUM_ITERATIONS = 100;
    uint64_t times[NUM_ITERATIONS];
    
    for (int i = 0; i < NUM_ITERATIONS; ++i) {
        times[i] = get_time_us();
        std::this_thread::sleep_for(std::chrono::microseconds(CONTROL_PERIOD_US));
    }

    // 统计周期误差
    int64_t max_jitter = 0;
    int64_t sum_jitter = 0;
    
    for (int i = 1; i < NUM_ITERATIONS; ++i) {
        int64_t period = static_cast<int64_t>(times[i] - times[i-1]);
        int64_t jitter = std::abs(period - static_cast<int64_t>(CONTROL_PERIOD_US));
        if (jitter > max_jitter) max_jitter = jitter;
        sum_jitter += jitter;
    }

    double avg_jitter = static_cast<double>(sum_jitter) / (NUM_ITERATIONS - 1);
    LOG_INFO("测试完成: 平均抖动=%.1fus, 最大抖动=%ldus", avg_jitter, (long)max_jitter);
    LOG_INFO("实时定时测试完成");
    return 0;
}
