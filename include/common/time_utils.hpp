/**
 * @file time_utils.hpp
 * @brief 时间工具函数
 * @author Zomnk
 * @date 2026-02-01
 */

#ifndef ODROID_COMMON_TIME_UTILS_HPP
#define ODROID_COMMON_TIME_UTILS_HPP

#include <cstdint>
#include <thread>
#include <chrono>
#include "constants.hpp"

namespace odroid {

// get_time_us() 和 get_time_ns() 已在 constants.hpp 中定义

/**
 * @brief 高精度计时器
 */
class Timer {
public:
    Timer() { reset(); }

    void reset() { start_time_ = get_time_ns(); }

    uint64_t elapsed_ns() const { return get_time_ns() - start_time_; }
    uint64_t elapsed_us() const { return elapsed_ns() / 1000; }
    uint64_t elapsed_ms() const { return elapsed_ns() / 1000000; }
    double elapsed_sec() const { return static_cast<double>(elapsed_ns()) / 1e9; }

private:
    uint64_t start_time_;
};

/**
 * @brief 周期定时器
 */
class PeriodicTimer {
public:
    explicit PeriodicTimer(uint64_t period_us)
        : period_us_(period_us), next_time_(get_time_us() + period_us) {}

    bool wait() {
        uint64_t now = get_time_us();
        if (now >= next_time_) {
            next_time_ += period_us_;
            return true;  // 超时
        }

        while (get_time_us() < next_time_) {
            std::this_thread::sleep_for(std::chrono::microseconds(10));
        }
        next_time_ += period_us_;
        return false;
    }

    void reset() { next_time_ = get_time_us() + period_us_; }

private:
    uint64_t period_us_;
    uint64_t next_time_;
};

inline void sleep_us(uint64_t us) {
    std::this_thread::sleep_for(std::chrono::microseconds(us));
}

inline void sleep_ms(uint64_t ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

} // namespace odroid

#endif // ODROID_COMMON_TIME_UTILS_HPP