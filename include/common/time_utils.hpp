/**
 * @file time_utils.hpp
 * @brief 高精度时间工具
 * @author Zomnk
 * @date 2026-02-01
 */

#ifndef ODROID_COMMON_TIME_UTILS_HPP
#define ODROID_COMMON_TIME_UTILS_HPP

#include <cstdint>
#include <ctime>
#include <chrono>

namespace odroid {

/**
 * @brief 获取当前时间 (纳秒)
 */
inline uint64_t get_time_ns() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + ts.tv_nsec;
}

/**
 * @brief 获取当前时间 (微秒)
 */
inline uint64_t get_time_us() {
    return get_time_ns() / 1000;
}

/**
 * @brief 获取当前时间 (毫秒)
 */
inline uint64_t get_time_ms() {
    return get_time_ns() / 1000000;
}

/**
 * @brief 获取当前时间 (秒, 浮点数)
 */
inline double get_time_sec() {
    return static_cast<double>(get_time_ns()) / 1e9;
}

/**
 * @brief 高精度休眠 (纳秒)
 */
inline void sleep_ns(uint64_t ns) {
    struct timespec ts;
    ts.tv_sec = ns / 1000000000ULL;
    ts.tv_nsec = ns % 1000000000ULL;
    clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, nullptr);
}

/**
 * @brief 高精度休眠 (微秒)
 */
inline void sleep_us(uint64_t us) {
    sleep_ns(us * 1000);
}

/**
 * @brief 高精度休眠 (毫秒)
 */
inline void sleep_ms(uint64_t ms) {
    sleep_ns(ms * 1000000);
}

/**
 * @brief 休眠直到指定时刻 (纳秒)
 */
inline void sleep_until_ns(uint64_t target_ns) {
    struct timespec ts;
    ts.tv_sec = target_ns / 1000000000ULL;
    ts.tv_nsec = target_ns % 1000000000ULL;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
}

/**
 * @brief 简单计时器类
 */
class Timer {
public:
    Timer() : start_time_(get_time_ns()) {}

    void reset() { start_time_ = get_time_ns(); }
    
    uint64_t elapsed_ns() const { return get_time_ns() - start_time_; }
    uint64_t elapsed_us() const { return elapsed_ns() / 1000; }
    uint64_t elapsed_ms() const { return elapsed_ns() / 1000000; }
    double elapsed_sec() const { return static_cast<double>(elapsed_ns()) / 1e9; }

private:
    uint64_t start_time_;
};

/**
 * @brief 周期计时器 (用于RT循环)
 */
class PeriodicTimer {
public:
    explicit PeriodicTimer(uint64_t period_us) 
        : period_ns_(period_us * 1000), next_time_(get_time_ns() + period_ns_) {}

    /**
     * @brief 等待直到下一个周期
     * @return 是否发生了超时(错过了deadline)
     */
    bool wait() {
        uint64_t now = get_time_ns();
        if (now >= next_time_) {
            // 错过了deadline
            next_time_ = now + period_ns_;
            return true;
        }
        sleep_until_ns(next_time_);
        next_time_ += period_ns_;
        return false;
    }

    /**
     * @brief 重置定时器
     */
    void reset() {
        next_time_ = get_time_ns() + period_ns_;
    }

    /**
     * @brief 获取上次循环的实际周期
     */
    uint64_t get_actual_period_us() const {
        static uint64_t last_time = 0;
        uint64_t now = get_time_ns();
        uint64_t period = (last_time == 0) ? period_ns_ : (now - last_time);
        last_time = now;
        return period / 1000;
    }

private:
    uint64_t period_ns_;
    uint64_t next_time_;
};

/**
 * @brief 运行时统计
 */
class RuntimeStats {
public:
    RuntimeStats() : count_(0), sum_(0), sum_sq_(0), min_(UINT64_MAX), max_(0) {}

    void update(uint64_t value_us) {
        count_++;
        sum_ += value_us;
        sum_sq_ += value_us * value_us;
        if (value_us < min_) min_ = value_us;
        if (value_us > max_) max_ = value_us;
    }

    void reset() {
        count_ = 0;
        sum_ = 0;
        sum_sq_ = 0;
        min_ = UINT64_MAX;
        max_ = 0;
    }

    uint64_t count() const { return count_; }
    double mean_us() const { return count_ > 0 ? static_cast<double>(sum_) / count_ : 0; }
    uint64_t min_us() const { return min_ == UINT64_MAX ? 0 : min_; }
    uint64_t max_us() const { return max_; }
    
    double std_dev_us() const {
        if (count_ < 2) return 0;
        double mean = mean_us();
        double variance = static_cast<double>(sum_sq_) / count_ - mean * mean;
        return variance > 0 ? std::sqrt(variance) : 0;
    }

private:
    uint64_t count_;
    uint64_t sum_;
    uint64_t sum_sq_;
    uint64_t min_;
    uint64_t max_;
};

} // namespace odroid

#endif // ODROID_COMMON_TIME_UTILS_HPP
