/**
 * @file rt_thread.hpp
 * @brief 实时线程封装类
 * @author Zomnk
 * @date 2026-02-01
 */

#ifndef ODROID_REALTIME_RT_THREAD_HPP
#define ODROID_REALTIME_RT_THREAD_HPP

#include <thread>
#include <atomic>
#include <functional>

#include "rt_utils.hpp"
#include "common/time_utils.hpp"

namespace odroid {

/**
 * @brief 实时线程类 - 周期性执行任务
 */
class RTThread {
public:
    using TaskFunction = std::function<void()>;
    
    RTThread() = default;
    ~RTThread() { stop(); }
    
    // 禁止拷贝
    RTThread(const RTThread&) = delete;
    RTThread& operator=(const RTThread&) = delete;
    
    /**
     * @brief 启动实时线程
     * @param task 周期性执行的任务函数
     * @param period_us 周期 (微秒)
     * @param priority 实时优先级 (1-99)
     * @param cpu_id 绑定的CPU核心
     * @return 成功返回true
     */
    bool start(TaskFunction task, uint64_t period_us, int priority, int cpu_id) {
        if (running_) {
            LOG_WARN("线程已在运行");
            return false;
        }
        
        task_ = std::move(task);
        period_us_ = period_us;
        priority_ = priority;
        cpu_id_ = cpu_id;
        running_ = true;
        
        thread_ = std::thread(&RTThread::thread_func, this);
        return true;
    }
    
    /**
     * @brief 停止线程
     */
    void stop() {
        if (running_) {
            running_ = false;
            if (thread_.joinable()) {
                thread_.join();
            }
            LOG_INFO("实时线程已停止");
        }
    }
    
    /**
     * @brief 检查线程是否在运行
     */
    bool is_running() const { return running_; }
    
    /**
     * @brief 获取实际周期统计
     */
    uint64_t get_cycle_count() const { return cycle_count_; }
    uint64_t get_max_latency_us() const { return max_latency_us_; }
    double get_avg_latency_us() const { 
        return cycle_count_ > 0 ? static_cast<double>(total_latency_us_) / cycle_count_ : 0; 
    }
    
private:
    void thread_func() {
        // 配置实时环境
        setup_realtime_thread(priority_, cpu_id_);
        
        PeriodicTimer timer(period_us_);
        uint64_t last_time = get_time_us();
        
        LOG_INFO("实时线程启动: 周期=%lu us, 优先级=%d, CPU=%d", 
                 period_us_, priority_, cpu_id_);
        
        while (running_) {
            uint64_t now = get_time_us();
            uint64_t latency = (now > last_time) ? (now - last_time - period_us_) : 0;
            
            // 更新统计
            cycle_count_++;
            total_latency_us_ += latency;
            if (latency > max_latency_us_) {
                max_latency_us_ = latency;
            }
            
            last_time = now;
            
            // 执行任务前再次检查 running_ 状态
            if (running_ && task_) {
                task_();
            }
            
            // 等待下一个周期前检查是否需要退出
            if (!running_) {
                break;
            }
            timer.wait();
        }
    }
    
    std::thread thread_;
    TaskFunction task_;
    uint64_t period_us_ = 1000;
    int priority_ = 80;
    int cpu_id_ = 0;
    std::atomic<bool> running_{false};
    
    // 统计信息
    std::atomic<uint64_t> cycle_count_{0};
    std::atomic<uint64_t> max_latency_us_{0};
    std::atomic<uint64_t> total_latency_us_{0};
};

} // namespace odroid

#endif // ODROID_REALTIME_RT_THREAD_HPP