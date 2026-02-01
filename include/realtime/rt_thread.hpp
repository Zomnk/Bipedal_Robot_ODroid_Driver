/**
 * @file rt_thread.hpp
 * @brief 实时线程封装类
 * @author Zomnk
 * @date 2026-02-01
 */

#ifndef ODROID_REALTIME_RT_THREAD_HPP
#define ODROID_REALTIME_RT_THREAD_HPP

#include <atomic>
#include <functional>
#include <thread>
#include <string>

#include "common/logger.hpp"
#include "common/time_utils.hpp"
#include "realtime/rt_utils.hpp"

namespace odroid {

/**
 * @brief 实时线程封装类
 */
class RTThread {
public:
    using TaskFunc = std::function<void()>;

    /**
     * @brief 构造函数
     * @param name 线程名称
     * @param period_us 周期 (微秒), 0表示非周期性线程
     * @param priority SCHED_FIFO优先级 (1-99)
     * @param cpu_core CPU亲和性 (-1表示不设置)
     */
    RTThread(const std::string& name, uint64_t period_us, int priority, int cpu_core = -1)
        : name_(name)
        , period_us_(period_us)
        , priority_(priority)
        , cpu_core_(cpu_core)
        , running_(false)
        , loop_count_(0)
        , missed_deadlines_(0) {}

    ~RTThread() {
        stop();
    }

    // 禁止拷贝
    RTThread(const RTThread&) = delete;
    RTThread& operator=(const RTThread&) = delete;

    /**
     * @brief 设置循环任务
     */
    void set_loop_task(TaskFunc task) {
        loop_task_ = std::move(task);
    }

    /**
     * @brief 设置初始化任务 (在RT配置后执行一次)
     */
    void set_init_task(TaskFunc task) {
        init_task_ = std::move(task);
    }

    /**
     * @brief 设置清理任务 (在退出前执行一次)
     */
    void set_cleanup_task(TaskFunc task) {
        cleanup_task_ = std::move(task);
    }

    /**
     * @brief 启动线程
     * @return 是否成功
     */
    bool start() {
        if (running_.load()) {
            LOG_WARN("Thread '%s' already running", name_.c_str());
            return false;
        }

        if (!loop_task_) {
            LOG_ERROR("Thread '%s' has no loop task", name_.c_str());
            return false;
        }

        running_.store(true);
        thread_ = std::thread(&RTThread::thread_func, this);

        LOG_INFO("Thread '%s' started (period=%lu us, priority=%d, cpu=%d)",
                 name_.c_str(), (unsigned long)period_us_, priority_, cpu_core_);
        return true;
    }

    /**
     * @brief 停止线程
     */
    void stop() {
        if (!running_.load()) return;

        running_.store(false);
        if (thread_.joinable()) {
            thread_.join();
        }

        LOG_INFO("Thread '%s' stopped (loops=%lu, missed=%lu)",
                 name_.c_str(), 
                 (unsigned long)loop_count_.load(), 
                 (unsigned long)missed_deadlines_.load());
    }

    /**
     * @brief 检查是否运行中
     */
    bool is_running() const { return running_.load(); }

    /**
     * @brief 获取循环计数
     */
    uint64_t get_loop_count() const { return loop_count_.load(); }

    /**
     * @brief 获取错过deadline的次数
     */
    uint64_t get_missed_deadlines() const { return missed_deadlines_.load(); }

    /**
     * @brief 获取运行时统计
     */
    const RuntimeStats& get_stats() const { return stats_; }

    /**
     * @brief 重置统计数据
     */
    void reset_stats() {
        loop_count_.store(0);
        missed_deadlines_.store(0);
        stats_.reset();
    }

private:
    void thread_func() {
        // 设置RT属性
        if (priority_ > 0) {
            set_thread_priority(priority_);
        }
        if (cpu_core_ >= 0) {
            set_thread_affinity(cpu_core_);
        }
        prefault_stack();

        // 执行初始化任务
        if (init_task_) {
            init_task_();
        }

        // 周期性循环
        if (period_us_ > 0) {
            PeriodicTimer timer(period_us_);

            while (running_.load()) {
                Timer loop_timer;

                // 执行任务
                loop_task_();

                // 统计执行时间
                uint64_t exec_time = loop_timer.elapsed_us();
                stats_.update(exec_time);
                loop_count_++;

                // 等待下一个周期
                if (timer.wait()) {
                    missed_deadlines_++;
                }
            }
        } else {
            // 非周期性线程，单次执行
            loop_task_();
        }

        // 执行清理任务
        if (cleanup_task_) {
            cleanup_task_();
        }
    }

    std::string name_;
    uint64_t period_us_;
    int priority_;
    int cpu_core_;

    std::atomic<bool> running_;
    std::thread thread_;

    TaskFunc loop_task_;
    TaskFunc init_task_;
    TaskFunc cleanup_task_;

    std::atomic<uint64_t> loop_count_;
    std::atomic<uint64_t> missed_deadlines_;
    RuntimeStats stats_;
};

} // namespace odroid

#endif // ODROID_REALTIME_RT_THREAD_HPP
