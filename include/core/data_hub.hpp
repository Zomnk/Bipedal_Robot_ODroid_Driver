/**
 * @file data_hub.hpp
 * @brief 数据交换中心 (线程间通信)
 * @author Zomnk
 * @date 2026-02-01
 */

#ifndef ODROID_CORE_DATA_HUB_HPP
#define ODROID_CORE_DATA_HUB_HPP

#include <atomic>
#include <mutex>

#include "common/types.hpp"
#include "common/time_utils.hpp"
#include "realtime/lock_free_buffer.hpp"

namespace odroid {

/**
 * @brief 数据交换中心
 *
 * 用于不同线程/模块间的数据交换:
 * - 控制指令: 上层 -> SPI线程 -> STM32
 * - 反馈数据: STM32 -> SPI线程 -> 上层
 */
class DataHub {
public:
    static DataHub& instance() {
        static DataHub hub;
        return hub;
    }

    //--------------------------------------------------------------------------
    // 控制指令 (写入端: 上层/测试程序, 读取端: SPI线程)
    //--------------------------------------------------------------------------

    /**
     * @brief 设置控制指令
     */
    void set_command(const RobotCommand& cmd) {
        command_buffer_.write(cmd);
        command_updated_.store(true, std::memory_order_release);
        last_command_time_us_ = get_time_us();
    }

    /**
     * @brief 获取控制指令
     * @return 是否有新指令
     */
    bool get_command(RobotCommand& cmd) {
        bool has_new = command_buffer_.read(cmd);
        if (has_new) {
            command_updated_.store(false, std::memory_order_release);
        }
        return has_new;
    }

    /**
     * @brief 检查是否有新控制指令
     */
    bool has_new_command() const {
        return command_updated_.load(std::memory_order_acquire);
    }

    /**
     * @brief 获取上次指令时间
     */
    uint64_t get_last_command_time_us() const {
        return last_command_time_us_;
    }

    //--------------------------------------------------------------------------
    // 反馈数据 (写入端: SPI线程, 读取端: 上层/测试程序)
    //--------------------------------------------------------------------------

    /**
     * @brief 设置反馈数据
     */
    void set_feedback(const RobotFeedback& fb) {
        feedback_buffer_.write(fb);
        feedback_updated_.store(true, std::memory_order_release);
        feedback_count_++;
    }

    /**
     * @brief 获取反馈数据
     * @return 是否有新反馈
     */
    bool get_feedback(RobotFeedback& fb) {
        bool has_new = feedback_buffer_.read(fb);
        if (has_new) {
            feedback_updated_.store(false, std::memory_order_release);
        }
        return has_new;
    }

    /**
     * @brief 检查是否有新反馈
     */
    bool has_new_feedback() const {
        return feedback_updated_.load(std::memory_order_acquire);
    }

    /**
     * @brief 获取反馈计数
     */
    uint64_t get_feedback_count() const {
        return feedback_count_.load();
    }

    //--------------------------------------------------------------------------
    // 系统状态
    //--------------------------------------------------------------------------

    /**
     * @brief 设置系统运行状态
     */
    void set_running(bool running) {
        running_.store(running, std::memory_order_release);
    }

    /**
     * @brief 检查系统是否运行
     */
    bool is_running() const {
        return running_.load(std::memory_order_acquire);
    }

    /**
     * @brief 设置紧急停止
     */
    void set_emergency_stop(bool stop) {
        emergency_stop_.store(stop, std::memory_order_release);
    }

    /**
     * @brief 检查是否紧急停止
     */
    bool is_emergency_stop() const {
        return emergency_stop_.load(std::memory_order_acquire);
    }

    //--------------------------------------------------------------------------
    // 统计重置
    //--------------------------------------------------------------------------

    void reset_stats() {
        feedback_count_.store(0);
    }

private:
    DataHub()
        : running_(false)
        , emergency_stop_(false)
        , command_updated_(false)
        , feedback_updated_(false)
        , last_command_time_us_(0)
        , feedback_count_(0) {}

    // 控制指令缓冲区
    TripleBuffer<RobotCommand> command_buffer_;
    std::atomic<bool> command_updated_;
    uint64_t last_command_time_us_;

    // 反馈数据缓冲区
    TripleBuffer<RobotFeedback> feedback_buffer_;
    std::atomic<bool> feedback_updated_;
    std::atomic<uint64_t> feedback_count_;

    // 系统状态
    std::atomic<bool> running_;
    std::atomic<bool> emergency_stop_;
};

} // namespace odroid

#endif // ODROID_CORE_DATA_HUB_HPP
