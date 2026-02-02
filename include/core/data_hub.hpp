/**
 * @file data_hub.hpp
 * @brief 数据中心 - 管理机器人状态和指令
 * @author Zomnk
 * @date 2026-02-01
 */

#ifndef ODROID_CORE_DATA_HUB_HPP
#define ODROID_CORE_DATA_HUB_HPP

#include <mutex>
#include <atomic>

#include "common/types.hpp"
#include "common/constants.hpp"
#include "realtime/lock_free_buffer.hpp"

namespace odroid {

/**
 * @brief 数据中心类 - 线程安全的数据存储和交换
 */
class DataHub {
public:
    static DataHub& instance() {
        static DataHub hub;
        return hub;
    }
    
    //==========================================================================
    // 控制指令接口 (上层算法 -> SPI发送)
    //==========================================================================
    
    /**
     * @brief 设置机器人控制指令
     * @param cmd 控制指令
     */
    void set_command(const RobotCommand& cmd) {
        command_buffer_.write(cmd);
    }
    
    /**
     * @brief 获取最新的控制指令
     * @param cmd 输出参数
     * @return 有有效指令返回true
     */
    bool get_command(RobotCommand& cmd) {
        return command_buffer_.read(cmd);
    }
    
    //==========================================================================
    // 反馈数据接口 (SPI接收 -> 上层算法)
    //==========================================================================
    
    /**
     * @brief 更新机器人反馈数据
     * @param feedback 反馈数据
     */
    void update_feedback(const RobotFeedback& feedback) {
        feedback_buffer_.write(feedback);
    }
    
    /**
     * @brief 获取最新的反馈数据
     * @param feedback 输出参数
     * @return 有有效数据返回true
     */
    bool get_feedback(RobotFeedback& feedback) {
        return feedback_buffer_.read(feedback);
    }
    
    //==========================================================================
    // SPI缓冲区接口 (协议层使用)
    //==========================================================================
    
    /**
     * @brief 获取SPI发送缓冲区引用
     */
    SPITxBuffer& get_tx_buffer() { return tx_buffer_; }
    const SPITxBuffer& get_tx_buffer() const { return tx_buffer_; }
    
    /**
     * @brief 获取SPI接收缓冲区引用
     */
    SPIRxBuffer& get_rx_buffer() { return rx_buffer_; }
    const SPIRxBuffer& get_rx_buffer() const { return rx_buffer_; }
    
    //==========================================================================
    // 状态信息
    //==========================================================================
    
    /**
     * @brief 增加SPI传输计数
     */
    void increment_transfer_count() { transfer_count_++; }
    
    /**
     * @brief 获取传输计数
     */
    uint64_t get_transfer_count() const { return transfer_count_; }
    
    /**
     * @brief 检查通信是否正常
     */
    bool is_communication_ok() const { return communication_ok_; }
    void set_communication_ok(bool ok) { communication_ok_ = ok; }
    
private:
    DataHub() 
        : transfer_count_(0)
        , communication_ok_(false) {}
    
    // 使用最新值缓冲区进行无锁通信
    LatestValueBuffer<RobotCommand> command_buffer_;
    LatestValueBuffer<RobotFeedback> feedback_buffer_;
    
    // SPI原始缓冲区
    SPITxBuffer tx_buffer_;
    SPIRxBuffer rx_buffer_;
    
    // 状态信息
    std::atomic<uint64_t> transfer_count_;
    std::atomic<bool> communication_ok_;
};

} // namespace odroid

#endif // ODROID_CORE_DATA_HUB_HPP