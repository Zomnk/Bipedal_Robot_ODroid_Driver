/**
 * @file robot_interface.hpp
 * @brief 机器人接口类 - 整合所有功能的顶层接口
 * @author Zomnk
 * @date 2026-02-01
 */

#ifndef ODROID_CORE_ROBOT_INTERFACE_HPP
#define ODROID_CORE_ROBOT_INTERFACE_HPP

#include "common/types.hpp"
#include "common/constants.hpp"
#include "common/logger.hpp"
#include "communication/spi_driver.hpp"
#include "communication/protocol.hpp"
#include "realtime/rt_thread.hpp"
#include "core/data_hub.hpp"

namespace odroid {

/**
 * @brief 机器人接口类 - 提供高层API
 */
class RobotInterface {
public:
    RobotInterface() = default;
    ~RobotInterface() { stop(); }
    
    // 禁止拷贝
    RobotInterface(const RobotInterface&) = delete;
    RobotInterface& operator=(const RobotInterface&) = delete;
    
    /**
     * @brief 初始化机器人接口
     * @param config SPI配置
     * @return 成功返回true
     */
    bool init(const SPIConfig& config = SPIConfig{}) {
        LOG_INFO("初始化机器人接口...");
        
        // 初始化SPI驱动
        if (!spi_driver_.init(config)) {
            LOG_ERROR("SPI驱动初始化失败");
            return false;
        }
        
        initialized_ = true;
        LOG_INFO("机器人接口初始化成功");
        return true;
    }
    
    /**
     * @brief 启动通信线程
     * @param period_us 通信周期 (微秒)
     * @return 成功返回true
     */
    bool start(uint64_t period_us = CONTROL_PERIOD_US) {
        if (!initialized_) {
            LOG_ERROR("接口未初始化");
            return false;
        }
        
        if (running_) {
            LOG_WARN("通信线程已在运行");
            return false;
        }
        
        // 启动SPI通信线程
        running_ = spi_thread_.start(
            [this]() { this->spi_loop(); },
            period_us,
            RT_PRIORITY_SPI,
            CPU_CORE_SPI
        );
        
        if (running_) {
            LOG_INFO("通信线程已启动, 周期=%lu us", period_us);
        }
        
        return running_;
    }
    
    /**
     * @brief 停止通信线程
     */
    void stop() {
        if (running_) {
            spi_thread_.stop();
            running_ = false;
            LOG_INFO("通信线程已停止");
        }
        spi_driver_.close();
    }
    
    //==========================================================================
    // 高层控制接口
    //==========================================================================
    
    /**
     * @brief 发送机器人控制指令
     * @param cmd 控制指令
     */
    void send_command(const RobotCommand& cmd) {
        DataHub::instance().set_command(cmd);
    }
    
    /**
     * @brief 获取机器人反馈数据
     * @param feedback 反馈数据
     * @return 有有效数据返回true
     */
    bool get_feedback(RobotFeedback& feedback) {
        return DataHub::instance().get_feedback(feedback);
    }
    
    /**
     * @brief 发送单腿控制指令
     * @param leg 腿编号 (0=左腿, 1=右腿)
     * @param cmd 腿控制指令
     */
    void send_leg_command(int leg, const LegCommand& cmd) {
        RobotCommand robot_cmd;
        if (DataHub::instance().get_command(robot_cmd)) {
            // 基于当前指令更新
        }
        
        if (leg == 0) {
            robot_cmd.left_leg = cmd;
        } else {
            robot_cmd.right_leg = cmd;
        }
        robot_cmd.timestamp_us = get_time_us();
        
        DataHub::instance().set_command(robot_cmd);
    }
    
    //==========================================================================
    // 状态查询接口
    //==========================================================================
    
    /**
     * @brief 检查是否已初始化
     */
    bool is_initialized() const { return initialized_; }
    
    /**
     * @brief 检查通信线程是否在运行
     */
    bool is_running() const { return running_; }
    
    /**
     * @brief 获取通信统计信息
     */
    uint64_t get_transfer_count() const {
        return DataHub::instance().get_transfer_count();
    }
    
    uint64_t get_cycle_count() const {
        return spi_thread_.get_cycle_count();
    }
    
    double get_avg_latency_us() const {
        return spi_thread_.get_avg_latency_us();
    }
    
    uint64_t get_max_latency_us() const {
        return spi_thread_.get_max_latency_us();
    }
    
    /**
     * @brief 打印通信统计信息
     */
    void print_stats() {
        LOG_INFO("=== Communication Stats ===");
        LOG_INFO("Transfer count: %lu", get_transfer_count());
        LOG_INFO("Cycle count: %lu", get_cycle_count());
        LOG_INFO("Avg latency: %.2f us", get_avg_latency_us());
        LOG_INFO("Max latency: %lu us", get_max_latency_us());
    }
    
    /**
     * @brief 打印原始SPI数据 (用于调试)
     */
    void print_raw_data() {
        auto& hub = DataHub::instance();
        const auto& rx = hub.get_rx_buffer();
        LOG_INFO("=== Raw RX Data (test mode expected: 0x0000+, 0x1001, 0x1002...) ===");
        LOG_INFO("  [0-3]: 0x%04X 0x%04X 0x%04X 0x%04X", 
                 rx.data[0], rx.data[1], rx.data[2], rx.data[3]);
        LOG_INFO("  [4-7]: 0x%04X 0x%04X 0x%04X 0x%04X", 
                 rx.data[4], rx.data[5], rx.data[6], rx.data[7]);
        LOG_INFO("  [56-59] (last 4): 0x%04X 0x%04X 0x%04X 0x%04X",
                 rx.data[56], rx.data[57], rx.data[58], rx.data[59]);
    }
    
    /**
     * @brief 打印原始字节数据 (用于深度调试)
     */
    void print_raw_bytes() {
        auto& hub = DataHub::instance();
        const auto& rx = hub.get_rx_buffer();
        // 把前8个words作为字节打印
        const uint8_t* bytes = reinterpret_cast<const uint8_t*>(rx.data);
        LOG_INFO("=== Raw RX Bytes (first 16 bytes) ===");
        LOG_INFO("  [0-7]:  %02X %02X %02X %02X %02X %02X %02X %02X",
                 bytes[0], bytes[1], bytes[2], bytes[3], 
                 bytes[4], bytes[5], bytes[6], bytes[7]);
        LOG_INFO("  [8-15]: %02X %02X %02X %02X %02X %02X %02X %02X",
                 bytes[8], bytes[9], bytes[10], bytes[11], 
                 bytes[12], bytes[13], bytes[14], bytes[15]);
    }
    
private:
    /**
     * @brief SPI通信循环 (在实时线程中执行)
     */
    void spi_loop() {
        auto& hub = DataHub::instance();
        
        // 获取最新控制指令并编码
        RobotCommand cmd;
        if (hub.get_command(cmd)) {
            Protocol::encode_robot_cmd(cmd, hub.get_tx_buffer());
        }
        
        // 执行SPI传输
        if (spi_driver_.transfer(hub.get_tx_buffer(), hub.get_rx_buffer())) {
            // 解码反馈数据
            RobotFeedback feedback;
            Protocol::decode_robot_feedback(hub.get_rx_buffer(), feedback);
            hub.update_feedback(feedback);
            
            hub.increment_transfer_count();
            hub.set_communication_ok(true);
        } else {
            hub.set_communication_ok(false);
        }
    }
    
    SPIDriver spi_driver_;
    RTThread spi_thread_;
    bool initialized_ = false;
    bool running_ = false;
};

} // namespace odroid

#endif // ODROID_CORE_ROBOT_INTERFACE_HPP