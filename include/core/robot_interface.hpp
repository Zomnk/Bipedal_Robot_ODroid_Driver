/**
 * @file robot_interface.hpp
 * @brief 机器人接口主类
 * @author Zomnk
 * @date 2026-02-01
 */

#ifndef ODROID_CORE_ROBOT_INTERFACE_HPP
#define ODROID_CORE_ROBOT_INTERFACE_HPP

#include <memory>
#include <atomic>
#include <functional>

#include "common/types.hpp"
#include "common/constants.hpp"
#include "common/logger.hpp"
#include "common/time_utils.hpp"
#include "realtime/rt_thread.hpp"
#include "realtime/rt_utils.hpp"
#include "communication/spi_driver.hpp"
#include "communication/protocol.hpp"
#include "core/data_hub.hpp"

namespace odroid {

/**
 * @brief SPI通信线程任务
 */
class SPICommTask {
public:
    SPICommTask() : initialized_(false) {}

    /**
     * @brief 初始化
     */
    bool init(const SPIConfig& config = SPIConfig()) {
        if (!spi_.open(config)) {
            return false;
        }
        initialized_ = true;
        return true;
    }

    /**
     * @brief 执行一次SPI通信
     */
    void execute() {
        if (!initialized_) return;

        // 获取控制指令
        RobotCommand cmd = DataHub::instance().get_command();

        // 编码发送数据
        SPITxBuffer tx_buf;
        Protocol::encode_command(cmd, tx_buf);

        // SPI传输
        SPIRxBuffer rx_buf;
        spi_.transfer(tx_buf, rx_buf);

        // 解码反馈数据
        RobotFeedback fb;
        Protocol::decode_feedback(rx_buf, fb);
        fb.timestamp_us = get_time_us();

        // 更新数据中心
        DataHub::instance().update_feedback(fb);
    }

    /**
     * @brief 关闭
     */
    void close() {
        spi_.close();
        initialized_ = false;
    }

private:
    SPIDriver spi_;
    bool initialized_;
};

/**
 * @brief 机器人接口主类
 *
 * 负责:
 * - 初始化SPI通信
 * - 启动实时线程
 * - 提供控制接口
 */
class RobotInterface {
public:
    using FeedbackCallback = std::function<void(const RobotFeedback&)>;

    RobotInterface() : running_(false), callback_(nullptr) {}

    ~RobotInterface() {
        stop();
    }

    /**
     * @brief 初始化机器人接口
     */
    bool init(const SPIConfig& spi_config = SPIConfig()) {
        if (!spi_task_.init(spi_config)) {
            LOG_ERROR("Failed to init SPI");
            return false;
        }

        LOG_INFO("RobotInterface initialized");
        return true;
    }

    /**
     * @brief 启动实时控制线程
     */
    bool start() {
        if (running_) {
            LOG_WARN("Already running");
            return true;
        }

        running_ = true;

        // 创建SPI线程
        RTThreadConfig spi_cfg;
        spi_cfg.name = "spi_thread";
        spi_cfg.priority = RT_PRIORITY_SPI;
        spi_cfg.cpu_core = CPU_CORE_SPI;
        spi_cfg.period_us = SPI_PERIOD_US;

        spi_thread_ = std::make_unique<RTThread>(spi_cfg, [this]() {
            this->spi_loop();
        });

        if (!spi_thread_->start()) {
            LOG_ERROR("Failed to start SPI thread");
            running_ = false;
            return false;
        }

        LOG_INFO("RobotInterface started");
        return true;
    }

    /**
     * @brief 停止
     */
    void stop() {
        if (!running_) return;

        running_ = false;

        if (spi_thread_) {
            spi_thread_->stop();
            spi_thread_.reset();
        }

        spi_task_.close();
        LOG_INFO("RobotInterface stopped");
    }

    /**
     * @brief 设置控制指令
     */
    void set_command(const RobotCommand& cmd) {
        DataHub::instance().set_command(cmd);
    }

    /**
     * @brief 获取反馈数据
     */
    RobotFeedback get_feedback() const {
        return DataHub::instance().get_feedback();
    }

    /**
     * @brief 设置反馈回调
     */
    void set_feedback_callback(FeedbackCallback cb) {
        callback_ = cb;
    }

    /**
     * @brief 是否正在运行
     */
    bool is_running() const {
        return running_;
    }

private:
    /**
     * @brief SPI通信循环
     */
    void spi_loop() {
        spi_task_.execute();

        // 调用回调
        if (callback_) {
            callback_(DataHub::instance().get_feedback());
        }
    }

    std::atomic<bool> running_;
    SPICommTask spi_task_;
    std::unique_ptr<RTThread> spi_thread_;
    FeedbackCallback callback_;
};

} // namespace odroid

#endif // ODROID_CORE_ROBOT_INTERFACE_HPP
