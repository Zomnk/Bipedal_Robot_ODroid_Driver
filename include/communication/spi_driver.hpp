/**
 * @file spi_driver.hpp
 * @brief SPI驱动层 - 基于Linux spidev
 * @author Zomnk
 * @date 2026-02-01
 */

#ifndef ODROID_COMMUNICATION_SPI_DRIVER_HPP
#define ODROID_COMMUNICATION_SPI_DRIVER_HPP

#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#include "common/types.hpp"
#include "common/constants.hpp"
#include "common/logger.hpp"

namespace odroid {

/**
 * @brief SPI驱动类 - 16位主机模式
 */
class SPIDriver {
public:
    SPIDriver() = default;
    ~SPIDriver() { close(); }
    
    // 禁止拷贝
    SPIDriver(const SPIDriver&) = delete;
    SPIDriver& operator=(const SPIDriver&) = delete;
    
    /**
     * @brief 初始化SPI设备
     * @param config SPI配置
     * @return 成功返回true
     */
    bool init(const SPIConfig& config = SPIConfig{}) {
        config_ = config;
        
        // 打开SPI设备
        fd_ = ::open(config_.device, O_RDWR);
        if (fd_ < 0) {
            LOG_ERROR("无法打开SPI设备: %s", config_.device);
            return false;
        }
        
        // 配置SPI模式
        uint8_t mode = config_.mode;
        if (ioctl(fd_, SPI_IOC_WR_MODE, &mode) < 0) {
            LOG_ERROR("设置SPI模式失败");
            ::close(fd_);
            fd_ = -1;
            return false;
        }
        
        // 配置位宽
        uint8_t bits = config_.bits_per_word;
        if (ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
            LOG_ERROR("设置SPI位宽失败");
            ::close(fd_);
            fd_ = -1;
            return false;
        }
        
        // 配置速度
        uint32_t speed = config_.speed_hz;
        if (ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
            LOG_ERROR("设置SPI速度失败");
            ::close(fd_);
            fd_ = -1;
            return false;
        }
        
        is_open_ = true;
        LOG_INFO("SPI初始化成功: %s, %d Hz, %d bits", 
                 config_.device, config_.speed_hz, config_.bits_per_word);
        return true;
    }
    
    /**
     * @brief 关闭SPI设备
     */
    void close() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
            is_open_ = false;
            LOG_INFO("SPI设备已关闭");
        }
    }
    
    /**
     * @brief 全双工SPI传输
     * @param tx_buffer 发送缓冲区
     * @param rx_buffer 接收缓冲区
     * @return 成功返回true
     */
    bool transfer(const SPITxBuffer& tx_buffer, SPIRxBuffer& rx_buffer) {
        if (!is_open_) {
            LOG_ERROR("SPI设备未打开");
            return false;
        }
        
        // 准备传输结构
        // 注意: 我们需要发送的数据量取最大值
        constexpr size_t tx_bytes = SPI_TX_WORDS * 2;
        constexpr size_t rx_bytes = SPI_RX_WORDS * 2;
        constexpr size_t max_bytes = (tx_bytes > rx_bytes) ? tx_bytes : rx_bytes;
        
        // 使用更大的缓冲区进行传输
        uint8_t tx_buf[max_bytes] = {0};
        uint8_t rx_buf[max_bytes] = {0};
        
        // 复制发送数据
        std::memcpy(tx_buf, tx_buffer.data, tx_bytes);
        
        struct spi_ioc_transfer tr = {};
        tr.tx_buf = reinterpret_cast<unsigned long>(tx_buf);
        tr.rx_buf = reinterpret_cast<unsigned long>(rx_buf);
        tr.len = max_bytes;
        tr.speed_hz = config_.speed_hz;
        tr.bits_per_word = config_.bits_per_word;
        tr.delay_usecs = 0;
        
        if (ioctl(fd_, SPI_IOC_MESSAGE(1), &tr) < 0) {
            LOG_ERROR("SPI传输失败");
            return false;
        }
        
        // 复制接收数据
        std::memcpy(rx_buffer.data, rx_buf, rx_bytes);
        
        transfer_count_++;
        return true;
    }
    
    /**
     * @brief 获取传输计数
     */
    uint64_t get_transfer_count() const { return transfer_count_; }
    
    /**
     * @brief 检查设备是否已打开
     */
    bool is_open() const { return is_open_; }
    
private:
    int fd_ = -1;
    bool is_open_ = false;
    SPIConfig config_;
    uint64_t transfer_count_ = 0;
};

} // namespace odroid

#endif // ODROID_COMMUNICATION_SPI_DRIVER_HPP