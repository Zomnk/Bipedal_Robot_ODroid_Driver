/**
 * @file spi_driver.hpp
 * @brief Linux SPI主机驱动
 * @author Zomnk
 * @date 2026-02-01
 */

#ifndef ODROID_COMMUNICATION_SPI_DRIVER_HPP
#define ODROID_COMMUNICATION_SPI_DRIVER_HPP

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#include "common/types.hpp"
#include "common/constants.hpp"
#include "common/logger.hpp"

namespace odroid {

/**
 * @brief SPI驱动配置
 */
struct SPIConfig {
    std::string device = SPI_DEVICE_DEFAULT;
    uint32_t speed_hz = SPI_SPEED_HZ;
    uint8_t bits_per_word = SPI_BITS_PER_WORD;
    uint8_t mode = SPI_MODE_0;
    
    // STM32 SPI配置: MSB first, CPOL=0, CPHA=0
    bool lsb_first = false;
};

/**
 * @brief SPI驱动类
 */
class SPIDriver {
public:
    SPIDriver() : fd_(-1), is_open_(false) {}
    
    ~SPIDriver() {
        close();
    }

    // 禁止拷贝
    SPIDriver(const SPIDriver&) = delete;
    SPIDriver& operator=(const SPIDriver&) = delete;

    /**
     * @brief 打开SPI设备
     * @param config SPI配置
     * @return 是否成功
     */
    bool open(const SPIConfig& config = SPIConfig()) {
        if (is_open_) {
            LOG_WARN("SPI device already open");
            return true;
        }

        config_ = config;

        // 打开设备
        fd_ = ::open(config_.device.c_str(), O_RDWR);
        if (fd_ < 0) {
            LOG_ERROR("Failed to open SPI device %s: %s", 
                     config_.device.c_str(), strerror(errno));
            return false;
        }

        // 设置SPI模式
        if (ioctl(fd_, SPI_IOC_WR_MODE, &config_.mode) < 0) {
            LOG_ERROR("Failed to set SPI mode: %s", strerror(errno));
            ::close(fd_);
            fd_ = -1;
            return false;
        }

        // 设置位宽
        if (ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &config_.bits_per_word) < 0) {
            LOG_ERROR("Failed to set SPI bits per word: %s", strerror(errno));
            ::close(fd_);
            fd_ = -1;
            return false;
        }

        // 设置速度
        if (ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &config_.speed_hz) < 0) {
            LOG_ERROR("Failed to set SPI speed: %s", strerror(errno));
            ::close(fd_);
            fd_ = -1;
            return false;
        }

        // 设置LSB/MSB
        uint8_t lsb = config_.lsb_first ? 1 : 0;
        if (ioctl(fd_, SPI_IOC_WR_LSB_FIRST, &lsb) < 0) {
            LOG_WARN("Failed to set SPI LSB first: %s", strerror(errno));
            // 非致命错误，继续
        }

        is_open_ = true;
        LOG_INFO("SPI device %s opened: mode=%d, bits=%d, speed=%u Hz",
                config_.device.c_str(), config_.mode, 
                config_.bits_per_word, config_.speed_hz);

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
            LOG_INFO("SPI device closed");
        }
    }

    /**
     * @brief 检查设备是否打开
     */
    bool is_open() const { return is_open_; }

    /**
     * @brief 全双工传输 (16位模式)
     * @param tx_buf 发送缓冲区 (uint16_t数组)
     * @param rx_buf 接收缓冲区 (uint16_t数组)
     * @param len 传输的uint16_t数量
     * @return 是否成功
     */
    bool transfer_16bit(const uint16_t* tx_buf, uint16_t* rx_buf, size_t len) {
        if (!is_open_) {
            LOG_ERROR("SPI device not open");
            return false;
        }

        struct spi_ioc_transfer tr;
        memset(&tr, 0, sizeof(tr));
        
        tr.tx_buf = reinterpret_cast<unsigned long>(tx_buf);
        tr.rx_buf = reinterpret_cast<unsigned long>(rx_buf);
        tr.len = len * 2;  // 字节数
        tr.speed_hz = config_.speed_hz;
        tr.bits_per_word = config_.bits_per_word;
        tr.delay_usecs = 0;

        int ret = ioctl(fd_, SPI_IOC_MESSAGE(1), &tr);
        if (ret < 0) {
            LOG_ERROR("SPI transfer failed: %s", strerror(errno));
            transfer_errors_++;
            return false;
        }

        transfer_count_++;
        return true;
    }

    /**
     * @brief 全双工传输 (使用SPIBuffer)
     * @param tx_buf 发送缓冲区
     * @param rx_buf 接收缓冲区
     * @return 是否成功
     */
    bool transfer(const SPITxBuffer& tx_buf, SPIRxBuffer& rx_buf) {
        // 需要处理大小不匹配的情况
        // TX: CONTROL_DATA_NUM (40) uint16_t
        // RX: FEEDBACK_DATA_NUM (60) uint16_t
        // 传输时使用较大的那个
        
        static_assert(FEEDBACK_DATA_NUM >= CONTROL_DATA_NUM, 
                      "RX buffer must be >= TX buffer");
        
        // 扩展TX缓冲区到RX大小，填充0
        uint16_t tx_extended[FEEDBACK_DATA_NUM] = {0};
        memcpy(tx_extended, tx_buf.data, sizeof(tx_buf.data));
        
        return transfer_16bit(tx_extended, rx_buf.data, FEEDBACK_DATA_NUM);
    }

    /**
     * @brief 获取传输统计
     */
    uint64_t get_transfer_count() const { return transfer_count_; }
    uint64_t get_transfer_errors() const { return transfer_errors_; }

    /**
     * @brief 重置统计
     */
    void reset_stats() {
        transfer_count_ = 0;
        transfer_errors_ = 0;
    }

private:
    int fd_;
    bool is_open_;
    SPIConfig config_;
    
    uint64_t transfer_count_ = 0;
    uint64_t transfer_errors_ = 0;
};

} // namespace odroid

#endif // ODROID_COMMUNICATION_SPI_DRIVER_HPP
