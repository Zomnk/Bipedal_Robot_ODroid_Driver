/**
 * @file spi_driver.hpp
 * @brief SPI驱动层 - 基于Linux spidev
 * @author Zomnk
 * @date 2026-02-01
 * 
 * @note 重要: ODroid-C4 (ARM64) 和 STM32F405 (ARM32) 都是小端序
 *       SPI 16位模式下需要正确处理字节顺序
 *       无NSS模式: 仅使用 MOSI, MISO, SCK 三线
 */

#ifndef ODROID_COMMUNICATION_SPI_DRIVER_HPP
#define ODROID_COMMUNICATION_SPI_DRIVER_HPP

#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <endian.h>

#include "common/types.hpp"
#include "common/constants.hpp"
#include "common/logger.hpp"

namespace odroid {

/**
 * @brief SPI驱动类 - 16位主机模式 (无NSS)
 * @note 由于没有NSS片选信号，STM32从机需要始终处于接收状态
 *       全双工传输: ODroid发送控制指令同时接收STM32的反馈数据
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
        
        // 配置SPI模式 (CPOL=0, CPHA=0, 与STM32一致)
        // 添加 SPI_NO_CS 标志，因为我们没有使用硬件片选
        uint32_t mode32 = config_.mode | SPI_NO_CS;
        if (ioctl(fd_, SPI_IOC_WR_MODE32, &mode32) < 0) {
            LOG_ERROR("设置SPI模式失败");
            ::close(fd_);
            fd_ = -1;
            return false;
        }
        
        // 配置位宽 - 使用8位模式，手动处理16位数据
        // 这样可以更好地控制字节序
        uint8_t bits = 8;  // 使用8位传输，手动处理16位
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
        LOG_INFO("SPI初始化成功: %s, %d Hz, 8-bit mode (16-bit data)", 
                 config_.device, config_.speed_hz);
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
     * @brief 全双工SPI传输 (16位数据，大端序传输)
     * @param tx_buffer 发送缓冲区 (控制指令, 40 words)
     * @param rx_buffer 接收缓冲区 (反馈数据, 60 words)
     * @return 成功返回true
     * 
     * @note 传输长度使用60 words (较大值)，以确保能接收完整的反馈数据
     *       STM32发送60 words，ODroid发送40 words (后20 words为0)
     */
    bool transfer(const SPITxBuffer& tx_buffer, SPIRxBuffer& rx_buffer) {
        if (!is_open_) {
            LOG_ERROR("SPI设备未打开");
            return false;
        }
        
        // 传输长度：使用较大值60 words = 120 bytes
        constexpr size_t transfer_words = SPI_RX_WORDS;  // 60 words
        constexpr size_t transfer_bytes = transfer_words * 2;  // 120 bytes
        
        // 准备发送缓冲区（大端序转换）
        uint8_t tx_buf[transfer_bytes] = {0};
        uint8_t rx_buf[transfer_bytes] = {0};
        
        // 将16位数据转换为大端序字节流
        // STM32 SPI 16位模式：高字节先传输
        for (size_t i = 0; i < SPI_TX_WORDS; i++) {
            uint16_t val = tx_buffer.data[i];
            tx_buf[i * 2]     = (val >> 8) & 0xFF;  // 高字节
            tx_buf[i * 2 + 1] = val & 0xFF;         // 低字节
        }
        // 剩余部分 (40-59) 填充0，已由初始化完成
        
        struct spi_ioc_transfer tr = {};
        tr.tx_buf = reinterpret_cast<unsigned long>(tx_buf);
        tr.rx_buf = reinterpret_cast<unsigned long>(rx_buf);
        tr.len = transfer_bytes;
        tr.speed_hz = config_.speed_hz;
        tr.bits_per_word = 8;
        tr.delay_usecs = 0;
        tr.cs_change = 0;  // 传输完成后不改变CS状态
        
        int ret = ioctl(fd_, SPI_IOC_MESSAGE(1), &tr);
        if (ret < 0) {
            LOG_ERROR("SPI传输失败: ret=%d", ret);
            return false;
        }
        
        // 将接收到的大端序字节流转换回16位数据
        for (size_t i = 0; i < SPI_RX_WORDS; i++) {
            uint8_t high = rx_buf[i * 2];
            uint8_t low = rx_buf[i * 2 + 1];
            rx_buffer.data[i] = (static_cast<uint16_t>(high) << 8) | low;
        }
        
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