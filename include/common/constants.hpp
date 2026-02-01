/**
 * @file constants.hpp
 * @brief 系统常量定义
 * @author Zomnk
 * @date 2026-02-01
 */

#ifndef ODROID_COMMON_CONSTANTS_HPP
#define ODROID_COMMON_CONSTANTS_HPP

#include <cstdint>

namespace odroid {

//==============================================================================
// 数学常量
//==============================================================================

constexpr double PI = 3.14159265358979323846;
constexpr double DEG_TO_RAD = PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / PI;

//==============================================================================
// 时间常量
//==============================================================================

constexpr uint64_t US_PER_MS = 1000;
constexpr uint64_t US_PER_S = 1000000;
constexpr uint64_t NS_PER_US = 1000;
constexpr uint64_t NS_PER_MS = 1000000;
constexpr uint64_t NS_PER_S = 1000000000;

//==============================================================================
// 控制周期
//==============================================================================

// SPI通信周期 (1kHz)
constexpr uint64_t SPI_PERIOD_US = 1000;
// 插值器周期 (500Hz) - 暂未使用
constexpr uint64_t INTERP_PERIOD_US = 2000;
// TCP通信周期 (200Hz) - 暂未使用
constexpr uint64_t TCP_PERIOD_US = 5000;

//==============================================================================
// RT线程优先级
//==============================================================================

constexpr int RT_PRIORITY_MAX = 99;
constexpr int RT_PRIORITY_SPI = 99;         // SPI通信 (最高)
constexpr int RT_PRIORITY_INTERP = 80;      // 插值器
constexpr int RT_PRIORITY_TCP_RX = 50;      // TCP接收
constexpr int RT_PRIORITY_TCP_TX = 50;      // TCP发送
constexpr int RT_PRIORITY_LOGGER = 10;      // 日志

//==============================================================================
// CPU亲和性
//==============================================================================

// ODroid-C4有4个核心 (0-3)
constexpr int CPU_CORE_SPI = 3;             // SPI通信专用核心
constexpr int CPU_CORE_INTERP = 2;          // 插值器专用核心
constexpr int CPU_CORE_TCP = 1;             // TCP通信核心
constexpr int CPU_CORE_OTHER = 0;           // 其他任务核心

//==============================================================================
// SPI配置
//==============================================================================

// 默认SPI设备
constexpr const char* SPI_DEVICE_DEFAULT = "/dev/spidev0.0";
// SPI速度 (Hz)
constexpr uint32_t SPI_SPEED_HZ = 10000000;  // 10MHz
// SPI位宽
constexpr uint8_t SPI_BITS_PER_WORD = 16;

//==============================================================================
// 超时配置
//==============================================================================

// STM32通信超时 (毫秒)
constexpr uint32_t STM32_TIMEOUT_MS = 100;
// Jetson连接超时 (毫秒)
constexpr uint32_t JETSON_TIMEOUT_MS = 1000;

//==============================================================================
// 缓冲区大小
//==============================================================================

// 无锁队列容量
constexpr size_t QUEUE_CAPACITY = 64;
// 日志缓冲区大小
constexpr size_t LOG_BUFFER_SIZE = 4096;

} // namespace odroid

#endif // ODROID_COMMON_CONSTANTS_HPP
