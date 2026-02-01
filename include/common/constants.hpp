/**
 * @file constants.hpp
 * @brief 系统常量定义
 */

#ifndef ODROID_COMMON_CONSTANTS_HPP
#define ODROID_COMMON_CONSTANTS_HPP

#include <cstdint>

namespace odroid {

//==============================================================================
// 电机参数常量 - 达妙电机
//==============================================================================

// DM6006 (髋关节)
constexpr float DM6006_POS_MAX    = 12.5f;     // rad
constexpr float DM6006_VEL_MAX    = 45.0f;     // rad/s
constexpr float DM6006_TORQUE_MAX = 12.0f;     // Nm
constexpr float DM6006_KP_MAX     = 500.0f;
constexpr float DM6006_KD_MAX     = 5.0f;

// DM4340 (大腿/小腿/膝关节)
constexpr float DM4340_POS_MAX    = 12.5f;     // rad
constexpr float DM4340_VEL_MAX    = 30.0f;     // rad/s
constexpr float DM4340_TORQUE_MAX = 10.0f;     // Nm
constexpr float DM4340_KP_MAX     = 500.0f;
constexpr float DM4340_KD_MAX     = 5.0f;

// DM8006 (踝关节)
constexpr float DM8006_POS_MAX    = 12.5f;     // rad
constexpr float DM8006_VEL_MAX    = 25.0f;     // rad/s
constexpr float DM8006_TORQUE_MAX = 24.0f;     // Nm
constexpr float DM8006_KP_MAX     = 500.0f;
constexpr float DM8006_KD_MAX     = 5.0f;

//==============================================================================
// IMU参数常量
//==============================================================================

constexpr float IMU_ACCEL_MAX = 160.0f;        // m/s^2 (约16g)
constexpr float IMU_GYRO_MAX  = 2000.0f;       // deg/s
constexpr float IMU_EULER_MAX = 180.0f;        // deg

//==============================================================================
// SPI通信常量
//==============================================================================

constexpr size_t SPI_TX_WORDS = 80;            // 发送缓冲区大小 (uint16_t)
constexpr size_t SPI_RX_WORDS = 84;            // 接收缓冲区大小 (uint16_t)
constexpr size_t SPI_TX_BYTES = SPI_TX_WORDS * 2;
constexpr size_t SPI_RX_BYTES = SPI_RX_WORDS * 2;

constexpr uint32_t SPI_SPEED_HZ = 10000000;    // 10 MHz

//==============================================================================
// 实时控制常量
//==============================================================================

constexpr uint32_t CONTROL_PERIOD_US = 1000;   // 1ms控制周期
constexpr uint32_t CONTROL_FREQ_HZ   = 1000;   // 1kHz控制频率

//==============================================================================
// 时间工具函数
//==============================================================================

inline uint64_t get_time_us() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000ULL + 
           static_cast<uint64_t>(ts.tv_nsec) / 1000ULL;
}

} // namespace odroid

#endif // ODROID_COMMON_CONSTANTS_HPP
