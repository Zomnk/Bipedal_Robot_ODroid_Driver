/**
 * @file constants.hpp
 * @brief 系统常量定义
 * @author Zomnk
 * @date 2026-02-01
 */

#ifndef ODROID_COMMON_CONSTANTS_HPP
#define ODROID_COMMON_CONSTANTS_HPP

#include <cstdint>
#include <cstddef>
#include <ctime>

namespace odroid {

//==============================================================================
// 电机参数常量 - 达妙电机
//==============================================================================

// DM6006 (髋关节 - Hip)
constexpr float DM6006_POS_MAX    = 12.5f;     // rad
constexpr float DM6006_VEL_MAX    = 45.0f;     // rad/s
constexpr float DM6006_TORQUE_MAX = 12.0f;     // Nm
constexpr float DM6006_KP_MAX     = 500.0f;
constexpr float DM6006_KD_MAX     = 5.0f;

// DM4340 (大腿Thigh/小腿Calf/膝关节Knee)
constexpr float DM4340_POS_MAX    = 12.5f;     // rad
constexpr float DM4340_VEL_MAX    = 30.0f;     // rad/s
constexpr float DM4340_TORQUE_MAX = 27.0f;     // Nm
constexpr float DM4340_KP_MAX     = 500.0f;
constexpr float DM4340_KD_MAX     = 5.0f;

// DM8006 (踝关节 - Ankle)
constexpr float DM8006_POS_MAX    = 12.5f;     // rad
constexpr float DM8006_VEL_MAX    = 25.0f;     // rad/s
constexpr float DM8006_TORQUE_MAX = 40.0f;     // Nm
constexpr float DM8006_KP_MAX     = 500.0f;
constexpr float DM8006_KD_MAX     = 5.0f;

//==============================================================================
// SPI通信常量
//==============================================================================

constexpr const char* SPI_DEVICE_DEFAULT = "/dev/spidev0.0";
constexpr uint32_t SPI_SPEED_HZ      = 10000000;  // 10MHz
constexpr uint8_t  SPI_BITS_PER_WORD = 16;        // 16位模式
constexpr uint8_t  SPI_MODE          = 0;         // CPOL=0, CPHA=0

// 数据布局常量
constexpr size_t MOTORS_PER_LEG     = 5;          // 每腿5个电机
constexpr size_t MOTOR_PARAMS       = 4;          // 每个电机4个参数
constexpr size_t IMU_PARAMS         = 10;         // 每个IMU 10个参数
constexpr size_t NUM_IMUS           = 2;          // 2个IMU

// TX: 10电机 x 4参数 = 40 words
constexpr size_t SPI_TX_WORDS       = MOTORS_PER_LEG * 2 * MOTOR_PARAMS;  // 40
// RX: 40电机反馈 + 20 IMU数据 = 60 words
constexpr size_t SPI_RX_WORDS       = SPI_TX_WORDS + NUM_IMUS * IMU_PARAMS;  // 60

constexpr size_t CONTROL_DATA_NUM   = SPI_TX_WORDS;   // 40
constexpr size_t FEEDBACK_DATA_NUM  = SPI_RX_WORDS;   // 60

//==============================================================================
// 实时线程常量
//==============================================================================

constexpr int RT_PRIORITY_MAX  = 99;    // SCHED_FIFO最大优先级
constexpr int RT_PRIORITY_SPI  = 90;    // SPI通信线程优先级
constexpr int RT_PRIORITY_MAIN = 80;    // 主控制线程优先级

constexpr int CPU_CORE_SPI     = 2;     // SPI线程绑定核心
constexpr int CPU_CORE_MAIN    = 3;     // 主线程绑定核心

//==============================================================================
// 控制周期常量
//==============================================================================

constexpr uint32_t CONTROL_FREQ_HZ   = 1000;      // 1kHz控制频率
constexpr uint32_t CONTROL_PERIOD_US = 1000000 / CONTROL_FREQ_HZ;  // 1000us
constexpr uint32_t SPI_PERIOD_US     = CONTROL_PERIOD_US;          // SPI周期=控制周期

//==============================================================================
// 工具函数
//==============================================================================

inline uint64_t get_time_us() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000ULL +
           static_cast<uint64_t>(ts.tv_nsec) / 1000ULL;
}

inline uint64_t get_time_ns() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL +
           static_cast<uint64_t>(ts.tv_nsec);
}

} // namespace odroid

#endif // ODROID_COMMON_CONSTANTS_HPP