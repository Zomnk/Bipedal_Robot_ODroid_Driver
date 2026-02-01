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
constexpr float DM4340_TORQUE_MAX = 10.0f;     // Nm
constexpr float DM4340_KP_MAX     = 500.0f;
constexpr float DM4340_KD_MAX     = 5.0f;

// DM8006 (踝关节 - Ankle)
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

// SPI设备配置
constexpr const char* SPI_DEVICE_DEFAULT = "/dev/spidev0.0";
constexpr uint32_t SPI_SPEED_HZ = 10000000;    // 10 MHz
constexpr uint8_t SPI_BITS_PER_WORD = 16;      // 16位模式 (与STM32匹配)

// 数据缓冲区大小
// TX: 10个电机 * 4参数 = 40 words (位置/速度/kp/kd)
// RX: 10个电机 * 4参数 + 2个IMU * 10参数 = 40 + 20 = 60 words
constexpr size_t CONTROL_DATA_NUM = 40;        // 控制数据: 10电机 * 4参数
constexpr size_t FEEDBACK_DATA_NUM = 60;       // 反馈数据: 10电机*4 + 2IMU*10
constexpr size_t SPI_TX_WORDS = CONTROL_DATA_NUM;
constexpr size_t SPI_RX_WORDS = FEEDBACK_DATA_NUM;
constexpr size_t SPI_TX_BYTES = SPI_TX_WORDS * 2;
constexpr size_t SPI_RX_BYTES = SPI_RX_WORDS * 2;

//==============================================================================
// 实时控制常量
//==============================================================================

constexpr uint32_t CONTROL_PERIOD_US = 1000;   // 1ms控制周期
constexpr uint32_t CONTROL_FREQ_HZ   = 1000;   // 1kHz控制频率
constexpr uint64_t SPI_PERIOD_US     = 1000;   // SPI通信周期 (与控制周期相同)

//==============================================================================
// 实时线程配置
//==============================================================================

// 线程优先级 (SCHED_FIFO, 1-99, 越高优先级越高)
constexpr int RT_PRIORITY_MAX     = 99;
constexpr int RT_PRIORITY_SPI     = 90;        // SPI通信线程 (最高)
constexpr int RT_PRIORITY_CONTROL = 80;        // 控制线程
constexpr int RT_PRIORITY_LOGGING = 20;        // 日志线程 (最低)

// CPU核心绑定 (ODroid-C4有4个大核+2个小核)
constexpr int CPU_CORE_SPI     = 3;            // SPI线程绑定到核心3
constexpr int CPU_CORE_CONTROL = 2;            // 控制线程绑定到核心2
constexpr int CPU_CORE_MAIN    = 0;            // 主线程绑定到核心0

//==============================================================================
// 时间工具函数
//==============================================================================

/**
 * @brief 获取当前时间 (微秒)
 */
inline uint64_t get_time_us() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000ULL +
           static_cast<uint64_t>(ts.tv_nsec) / 1000ULL;
}

} // namespace odroid

#endif // ODROID_COMMON_CONSTANTS_HPP
