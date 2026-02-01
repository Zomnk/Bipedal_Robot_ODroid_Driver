/**
 * @file rt_utils.hpp
 * @brief 实时线程工具函数
 * @author Zomnk
 * @date 2026-02-01
 */

#ifndef ODROID_REALTIME_RT_UTILS_HPP
#define ODROID_REALTIME_RT_UTILS_HPP

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <sched.h>
#include <pthread.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <unistd.h>

#include "common/logger.hpp"
#include "common/constants.hpp"

namespace odroid {

/**
 * @brief 设置线程调度策略为SCHED_FIFO
 * @param priority 线程优先级 (1-99)
 * @return 是否成功
 */
inline bool set_thread_priority(int priority) {
    struct sched_param param;
    param.sched_priority = priority;

    int ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    if (ret != 0) {
        LOG_ERROR("Failed to set SCHED_FIFO priority %d: %s", priority, strerror(ret));
        return false;
    }

    LOG_INFO("Set thread priority to SCHED_FIFO:%d", priority);
    return true;
}

/**
 * @brief 设置线程CPU亲和性
 * @param cpu_core CPU核心编号
 * @return 是否成功
 */
inline bool set_thread_affinity(int cpu_core) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_core, &cpuset);

    int ret = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
    if (ret != 0) {
        LOG_ERROR("Failed to set CPU affinity to core %d: %s", cpu_core, strerror(ret));
        return false;
    }

    LOG_INFO("Set thread affinity to CPU core %d", cpu_core);
    return true;
}

/**
 * @brief 锁定所有内存，防止页面交换
 * @return 是否成功
 */
inline bool lock_memory() {
    int ret = mlockall(MCL_CURRENT | MCL_FUTURE);
    if (ret != 0) {
        LOG_ERROR("Failed to lock memory: %s", strerror(errno));
        return false;
    }

    LOG_INFO("Memory locked successfully");
    return true;
}

/**
 * @brief 解锁内存
 */
inline void unlock_memory() {
    munlockall();
    LOG_INFO("Memory unlocked");
}

/**
 * @brief 预分配栈空间以避免RT期间的页面错误
 * @param stack_size 栈大小 (字节)
 */
inline void prefault_stack(size_t stack_size = 8 * 1024) {
    volatile char* stack = static_cast<volatile char*>(alloca(stack_size));
    for (size_t i = 0; i < stack_size; i += 4096) {
        stack[i] = 0;
    }
    (void)stack;  // 防止unused warning
    LOG_DEBUG("Stack prefaulted: %zu bytes", stack_size);
}

/**
 * @brief 检查是否拥有RT权限
 * @return 是否有RT权限
 */
inline bool check_rt_capabilities() {
    struct rlimit rlim;
    if (getrlimit(RLIMIT_RTPRIO, &rlim) != 0) {
        LOG_ERROR("Failed to get RLIMIT_RTPRIO: %s", strerror(errno));
        return false;
    }

    LOG_INFO("RLIMIT_RTPRIO: soft=%lu, hard=%lu",
             (unsigned long)rlim.rlim_cur, (unsigned long)rlim.rlim_max);

    if (static_cast<int>(rlim.rlim_cur) < RT_PRIORITY_MAX) {
        LOG_WARN("RT priority limit (%lu) is less than required (%d)",
                 (unsigned long)rlim.rlim_cur, RT_PRIORITY_MAX);
        LOG_WARN("Run 'sudo setcap cap_sys_nice+ep <executable>' or add to /etc/security/limits.conf");
        return false;
    }

    return true;
}

/**
 * @brief 检查是否有SPI设备访问权限
 * @param device SPI设备路径
 * @return 是否有权限
 */
inline bool check_spi_permission(const char* device = SPI_DEVICE_DEFAULT) {
    if (access(device, R_OK | W_OK) != 0) {
        LOG_ERROR("No access to %s: %s", device, strerror(errno));
        LOG_WARN("Run 'sudo usermod -a -G spi $USER' and re-login, or use sudo");
        return false;
    }
    return true;
}

/**
 * @brief 初始化RT环境
 * @return 是否成功
 */
inline bool init_rt_environment() {
    LOG_INFO("Initializing RT environment...");

    // 检查RT权限
    if (!check_rt_capabilities()) {
        LOG_WARN("RT capabilities check failed, continuing without RT priority");
    }

    // 锁定内存
    if (!lock_memory()) {
        LOG_WARN("Memory lock failed, continuing anyway");
    }

    // 预分配栈空间
    prefault_stack();

    LOG_INFO("RT environment initialized");
    return true;
}

/**
 * @brief 获取CPU核心数量
 * @return CPU核心数
 */
inline int get_cpu_count() {
    return static_cast<int>(sysconf(_SC_NPROCESSORS_ONLN));
}

/**
 * @brief 打印系统信息
 */
inline void print_system_info() {
    LOG_INFO("=== System Information ===");
    LOG_INFO("CPU cores: %d", get_cpu_count());

    // 内核版本
    FILE* fp = fopen("/proc/version", "r");
    if (fp) {
        char buf[256];
        if (fgets(buf, sizeof(buf), fp)) {
            buf[80] = '\0';  // 截断
            LOG_INFO("Kernel: %s...", buf);
        }
        fclose(fp);
    }

    // 检查RT内核
    fp = fopen("/sys/kernel/realtime", "r");
    if (fp) {
        int rt = 0;
        if (fscanf(fp, "%d", &rt) == 1 && rt == 1) {
            LOG_INFO("RT-PREEMPT kernel detected");
        }
        fclose(fp);
    } else {
        LOG_WARN("Not a RT-PREEMPT kernel (non-critical for testing)");
    }

    LOG_INFO("==========================");
}

} // namespace odroid

#endif // ODROID_REALTIME_RT_UTILS_HPP
