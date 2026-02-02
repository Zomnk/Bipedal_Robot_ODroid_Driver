/**
 * @file rt_utils.hpp
 * @brief 实时线程工具函数
 * @author Zomnk
 * @date 2026-02-01
 */

#ifndef ODROID_REALTIME_RT_UTILS_HPP
#define ODROID_REALTIME_RT_UTILS_HPP

#include <cstdint>
#include <sched.h>
#include <pthread.h>
#include <sys/mman.h>

#include "common/constants.hpp"
#include "common/logger.hpp"

namespace odroid {

/**
 * @brief 设置线程为实时优先级 (SCHED_FIFO)
 * @param priority 优先级 (1-99, 99最高)
 * @return 成功返回true
 */
inline bool set_realtime_priority(int priority) {
    if (priority < 1 || priority > 99) {
        LOG_ERROR("无效的实时优先级: %d (应为1-99)", priority);
        return false;
    }
    
    struct sched_param param;
    param.sched_priority = priority;
    
    if (sched_setscheduler(0, SCHED_FIFO, &param) != 0) {
        LOG_ERROR("设置SCHED_FIFO失败, 需要root权限");
        return false;
    }
    
    LOG_INFO("已设置实时优先级: %d", priority);
    return true;
}

/**
 * @brief 绑定当前线程到指定CPU核心
 * @param cpu_id CPU核心编号 (从0开始)
 * @return 成功返回true
 */
inline bool set_cpu_affinity(int cpu_id) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_id, &cpuset);
    
    if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) != 0) {
        LOG_ERROR("设置CPU亲和性失败: CPU %d", cpu_id);
        return false;
    }
    
    LOG_INFO("已绑定到CPU核心: %d", cpu_id);
    return true;
}

/**
 * @brief 锁定当前进程内存，防止换页
 * @return 成功返回true
 */
inline bool lock_memory() {
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        LOG_ERROR("锁定内存失败, 需要root权限");
        return false;
    }
    
    LOG_INFO("已锁定进程内存");
    return true;
}

/**
 * @brief 配置实时线程环境
 * @param priority 实时优先级
 * @param cpu_id CPU核心编号
 * @return 成功返回true
 */
inline bool setup_realtime_thread(int priority, int cpu_id) {
    bool success = true;
    
    // 锁定内存
    if (!lock_memory()) {
        success = false;
    }
    
    // 设置CPU亲和性
    if (!set_cpu_affinity(cpu_id)) {
        success = false;
    }
    
    // 设置实时优先级
    if (!set_realtime_priority(priority)) {
        success = false;
    }
    
    return success;
}

} // namespace odroid

#endif // ODROID_REALTIME_RT_UTILS_HPP