/**
 * @file lock_free_buffer.hpp
 * @brief 无锁环形缓冲区 - 用于实时线程间通信
 * @author Zomnk
 * @date 2026-02-01
 */

#ifndef ODROID_REALTIME_LOCK_FREE_BUFFER_HPP
#define ODROID_REALTIME_LOCK_FREE_BUFFER_HPP

#include <atomic>
#include <array>
#include <cstddef>

namespace odroid {

/**
 * @brief 无锁单生产者单消费者缓冲区
 * @tparam T 数据类型
 * @tparam N 缓冲区大小 (必须是2的幂)
 */
template<typename T, size_t N>
class LockFreeBuffer {
    static_assert((N & (N - 1)) == 0, "缓冲区大小必须是2的幂");
    
public:
    LockFreeBuffer() : head_(0), tail_(0) {}
    
    /**
     * @brief 写入数据 (生产者调用)
     * @param item 要写入的数据
     * @return 成功返回true, 缓冲区满返回false
     */
    bool push(const T& item) {
        size_t head = head_.load(std::memory_order_relaxed);
        size_t next_head = (head + 1) & (N - 1);
        
        if (next_head == tail_.load(std::memory_order_acquire)) {
            return false;  // 缓冲区满
        }
        
        buffer_[head] = item;
        head_.store(next_head, std::memory_order_release);
        return true;
    }
    
    /**
     * @brief 读取数据 (消费者调用)
     * @param item 读取的数据存放位置
     * @return 成功返回true, 缓冲区空返回false
     */
    bool pop(T& item) {
        size_t tail = tail_.load(std::memory_order_relaxed);
        
        if (tail == head_.load(std::memory_order_acquire)) {
            return false;  // 缓冲区空
        }
        
        item = buffer_[tail];
        tail_.store((tail + 1) & (N - 1), std::memory_order_release);
        return true;
    }
    
    /**
     * @brief 检查缓冲区是否为空
     */
    bool empty() const {
        return head_.load(std::memory_order_acquire) == 
               tail_.load(std::memory_order_acquire);
    }
    
    /**
     * @brief 检查缓冲区是否已满
     */
    bool full() const {
        size_t next_head = (head_.load(std::memory_order_acquire) + 1) & (N - 1);
        return next_head == tail_.load(std::memory_order_acquire);
    }
    
    /**
     * @brief 获取当前元素数量
     */
    size_t size() const {
        size_t head = head_.load(std::memory_order_acquire);
        size_t tail = tail_.load(std::memory_order_acquire);
        return (head - tail + N) & (N - 1);
    }
    
    /**
     * @brief 清空缓冲区
     */
    void clear() {
        tail_.store(head_.load(std::memory_order_acquire), std::memory_order_release);
    }
    
private:
    std::array<T, N> buffer_;
    alignas(64) std::atomic<size_t> head_;  // 缓存行对齐，避免伪共享
    alignas(64) std::atomic<size_t> tail_;
};

/**
 * @brief 最新值缓冲区 - 只保留最新的值
 * @tparam T 数据类型
 */
template<typename T>
class LatestValueBuffer {
public:
    LatestValueBuffer() : has_value_(false) {}
    
    /**
     * @brief 写入最新值 (生产者调用)
     */
    void write(const T& item) {
        buffer_[write_index_.load(std::memory_order_relaxed)] = item;
        write_index_.store(1 - write_index_.load(std::memory_order_relaxed), 
                          std::memory_order_release);
        has_value_.store(true, std::memory_order_release);
    }
    
    /**
     * @brief 读取最新值 (消费者调用)
     * @param item 读取的数据存放位置
     * @return 有数据返回true
     */
    bool read(T& item) {
        if (!has_value_.load(std::memory_order_acquire)) {
            return false;
        }
        
        // 读取非写入的那个缓冲区
        item = buffer_[1 - write_index_.load(std::memory_order_acquire)];
        return true;
    }
    
    /**
     * @brief 检查是否有数据
     */
    bool has_value() const {
        return has_value_.load(std::memory_order_acquire);
    }
    
private:
    std::array<T, 2> buffer_;
    std::atomic<int> write_index_{0};
    std::atomic<bool> has_value_;
};

} // namespace odroid

#endif // ODROID_REALTIME_LOCK_FREE_BUFFER_HPP