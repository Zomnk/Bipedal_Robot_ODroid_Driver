/**
 * @file lock_free_buffer.hpp
 * @brief 无锁环形缓冲区 (单生产者-单消费者)
 * @author Zomnk
 * @date 2026-02-01
 */

#ifndef ODROID_REALTIME_LOCK_FREE_BUFFER_HPP
#define ODROID_REALTIME_LOCK_FREE_BUFFER_HPP

#include <atomic>
#include <array>
#include <cstddef>
#include <type_traits>

namespace odroid {

/**
 * @brief SPSC无锁环形队列
 * @tparam T 数据类型
 * @tparam Capacity 容量 (必须是2的幂)
 */
template <typename T, size_t Capacity>
class SPSCQueue {
    static_assert((Capacity & (Capacity - 1)) == 0, "Capacity must be power of 2");
    static_assert(Capacity >= 2, "Capacity must be at least 2");

public:
    SPSCQueue() : head_(0), tail_(0) {}

    /**
     * @brief 入队
     * @param item 要入队的数据
     * @return 是否成功 (队列满时返回false)
     */
    bool push(const T& item) {
        size_t head = head_.load(std::memory_order_relaxed);
        size_t next = (head + 1) & (Capacity - 1);
        
        if (next == tail_.load(std::memory_order_acquire)) {
            return false;  // 队列满
        }
        
        buffer_[head] = item;
        head_.store(next, std::memory_order_release);
        return true;
    }

    /**
     * @brief 出队
     * @param item 输出参数
     * @return 是否成功 (队列空时返回false)
     */
    bool pop(T& item) {
        size_t tail = tail_.load(std::memory_order_relaxed);
        
        if (tail == head_.load(std::memory_order_acquire)) {
            return false;  // 队列空
        }
        
        item = buffer_[tail];
        tail_.store((tail + 1) & (Capacity - 1), std::memory_order_release);
        return true;
    }

    /**
     * @brief 查看队首但不移除
     */
    bool peek(T& item) const {
        size_t tail = tail_.load(std::memory_order_relaxed);
        
        if (tail == head_.load(std::memory_order_acquire)) {
            return false;
        }
        
        item = buffer_[tail];
        return true;
    }

    /**
     * @brief 检查是否为空
     */
    bool empty() const {
        return head_.load(std::memory_order_acquire) == 
               tail_.load(std::memory_order_acquire);
    }

    /**
     * @brief 检查是否已满
     */
    bool full() const {
        size_t head = head_.load(std::memory_order_acquire);
        size_t tail = tail_.load(std::memory_order_acquire);
        return ((head + 1) & (Capacity - 1)) == tail;
    }

    /**
     * @brief 获取当前元素数量
     */
    size_t size() const {
        size_t head = head_.load(std::memory_order_acquire);
        size_t tail = tail_.load(std::memory_order_acquire);
        return (head - tail + Capacity) & (Capacity - 1);
    }

    /**
     * @brief 获取容量
     */
    constexpr size_t capacity() const { return Capacity - 1; }

    /**
     * @brief 清空队列
     */
    void clear() {
        tail_.store(head_.load(std::memory_order_relaxed), 
                    std::memory_order_relaxed);
    }

private:
    alignas(64) std::atomic<size_t> head_;
    alignas(64) std::atomic<size_t> tail_;
    alignas(64) std::array<T, Capacity> buffer_;
};

/**
 * @brief 三缓冲区 (用于最新值交换，无锁)
 * 适用于：生产者不断产生新值，消费者只需要最新值的场景
 * @tparam T 数据类型
 */
template <typename T>
class TripleBuffer {
public:
    TripleBuffer() : write_idx_(0), read_idx_(1), ready_idx_(2), new_data_(false) {
        for (int i = 0; i < 3; ++i) {
            buffers_[i] = T{};
        }
    }

    /**
     * @brief 写入新数据
     */
    void write(const T& data) {
        buffers_[write_idx_] = data;
        
        // 交换 write 和 ready
        int expected = ready_idx_.load(std::memory_order_relaxed);
        while (!ready_idx_.compare_exchange_weak(expected, write_idx_,
                                                  std::memory_order_release,
                                                  std::memory_order_relaxed)) {
            expected = ready_idx_.load(std::memory_order_relaxed);
        }
        write_idx_ = expected;
        new_data_.store(true, std::memory_order_release);
    }

    /**
     * @brief 读取最新数据
     * @return 是否有新数据
     */
    bool read(T& data) {
        if (!new_data_.load(std::memory_order_acquire)) {
            data = buffers_[read_idx_];
            return false;  // 返回旧数据
        }
        
        // 交换 read 和 ready
        int expected = ready_idx_.load(std::memory_order_relaxed);
        while (!ready_idx_.compare_exchange_weak(expected, read_idx_,
                                                  std::memory_order_acquire,
                                                  std::memory_order_relaxed)) {
            expected = ready_idx_.load(std::memory_order_relaxed);
        }
        read_idx_ = expected;
        new_data_.store(false, std::memory_order_release);
        
        data = buffers_[read_idx_];
        return true;  // 返回新数据
    }

    /**
     * @brief 直接获取最新值的引用 (只读)
     */
    const T& peek() const {
        return buffers_[read_idx_];
    }

    /**
     * @brief 检查是否有新数据
     */
    bool has_new_data() const {
        return new_data_.load(std::memory_order_acquire);
    }

private:
    std::array<T, 3> buffers_;
    int write_idx_;
    int read_idx_;
    std::atomic<int> ready_idx_;
    std::atomic<bool> new_data_;
};

} // namespace odroid

#endif // ODROID_REALTIME_LOCK_FREE_BUFFER_HPP
