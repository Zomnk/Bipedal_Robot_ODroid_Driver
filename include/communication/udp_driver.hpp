/**
 * @file udp_driver.hpp
 * @brief UDP通信驱动 (ODroid <-> Jetson)
 * @author Zomnk
 * @date 2026-02-02
 *
 * @note 实现与Jetson之间的UDP双向通信
 * 通信模式:
 *   - ODroid作为客户端，主动发送观测数据
 *   - Jetson作为服务端，返回RL推理结果
 */

#ifndef ODROID_COMMUNICATION_UDP_DRIVER_HPP
#define ODROID_COMMUNICATION_UDP_DRIVER_HPP

#include <cstdint>
#include <cstring>
#include <string>
#include <atomic>
#include <mutex>
#include <thread>
#include <chrono>
#include <future>
#include <functional>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include "common/logger.hpp"
#include "common/time_utils.hpp"
#include "communication/jetson_protocol.hpp"

namespace odroid {

/**
 * @brief UDP通信配置
 */
struct UDPConfig {
    std::string jetson_ip = JETSON_DEFAULT_IP;
    int jetson_port = JETSON_DEFAULT_PORT;
    int recv_timeout_ms = 10;      // 接收超时 (ms)
    int send_interval_us = 2000;   // 发送间隔 (us), 500Hz
    bool enable_blocking = false;  // 是否阻塞模式
};

/**
 * @brief UDP通信驱动类
 */
class UDPDriver {
public:
    using ResponseCallback = std::function<void(const JetsonResponse&)>;

    UDPDriver() = default;
    ~UDPDriver() {
        close();
    }

    // 禁止拷贝
    UDPDriver(const UDPDriver&) = delete;
    UDPDriver& operator=(const UDPDriver&) = delete;

    /**
     * @brief 初始化UDP连接
     * @param config UDP配置
     * @return 是否成功
     */
    bool init(const UDPConfig& config = UDPConfig()) {
        config_ = config;

        // 创建UDP socket
        socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd_ < 0) {
            LOG_ERROR("Failed to create UDP socket: %s", strerror(errno));
            return false;
        }

        // 绑定本地端口（重要：必须绑定才能接收数据！）
        struct sockaddr_in local_addr;
        memset(&local_addr, 0, sizeof(local_addr));
        local_addr.sin_family = AF_INET;
        local_addr.sin_addr.s_addr = INADDR_ANY;  // 监听所有网卡
        local_addr.sin_port = htons(config_.jetson_port);  // 使用相同端口
        
        if (bind(socket_fd_, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
            LOG_ERROR("Failed to bind UDP socket to port %d: %s", 
                     config_.jetson_port, strerror(errno));
            ::close(socket_fd_);
            socket_fd_ = -1;
            return false;
        }
        
        LOG_INFO("UDP socket bound to local port: %d", config_.jetson_port);

        // 设置目标地址 (Jetson)
        memset(&server_addr_, 0, sizeof(server_addr_));
        server_addr_.sin_family = AF_INET;
        server_addr_.sin_port = htons(config_.jetson_port);
        if (inet_pton(AF_INET, config_.jetson_ip.c_str(), &server_addr_.sin_addr) <= 0) {
            LOG_ERROR("Invalid Jetson IP address: %s", config_.jetson_ip.c_str());
            ::close(socket_fd_);
            socket_fd_ = -1;
            return false;
        }

        // 设置接收超时
        if (config_.recv_timeout_ms > 0) {
            struct timeval tv;
            tv.tv_sec = config_.recv_timeout_ms / 1000;
            tv.tv_usec = (config_.recv_timeout_ms % 1000) * 1000;
            setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        }

        // 设置非阻塞模式
        if (!config_.enable_blocking) {
            int flags = fcntl(socket_fd_, F_GETFL, 0);
            fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);
        }

        initialized_ = true;
        LOG_INFO("UDP driver initialized: %s:%d", 
                 config_.jetson_ip.c_str(), config_.jetson_port);
        return true;
    }

    /**
     * @brief 关闭UDP连接
     */
    void close() {
        stop();
        if (socket_fd_ >= 0) {
            ::close(socket_fd_);
            socket_fd_ = -1;
        }
        initialized_ = false;
    }

    /**
     * @brief 发送观测数据到Jetson
     * @param request 观测数据
     * @return 发送字节数, -1表示失败
     */
    int send(const JetsonRequest& request) {
        if (!initialized_ || socket_fd_ < 0) {
            return -1;
        }

        socklen_t addr_len = sizeof(server_addr_);
        int sent = sendto(socket_fd_, &request, sizeof(JetsonRequest), 0,
                         (struct sockaddr*)&server_addr_, addr_len);
        
        if (sent > 0) {
            stats_.tx_count++;
            stats_.tx_bytes += sent;
        } else if (sent < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            stats_.tx_errors++;
            LOG_DEBUG("UDP send failed: %s", strerror(errno));
        }
        
        return sent;
    }

    /**
     * @brief 接收Jetson的动作数据
     * @param response 输出: 动作数据
     * @return 接收字节数, 0表示无数据, -1表示错误
     */
    int receive(JetsonResponse& response) {
        if (!initialized_ || socket_fd_ < 0) {
            return -1;
        }

        socklen_t addr_len = sizeof(server_addr_);
        int received = recvfrom(socket_fd_, &response, sizeof(JetsonResponse), 0,
                               (struct sockaddr*)&server_addr_, &addr_len);
        
        if (received > 0) {
            stats_.rx_count++;
            stats_.rx_bytes += received;
            last_response_ = response;
            last_response_time_us_ = get_time_us();
        } else if (received < 0) {
            // 检查是否为临时错误（无数据可读）
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return 0;  // 无数据可读，正常情况
            }
            // 其他错误（如socket已关闭）
            if (errno == EBADF || errno == ECONNRESET || errno == ENOTCONN) {
                return -1;  // Socket错误，应退出
            }
            stats_.rx_errors++;
            return 0;  // 其他错误，暂不退出
        }
        
        return received;
    }

    /**
     * @brief 同步发送并等待接收 (阻塞模式)
     * @param request 发送的观测数据
     * @param response 输出: 接收的动作数据
     * @param timeout_ms 超时时间 (ms)
     * @return 是否成功
     */
    bool send_receive(const JetsonRequest& request, JetsonResponse& response, 
                      int timeout_ms = 10) {
        if (send(request) < 0) {
            return false;
        }

        // 等待接收
        uint64_t start = get_time_us();
        while (get_time_us() - start < timeout_ms * 1000) {
            if (receive(response) > 0) {
                return true;
            }
            usleep(100);  // 等待100us后重试
        }
        
        stats_.timeouts++;
        return false;
    }

    /**
     * @brief 设置响应回调函数
     */
    void set_response_callback(ResponseCallback callback) {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        response_callback_ = callback;
    }

    /**
     * @brief 启动异步接收线程
     */
    bool start_async_receive() {
        if (!initialized_ || running_) {
            return false;
        }
        
        running_ = true;
        recv_thread_ = std::thread(&UDPDriver::receive_thread_func, this);
        LOG_INFO("UDP async receive thread started");
        return true;
    }

    /**
     * @brief 停止异步接收线程
     */
    void stop() {
        if (!running_) {
            return;  // 已经停止
        }
        
        running_ = false;
        
        // 强制关闭socket，让阻塞的recvfrom()立即返回EBADF错误
        if (socket_fd_ >= 0) {
            shutdown(socket_fd_, SHUT_RDWR);
        }
        
        // 等待线程退出（最多1秒）
        if (recv_thread_.joinable()) {
            // 使用future来实现带超时的join
            auto join_future = std::async(std::launch::async, [this]() {
                if (recv_thread_.joinable()) {
                    recv_thread_.join();
                }
            });
            
            if (join_future.wait_for(std::chrono::seconds(1)) == std::future_status::timeout) {
                LOG_WARN("UDP receive thread did not exit in 1 second, detaching...");
                recv_thread_.detach();
            }
        }
    }

    /**
     * @brief 获取最新的响应数据
     */
    JetsonResponse get_last_response() const {
        return last_response_;
    }

    /**
     * @brief 获取最后接收时间
     */
    uint64_t get_last_response_time() const {
        return last_response_time_us_;
    }

    /**
     * @brief 检查连接是否活跃
     * @param timeout_ms 超时判断阈值
     */
    bool is_connected(int timeout_ms = 100) const {
        if (!initialized_) return false;
        uint64_t now = get_time_us();
        return (now - last_response_time_us_) < timeout_ms * 1000;
    }

    /**
     * @brief 获取统计信息
     */
    struct Stats {
        uint64_t tx_count = 0;
        uint64_t rx_count = 0;
        uint64_t tx_bytes = 0;
        uint64_t rx_bytes = 0;
        uint64_t tx_errors = 0;
        uint64_t rx_errors = 0;
        uint64_t timeouts = 0;
    };

    Stats get_stats() const { return stats_; }

    void print_stats() const {
        LOG_INFO("=== UDP Communication Statistics ===");
        LOG_INFO("TX: %lu packets, %lu bytes, %lu errors",
                 stats_.tx_count, stats_.tx_bytes, stats_.tx_errors);
        LOG_INFO("RX: %lu packets, %lu bytes, %lu errors",
                 stats_.rx_count, stats_.rx_bytes, stats_.rx_errors);
        LOG_INFO("Timeouts: %lu", stats_.timeouts);
        if (stats_.tx_count > 0 && stats_.rx_count > 0) {
            float loss_rate = 1.0f - (float)stats_.rx_count / stats_.tx_count;
            LOG_INFO("Packet loss rate: %.2f%%", loss_rate * 100);
        }
    }

private:
    void receive_thread_func() {
        JetsonResponse response;
        while (running_) {
            int recv_result = receive(response);
            
            if (recv_result > 0) {
                std::lock_guard<std::mutex> lock(callback_mutex_);
                if (response_callback_) {
                    response_callback_(response);
                }
            } else if (recv_result < 0) {
                // Socket错误，可能已关闭
                if (!running_) {
                    break;  // 立即退出
                }
            }
            
            usleep(500);  // 2000Hz轮询
        }
        LOG_DEBUG("UDP receive thread exited");
    }

private:
    UDPConfig config_;
    int socket_fd_ = -1;
    struct sockaddr_in server_addr_;
    
    bool initialized_ = false;
    std::atomic<bool> running_{false};
    std::thread recv_thread_;
    
    std::mutex callback_mutex_;
    ResponseCallback response_callback_;
    
    JetsonResponse last_response_;
    std::atomic<uint64_t> last_response_time_us_{0};
    
    Stats stats_;
};

} // namespace odroid

#endif // ODROID_COMMUNICATION_UDP_DRIVER_HPP
