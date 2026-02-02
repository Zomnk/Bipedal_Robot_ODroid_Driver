/**
 * @file jetson_interface.hpp
 * @brief Jetson通信接口 - 数据转换与通信管理
 * @author Zomnk
 * @date 2026-02-02
 *
 * @note 负责:
 *   1. 将RobotFeedback转换为JetsonRequest (观测数据)
 *   2. 将JetsonResponse转换为RobotCommand (动作数据)
 *   3. 管理与Jetson的UDP通信
 */

#ifndef ODROID_CORE_JETSON_INTERFACE_HPP
#define ODROID_CORE_JETSON_INTERFACE_HPP

#include <atomic>
#include <mutex>
#include <thread>
#include <cmath>

#include "common/types.hpp"
#include "common/logger.hpp"
#include "common/time_utils.hpp"
#include "communication/udp_driver.hpp"
#include "communication/jetson_protocol.hpp"

namespace odroid {

/**
 * @brief Jetson接口配置
 */
struct JetsonInterfaceConfig {
    UDPConfig udp_config;
    
    // 控制参数 (PD增益)
    float kp = 3.0f;           // 位置增益
    float kd = 0.15f;          // 速度增益
    
    // 初始站立姿态 (rad)
    float init_pos[NUM_JOINTS] = {
        // 左腿: Yaw, Roll, Pitch, Knee, Ankle
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        // 右腿: Yaw, Roll, Pitch, Knee, Ankle
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f
    };
    
    // 通信频率
    int comm_rate_hz = 500;    // 与Jetson通信频率
};

/**
 * @brief Jetson通信接口类
 */
class JetsonInterface {
public:
    JetsonInterface() = default;
    ~JetsonInterface() {
        stop();
    }

    /**
     * @brief 初始化接口
     */
    bool init(const JetsonInterfaceConfig& config = JetsonInterfaceConfig()) {
        config_ = config;
        
        // 初始化UDP驱动
        if (!udp_driver_.init(config_.udp_config)) {
            LOG_ERROR("Failed to init UDP driver for Jetson");
            return false;
        }
        
        // 初始化init_pos
        for (int i = 0; i < NUM_JOINTS; i++) {
            current_init_pos_[i] = config_.init_pos[i];
        }
        
        initialized_ = true;
        LOG_INFO("Jetson interface initialized");
        return true;
    }

    /**
     * @brief 启动通信线程
     */
    bool start() {
        if (!initialized_ || running_) {
            return false;
        }
        
        running_ = true;
        // 启动异步接收
        udp_driver_.start_async_receive();
        
        LOG_INFO("Jetson interface started");
        return true;
    }

    /**
     * @brief 停止通信
     */
    void stop() {
        running_ = false;
        udp_driver_.stop();
        udp_driver_.close();
    }

    /**
     * @brief 发送机器人状态到Jetson
     * @param feedback 机器人反馈数据 (来自STM32)
     * @param command 用户控制指令 [vx, vy, yaw_rate, ...]
     */
    void send_observation(const RobotFeedback& feedback, const float* command = nullptr) {
        JetsonRequest request;
        
        // 转换反馈数据为Jetson请求格式
        convert_feedback_to_request(feedback, request);
        
        // 添加控制指令
        if (command) {
            for (int i = 0; i < NUM_CMD; i++) {
                request.command[i] = command[i];
            }
        }
        
        // 设置初始位置
        for (int i = 0; i < NUM_JOINTS; i++) {
            request.init_pos[i] = current_init_pos_[i];
        }
        
        request.trigger = 1.0f;  // 有效数据标志
        
        // 发送
        udp_driver_.send(request);
    }

    /**
     * @brief 获取Jetson的动作输出并转换为机器人指令
     * @param cmd 输出: 机器人控制指令
     * @return 是否有新数据
     */
    bool get_action(RobotCommand& cmd) {
        JetsonResponse response = udp_driver_.get_last_response();
        
        // 检查是否有有效数据
        if (!udp_driver_.is_connected(100)) {
            return false;
        }
        
        // 转换Jetson响应为机器人指令
        convert_response_to_command(response, cmd);
        
        return true;
    }

    /**
     * @brief 同步通信: 发送观测并等待动作
     * @param feedback 机器人反馈
     * @param command 用户控制指令
     * @param robot_cmd 输出: 机器人控制指令
     * @param timeout_ms 超时时间
     * @return 是否成功
     */
    bool sync_communicate(const RobotFeedback& feedback, 
                          const float* command,
                          RobotCommand& robot_cmd,
                          int timeout_ms = 10) {
        JetsonRequest request;
        JetsonResponse response;
        
        // 构建请求
        convert_feedback_to_request(feedback, request);
        if (command) {
            for (int i = 0; i < NUM_CMD; i++) {
                request.command[i] = command[i];
            }
        }
        for (int i = 0; i < NUM_JOINTS; i++) {
            request.init_pos[i] = current_init_pos_[i];
        }
        request.trigger = 1.0f;
        
        // 同步发送接收
        if (!udp_driver_.send_receive(request, response, timeout_ms)) {
            return false;
        }
        
        // 转换响应
        convert_response_to_command(response, robot_cmd);
        return true;
    }

    /**
     * @brief 设置初始站立位置
     */
    void set_init_pos(const float* init_pos) {
        for (int i = 0; i < NUM_JOINTS; i++) {
            current_init_pos_[i] = init_pos[i];
        }
    }

    /**
     * @brief 设置PD增益
     */
    void set_gains(float kp, float kd) {
        config_.kp = kp;
        config_.kd = kd;
    }

    /**
     * @brief 检查连接状态
     */
    bool is_connected() const {
        return udp_driver_.is_connected(100);
    }

    /**
     * @brief 打印统计信息
     */
    void print_stats() const {
        udp_driver_.print_stats();
    }

private:
    /**
     * @brief 将RobotFeedback转换为JetsonRequest
     */
    void convert_feedback_to_request(const RobotFeedback& feedback, JetsonRequest& request) {
        // 使用Waveshare IMU数据 (更稳定的10轴IMU)
        const IMUFeedback& imu = feedback.imu[1];  // [1] = Waveshare
        
        // 欧拉角 (rad)
        request.eu_ang[0] = imu.euler[0];  // roll
        request.eu_ang[1] = imu.euler[1];  // pitch
        request.eu_ang[2] = imu.euler[2];  // yaw
        
        // 角速度 (deg/s -> rad/s)
        request.omega[0] = imu.gyro[0] * M_PI / 180.0f;
        request.omega[1] = imu.gyro[1] * M_PI / 180.0f;
        request.omega[2] = imu.gyro[2] * M_PI / 180.0f;
        
        // 加速度 (g -> m/s^2)
        request.acc[0] = imu.accel[0] * 9.81f;
        request.acc[1] = imu.accel[1] * 9.81f;
        request.acc[2] = imu.accel[2] * 9.81f;
        
        // 左腿关节数据
        request.q[L_YAW]   = feedback.left_leg.yaw.position;
        request.q[L_ROLL]  = feedback.left_leg.roll.position;
        request.q[L_PITCH] = feedback.left_leg.pitch.position;
        request.q[L_KNEE]  = feedback.left_leg.knee.position;
        request.q[L_ANKLE] = feedback.left_leg.ankle.position;
        
        request.dq[L_YAW]   = feedback.left_leg.yaw.velocity;
        request.dq[L_ROLL]  = feedback.left_leg.roll.velocity;
        request.dq[L_PITCH] = feedback.left_leg.pitch.velocity;
        request.dq[L_KNEE]  = feedback.left_leg.knee.velocity;
        request.dq[L_ANKLE] = feedback.left_leg.ankle.velocity;
        
        request.tau[L_YAW]   = feedback.left_leg.yaw.torque;
        request.tau[L_ROLL]  = feedback.left_leg.roll.torque;
        request.tau[L_PITCH] = feedback.left_leg.pitch.torque;
        request.tau[L_KNEE]  = feedback.left_leg.knee.torque;
        request.tau[L_ANKLE] = feedback.left_leg.ankle.torque;
        
        // 右腿关节数据
        request.q[R_YAW]   = feedback.right_leg.yaw.position;
        request.q[R_ROLL]  = feedback.right_leg.roll.position;
        request.q[R_PITCH] = feedback.right_leg.pitch.position;
        request.q[R_KNEE]  = feedback.right_leg.knee.position;
        request.q[R_ANKLE] = feedback.right_leg.ankle.position;
        
        request.dq[R_YAW]   = feedback.right_leg.yaw.velocity;
        request.dq[R_ROLL]  = feedback.right_leg.roll.velocity;
        request.dq[R_PITCH] = feedback.right_leg.pitch.velocity;
        request.dq[R_KNEE]  = feedback.right_leg.knee.velocity;
        request.dq[R_ANKLE] = feedback.right_leg.ankle.velocity;
        
        request.tau[R_YAW]   = feedback.right_leg.yaw.torque;
        request.tau[R_ROLL]  = feedback.right_leg.roll.torque;
        request.tau[R_PITCH] = feedback.right_leg.pitch.torque;
        request.tau[R_KNEE]  = feedback.right_leg.knee.torque;
        request.tau[R_ANKLE] = feedback.right_leg.ankle.torque;
    }

    /**
     * @brief 将JetsonResponse转换为RobotCommand
     */
    void convert_response_to_command(const JetsonResponse& response, RobotCommand& cmd) {
        float kp = config_.kp;
        float kd = config_.kd;
        
        // 左腿
        cmd.left_leg.yaw.position   = response.q_exp[L_YAW];
        cmd.left_leg.yaw.velocity   = 0.0f;
        cmd.left_leg.yaw.kp = kp;
        cmd.left_leg.yaw.kd = kd;
        
        cmd.left_leg.roll.position  = response.q_exp[L_ROLL];
        cmd.left_leg.roll.velocity  = 0.0f;
        cmd.left_leg.roll.kp = kp;
        cmd.left_leg.roll.kd = kd;
        
        cmd.left_leg.pitch.position = response.q_exp[L_PITCH];
        cmd.left_leg.pitch.velocity = 0.0f;
        cmd.left_leg.pitch.kp = kp;
        cmd.left_leg.pitch.kd = kd;
        
        cmd.left_leg.knee.position  = response.q_exp[L_KNEE];
        cmd.left_leg.knee.velocity  = 0.0f;
        cmd.left_leg.knee.kp = kp;
        cmd.left_leg.knee.kd = kd;
        
        cmd.left_leg.ankle.position = response.q_exp[L_ANKLE];
        cmd.left_leg.ankle.velocity = 0.0f;
        cmd.left_leg.ankle.kp = kp;
        cmd.left_leg.ankle.kd = kd;
        
        // 右腿
        cmd.right_leg.yaw.position   = response.q_exp[R_YAW];
        cmd.right_leg.yaw.velocity   = 0.0f;
        cmd.right_leg.yaw.kp = kp;
        cmd.right_leg.yaw.kd = kd;
        
        cmd.right_leg.roll.position  = response.q_exp[R_ROLL];
        cmd.right_leg.roll.velocity  = 0.0f;
        cmd.right_leg.roll.kp = kp;
        cmd.right_leg.roll.kd = kd;
        
        cmd.right_leg.pitch.position = response.q_exp[R_PITCH];
        cmd.right_leg.pitch.velocity = 0.0f;
        cmd.right_leg.pitch.kp = kp;
        cmd.right_leg.pitch.kd = kd;
        
        cmd.right_leg.knee.position  = response.q_exp[R_KNEE];
        cmd.right_leg.knee.velocity  = 0.0f;
        cmd.right_leg.knee.kp = kp;
        cmd.right_leg.knee.kd = kd;
        
        cmd.right_leg.ankle.position = response.q_exp[R_ANKLE];
        cmd.right_leg.ankle.velocity = 0.0f;
        cmd.right_leg.ankle.kp = kp;
        cmd.right_leg.ankle.kd = kd;
        
        cmd.timestamp_us = get_time_us();
    }

private:
    JetsonInterfaceConfig config_;
    UDPDriver udp_driver_;
    
    bool initialized_ = false;
    std::atomic<bool> running_{false};
    
    float current_init_pos_[NUM_JOINTS] = {0};
};

} // namespace odroid

#endif // ODROID_CORE_JETSON_INTERFACE_HPP
