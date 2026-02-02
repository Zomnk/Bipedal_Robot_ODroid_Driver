/**
 * @file jetson_protocol.hpp
 * @brief ODroid与Jetson之间的UDP通信协议定义
 * @author Zomnk
 * @date 2026-02-02
 *
 * @note 与sim2sim_lcm项目中的协议保持一致
 * 通信架构:
 *   ODroid (观测数据采集) ----UDP----> Jetson (RL推理)
 *   ODroid (动作执行)     <---UDP---- Jetson (动作输出)
 */

#ifndef ODROID_COMMUNICATION_JETSON_PROTOCOL_HPP
#define ODROID_COMMUNICATION_JETSON_PROTOCOL_HPP

#include <cstdint>
#include <cstring>

namespace odroid {

//==============================================================================
// 常量定义
//==============================================================================

constexpr int NUM_JOINTS = 10;           // 双足机器人10个关节 (5*2腿)
constexpr int NUM_CMD = 4;               // 控制指令数 (vx, vy, yaw_rate, ...)
constexpr int JETSON_DEFAULT_PORT = 10000;
constexpr const char* JETSON_DEFAULT_IP = "192.168.1.10";  // Jetson默认IP

//==============================================================================
// 消息结构定义 (与sim2sim_lcm/udp_publish_tinker.h一致)
//==============================================================================

/**
 * @brief 观测数据 (ODroid -> Jetson)
 * @note 发送给Jetson进行RL推理的输入数据
 */
#pragma pack(push, 1)
struct JetsonRequest {
    float trigger;          // 触发标志 (1=有效数据)
    float command[NUM_CMD]; // 控制指令 [vx, vy, yaw_rate, ...]
    float eu_ang[3];        // 欧拉角 [roll, pitch, yaw] (rad)
    float omega[3];         // 角速度 [x, y, z] (rad/s)
    float acc[3];           // 加速度 [x, y, z] (m/s^2)
    float q[NUM_JOINTS];    // 关节位置 (rad)
    float dq[NUM_JOINTS];   // 关节速度 (rad/s)
    float tau[NUM_JOINTS];  // 关节力矩 (Nm)
    float init_pos[NUM_JOINTS];  // 初始位置 (rad)
    
    JetsonRequest() {
        memset(this, 0, sizeof(JetsonRequest));
        trigger = 1.0f;  // 默认有效
    }
};
#pragma pack(pop)

/**
 * @brief 动作数据 (Jetson -> ODroid)
 * @note Jetson RL推理输出的关节目标值
 */
#pragma pack(push, 1)
struct JetsonResponse {
    float q_exp[NUM_JOINTS];    // 期望关节位置 (rad)
    float dq_exp[NUM_JOINTS];   // 期望关节速度 (rad/s) - 预留
    float tau_exp[NUM_JOINTS];  // 期望力矩 (Nm) - 预留
    
    JetsonResponse() {
        memset(this, 0, sizeof(JetsonResponse));
    }
};
#pragma pack(pop)

//==============================================================================
// 关节映射 (ODroid腿部结构 <-> RL模型关节顺序)
//==============================================================================

/**
 * @brief 关节索引定义
 * @note 双足机器人关节顺序: 左腿5个 + 右腿5个
 *       每条腿: Yaw, Roll, Pitch, Knee, Ankle
 */
enum JointIndex {
    // 左腿
    L_YAW = 0,
    L_ROLL = 1,
    L_PITCH = 2,
    L_KNEE = 3,
    L_ANKLE = 4,
    // 右腿
    R_YAW = 5,
    R_ROLL = 6,
    R_PITCH = 7,
    R_KNEE = 8,
    R_ANKLE = 9
};

/**
 * @brief 默认站立姿态
 * @note 可根据实际机器人调整
 */
constexpr float DEFAULT_INIT_POS[NUM_JOINTS] = {
    // 左腿: Yaw, Roll, Pitch, Knee, Ankle
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    // 右腿: Yaw, Roll, Pitch, Knee, Ankle
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f
};

} // namespace odroid

#endif // ODROID_COMMUNICATION_JETSON_PROTOCOL_HPP
