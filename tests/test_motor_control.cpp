/**
 * @file test_motor_control.cpp
 * @brief 电机手动控制测试程序
 * @author Zomnk
 * @date 2026-02-01
 * 
 * 交互式电机控制测试，用于调试阶段手动发送控制指令
 * 
 * 编译: mkdir build && cd build && cmake .. && make test_motor_control
 * 运行: sudo ./test_motor_control
 * 
 * 命令:
 *   h           - 显示帮助
 *   s           - 显示当前状态
 *   z           - 零位 (所有电机归零)
 *   e           - 紧急停止
 *   r           - 恢复运行
 *   m <id> <pos> [vel] [kp] [kd]  - 设置单个电机
 *   l <pos>     - 左腿所有关节设置相同位置
 *   q           - 退出
 * 
 * 电机ID:
 *   0-4: 左腿 (hip, thigh, calf, knee, ankle)
 *   5-9: 右腿 (hip, thigh, calf, knee, ankle)
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <thread>
#include <atomic>

#include "common/types.hpp"
#include "common/constants.hpp"
#include "common/logger.hpp"
#include "common/time_utils.hpp"
#include "realtime/rt_utils.hpp"
#include "realtime/rt_thread.hpp"
#include "communication/spi_driver.hpp"
#include "communication/protocol.hpp"
#include "core/data_hub.hpp"

using namespace odroid;

// 全局变量
static std::atomic<bool> g_running{true};
static RobotCommand g_cmd;
static RobotFeedback g_fb;
static std::mutex g_mutex;

void signal_handler(int sig) {
    (void)sig;
    g_running = false;
}

// 电机名称
const char* motor_names[] = {
    "L-Hip", "L-Thigh", "L-Calf", "L-Knee", "L-Ankle",
    "R-Hip", "R-Thigh", "R-Calf", "R-Knee", "R-Ankle"
};

// 获取电机指令引用
MotorCommand& get_motor_cmd(RobotCommand& cmd, int id) {
    switch (id) {
        case 0: return cmd.left_leg.hip;
        case 1: return cmd.left_leg.thigh;
        case 2: return cmd.left_leg.calf;
        case 3: return cmd.left_leg.knee;
        case 4: return cmd.left_leg.ankle;
        case 5: return cmd.right_leg.hip;
        case 6: return cmd.right_leg.thigh;
        case 7: return cmd.right_leg.calf;
        case 8: return cmd.right_leg.knee;
        case 9: return cmd.right_leg.ankle;
        default: return cmd.left_leg.hip;
    }
}

// 获取电机反馈引用
const MotorFeedback& get_motor_fb(const RobotFeedback& fb, int id) {
    switch (id) {
        case 0: return fb.left_leg.hip;
        case 1: return fb.left_leg.thigh;
        case 2: return fb.left_leg.calf;
        case 3: return fb.left_leg.knee;
        case 4: return fb.left_leg.ankle;
        case 5: return fb.right_leg.hip;
        case 6: return fb.right_leg.thigh;
        case 7: return fb.right_leg.calf;
        case 8: return fb.right_leg.knee;
        case 9: return fb.right_leg.ankle;
        default: return fb.left_leg.hip;
    }
}

void print_help() {
    printf("\n=== Motor Control Commands ===\n");
    printf("  h                           - Show this help\n");
    printf("  s                           - Show current state\n");
    printf("  z                           - Zero all motors\n");
    printf("  e                           - Emergency stop\n");
    printf("  r                           - Resume operation\n");
    printf("  m <id> <pos> [vel] [kp] [kd] - Set motor (id: 0-9)\n");
    printf("  l <pos>                     - Set all left leg joints\n");
    printf("  w                           - Wave test (sine motion)\n");
    printf("  q                           - Quit\n");
    printf("\nMotor IDs:\n");
    printf("  0-4: Left leg  (hip, thigh, calf, knee, ankle)\n");
    printf("  5-9: Right leg (hip, thigh, calf, knee, ankle)\n");
    printf("\n");
}

void print_state() {
    std::lock_guard<std::mutex> lock(g_mutex);
    
    printf("\n=== Current State ===\n");
    printf("%-10s | %-8s %-8s %-6s %-6s | %-8s %-8s %-8s %-6s\n",
           "Motor", "Cmd Pos", "Cmd Vel", "Kp", "Kd",
           "Fb Pos", "Fb Vel", "Torque", "Temp");
    printf("-----------|-------------------------------|------------------------------------\n");
    
    for (int i = 0; i < 10; i++) {
        const MotorCommand& cmd = get_motor_cmd(g_cmd, i);
        const MotorFeedback& fb = get_motor_fb(g_fb, i);
        
        printf("%-10s | %8.3f %8.3f %6.1f %6.2f | %8.3f %8.3f %8.3f %6.1f\n",
               motor_names[i],
               cmd.position, cmd.velocity, cmd.kp, cmd.kd,
               fb.position, fb.velocity, fb.torque, fb.temperature);
    }
    
    printf("\nIMU[0]: accel=[%.2f, %.2f, %.2f], gyro=[%.1f, %.1f, %.1f]\n",
           g_fb.imu[0].accel[0], g_fb.imu[0].accel[1], g_fb.imu[0].accel[2],
           g_fb.imu[0].gyro[0], g_fb.imu[0].gyro[1], g_fb.imu[0].gyro[2]);
    printf("IMU[0]: euler=[%.1f, %.1f, %.1f]\n",
           g_fb.imu[0].euler[0], g_fb.imu[0].euler[1], g_fb.imu[0].euler[2]);
}

void zero_all_motors() {
    std::lock_guard<std::mutex> lock(g_mutex);
    g_cmd = RobotCommand{};
    
    // 设置默认PD参数
    for (int i = 0; i < 10; i++) {
        MotorCommand& cmd = get_motor_cmd(g_cmd, i);
        cmd.position = 0.0f;
        cmd.velocity = 0.0f;
        cmd.kp = 30.0f;
        cmd.kd = 1.0f;
    }
    
    DataHub::instance().set_command(g_cmd);
    printf("All motors set to zero position\n");
}

void set_motor(int id, float pos, float vel, float kp, float kd) {
    if (id < 0 || id > 9) {
        printf("Invalid motor ID: %d (must be 0-9)\n", id);
        return;
    }
    
    std::lock_guard<std::mutex> lock(g_mutex);
    MotorCommand& cmd = get_motor_cmd(g_cmd, id);
    cmd.position = pos;
    cmd.velocity = vel;
    cmd.kp = kp;
    cmd.kd = kd;
    
    DataHub::instance().set_command(g_cmd);
    printf("Motor %d (%s): pos=%.3f, vel=%.3f, kp=%.1f, kd=%.2f\n",
           id, motor_names[id], pos, vel, kp, kd);
}

void set_left_leg(float pos) {
    std::lock_guard<std::mutex> lock(g_mutex);
    
    for (int i = 0; i < 5; i++) {
        MotorCommand& cmd = get_motor_cmd(g_cmd, i);
        cmd.position = pos;
        cmd.velocity = 0.0f;
        cmd.kp = 30.0f;
        cmd.kd = 1.0f;
    }
    
    DataHub::instance().set_command(g_cmd);
    printf("Left leg all joints set to position: %.3f\n", pos);
}

// SPI通信线程
void spi_thread_func(SPIDriver& spi) {
    PeriodicTimer timer(SPI_PERIOD_US);
    
    while (g_running) {
        // 获取控制指令
        RobotCommand cmd;
        {
            std::lock_guard<std::mutex> lock(g_mutex);
            cmd = g_cmd;
        }
        
        // 检查紧急停止
        if (DataHub::instance().is_emergency_stop()) {
            cmd = RobotCommand{};
        }
        
        // 编码
        SPITxBuffer tx_buf;
        Protocol::encode_robot_cmd(cmd, tx_buf);
        
        // SPI传输
        SPIRxBuffer rx_buf;
        if (spi.transfer(tx_buf, rx_buf)) {
            // 解码
            RobotFeedback fb;
            Protocol::decode_robot_fb(rx_buf, fb);
            
            // 更新全局反馈
            {
                std::lock_guard<std::mutex> lock(g_mutex);
                g_fb = fb;
            }
            
            DataHub::instance().set_feedback(fb);
        }
        
        timer.wait();
    }
}

// 波形测试
void wave_test(SPIDriver& spi) {
    printf("Starting wave test... Press Enter to stop\n");
    
    PeriodicTimer timer(10000);  // 100Hz
    uint64_t t = 0;
    
    while (g_running) {
        // 检查是否有输入
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(0, &fds);  // stdin
        struct timeval tv = {0, 0};
        if (select(1, &fds, nullptr, nullptr, &tv) > 0) {
            getchar();  // 消耗输入
            break;
        }
        
        // 正弦波
        float phase = t * 0.001;  // 1Hz
        float pos = 0.3f * sin(2.0 * PI * phase);
        
        {
            std::lock_guard<std::mutex> lock(g_mutex);
            // 只控制左腿髋关节
            g_cmd.left_leg.hip.position = pos;
            g_cmd.left_leg.hip.velocity = 0.0f;
            g_cmd.left_leg.hip.kp = 30.0f;
            g_cmd.left_leg.hip.kd = 1.0f;
        }
        
        DataHub::instance().set_command(g_cmd);
        
        // 显示进度
        if (t % 100 == 0) {
            printf("\rWave: t=%.2f, pos=%.3f", phase, pos);
            fflush(stdout);
        }
        
        t++;
        timer.wait();
    }
    
    printf("\nWave test stopped\n");
}

int main(int argc, char* argv[]) {
    // 解析参数
    SPIConfig config;
    if (argc > 1) {
        config.device = argv[1];
    }

    Logger::instance().set_level(LogLevel::INFO);
    
    LOG_INFO("=== Motor Control Test ===");
    LOG_INFO("SPI Device: %s", config.device.c_str());

    // 信号处理
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // 检查权限
    if (!check_spi_permission(config.device.c_str())) {
        LOG_WARN("Try running with sudo");
    }

    // 打开SPI设备
    SPIDriver spi;
    if (!spi.open(config)) {
        LOG_ERROR("Failed to open SPI device");
        return 1;
    }

    // 初始化默认指令
    zero_all_motors();

    // 启动SPI通信线程
    std::thread spi_thread(spi_thread_func, std::ref(spi));

    // 主循环 - 命令行交互
    print_help();
    
    char line[256];
    while (g_running) {
        printf("> ");
        fflush(stdout);
        
        if (!fgets(line, sizeof(line), stdin)) {
            break;
        }
        
        // 解析命令
        char cmd;
        if (sscanf(line, " %c", &cmd) != 1) {
            continue;
        }
        
        switch (cmd) {
            case 'h':
                print_help();
                break;
                
            case 's':
                print_state();
                break;
                
            case 'z':
                zero_all_motors();
                break;
                
            case 'e':
                DataHub::instance().set_emergency_stop(true);
                printf("EMERGENCY STOP!\n");
                break;
                
            case 'r':
                DataHub::instance().set_emergency_stop(false);
                printf("Resumed\n");
                break;
                
            case 'm': {
                int id;
                float pos = 0, vel = 0, kp = 30, kd = 1;
                int n = sscanf(line, " m %d %f %f %f %f", &id, &pos, &vel, &kp, &kd);
                if (n >= 2) {
                    set_motor(id, pos, vel, kp, kd);
                } else {
                    printf("Usage: m <id> <pos> [vel] [kp] [kd]\n");
                }
                break;
            }
                
            case 'l': {
                float pos;
                if (sscanf(line, " l %f", &pos) == 1) {
                    set_left_leg(pos);
                } else {
                    printf("Usage: l <pos>\n");
                }
                break;
            }
                
            case 'w':
                wave_test(spi);
                break;
                
            case 'q':
                g_running = false;
                break;
                
            default:
                printf("Unknown command: %c\n", cmd);
                break;
        }
    }

    // 停止
    g_running = false;
    spi_thread.join();
    spi.close();

    LOG_INFO("Test completed");
    return 0;
}
