# ODroid-C4 Robot Driver

基于ODroid-C4的双足机器人上位机驱动程序，负责：
- **SPI通信**：与STM32通信（1kHz），控制10个电机和读取IMU数据
- **UDP通信**：与Jetson通信（500Hz），转发观测数据和接收动作指令
- **数据桥接**：Jetson ↔ ODroid ↔ STM32 完整数据流
- **实时性保证**：RT线程、CPU隔离、内存锁定

## 系统要求

- **硬件**: ODroid-C4
- **操作系统**: Ubuntu 20.04 Server
- **内核**: Linux 5.10+ (推荐RT-PREEMPT补丁)
- **编译器**: GCC 9.4.0+, C++17
- **CMake**: 3.16+
- **网络**: 与Jetson同一局域网

## 目录结构

```
ODroid/
├── CMakeLists.txt              # CMake构建配置
├── README.md                   # 本文件
├── include/
│   ├── common/
│   │   ├── types.hpp           # 数据类型定义
│   │   ├── constants.hpp       # 系统常量
│   │   ├── logger.hpp          # 日志系统
│   │   └── time_utils.hpp      # 时间工具
│   ├── realtime/
│   │   ├── rt_utils.hpp        # RT工具函数
│   │   ├── rt_thread.hpp       # RT线程封装
│   │   └── lock_free_buffer.hpp # 无锁缓冲区
│   ├── communication/
│   │   ├── spi_driver.hpp      # SPI驱动
│   │   ├── udp_driver.hpp      # UDP驱动（与Jetson通信）
│   │   ├── protocol.hpp        # SPI通信协议
│   │   └── jetson_protocol.hpp # UDP通信协议
│   └── core/
│       ├── data_hub.hpp        # 数据交换中心
│       ├── robot_interface.hpp # 机器人接口（SPI）
│       └── jetson_interface.hpp # Jetson接口（UDP）
├── src/
│   └── main.cpp                # 主程序（数据桥接）
├── tests/
│   ├── test_spi_comm.cpp       # SPI通信测试
│   ├── test_motor_control.cpp  # 电机控制测试
│   ├── test_rt_timing.cpp      # 实时性测试
│   ├── test_jetson_comm.cpp    # UDP通信测试
│   └── test_full_loop.cpp      # 完整数据流测试
└── scripts/
    ├── setup_rt_permissions.sh # RT权限配置
    └── build.sh                # 编译脚本
```

## 快速开始

### 1. 配置权限

```bash
cd scripts
chmod +x setup_rt_permissions.sh
sudo ./setup_rt_permissions.sh
# 重新登录以生效
```

### 2. 编译

```bash
chmod +x scripts/build.sh
./scripts/build.sh

# 或手动编译
mkdir build && cd build
cmake ..
make -j4
```

### 3. 运行测试

```bash
cd build

# 1. SPI通信测试（与STM32）
sudo ./test_spi_comm

# 2. 实时性能测试
sudo ./test_rt_timing

# 3. 电机手动控制测试
sudo ./test_motor_control

# 4. UDP通信测试（与Jetson）
sudo ./test_jetson_comm 192.168.5.141 10000

# 5. 完整数据流测试（Jetson-ODroid-STM32）
#    注意：先在Jetson端运行 test_motors
sudo ./test_full_loop 192.168.5.141 10000
```

### 4. 运行主程序

```bash
# 生产环境：使用robot_driver（推荐）
sudo ./robot_driver 192.168.5.141 10000
#                    ↑ Jetson IP   ↑ UDP端口

# 开发/调试：使用test_full_loop
sudo ./test_full_loop 192.168.5.141 10000
```

**主程序功能**：
- ✅ 自动数据桥接（STM32 ↔ ODroid ↔ Jetson）
- ✅ 每秒打印详细状态（关节、IMU、通信统计）
- ✅ 支持渐进式标定模式
- ✅ 实时监控和错误检测

## 通信协议

### SPI通信（ODroid ↔ STM32）

**频率**: 1kHz  
**设备**: `/dev/spidev0.0`  
**速度**: 10MHz  
**模式**: Mode 0 (CPOL=0, CPHA=0)

#### SPI引脚连接

ODroid-C4 → STM32:
- MOSI: Pin 19
- MISO: Pin 21
- SCLK: Pin 23
- CS0:  Pin 24

#### 控制数据 (ODroid → STM32)

| 偏移 | 内容 | 数量 |
|------|------|------|
| 0-19 | 左腿5关节 × 4参数 | 20 × uint16 |
| 20-39 | 右腿5关节 × 4参数 | 20 × uint16 |

每个关节: [position, velocity, kp, kd]

#### 反馈数据 (STM32 → ODroid)

| 偏移 | 内容 | 数量 |
|------|------|------|
| 0-14 | 左腿5关节 × 3参数 | 15 × uint16 |
| 15-29 | 右腿5关节 × 3参数 | 15 × uint16 |
| 30-39 | IMU0数据 | 10 × uint16 |
| 40-49 | IMU1数据（Waveshare） | 10 × uint16 |
| 50-59 | 保留 | 10 × uint16 |

每个关节: [position, velocity, torque]  
每个IMU: [euler_x, euler_y, euler_z, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, 保留]

### UDP通信（ODroid ↔ Jetson）

**频率**: 500Hz  
**端口**: 10000 (默认)  
**协议**: 固定长度二进制包

#### JetsonRequest（观测数据：ODroid → Jetson）

```cpp
struct JetsonRequest {  // 216 bytes
    float omega[3];      // 角速度 (rad/s)
    float eu_ang[3];     // 欧拉角 (rad)
    float q[10];         // 关节位置 (rad)
    float dq[10];        // 关节速度 (rad/s)
    float init_pos[10];  // 初始位置 (rad)
    float trigger;       // 触发标志
    float command[4];    // 用户指令 [vx, vy, yaw_rate, 保留]
    float acc[3];        // 加速度 (g)
    float tau[10];       // 力矩反馈 (Nm)
};
```

#### JetsonResponse（动作指令：Jetson → ODroid）

```cpp
struct JetsonResponse {  // 120 bytes
    float q_exp[10];     // 期望位置 (rad)
    float dq_exp[10];    // 期望速度 (rad/s)
    float tau_exp[10];   // 期望力矩 (Nm)
};
```

**特殊模式**：
- **标定模式**: `dq_exp[0] = -999.0`, `tau_exp[0] = joint_id (0-9)`
  - 只对当前标定关节设置 kp=0, kd=0（卸载扭矩）
  - 其他已标定关节保持在标定位置

### 编码规则

浮点数 [-max, +max] ↔ uint16 [0, 65535]

```cpp
encode: uint16 = (value + max) / (2 * max) * 65535
decode: value = uint16 / 65535 * 2 * max - max
```

## 数据流架构

```
┌─────────┐  UDP(500Hz)  ┌─────────┐  SPI(1kHz)   ┌─────────┐
│ Jetson  │ ←─────────→  │ ODroid  │ ←─────────→  │ STM32   │
│  (AI)   │   动作/观测   │ (桥接)  │   指令/反馈   │ (控制)  │
└─────────┘              └─────────┘              └─────────┘
     ↓                        ↓                        ↓
  策略网络               数据转换/              10个电机控制
  推理计算               协议桥接                2个IMU读取
```

**线程模型**：
- **主线程**：监控和统计（10Hz）
- **SPI线程**：实时线程，RT优先级90，1kHz，绑定CPU核心2
- **UDP接收线程**：普通线程，500Hz异步接收

## 电机配置

### 关节布局

```
左腿 (0-4):  Yaw, Roll, Pitch, Knee, Ankle
右腿 (5-9):  Yaw, Roll, Pitch, Knee, Ankle
```

### 电机参数

| 型号 | 位置(rad) | 速度(rad/s) | 力矩(Nm) | Kp | Kd |
|------|-----------|-------------|----------|-----|-----|
| DM6006 | ±12.5 | ±45.0 | ±18.0 | 500 | 5.0 |
| DM4340 | ±12.5 | ±30.0 | ±10.0 | 500 | 5.0 |
| DM8006 | ±12.5 | ±45.0 | ±40.0 | 500 | 5.0 |

## 实时性优化

### 1. 安装RT内核

```bash
# ODroid-C4
sudo apt install linux-image-5.10.0-odroid-rt-arm64
```

### 2. CPU隔离

编辑 `/boot/boot.ini`:
```
# 添加到bootargs
isolcpus=2,3 nohz_full=2,3 rcu_nocbs=2,3
```

### 3. 禁用CPU调频

```bash
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
```

## 调试技巧

### 1. 检查网络连接

```bash
# 查看ODroid IP
ifconfig

# 测试与Jetson连通性
ping 192.168.5.141

# 检查端口占用
sudo netstat -tulnp | grep 10000

# 抓包分析UDP通信
sudo tcpdump -i any -n udp port 10000 -X
```

### 2. 检查SPI设备

```bash
ls -la /dev/spidev*
```

### 3. 检查RT权限

```bash
ulimit -r  # 应显示99
```

### 4. 监控延迟

```bash
sudo ./test_rt_timing 1000 10000  # 1ms周期，10000次
```

### 5. 手动电机控制

```bash
sudo ./test_motor_control
# 命令:
#   m 0 0.5       # 左腿髋关节设置到0.5rad
#   s             # 显示状态
#   z             # 零位
#   e             # 紧急停止
```

### 6. 查看实时日志

```bash
# 主程序输出示例
sudo ./robot_driver 192.168.5.141 10000

# 预期输出：
# 开始数据交换，等待Jetson连接...
# ✓ Jetson已连接！数据交换已建立
# 运行时间: 1.0 秒 | 主循环: 500
# 通信计数: 发送=500, 接收=450, 桥接=450
# 左腿关节状态: ...
```

## 故障排除

### UDP通信无法连接

**症状**：ODroid 和 Jetson 无法建立连接

**检查清单**：
1. **网络连通性**
   ```bash
   ping 192.168.5.141  # 从ODroid ping Jetson
   ```
2. **IP地址正确性**
   - ODroid 启动时传入 **Jetson IP**
   - Jetson 启动时传入 **ODroid IP**
3. **端口占用**
   ```bash
   sudo netstat -tulnp | grep 10000
   # 如果被占用，kill 相关进程或更换端口
   ```
4. **防火墙设置**
   ```bash
   sudo ufw status
   sudo ufw allow 10000/udp
   ```
5. **启动顺序**
   - **推荐**：先启动 ODroid（主动发送数据）
   - 再启动 Jetson（接收并响应）

**详细排查**：参考 `UDP_CONNECTION_TROUBLESHOOTING.md`

### 连接建立很慢

**原因**：可能使用了旧版本的 `main.cpp`，包含等待循环

**解决**：确保使用最新版本（无等待阶段，立即开始数据交换）

### SPI传输失败

1. 检查连线
2. 检查CS信号
3. 降低SPI速度: `-s 1000000`
4. 检查STM32是否正常运行

### 错过deadline

1. 检查是否为RT内核
   ```bash
   uname -a | grep PREEMPT
   ```
2. 检查CPU隔离配置
   ```bash
   cat /proc/cmdline | grep isolcpus
   ```
3. 检查其他高优先级进程
   ```bash
   ps -eLo pid,tid,class,rtprio,ni,pri,psr,pcpu,stat,comm | grep FF
   ```

### 权限问题

1. 运行 `setup_rt_permissions.sh`
2. 重新登录
3. 使用 `sudo` 运行程序
4. 检查 `/etc/security/limits.conf`

### 数据异常

1. **观测数据全为零**：检查SPI连接和STM32状态
2. **动作指令未生效**：检查Jetson是否正常发送
3. **IMU数据异常**：检查IMU接线和STM32固件

## 重要更新日志

### 2026-02-04
- ✅ 修复 UDP 通信 bug（ODroid 端缺少 `bind()` 调用）
- ✅ 优化 `main.cpp` 启动流程，删除等待阶段，加快连接速度
- ✅ 添加数据桥接逻辑，实现 Jetson-ODroid-STM32 完整数据流
- ✅ 实现渐进式标定功能（只卸载当前标定关节的扭矩）
- ✅ 添加详细的状态监控输出（每秒打印关节、IMU、通信统计）
- ✅ 创建 `UDP_CONNECTION_TROUBLESHOOTING.md` 故障排查文档

## 最佳实践

### 生产环境运行

```bash
# 1. 启动ODroid主程序
sudo ./robot_driver 192.168.5.141 10000

# 2. 在Jetson端启动RL策略或测试程序
# （在Jetson上运行）

# 3. 观察ODroid输出，确认连接建立
# ✓ Jetson已连接！数据交换已建立

# 4. 监控关节状态和通信统计
```

### 开发调试

```bash
# 使用test_full_loop.cpp（更详细的日志）
sudo ./test_full_loop 192.168.5.141 10000

# 优点：
# - 每500ms打印统计信息
# - 显示最后接收的动作
# - 显示电机实际位置
```

### 关节标定

```bash
# 在Jetson端运行标定程序
./calibrate_robot

# ODroid端使用任一程序即可（自动支持标定模式）
sudo ./robot_driver 192.168.5.141 10000
# 或
sudo ./test_full_loop 192.168.5.141 10000
```

## 相关文档

- [UDP 连接故障排查](UDP_CONNECTION_TROUBLESHOOTING.md)
- [SPI 通信协议](include/communication/protocol.hpp)
- [Jetson UDP 协议](include/communication/jetson_protocol.hpp)

## 技术支持

如遇到问题，请提供以下信息：
1. ODroid 和 Jetson 的 IP 地址
2. `ping` 测试结果
3. 端口占用情况（`netstat`）
4. 完整的启动命令和输出
5. `tcpdump` 抓包结果（如有）

## License

MIT License
