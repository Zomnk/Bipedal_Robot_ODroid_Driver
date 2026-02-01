# ODroid-C4 Robot Driver

基于ODroid-C4的双足机器人上位机驱动程序，通过SPI与STM32通信。

## 系统要求

- **硬件**: ODroid-C4
- **操作系统**: Ubuntu 20.04 Server
- **内核**: Linux 5.10+ (推荐RT-PREEMPT补丁)
- **编译器**: GCC 9.4.0+
- **CMake**: 3.16+

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
│   │   └── protocol.hpp        # 通信协议
│   └── core/
│       ├── data_hub.hpp        # 数据交换中心
│       └── robot_interface.hpp # 机器人接口
├── src/
│   └── main.cpp                # 主程序
├── tests/
│   ├── test_spi_comm.cpp       # SPI通信测试
│   ├── test_motor_control.cpp  # 电机控制测试
│   └── test_rt_timing.cpp      # 实时性测试
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
# 实时性测试
sudo ./build-debug/test_rt_timing

# SPI通信测试
sudo ./build-debug/test_spi_comm

# 电机手动控制测试
sudo ./build-debug/test_motor_control
```

### 4. 运行主程序

```bash
sudo ./build-debug/robot_driver
```

## SPI连接

ODroid-C4 SPI引脚:
- MOSI: Pin 19
- MISO: Pin 21
- SCLK: Pin 23
- CS0:  Pin 24

STM32 SPI配置:
- 16位数据宽度
- MSB优先
- CPOL=0, CPHA=0 (Mode 0)
- 10MHz时钟

## 数据格式

### 控制数据 (ODroid → STM32)

| 偏移 | 内容 | 数量 |
|------|------|------|
| 0-19 | 左腿5关节 × 4参数 | 20 × uint16 |
| 20-39 | 右腿5关节 × 4参数 | 20 × uint16 |

每个关节: [position, velocity, kp, kd]

### 反馈数据 (STM32 → ODroid)

| 偏移 | 内容 | 数量 |
|------|------|------|
| 0-15 | 左腿4关节 × 4参数 | 16 × uint16 |
| 16-31 | 右腿4关节 × 4参数 | 16 × uint16 |
| 32-41 | IMU0数据 | 10 × uint16 |
| 42-51 | IMU1数据 | 10 × uint16 |
| 52-59 | 保留 | 8 × uint16 |

每个关节: [position, velocity, torque, temperature]

### 编码规则

浮点数 [-max, +max] ↔ uint16 [0, 65535]

```
encode: uint16 = (value + max) / (2 * max) * 65535
decode: value = uint16 / 65535 * 2 * max - max
```

## 电机参数

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

### 1. 检查SPI设备

```bash
ls -la /dev/spidev*
```

### 2. 检查RT权限

```bash
ulimit -r  # 应显示99
```

### 3. 监控延迟

```bash
sudo ./test_rt_timing 1000 10000  # 1ms周期，10000次
```

### 4. 手动电机控制

```bash
sudo ./test_motor_control
# 命令:
#   m 0 0.5       # 左腿髋关节设置到0.5rad
#   s             # 显示状态
#   z             # 零位
#   e             # 紧急停止
```

## 故障排除

### SPI传输失败
1. 检查连线
2. 检查CS信号
3. 降低SPI速度: `-s 1000000`

### 错过deadline
1. 检查是否为RT内核
2. 检查CPU隔离配置
3. 检查其他高优先级进程

### 权限问题
1. 运行 `setup_rt_permissions.sh`
2. 重新登录
3. 使用 `sudo`

## License

MIT License
