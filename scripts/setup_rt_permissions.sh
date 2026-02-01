#!/bin/bash
#
# setup_rt_permissions.sh
# 设置实时优先级和SPI访问权限
#
# 使用方法:
#   chmod +x setup_rt_permissions.sh
#   sudo ./setup_rt_permissions.sh
#

set -e

echo "=== ODroid RT Permission Setup ==="

# 获取当前用户
if [ -n "$SUDO_USER" ]; then
    TARGET_USER=$SUDO_USER
else
    TARGET_USER=$(whoami)
fi

echo "Setting up permissions for user: $TARGET_USER"

# 1. 创建realtime组 (如果不存在)
if ! getent group realtime > /dev/null 2>&1; then
    echo "Creating 'realtime' group..."
    groupadd realtime
fi

# 2. 将用户添加到realtime组
echo "Adding $TARGET_USER to 'realtime' group..."
usermod -a -G realtime $TARGET_USER

# 3. 将用户添加到spi组 (如果存在)
if getent group spi > /dev/null 2>&1; then
    echo "Adding $TARGET_USER to 'spi' group..."
    usermod -a -G spi $TARGET_USER
fi

# 4. 配置limits.conf
LIMITS_FILE="/etc/security/limits.d/99-realtime.conf"
echo "Configuring $LIMITS_FILE..."

cat > $LIMITS_FILE << 'EOF'
# Real-time priority limits for ODroid robot driver
@realtime   soft    rtprio          99
@realtime   hard    rtprio          99
@realtime   soft    memlock         unlimited
@realtime   hard    memlock         unlimited
@realtime   soft    nice            -20
@realtime   hard    nice            -20
EOF

echo "Limits configured:"
cat $LIMITS_FILE

# 5. 配置udev规则 (SPI访问)
UDEV_FILE="/etc/udev/rules.d/99-spi.rules"
echo "Configuring $UDEV_FILE..."

cat > $UDEV_FILE << 'EOF'
# SPI device permissions for ODroid
SUBSYSTEM=="spidev", GROUP="spi", MODE="0660"
SUBSYSTEM=="spidev", GROUP="realtime", MODE="0660"
EOF

# 重新加载udev规则
udevadm control --reload-rules
udevadm trigger

echo "SPI udev rules configured"

# 6. 检查RT内核
echo ""
echo "=== System Check ==="

if [ -f /sys/kernel/realtime ]; then
    RT_ENABLED=$(cat /sys/kernel/realtime)
    if [ "$RT_ENABLED" == "1" ]; then
        echo "[OK] RT-PREEMPT kernel detected"
    else
        echo "[WARN] RT-PREEMPT kernel not enabled"
    fi
else
    echo "[WARN] Not a RT-PREEMPT kernel"
    echo "       Consider installing: linux-image-*-rt-arm64"
fi

# 7. 检查CPU隔离
CMDLINE=$(cat /proc/cmdline)
if echo "$CMDLINE" | grep -q "isolcpus"; then
    echo "[OK] CPU isolation configured"
else
    echo "[WARN] CPU isolation not configured"
    echo "       Consider adding 'isolcpus=2,3' to kernel cmdline"
fi

# 8. 检查SPI设备
echo ""
echo "=== SPI Devices ==="
ls -la /dev/spidev* 2>/dev/null || echo "No SPI devices found"

# 9. 设置CPU调度器为performance
echo ""
echo "=== CPU Governor ==="
for cpu in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
    if [ -f "$cpu" ]; then
        echo "performance" > "$cpu" 2>/dev/null || true
    fi
done
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || echo "N/A"

echo ""
echo "=== Setup Complete ==="
echo ""
echo "IMPORTANT: Please log out and log back in for group changes to take effect!"
echo ""
echo "To verify setup after re-login:"
echo "  1. Check groups: groups"
echo "  2. Check rtprio: ulimit -r"
echo "  3. Test SPI access: ls -la /dev/spidev0.0"
echo ""
