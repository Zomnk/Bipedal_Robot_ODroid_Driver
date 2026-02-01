#!/bin/bash
#
# build.sh
# 编译脚本
#
# 使用方法:
#   chmod +x build.sh
#   ./build.sh [debug|release]
#

set -e

BUILD_TYPE=${1:-debug}

echo "=== Building ODroid Robot Driver ==="
echo "Build type: $BUILD_TYPE"

# 创建build目录
BUILD_DIR="build-$BUILD_TYPE"
mkdir -p $BUILD_DIR
cd $BUILD_DIR

# 配置
if [ "$BUILD_TYPE" == "release" ]; then
    cmake -DCMAKE_BUILD_TYPE=Release ..
else
    cmake -DCMAKE_BUILD_TYPE=Debug ..
fi

# 编译
make -j$(nproc)

echo ""
echo "=== Build Complete ==="
echo "Executables in $BUILD_DIR:"
ls -la robot_driver test_* 2>/dev/null || echo "Build may have failed"
echo ""
echo "To run:"
echo "  sudo ./$BUILD_DIR/robot_driver"
echo "  sudo ./$BUILD_DIR/test_spi_comm"
echo "  sudo ./$BUILD_DIR/test_motor_control"
echo "  sudo ./$BUILD_DIR/test_rt_timing"
