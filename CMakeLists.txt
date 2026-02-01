cmake_minimum_required(VERSION 3.16)
project(odroid_robot_driver VERSION 1.0.0 LANGUAGES CXX)

# C++17标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -DDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -mtune=native -ffast-math -funroll-loops -DNDEBUG")

# 默认使用Debug模式
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")

# 查找依赖
find_package(Threads REQUIRED)

# 可选 Eigen3 (用于EKF, 当前阶段可选)
find_package(Eigen3 3.3 QUIET)
if(Eigen3_FOUND)
    message(STATUS "Eigen3 found: ${EIGEN3_INCLUDE_DIR}")
    add_definitions(-DHAS_EIGEN)
else()
    message(STATUS "Eigen3 not found, EKF will be disabled")
endif()

# Header-only库 (所有代码都在.hpp头文件中)
add_library(robot_driver_lib INTERFACE)

target_include_directories(robot_driver_lib INTERFACE
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(robot_driver_lib INTERFACE
    Threads::Threads
    rt      # POSIX实时扩展
)

if(Eigen3_FOUND)
    target_link_libraries(robot_driver_lib INTERFACE Eigen3::Eigen)
endif()

# 主程序
add_executable(robot_driver src/main.cpp)
target_link_libraries(robot_driver robot_driver_lib)

# 测试程序 - SPI通信测试
add_executable(test_spi_comm tests/test_spi_comm.cpp)
target_link_libraries(test_spi_comm robot_driver_lib)

# 测试程序 - 电机控制测试
add_executable(test_motor_control tests/test_motor_control.cpp)
target_link_libraries(test_motor_control robot_driver_lib)

# 测试程序 - RT性能测试
add_executable(test_rt_timing tests/test_rt_timing.cpp)
target_link_libraries(test_rt_timing robot_driver_lib)

# 安装
install(TARGETS robot_driver DESTINATION bin)
install(DIRECTORY scripts/ DESTINATION bin
        FILE_PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
                         GROUP_EXECUTE GROUP_READ)
