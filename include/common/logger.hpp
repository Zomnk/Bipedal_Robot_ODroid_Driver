/**
 * @file logger.hpp
 * @brief 轻量级RT安全日志系统
 * @author Zomnk
 * @date 2026-02-01
 */

#ifndef ODROID_COMMON_LOGGER_HPP
#define ODROID_COMMON_LOGGER_HPP

#include <cstdio>
#include <cstdarg>
#include <ctime>
#include <chrono>
#include <mutex>
#include <string>

namespace odroid {

/**
 * @brief 日志级别
 */
enum class LogLevel : int {
    TRACE = 0,
    DEBUG = 1,
    INFO = 2,
    WARN = 3,
    ERROR = 4,
    FATAL = 5
};

/**
 * @brief 简单日志类 (非RT安全版本用于调试阶段)
 */
class Logger {
public:
    static Logger& instance() {
        static Logger logger;
        return logger;
    }

    void set_level(LogLevel level) { min_level_ = level; }
    void set_show_timestamp(bool show) { show_timestamp_ = show; }

    void log(LogLevel level, const char* file, int line, const char* fmt, ...) {
        if (level < min_level_) return;

        std::lock_guard<std::mutex> lock(mutex_);

        // 时间戳
        if (show_timestamp_) {
            auto now = std::chrono::system_clock::now();
            auto time = std::chrono::system_clock::to_time_t(now);
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch()) % 1000;
            
            char time_buf[32];
            std::strftime(time_buf, sizeof(time_buf), "%H:%M:%S", std::localtime(&time));
            printf("[%s.%03ld] ", time_buf, ms.count());
        }

        // 日志级别
        printf("[%s] ", level_str(level));

        // 文件和行号 (调试模式)
        if (level >= LogLevel::WARN) {
            const char* filename = strrchr(file, '/');
            if (!filename) filename = strrchr(file, '\\');
            filename = filename ? filename + 1 : file;
            printf("(%s:%d) ", filename, line);
        }

        // 消息内容
        va_list args;
        va_start(args, fmt);
        vprintf(fmt, args);
        va_end(args);
        printf("\n");
        fflush(stdout);
    }

private:
    Logger() : min_level_(LogLevel::DEBUG), show_timestamp_(true) {}
    
    const char* level_str(LogLevel level) {
        switch (level) {
            case LogLevel::TRACE: return "TRACE";
            case LogLevel::DEBUG: return "DEBUG";
            case LogLevel::INFO:  return " INFO";
            case LogLevel::WARN:  return " WARN";
            case LogLevel::ERROR: return "ERROR";
            case LogLevel::FATAL: return "FATAL";
            default: return "?????";
        }
    }

    LogLevel min_level_;
    bool show_timestamp_;
    std::mutex mutex_;
};

// 日志宏
#define LOG_TRACE(fmt, ...) \
    odroid::Logger::instance().log(odroid::LogLevel::TRACE, __FILE__, __LINE__, fmt, ##__VA_ARGS__)
#define LOG_DEBUG(fmt, ...) \
    odroid::Logger::instance().log(odroid::LogLevel::DEBUG, __FILE__, __LINE__, fmt, ##__VA_ARGS__)
#define LOG_INFO(fmt, ...) \
    odroid::Logger::instance().log(odroid::LogLevel::INFO, __FILE__, __LINE__, fmt, ##__VA_ARGS__)
#define LOG_WARN(fmt, ...) \
    odroid::Logger::instance().log(odroid::LogLevel::WARN, __FILE__, __LINE__, fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) \
    odroid::Logger::instance().log(odroid::LogLevel::ERROR, __FILE__, __LINE__, fmt, ##__VA_ARGS__)
#define LOG_FATAL(fmt, ...) \
    odroid::Logger::instance().log(odroid::LogLevel::FATAL, __FILE__, __LINE__, fmt, ##__VA_ARGS__)

} // namespace odroid

#endif // ODROID_COMMON_LOGGER_HPP
