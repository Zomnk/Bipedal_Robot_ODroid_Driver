/**
 * @file logger.hpp
 * @brief 日志系统
 * @author Zomnk
 * @date 2026-02-01
 */

#ifndef ODROID_COMMON_LOGGER_HPP
#define ODROID_COMMON_LOGGER_HPP

#include <cstdio>
#include <cstdarg>
#include <ctime>
#include <mutex>

// 防止与系统DEBUG宏冲突
#undef DEBUG

namespace odroid {

enum class LogLevel {
    DEBUG = 0,
    INFO,
    WARN,
    ERROR,
    FATAL
};

class Logger {
public:
    static Logger& instance() {
        static Logger logger;
        return logger;
    }

    void set_level(LogLevel level) { level_ = level; }
    LogLevel get_level() const { return level_; }

    void log(LogLevel level, const char* file, int line, const char* fmt, ...) {
        if (level < level_) return;

        std::lock_guard<std::mutex> lock(mutex_);

        // 时间戳
        time_t now = time(nullptr);
        struct tm* t = localtime(&now);
        printf("[%02d:%02d:%02d] ", t->tm_hour, t->tm_min, t->tm_sec);

        // 级别
        const char* level_str[] = {"DEBUG", "INFO", "WARN", "ERROR", "FATAL"};
        printf("[%s] ", level_str[static_cast<int>(level)]);

        // 消息
        va_list args;
        va_start(args, fmt);
        vprintf(fmt, args);
        va_end(args);

        printf("\n");
        fflush(stdout);
    }

private:
    Logger() : level_(LogLevel::INFO) {}
    LogLevel level_;
    std::mutex mutex_;
};

#define LOG_DEBUG(fmt, ...) odroid::Logger::instance().log(odroid::LogLevel::DEBUG, __FILE__, __LINE__, fmt, ##__VA_ARGS__)
#define LOG_INFO(fmt, ...)  odroid::Logger::instance().log(odroid::LogLevel::INFO, __FILE__, __LINE__, fmt, ##__VA_ARGS__)
#define LOG_WARN(fmt, ...)  odroid::Logger::instance().log(odroid::LogLevel::WARN, __FILE__, __LINE__, fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) odroid::Logger::instance().log(odroid::LogLevel::ERROR, __FILE__, __LINE__, fmt, ##__VA_ARGS__)
#define LOG_FATAL(fmt, ...) odroid::Logger::instance().log(odroid::LogLevel::FATAL, __FILE__, __LINE__, fmt, ##__VA_ARGS__)

} // namespace odroid

#endif // ODROID_COMMON_LOGGER_HPP