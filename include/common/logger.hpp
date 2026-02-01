/**
 * @file logger.hpp
 * @brief 日志系统
 */

#ifndef ODROID_COMMON_LOGGER_HPP
#define ODROID_COMMON_LOGGER_HPP

#include <cstdio>
#include <cstdarg>
#include <ctime>

// 取消可能由编译器定义的DEBUG宏，避免冲突
#ifdef DEBUG
#undef DEBUG
#endif

namespace odroid {

enum class LogLevel {
    DEBUG = 0,
    INFO  = 1,
    WARN  = 2,
    ERROR = 3,
    FATAL = 4
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

        time_t now = time(nullptr);
        struct tm* tm_info = localtime(&now);
        char time_buf[32];
        strftime(time_buf, sizeof(time_buf), "%H:%M:%S", tm_info);

        const char* level_str = get_level_str(level);
        fprintf(stderr, "[%s][%s] %s:%d: ", time_buf, level_str, file, line);

        va_list args;
        va_start(args, fmt);
        vfprintf(stderr, fmt, args);
        va_end(args);
        
        fprintf(stderr, "\n");
    }

private:
    Logger() : level_(LogLevel::INFO) {}
    
    const char* get_level_str(LogLevel level) {
        switch (level) {
            case LogLevel::DEBUG: return "DEBUG";
            case LogLevel::INFO:  return "INFO";
            case LogLevel::WARN:  return "WARN";
            case LogLevel::ERROR: return "ERROR";
            case LogLevel::FATAL: return "FATAL";
            default: return "UNKNOWN";
        }
    }

    LogLevel level_;
};

} // namespace odroid

//==============================================================================
// 日志宏定义
//==============================================================================

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

#endif // ODROID_COMMON_LOGGER_HPP
