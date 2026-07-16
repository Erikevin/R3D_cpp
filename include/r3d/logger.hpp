#pragma once
// Tiny thread-safe stderr logger with [LEVEL] prefix and emoji-free tags
// compatible with the Python output style. Header-only.

#include <cstdio>
#include <ctime>
#include <mutex>
#include <string>
#include <string_view>

namespace r3d::log {

enum class Level { Debug, Info, Warn, Error };

inline std::mutex& mux() {
    static std::mutex m;
    return m;
}

inline void log(Level lv, std::string_view msg) {
    const char* tag = "?";
    switch (lv) {
        case Level::Debug: tag = "DEBUG"; break;
        case Level::Info:  tag = "INFO";  break;
        case Level::Warn:  tag = "WARN";  break;
        case Level::Error: tag = "ERROR"; break;
    }
    std::time_t now = std::time(nullptr);
    std::tm tm{};
    localtime_r(&now, &tm);
    char tbuf[24];
    std::strftime(tbuf, sizeof(tbuf), "%H:%M:%S", &tm);
    std::lock_guard<std::mutex> lk(mux());
    std::fprintf(stderr, "[%s][%s] %.*s\n", tbuf, tag,
                 static_cast<int>(msg.size()), msg.data());
    std::fflush(stderr);
}

inline void info (std::string_view m) { log(Level::Info,  m); }
inline void warn (std::string_view m) { log(Level::Warn,  m); }
inline void error(std::string_view m) { log(Level::Error, m); }
inline void debug(std::string_view m) { log(Level::Debug, m); }

} // namespace r3d::log
