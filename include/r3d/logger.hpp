#pragma once
// Tiny thread-safe stderr logger with [LEVEL] prefix and emoji-free tags
// compatible with the Python output style. Header-only.

#include <chrono>
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
    auto now = std::chrono::system_clock::now();
    auto sec = std::chrono::time_point_cast<std::chrono::seconds>(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - sec).count();
    std::time_t t = std::chrono::system_clock::to_time_t(sec);
    std::tm tm{};
    localtime_r(&t, &tm);
    char tbuf[16];
    std::strftime(tbuf, sizeof(tbuf), "%H:%M:%S", &tm);
    std::lock_guard<std::mutex> lk(mux());
    std::fprintf(stderr, "[%s.%03lld][%s] %.*s\n", tbuf,
                 static_cast<long long>(ms), tag,
                 static_cast<int>(msg.size()), msg.data());
    std::fflush(stderr);
}

inline void info (std::string_view m) { log(Level::Info,  m); }
inline void warn (std::string_view m) { log(Level::Warn,  m); }
inline void error(std::string_view m) { log(Level::Error, m); }
inline void debug(std::string_view m) { log(Level::Debug, m); }

} // namespace r3d::log
