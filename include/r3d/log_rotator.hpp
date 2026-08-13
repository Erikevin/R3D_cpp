#pragma once
// 按需清空（truncate）持续追加的日志文件，零拷贝、低开销。
//
// 适用于进程持有固定 fd、以 O_APPEND 写入的日志（桥经 dup2 写 BRIDGE_LOG、
// PerformanceMonitor 写 PERF_LOG_PATH）：truncate(path) 对所有持有该 inode 的 fd 生效，
// O_APPEND 下次 write 自动追加到新末尾，故清空安全，不需要重启进程。
//
// 满足任一则清空：
//   * mtime 早于 keep_days 天前 —— 服务重启后遇到的陈旧文件（两天前的），清空重开；
//   * 大小超过 max_bytes       —— 持续运行的大小保护。
// 超期/超量内容直接丢弃（不拷贝归档），避免拷贝大文件的 IO 开销。清空后把 mtime 刷到
// 当前，防止"陈旧"判断反复触发。

#include <cstddef>
#include <string>

namespace r3d {

class LogRotator {
public:
    LogRotator(std::string path, std::size_t max_bytes, int keep_days);
    // mtime 早于 keep_days 或大小超 max_bytes 则清空文件（零拷贝）。
    void maintain() const;

private:
    std::string path_;
    std::size_t max_bytes_;
    int keep_days_;
};

}  // namespace r3d
