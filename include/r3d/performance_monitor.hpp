#pragma once
// 系统性能周期性采样，写入 config::PERF_LOG_PATH（/var/log/performance.log）。
// 指标：CPU 占用%、iowait%、内存%、swap%、磁盘读/写吞吐(MB/s)、磁盘忙碌率%。
//
// 采样源（零外部依赖）：
//   * CPU/iowait —— /proc/stat 首行 jiffies 两次采样的差。
//   * 内存/swap  —— /proc/meminfo（MemTotal/MemAvailable、SwapTotal/SwapFree）。
//   * 磁盘 IO    —— /proc/diskstats 中根文件系统所在整盘的 sectors/io_ticks 差。
//
// 自身的 PERF_LOG_PATH 与桥日志 BRIDGE_LOG 都经 LogRotator copytruncate 轮转 +
// 按 LOG_KEEP_DAYS 清理旧归档（桥进程 fd 固定，truncate 对其 O_APPEND 安全）。

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>

#include "log_rotator.hpp"

namespace r3d {

class PerformanceMonitor {
public:
    PerformanceMonitor();
    ~PerformanceMonitor();

    PerformanceMonitor(const PerformanceMonitor&) = delete;
    PerformanceMonitor& operator=(const PerformanceMonitor&) = delete;

    void start();
    void stop();

private:
    void task();
    void write_line(const std::string& line);

    bool sample_cpu(double& busy_pct, double& iowait_pct);
    bool sample_mem(double& pct, unsigned long& used_mb, unsigned long& total_mb);
    bool sample_swap(double& pct, unsigned long& used_mb, unsigned long& total_mb);
    bool sample_disk_io(double& read_mbs, double& write_mbs, double& util_pct);

    std::atomic<bool> running_{false};
    std::thread thread_;
    int fd_ = -1;  // PERF_LOG_PATH，O_APPEND
    LogRotator rotator_;
    LogRotator bridge_rotator_;

    // /proc/stat 基线
    unsigned long long prev_total_ = 0, prev_busy_ = 0, prev_iowait_ = 0;
    bool has_cpu_prev_ = false;
    // /proc/diskstats 基线
    unsigned long long prev_sectors_read_ = 0, prev_sectors_written_ = 0, prev_io_ms_ = 0;
    std::string disk_name_;
    bool has_disk_prev_ = false;
    std::chrono::steady_clock::time_point prev_clock_;
};

}  // namespace r3d
