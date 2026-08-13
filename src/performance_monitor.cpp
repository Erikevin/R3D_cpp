// 系统性能采样实现（CPU/iowait/内存/swap/磁盘 IO）。见 performance_monitor.hpp。

#include "r3d/performance_monitor.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fstream>
#include <sstream>
#include <string>

#include <fcntl.h>
#include <unistd.h>

#include "r3d/config.hpp"
#include "r3d/logger.hpp"

namespace r3d {

namespace {

// 读 /proc/stat 首行：cpu  user nice system idle iowait irq softirq steal ...
// total = 全部字段之和；busy = total - idle - iowait；iowait 单独保留。
bool read_cpu_jiffies(unsigned long long& total, unsigned long long& busy,
                      unsigned long long& iowait) {
    std::ifstream f("/proc/stat");
    if (!f) return false;
    std::string label;
    f >> label;  // "cpu"
    unsigned long long user = 0, nice = 0, system = 0, idle = 0, iow = 0,
                      irq = 0, softirq = 0, steal = 0;
    f >> user >> nice >> system >> idle >> iow >> irq >> softirq >> steal;
    unsigned long long idle_all = idle + iow;
    total = user + nice + system + idle_all + irq + softirq + steal;
    busy = total - idle_all;
    iowait = iow;
    return true;
}

// 读 /proc/meminfo 中指定键的值（单位 kB）；缺失返回 0。
unsigned long long meminfo_kb(const char* key) {
    std::ifstream f("/proc/meminfo");
    if (!f) return 0;
    std::string k;
    unsigned long long v = 0;
    while (f >> k) {
        if (k == key) {
            f >> v;
            return v;
        }
        std::string rest;
        std::getline(f, rest);
    }
    return 0;
}

// 去掉设备名末尾的分区号：mmcblk0p2→mmcblk0、nvme0n1p2→nvme0n1、sda2→sda。
std::string strip_partition(const std::string& dev) {
    std::size_t i = dev.size();
    while (i > 0 && std::isdigit(static_cast<unsigned char>(dev[i - 1]))) --i;
    if (i == dev.size()) return dev;          // 末尾无数字
    if (i > 0 && dev[i - 1] == 'p') return dev.substr(0, i - 1);  // ...pN
    return dev.substr(0, i);                                  // ...N
}

// /proc/diskstats 是否存在该设备行。
bool disk_in_stats(const std::string& name) {
    std::ifstream d("/proc/diskstats");
    std::string line;
    while (std::getline(d, line)) {
        std::istringstream iss(line);
        int maj = 0, min = 0;
        std::string nm;
        if (iss >> maj >> min >> nm && nm == name) return true;
    }
    return false;
}

// 探测根文件系统所在整盘名（供 /proc/diskstats 匹配）。
std::string detect_root_disk() {
    std::string raw;
    {
        std::ifstream m("/proc/mounts");
        std::string line;
        while (std::getline(m, line)) {
            std::istringstream iss(line);
            std::string dev, mp;
            if (iss >> dev >> mp && mp == "/") {
                raw = dev;
                break;
            }
        }
    }
    if (raw.empty()) return "";

    // /dev/root 通常是符号链接 → 解析真实设备。
    if (raw == "/dev/root" || raw == "root") {
        char buf[256];
        ssize_t n = ::readlink("/dev/root", buf, sizeof(buf) - 1);
        if (n > 0) {
            buf[n] = '\0';
            raw = buf;
        }
    }
    const std::size_t sl = raw.find_last_of('/');
    std::string b = (sl != std::string::npos) ? raw.substr(sl + 1) : raw;
    if (b.empty() || b[0] == '/') return "";

    // 先按原名查；查不到（分区名）再 strip 分区号查整盘。
    if (disk_in_stats(b)) return b;
    const std::string stripped = strip_partition(b);
    if (!stripped.empty() && stripped != b && disk_in_stats(stripped)) return stripped;
    return "";
}

// 读 /proc/diskstats 中 name 行的 sectors_read(字段3)、sectors_written(字段7)、
// io_ticks(字段10，累计 ms)。
bool read_disk_stats(const std::string& name, unsigned long long& sectors_read,
                     unsigned long long& sectors_written, unsigned long long& io_ms) {
    std::ifstream d("/proc/diskstats");
    std::string line;
    while (std::getline(d, line)) {
        std::istringstream iss(line);
        int maj = 0, min = 0;
        std::string nm;
        if (!(iss >> maj >> min >> nm) || nm != name) continue;
        unsigned long long f1 = 0, f2 = 0, f3 = 0, f4 = 0, f5 = 0, f6 = 0, f7 = 0,
                            f8 = 0, f9 = 0, f10 = 0;
        iss >> f1 >> f2 >> f3 >> f4 >> f5 >> f6 >> f7 >> f8 >> f9 >> f10;
        sectors_read = f3;
        sectors_written = f7;
        io_ms = f10;
        return true;
    }
    return false;
}

std::string now_stamp() {
    std::time_t t = std::time(nullptr);
    std::tm tm{};
    ::localtime_r(&t, &tm);
    char buf[24];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tm);
    return std::string(buf);
}

}  // namespace

PerformanceMonitor::PerformanceMonitor()
    : rotator_(config::PERF_LOG_PATH, config::LOG_ROTATE_BYTES, config::LOG_KEEP_DAYS),
      bridge_rotator_(config::BRIDGE_LOG, config::LOG_ROTATE_BYTES, config::LOG_KEEP_DAYS) {}

PerformanceMonitor::~PerformanceMonitor() { stop(); }

void PerformanceMonitor::start() {
    if (thread_.joinable()) return;
    running_ = true;

    fd_ = ::open(config::PERF_LOG_PATH, O_WRONLY | O_CREAT | O_APPEND, 0644);
    if (fd_ < 0) {
        log::warn(std::string("[性能] 打开日志失败: ") + config::PERF_LOG_PATH +
                  " (" + std::strerror(errno) + ")");
    }

    if (config::PERF_DISK_DEV && config::PERF_DISK_DEV[0] != '\0') {
        disk_name_ = config::PERF_DISK_DEV;
    } else {
        disk_name_ = detect_root_disk();
    }
    if (!disk_name_.empty()) {
        has_disk_prev_ = read_disk_stats(disk_name_, prev_sectors_read_,
                                         prev_sectors_written_, prev_io_ms_);
        prev_clock_ = std::chrono::steady_clock::now();
    }

    thread_ = std::thread([this] { task(); });
    log::info(std::string("[性能] 监控启动 → ") + config::PERF_LOG_PATH +
              (disk_name_.empty() ? "" : (std::string(" 磁盘=") + disk_name_)));
}

void PerformanceMonitor::stop() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

void PerformanceMonitor::write_line(const std::string& line) {
    if (fd_ < 0) return;
    std::string buf = line;
    buf.push_back('\n');
    const char* p = buf.data();
    std::size_t left = buf.size();
    while (left > 0) {
        ssize_t w = ::write(fd_, p, left);
        if (w < 0) {
            if (errno == EINTR) continue;
            break;
        }
        p += w;
        left -= static_cast<std::size_t>(w);
    }
}

bool PerformanceMonitor::sample_cpu(double& busy_pct, double& iowait_pct) {
    unsigned long long total = 0, busy = 0, iow = 0;
    if (!read_cpu_jiffies(total, busy, iow)) return false;
    if (!has_cpu_prev_) {
        prev_total_ = total;
        prev_busy_ = busy;
        prev_iowait_ = iow;
        has_cpu_prev_ = true;
        return false;  // 第一次无基线
    }
    const unsigned long long dt = total - prev_total_;
    const unsigned long long db = busy - prev_busy_;
    const unsigned long long di = iow - prev_iowait_;
    prev_total_ = total;
    prev_busy_ = busy;
    prev_iowait_ = iow;
    if (dt == 0) return false;
    busy_pct = static_cast<double>(db) / static_cast<double>(dt) * 100.0;
    iowait_pct = static_cast<double>(di) / static_cast<double>(dt) * 100.0;
    if (busy_pct > 100.0) busy_pct = 100.0;
    if (iowait_pct > 100.0) iowait_pct = 100.0;
    return true;
}

bool PerformanceMonitor::sample_mem(double& pct, unsigned long& used_mb,
                                    unsigned long& total_mb) {
    const unsigned long long total_kb = meminfo_kb("MemTotal:");
    const unsigned long long avail_kb = meminfo_kb("MemAvailable:");
    if (total_kb == 0) return false;
    const unsigned long long used_kb = (avail_kb <= total_kb) ? (total_kb - avail_kb) : 0;
    total_mb = static_cast<unsigned long>(total_kb / 1024);
    used_mb = static_cast<unsigned long>(used_kb / 1024);
    pct = static_cast<double>(used_kb) / static_cast<double>(total_kb) * 100.0;
    return true;
}

bool PerformanceMonitor::sample_swap(double& pct, unsigned long& used_mb,
                                     unsigned long& total_mb) {
    const unsigned long long total_kb = meminfo_kb("SwapTotal:");
    if (total_kb == 0) {
        pct = 0.0;
        used_mb = 0;
        total_mb = 0;
        return false;
    }
    const unsigned long long free_kb = meminfo_kb("SwapFree:");
    const unsigned long long used_kb = (free_kb <= total_kb) ? (total_kb - free_kb) : 0;
    total_mb = static_cast<unsigned long>(total_kb / 1024);
    used_mb = static_cast<unsigned long>(used_kb / 1024);
    pct = static_cast<double>(used_kb) / static_cast<double>(total_kb) * 100.0;
    return true;
}

bool PerformanceMonitor::sample_disk_io(double& read_mbs, double& write_mbs,
                                        double& util_pct) {
    if (disk_name_.empty() || !has_disk_prev_) return false;
    unsigned long long sr = 0, sw = 0, ioms = 0;
    if (!read_disk_stats(disk_name_, sr, sw, ioms)) return false;
    const auto now = std::chrono::steady_clock::now();
    const double secs = std::chrono::duration<double>(now - prev_clock_).count();
    if (secs <= 0.0) return false;
    const unsigned long long dsr = sr - prev_sectors_read_;
    const unsigned long long dsw = sw - prev_sectors_written_;
    const unsigned long long dioms = ioms - prev_io_ms_;
    prev_sectors_read_ = sr;
    prev_sectors_written_ = sw;
    prev_io_ms_ = ioms;
    prev_clock_ = now;
    // sectors 固定 512B；io_ticks 单位 ms。
    read_mbs = static_cast<double>(dsr) * 512.0 / secs / 1e6;
    write_mbs = static_cast<double>(dsw) * 512.0 / secs / 1e6;
    util_pct = static_cast<double>(dioms) / 1000.0 / secs * 100.0;
    if (util_pct > 100.0) util_pct = 100.0;
    return true;
}

void PerformanceMonitor::task() {
    while (running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(config::LOG_SAMPLE_INTERVAL_MS));
        if (!running_) break;

        double cpu = 0.0, iowait = 0.0, mem_pct = 0.0, swap_pct = 0.0;
        double rd = 0.0, wr = 0.0, util = 0.0;
        unsigned long mem_u = 0, mem_t = 0, swap_u = 0, swap_t = 0;

        if (sample_cpu(cpu, iowait)) {  // 首次无基线则跳过写入，下一轮起正常
            sample_mem(mem_pct, mem_u, mem_t);
            sample_swap(swap_pct, swap_u, swap_t);
            const bool has_disk = sample_disk_io(rd, wr, util);

            char buf[256];
            const int n = has_disk
                ? std::snprintf(buf, sizeof(buf),
                    "%s cpu=%.1f%% iowait=%.1f%% mem=%.1f%%(%lu/%luMB) swap=%.1f%%(%lu/%luMB) "
                    "disk_r=%.1fMB/s disk_w=%.1fMB/s disk_util=%.1f%%",
                    now_stamp().c_str(), cpu, iowait, mem_pct, mem_u, mem_t,
                    swap_pct, swap_u, swap_t, rd, wr, util)
                : std::snprintf(buf, sizeof(buf),
                    "%s cpu=%.1f%% iowait=%.1f%% mem=%.1f%%(%lu/%luMB) swap=%.1f%%(%lu/%luMB)",
                    now_stamp().c_str(), cpu, iowait, mem_pct, mem_u, mem_t,
                    swap_pct, swap_u, swap_t);
            if (n > 0) write_line(std::string(buf, static_cast<std::size_t>(n)));
        }

        // 维护两份日志：mtime 早于 LOG_KEEP_DAYS（两天前的）或大小超 LOG_ROTATE_BYTES
        // 则直接清空（零拷贝）。每轮检查开销仅两次 stat。
        rotator_.maintain();
        bridge_rotator_.maintain();
    }
}

}  // namespace r3d
