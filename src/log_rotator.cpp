// LogRotator 实现。见 log_rotator.hpp。

#include "r3d/log_rotator.hpp"

#include <ctime>

#include <fcntl.h>      // AT_FDCWD, utimensat
#include <sys/stat.h>
#include <unistd.h>

namespace r3d {

LogRotator::LogRotator(std::string path, std::size_t max_bytes, int keep_days)
    : path_(std::move(path)), max_bytes_(max_bytes), keep_days_(keep_days) {}

void LogRotator::maintain() const {
    struct stat st;
    if (::stat(path_.c_str(), &st) != 0) return;  // 文件不存在，由写入方创建即可

    const std::time_t now = std::time(nullptr);
    const std::time_t cutoff =
        now - static_cast<std::time_t>(keep_days_) * 86400;
    const bool stale = st.st_mtime < cutoff;
    const bool too_big = static_cast<std::size_t>(st.st_size) >= max_bytes_;
    if (!stale && !too_big) return;

    // 清空文件。失败（罕见）跳过，下次再试；不影响日志写入正确性。
    if (::truncate(path_.c_str(), 0) != 0) return;

    // 把 mtime/atime 刷到当前，避免基于 mtime 的"陈旧"判断反复触发清空。
    struct timespec ts[2];
    ts[0].tv_sec = now;  ts[0].tv_nsec = 0;  // atime
    ts[1].tv_sec = now;  ts[1].tv_nsec = 0;  // mtime
    ::utimensat(AT_FDCWD, path_.c_str(), ts, 0);
}

}  // namespace r3d
