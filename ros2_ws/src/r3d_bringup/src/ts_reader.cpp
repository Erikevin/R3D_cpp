#include "r3d_camodom/ts_reader.hpp"

#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace r3d_camodom {

TsReader::TsReader() {
  const char *user = std::getenv("USER");
  std::string u = (user && user[0]) ? std::string(user) : std::string("jetson");
  // path_ = "/home/" + u + "/timeshare";
  path_ = "/root/timeshare";
}

TsReader::~TsReader() { close(); }

bool TsReader::open() {
  fd_ = ::open(path_.c_str(), O_RDONLY | O_NONBLOCK);
  if (fd_ < 0) {
    return false;
  }
  // livox 新驱动写 32 字节(两个 timestamp);旧/未就绪时可能不足。按实际文件大小映射。
  struct stat st{};
  if (::fstat(fd_, &st) != 0 || st.st_size <= 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }
  size_t want = 32;
  if (static_cast<size_t>(st.st_size) < want) {
    want = static_cast<size_t>(st.st_size);
  }
  void *p = ::mmap(nullptr, want, PROT_READ, MAP_SHARED, fd_, 0);
  if (p == MAP_FAILED) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }
  map_ = p;
  map_len_ = want;
  return true;
}

int64_t TsReader::get_ns() {
  if (!map_ && !open()) {
    return 0;
  }
  // 16 字节：int64 high（偏移 0）, int64 low（偏移 8）；返回 low（native little-endian）。
  int64_t low = 0;
  std::memcpy(&low, static_cast<char *>(map_) + 8, sizeof(int64_t));
  return low;
}

int64_t TsReader::get_sys_ns() {
  // 第二个 timestamp 在偏移 16({high=秒, low=秒内纳秒})；仅当映射满 32 字节才有效。
  // 不做懒 open(避免双采集线程并发 open 竞争)；构造时已 open，未就绪则返回 0。
  if (map_len_ < 32) {
    return 0;
  }
  const char *base = static_cast<char *>(map_);
  int64_t sec = 0, ns = 0;
  // writer 分两次写 sec / ns，跨秒边界瞬间可能撕裂(旧sec+新ns)→重读至两次一致(有界)。
  for (int attempt = 0; attempt < 4; ++attempt) {
    int64_t s1 = 0, n1 = 0, s2 = 0, n2 = 0;
    std::memcpy(&s1, base + 16, sizeof(int64_t));
    std::memcpy(&n1, base + 24, sizeof(int64_t));
    std::memcpy(&s2, base + 16, sizeof(int64_t));
    std::memcpy(&n2, base + 24, sizeof(int64_t));
    sec = s2;
    ns = n2;
    if (s1 == s2 && n1 == n2) {
      break;
    }
  }
  // 合理性校验：系统UTC秒应 > 1e9(2020年后)，纳秒应落在 [0, 1e9)；否则视为未就绪/异常。
  if (sec < 1000000000LL || ns < 0 || ns >= 1000000000LL) {
    return 0;
  }
  return sec * 1000000000LL + ns;
}

void TsReader::close() {
  if (map_) {
    ::munmap(map_, map_len_);
    map_ = nullptr;
    map_len_ = 0;
  }
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

}  // namespace r3d_camodom
