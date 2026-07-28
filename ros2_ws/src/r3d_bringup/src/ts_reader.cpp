#include "r3d_camodom/ts_reader.hpp"

#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include <fcntl.h>
#include <sys/mman.h>
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
  void *p = ::mmap(nullptr, size_, PROT_READ, MAP_SHARED, fd_, 0);
  if (p == MAP_FAILED) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }
  map_ = p;
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

void TsReader::close() {
  if (map_) {
    ::munmap(map_, size_);
    map_ = nullptr;
  }
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

}  // namespace r3d_camodom
