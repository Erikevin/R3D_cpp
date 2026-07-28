#pragma once

// Livox 时间戳共享内存读取器（移植自 Python TimeStampReader）。
// 共享文件 /home/$USER/timeshare 共 16 字节：int64 high, int64 low（native little-endian）。
// get_ns() 返回 low（纳秒），失败返回 0。

#include <cstddef>
#include <cstdint>
#include <string>

namespace r3d_camodom {

class TsReader {
public:
  TsReader();
  ~TsReader();

  TsReader(const TsReader &) = delete;
  TsReader &operator=(const TsReader &) = delete;

  // 映射共享内存；成功返回 true。可提前调用以探测雷达栈是否就绪（get_ns 也会懒调用）。
  bool open();

  // 返回当前 Livox 纳秒时间戳；尚未映射则在首次调用时尝试 open()，失败返回 0。
  int64_t get_ns();
  void close();

private:
  std::string path_;
  int fd_ = -1;
  void *map_ = nullptr;
  size_t size_ = 16;
};

}  // namespace r3d_camodom
