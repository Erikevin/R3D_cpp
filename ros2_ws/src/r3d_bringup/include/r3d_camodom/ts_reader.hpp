#pragma once

// Livox 时间戳共享内存读取器（移植自 Python TimeStampReader）。
// 共享文件 /home/$USER/timeshare：livox 驱动写入两个 time_stamp（共 32 字节，native little-endian）：
//   [0] = {high, low}  雷达/主机 base_time（low 为纳秒）
//   [1] = {high, low}  系统 UTC（high=秒，low=秒内纳秒）
// get_ns() 返回 [0].low；get_sys_ns() 返回 [1] 的完整纳秒(high*1e9+low)。失败返回 0。

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

  // 返回第一个 timestamp 的 low（base_time 纳秒）；尚未映射则首次调用尝试 open()，失败返回 0。
  int64_t get_ns();
  // 返回第二个 timestamp（系统UTC）的完整纳秒 = high*1e9+low；未映射或文件不足 32 字节返回 0。
  int64_t get_sys_ns();
  void close();

private:
  std::string path_;
  int fd_ = -1;
  void *map_ = nullptr;
  size_t map_len_ = 0;   // 实际映射字节数（16 或 32，按文件大小而定）
};

}  // namespace r3d_camodom
