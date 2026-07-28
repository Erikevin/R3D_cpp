#pragma once

// V4L2 原生 MJPG 抓取封装：
//   - 直接以 V4L2_PIX_FMT_MJPEG 向驱动请求格式，MMAP 方式 streaming；
//   - grab() 取出原始 MJPG 字节，不做任何解码/编码（OpenCV 路径的解码→再编码在此被消除）；
//   - 内部排空到最新帧，降低触发延迟。
// 线程安全：grab() 内部加锁，单实例可被单调用者使用；不同实例互不影响。

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace r3d_camodom {

struct CamConfig {
  std::string device;      // 如 "/dev/video0"
  int width = 2592;
  int height = 1944;
  int fps = 10;
  bool auto_exposure = true;   // V4L2_CID_EXPOSURE_AUTO = APERTURE_PRIORITY
  bool auto_wb = true;         // V4L2_CID_AUTO_WHITE_BALANCE = 1
  int jpeg_quality = 90;       // V4L2_CID_JPEG_COMPRESSION_QUALITY（驱动支持则生效）
};

class V4L2Camera {
public:
  explicit V4L2Camera(const CamConfig &cfg);
  ~V4L2Camera();

  V4L2Camera(const V4L2Camera &) = delete;
  V4L2Camera &operator=(const V4L2Camera &) = delete;

  // 打开设备并完成 格式/帧率/控制/buffer/streaming 初始化。成功返回 true。
  bool open();
  bool is_open() const { return fd_ >= 0; }

  // 抓取一帧原始 MJPG 字节（排空到最新）。成功返回 true 并填充 out。
  bool grab(std::vector<uint8_t> &out);

  // 释放 streaming / mmap / fd。
  void release();

  const std::string &device() const { return cfg_.device; }
  int width() const { return cfg_.width; }
  int height() const { return cfg_.height; }

private:
  bool set_control(uint32_t id, int32_t value);

  CamConfig cfg_;
  int fd_ = -1;
  struct MappedBuffer {
    void *start = nullptr;
    size_t length = 0;
  };
  std::vector<MappedBuffer> buffers_;
  bool streaming_ = false;
  std::mutex mtx_;
};

}  // namespace r3d_camodom
