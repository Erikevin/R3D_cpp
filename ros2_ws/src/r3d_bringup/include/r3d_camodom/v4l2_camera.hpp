#pragma once

// V4L2 原生 MJPG 抓取封装：
//   - 直接以 V4L2_PIX_FMT_MJPEG 向驱动请求格式，MMAP 方式 streaming；
//   - grab() 取出原始 MJPG 字节，不做任何解码/编码（OpenCV 路径的解码→再编码在此被消除）；
//   - 内部排空到最新帧，降低触发延迟；
//   - 丢弃 V4L2_BUF_FLAG_ERROR 帧（驱动交付但数据损坏）与 SOI/EOI 标记异常的 MJPG，
//     避免坏字节落盘；并通过 set_logger 注入的回调上报告警；
//   - grab() 同时输出 v4l2_buffer.timestamp（驱动曝光时刻）的纳秒，供文件名/时间轴对齐。
// 线程安全：grab() 内部加锁，单实例可被单调用者使用；不同实例互不影响。

#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

namespace r3d_camodom {

struct CamConfig {
  std::string device;      // 如 "/dev/video0"
  int width = 2592;
  int height = 1944;
  int fps = 10;
  bool auto_exposure = false;   // V4L2_CID_EXPOSURE_AUTO = APERTURE_PRIORITY
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

  // 注入日志回调（错误帧 / MJPG 标记异常告警）。未注入则打到 stderr。
  using LogCb = std::function<void(const std::string &)>;
  void set_logger(LogCb cb) { log_ = std::move(cb); }

  // 抓取一帧原始 MJPG 字节（排空到最新，丢弃 ERROR/标记异常帧）。
  // 成功返回 true 并填充 out；frame_ns 非空时写入驱动曝光时刻（v4l2_buffer.timestamp）纳秒。
  bool grab(std::vector<uint8_t> &out, int64_t *frame_ns = nullptr);

  // 释放 streaming / mmap / fd。
  void release();

  const std::string &device() const { return cfg_.device; }
  int width() const { return cfg_.width; }
  int height() const { return cfg_.height; }

private:
  bool set_control(uint32_t id, int32_t value);
  // 走注入回调；未注入则 fprintf(stderr)。
  void log(const std::string &msg) const;

  CamConfig cfg_;
  int fd_ = -1;
  struct MappedBuffer {
    void *start = nullptr;
    size_t length = 0;
  };
  std::vector<MappedBuffer> buffers_;
  bool streaming_ = false;
  std::mutex mtx_;
  LogCb log_;                       // 可选日志回调（由上层注入 RCLCPP_WARN 等）
  std::size_t error_frames_ = 0;    // V4L2_BUF_FLAG_ERROR 累计计数
  std::size_t corrupt_frames_ = 0;  // MJPG SOI/EOI 标记异常累计计数
};

}  // namespace r3d_camodom
