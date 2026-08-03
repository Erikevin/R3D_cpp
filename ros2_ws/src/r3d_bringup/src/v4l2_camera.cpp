#include "r3d_camodom/v4l2_camera.hpp"

#include <linux/videodev2.h>

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

namespace r3d_camodom {

// 处理 EINTR 的 ioctl 包装。request 与 ioctl(2) 一致使用 unsigned long。
static int xioctl(int fd, unsigned long request, void *arg) {
  int r;
  do {
    r = ::ioctl(fd, request, arg);
  } while (r < 0 && errno == EINTR);
  return r;
}

static std::string to_hex(uint32_t v) {
  char buf[16];
  std::snprintf(buf, sizeof(buf), "%x", v);
  return std::string(buf);
}

void V4L2Camera::log(const std::string &msg) const {
  if (log_) {
    log_(msg);
  } else {
    std::fprintf(stderr, "%s\n", msg.c_str());
  }
}

V4L2Camera::V4L2Camera(const CamConfig &cfg) : cfg_(cfg) {}

V4L2Camera::~V4L2Camera() { release(); }

bool V4L2Camera::set_control(uint32_t id, int32_t value) {
  v4l2_control c{};
  c.id = id;
  c.value = value;
  return xioctl(fd_, VIDIOC_S_CTRL, &c) >= 0;
}

bool V4L2Camera::open() {
  fd_ = ::open(cfg_.device.c_str(), O_RDWR | O_NONBLOCK);
  if (fd_ < 0) {
    return false;
  }

  // 能力检查
  v4l2_capability cap{};
  if (xioctl(fd_, VIDIOC_QUERYCAP, &cap) < 0) {
    release();
    return false;
  }
  const uint32_t caps = cap.capabilities | cap.device_caps;
  if (!(caps & V4L2_CAP_VIDEO_CAPTURE)) {
    release();
    return false;
  }

  // 格式：MJPG
  v4l2_format fmt{};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = cfg_.width;
  fmt.fmt.pix.height = cfg_.height;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;
  if (xioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
    release();
    return false;
  }
  cfg_.width = fmt.fmt.pix.width;
  cfg_.height = fmt.fmt.pix.height;

  // 帧率
  v4l2_streamparm parm{};
  parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  parm.parm.capture.timeperframe.numerator = 1;
  parm.parm.capture.timeperframe.denominator = cfg_.fps;
  xioctl(fd_, VIDIOC_S_PARM, &parm);  // 失败不致命

  // 控制：自动曝光 / 自动白平衡 / JPEG 质量
  if (cfg_.auto_exposure) {
    set_control(V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_APERTURE_PRIORITY);
  }
  if (cfg_.auto_wb) {
    set_control(V4L2_CID_AUTO_WHITE_BALANCE, 1);
  }
  if (cfg_.jpeg_quality > 0) {
    set_control(V4L2_CID_JPEG_COMPRESSION_QUALITY, cfg_.jpeg_quality);
  }

  // 请求 MMAP buffer（count=2：便于排空到最新帧）
  v4l2_requestbuffers req{};
  req.count = 2;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if (xioctl(fd_, VIDIOC_REQBUFS, &req) < 0 || req.count < 1) {
    release();
    return false;
  }

  buffers_.resize(req.count);
  for (unsigned i = 0; i < req.count; ++i) {
    v4l2_buffer buf{};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if (xioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0) {
      release();
      return false;
    }
    buffers_[i].length = buf.length;
    buffers_[i].start =
        ::mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);
    if (buffers_[i].start == MAP_FAILED) {
      buffers_[i].start = nullptr;
      release();
      return false;
    }
  }

  // 入队
  for (unsigned i = 0; i < req.count; ++i) {
    v4l2_buffer buf{};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if (xioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
      release();
      return false;
    }
  }

  // 开始 streaming
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
    release();
    return false;
  }
  streaming_ = true;
  return true;
}

bool V4L2Camera::grab(std::vector<uint8_t> &out, int64_t *frame_ns) {
  std::lock_guard<std::mutex> lk(mtx_);
  if (fd_ < 0 || !streaming_) {
    return false;
  }

  // 阻塞等待至少一帧可读（最多 1000ms）
  pollfd pfd{};
  pfd.fd = fd_;
  pfd.events = POLLIN;
  if (::poll(&pfd, 1, 1000) <= 0) {
    return false;
  }

  // 排空到最新：每次 DQBUF 拿新帧，把上一帧归还；最后保留最新一帧。
  v4l2_buffer kept{};
  kept.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  kept.memory = V4L2_MEMORY_MMAP;
  bool have = false;
  while (true) {
    v4l2_buffer buf{};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
      if (errno == EAGAIN) {
        break;
      }
      return false;
    }
    // 错误帧：驱动已 DQBUF 交付，但 USB 传输/解码出错，数据无效。
    // 归还重填并跳过，绝不计入 kept，避免坏字节落盘。
    if (buf.flags & V4L2_BUF_FLAG_ERROR) {
      ++error_frames_;
      log("⚠️ [" + cfg_.device + "] V4L2 错误帧 flags=0x" + to_hex(buf.flags) +
          ", 丢弃该帧（累计 " + std::to_string(error_frames_) + "）");
      v4l2_buffer rel = buf;
      ::ioctl(fd_, VIDIOC_QBUF, &rel);  // 归还，让硬件重新填充
      // 非阻塞探测是否还有可立即读取的帧
      pollfd p{};
      p.fd = fd_;
      p.events = POLLIN;
      if (::poll(&p, 1, 0) <= 0) {
        break;
      }
      continue;
    }
    if (have) {
      v4l2_buffer rel = kept;
      ::ioctl(fd_, VIDIOC_QBUF, &rel);  // 丢弃旧帧，归还
    }
    kept = buf;
    have = true;

    // 非阻塞探测：是否还有可立即读取的帧
    pollfd p{};
    p.fd = fd_;
    p.events = POLLIN;
    if (::poll(&p, 1, 0) <= 0) {
      break;
    }
  }
  if (!have) {
    return false;
  }

  // 拷贝最新帧的 MJPG 字节（length 由驱动给出，bytesused 是有效字节）
  const auto &mb = buffers_[kept.index];
  const auto *base = static_cast<uint8_t *>(mb.start);
  const size_t n = kept.bytesused;
  // 完整性校验：MJPG 必须以 SOI(FFD8) 起、EOI(FFD9) 止，拦截驱动交付的不完整帧。
  if (n < 4 || base[0] != 0xFF || base[1] != 0xD8 || base[n - 2] != 0xFF || base[n - 1] != 0xD9) {
    ++corrupt_frames_;
    log("⚠️ [" + cfg_.device + "] MJPG 标记异常 size=" + std::to_string(n) +
        ", 丢弃该帧（累计 " + std::to_string(corrupt_frames_) + "）");
    v4l2_buffer rel = kept;
    ::ioctl(fd_, VIDIOC_QBUF, &rel);  // 归还，跳过本帧
    return false;
  }
  out.assign(base, base + n);

  // 帧时间戳（驱动曝光时刻，v4l2_buffer.timestamp）→ 纳秒，供文件名/时间轴对齐。
  // 注意时基取决于驱动（UVC 常为 CLOCK_MONOTONIC），与 system_clock 墙钟不同源。
  if (frame_ns) {
    *frame_ns = static_cast<int64_t>(kept.timestamp.tv_sec) * 1000000000LL +
                static_cast<int64_t>(kept.timestamp.tv_usec) * 1000LL;
  }

  // 归还最新帧 buffer
  v4l2_buffer rel = kept;
  ::ioctl(fd_, VIDIOC_QBUF, &rel);
  return true;
}

void V4L2Camera::release() {
  if (streaming_ && fd_ >= 0) {
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    xioctl(fd_, VIDIOC_STREAMOFF, &type);
    streaming_ = false;
  }
  for (auto &mb : buffers_) {
    if (mb.start && mb.length) {
      ::munmap(mb.start, mb.length);
    }
  }
  buffers_.clear();
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

}  // namespace r3d_camodom
