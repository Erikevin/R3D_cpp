// cam_tuner mock 测试 —— 无真实相机时验证整定闭环全流程。
//
// 原理：V4L2Camera 只经由 open/ioctl/mmap/munmap/poll/close 六个系统调用触达
// 设备，在本测试可执行文件内定义同名符号（ELF 符号解析：可执行文件自身的
// 定义优先于 libc），把 /dev/video* 映射成"亮度 ∝ exposure×gain"的模拟相机：
//   - QUERYCTRL/S_CTRL/G_CTRL 读写模拟寄存器；
//   - DQBUF 按 S_CTRL 后的当前参数合成对应灰度的 MJPG 帧（OpenCV imencode），
//     经 mmap 缓冲交付，帧节拍按 fps 模拟（STREAMON 后 ~250ms 起流）；
//   - 非 /dev/video* 的 fd 一律 dlsym(RTLD_NEXT) 透传真实 libc，OpenCV/运行库
//     内部调用不受影响。
//
// 场景（命令行参数选择，缺省全跑）：
//   dark     暗场景（默认参数下 mean≈30）→ 应升曝光/增益收敛（3 轮内）
//   bright   亮场景（mean≈200）           → 应压曝光收敛
//   saturated 削顶过曝（真实亮度 400，mean 钳 ~255、hl≈11%）→ 削顶感知步长 3 轮内收敛
//   normal   正常场景（mean≈70）          → 一轮即收敛
//   stalled  流卡死（DQBUF 持续 EINVAL） → 应回写默认参数 7.8ms/16 且 ok=true
//   slowfps  帧率 3fps                    → 预算尽退出，保留已调参数
//   noopen   设备不存在                   → ok=false
//   noexp    无曝光控制项                 → ok=false
//   pair     双机并行（cam_tune_pair）    → 全部 ok
//
// 构建产物仅本地验证用，不随包 install：colcon build 后跑
//   build/r3d_bringup/r3d_cam_tuner_mock

#include "r3d_ws/cam_tuner.hpp"

#include <linux/videodev2.h>

#include <opencv2/imgcodecs.hpp>

#include <dlfcn.h>
#include <poll.h>
#include <stdarg.h>
#include <sys/mman.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using Clock = std::chrono::steady_clock;

// ============================ mock 相机模型 ============================
namespace {

constexpr size_t kBufLen = 512 * 1024;  // 每 mmap buffer 容量（>= JPEG 实际大小）

struct MockCam {
  // 能力/寄存器（S_CTRL 写、帧合成读）
  bool has_exp = true, has_gain = true;
  int exp_min = 0, exp_max = 10000;   // exposure_absolute 原始单位（100µs）
  int gain_min = 16, gain_max = 248;
  int exp_raw = 78, gain = 16;        // 与默认起步值一致
  double scene = 70.0;                // 默认参数(7.8ms/16)下的画面 mean
  bool stall = false;                 // DQBUF 持续失败（流卡死/设备异常）
  double fps = 30.0;

  // 流状态
  bool streaming = false;
  Clock::time_point next_frame{};
  int frame_idx = 0;
  uint8_t *mem[2] = {nullptr, nullptr};  // mmap 缓冲
};

std::mutex g_mux;
std::map<int, MockCam *> g_fds;       // mock fd → 相机
std::map<std::string, MockCam *> g_paths;  // 设备路径 → 相机
int g_next_fd = 1000;

void mock_reset() {
  std::lock_guard<std::mutex> lk(g_mux);
  for (auto &kv : g_fds)
    for (auto *m : {kv.second->mem[0], kv.second->mem[1]}) delete[] m;
  g_fds.clear();
  g_paths.clear();
  g_next_fd = 1000;
}

// 注册一台模拟相机到设备路径
void mock_register(const std::string &dev, MockCam cam) {
  std::lock_guard<std::mutex> lk(g_mux);
  auto *c = new MockCam(cam);
  c->mem[0] = new uint8_t[kBufLen];
  c->mem[1] = new uint8_t[kBufLen];
  g_paths[dev] = c;
}

// 按 S_CTRL 后的当前参数合成一帧 MJPG 写入 mem[idx]，返回帧字节数
size_t mock_fill_frame(MockCam *c, int idx) {
  // 亮度模型：mean = scene × (exp/7.8ms) × (gain/16)，截到 255。
  // 噪声用帧序确定性伪随机（可复现，且双相机线程各自独立无共享状态）。
  double mean = c->scene * (c->exp_raw / 78.0) * (c->gain / 16.0);
  mean = std::min(mean, 255.0);
  const int noise = (c->frame_idx % 5) - 2;  // ±2
  int v = static_cast<int>(std::min(std::max(mean + noise, 0.0), 255.0));
  cv::Mat img(480, 640, CV_8UC3, cv::Scalar(v, v, v));
  if (mean > 235.0) {  // 高光模拟：接近饱和时撒 12% 的 ≥252 像素
    for (int y = 0; y < 480; y += 3)
      for (int x = 0; x < 640; x += 3) img.at<cv::Vec3b>(y, x) = {252, 252, 252};
  }
  std::vector<uint8_t> jpg;
  cv::imencode(".jpg", img, jpg, {cv::IMWRITE_JPEG_QUALITY, 80});
  std::memcpy(c->mem[idx], jpg.data(), jpg.size());
  return jpg.size();
}

}  // namespace

// ======================= 系统调用拦截（非 mock fd 透传） =======================
namespace {
// dlsym 懒加载真实函数
template <typename F>
F real_fn(const char *name) {
  static std::map<std::string, void *> cache;
  void *p = dlsym(RTLD_NEXT, name);
  if (!p) {
    std::fprintf(stderr, "[mock] dlsym(%s) 失败\n", name);
    std::abort();
  }
  return reinterpret_cast<F>(p);
}
}  // namespace

extern "C" int open(const char *path, int flags, ...) {
  std::lock_guard<std::mutex> lk(g_mux);
  auto it = g_paths.find(path);
  if (it != g_paths.end()) {
    g_fds[g_next_fd] = it->second;
    return g_next_fd++;
  }
  auto real = real_fn<int (*)(const char *, int, ...)>("open");
  return real(path, flags, 0);
}

extern "C" int close(int fd) {
  {
    std::lock_guard<std::mutex> lk(g_mux);
    if (g_fds.count(fd)) {
      g_fds.erase(fd);
      return 0;
    }
  }
  auto real = real_fn<int (*)(int)>("close");
  return real(fd);
}

extern "C" void *mmap(void *addr, size_t length, int prot, int flags, int fd,
                      off_t offset) {
  {
    std::lock_guard<std::mutex> lk(g_mux);
    auto it = g_fds.find(fd);
    if (it != g_fds.end() && (offset == 0 || offset == 1) && length <= kBufLen)
      return it->second->mem[offset];
  }
  auto real = real_fn<void *(*)(void *, size_t, int, int, int, off_t)>("mmap");
  return real(addr, length, prot, flags, fd, offset);
}

extern "C" int munmap(void *addr, size_t length) {
  // mock 缓冲不需要真实 unmap（随 mock_reset 统一回收）
  (void)addr;
  (void)length;
  return 0;
}

extern "C" int poll(struct pollfd *fds, nfds_t nfds, int timeout_ms) {
  if (nfds == 1) {
    std::unique_lock<std::mutex> lk(g_mux);
    auto it = g_fds.find(fds[0].fd);
    if (it != g_fds.end()) {
      MockCam *c = it->second;
      if (!c->streaming) {  // 未 STREAMON：无帧
        fds[0].revents = 0;
        return 0;
      }
      if (timeout_ms == 0) {  // 非阻塞探测
        fds[0].revents = Clock::now() >= c->next_frame ? POLLIN : 0;
        return fds[0].revents ? 1 : 0;
      }
      // 阻塞等帧：等到下一帧时隙或超时（DQBUF 才推进 next_frame）
      Clock::time_point deadline =
          Clock::now() + std::chrono::milliseconds(timeout_ms);
      while (Clock::now() < c->next_frame && Clock::now() < deadline)
        lk.unlock(), std::this_thread::sleep_for(std::chrono::microseconds(500)),
            lk.lock();
      if (Clock::now() >= c->next_frame) {
        fds[0].revents = POLLIN;
        return 1;
      }
      fds[0].revents = 0;
      return 0;
    }
  }
  auto real = real_fn<int (*)(struct pollfd *, nfds_t, int)>("poll");
  return real(fds, nfds, timeout_ms);
}

extern "C" int ioctl(int fd, unsigned long req, ...) {
  va_list ap;
  va_start(ap, req);
  void *arg = va_arg(ap, void *);
  va_end(ap);

  std::unique_lock<std::mutex> lk(g_mux);
  auto it = g_fds.find(fd);
  if (it == g_fds.end()) {
    lk.unlock();
    auto real = real_fn<int (*)(int, unsigned long, ...)>("ioctl");
    return real(fd, req, arg);
  }
  MockCam *c = it->second;

  switch (req) {
    case VIDIOC_QUERYCAP: {
      auto *cap = static_cast<v4l2_capability *>(arg);
      std::memset(cap, 0, sizeof(*cap));
      cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
      return 0;
    }
    case VIDIOC_S_FMT: {
      auto *f = static_cast<v4l2_format *>(arg);
      f->fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
      f->fmt.pix.sizeimage = static_cast<__u32>(kBufLen);
      return 0;
    }
    case VIDIOC_S_PARM:
      return 0;
    case VIDIOC_QUERYCTRL: {
      auto *q = static_cast<v4l2_queryctrl *>(arg);
      const uint32_t id = q->id;  // 先取出请求的 id，清零结构体后再比较
      std::memset(q, 0, sizeof(*q));
      q->id = id;
      if (id == V4L2_CID_EXPOSURE_ABSOLUTE) {
        if (!c->has_exp) return -1;
        q->minimum = c->exp_min;
        q->maximum = c->exp_max;
        return 0;
      }
      if (id == V4L2_CID_GAIN) {
        if (!c->has_gain) return -1;
        q->minimum = c->gain_min;
        q->maximum = c->gain_max;
        return 0;
      }
      q->minimum = 0;
      q->maximum = 4;
      return 0;
    }
    case VIDIOC_S_CTRL: {
      auto *ctl = static_cast<v4l2_control *>(arg);
      if (ctl->id == V4L2_CID_EXPOSURE_ABSOLUTE) c->exp_raw = ctl->value;
      else if (ctl->id == V4L2_CID_GAIN) c->gain = ctl->value;
      return 0;  // 其余控制项（EXPOSURE_AUTO/DFR/AWB…）一律接受
    }
    case VIDIOC_REQBUFS: {
      auto *r = static_cast<v4l2_requestbuffers *>(arg);
      r->count = 2;
      return 0;
    }
    case VIDIOC_QUERYBUF: {
      auto *b = static_cast<v4l2_buffer *>(arg);
      b->length = static_cast<__u32>(kBufLen);
      b->m.offset = b->index;
      return 0;
    }
    case VIDIOC_QBUF:
      return 0;
    case VIDIOC_DQBUF: {
      if (c->stall) return -1;  // 流卡死：设备异常持续失败
      auto *b = static_cast<v4l2_buffer *>(arg);
      b->index = (c->frame_idx++) % 2;
      b->bytesused = static_cast<__u32>(mock_fill_frame(c, b->index));
      b->flags = 0;
      auto now = Clock::now();
      b->timestamp.tv_sec =
          std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch())
              .count();
      b->timestamp.tv_usec = static_cast<__u32>(
          std::chrono::duration_cast<std::chrono::microseconds>(
              now.time_since_epoch()).count() % 1000000);
      c->next_frame = now +
          std::chrono::microseconds(static_cast<long long>(1e6 / c->fps));
      return 0;
    }
    case VIDIOC_STREAMON:
      c->streaming = true;
      c->next_frame = Clock::now() + std::chrono::milliseconds(250);  // 起流延迟
      return 0;
    case VIDIOC_STREAMOFF:
      c->streaming = false;
      return 0;
    default:
      return 0;
  }
}

// ============================ 测试场景与断言 ============================
namespace {

int g_fail = 0;

#define CHECK(cond, msg)                                                   \
  do {                                                                     \
    if (cond) {                                                            \
      std::printf("  ✅ %s\n", msg);                                       \
    } else {                                                               \
      std::printf("  ❌ %s （%s:%d）\n", msg, __FILE__, __LINE__);         \
      ++g_fail;                                                            \
    }                                                                      \
  } while (0)

void run_case(const char *name, MockCam cam, const std::string &dev,
              bool expect_ok, bool expect_conv) {
  std::printf("▶ 场景 %s\n", name);
  mock_reset();
  mock_register(dev, cam);
  r3d_ws::CamTuneConfig cfg;
  cfg.device = dev;
  cfg.side = "mock";
  const auto t0 = Clock::now();
  r3d_ws::CamTuneResult r = r3d_ws::cam_tune_one(cfg);
  const int ms = static_cast<int>(
      std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t0)
          .count());
  std::printf("  ok=%d converged=%d iters=%d mean=%.1f exp=%.2f gain=%d  (%dms)\n",
              (int)r.ok, (int)r.converged, r.iters, r.mean, r.exp_ms, r.gain, ms);
  CHECK(r.ok == expect_ok, "ok 符合预期");
  CHECK(r.converged == expect_conv, "converged 符合预期");
  CHECK(r.iters <= 3, "轮数 ≤3");
  CHECK(ms <= 3500, "耗时 ≤3.5s（预算 3s）");
  if (expect_conv)
    CHECK(std::fabs(r.mean - 70.0) <= 20.0, "末轮 mean 落在 70±20");
}

void case_pair() {
  std::printf("▶ 场景 pair（cam_tune_pair 双机并行）\n");
  mock_reset();
  MockCam right, left;  // config::TUNE_DEV0=video0(右) TUNE_DEV2=video2(左)
  right.scene = 200.0;  // 右亮
  left.scene = 40.0;    // 左暗（需升增益支；60 会落在死区一轮直过）
  left.gain_min = 0;    // 两台增益范围模拟实机差异
  left.gain_max = 128;
  mock_register("/dev/video0", right);
  mock_register("/dev/video2", left);
  const auto t0 = Clock::now();
  const bool ok = r3d_ws::cam_tune_pair();
  const int ms = static_cast<int>(
      std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t0)
          .count());
  std::printf("  pair_ok=%d (%dms)\n", (int)ok, ms);
  CHECK(ok, "双机均整定成功");
  CHECK(ms <= 3800, "总耗时 ≤3.8s（并行，含各自预算）");
}

}  // namespace

int main(int argc, char **argv) {
  std::vector<std::string> sel(argv + 1, argv + argc);
  auto want = [&](const char *s) {
    return sel.empty() ||
           std::find(sel.begin(), sel.end(), s) != sel.end();
  };

  if (want("dark")) {
    MockCam c;
    c.scene = 30.0;
    run_case("dark（默认参数下 mean≈30，应升曝光/增益收敛）", c, "/dev/video10",
             true, true);
  }
  if (want("bright")) {
    MockCam c;
    c.scene = 200.0;
    run_case("bright（mean≈200，应压曝光收敛）", c, "/dev/video11", true, true);
  }
  if (want("saturated")) {
    MockCam c;
    c.scene = 400.0;  // 真实亮度 400：mean 削顶 ~255、hl≈11%（mock 的 >235 撒高光）
    run_case("saturated（mean 削顶 ~255，削顶感知步长 3 轮内收敛）", c,
             "/dev/video16", true, true);
  }
  if (want("normal")) {
    MockCam c;
    c.scene = 70.0;
    run_case("normal（mean≈70，一轮即收敛）", c, "/dev/video12", true, true);
  }
  if (want("stalled")) {
    MockCam c;
    c.scene = 125.0;
    c.stall = true;
    run_case("stalled（流卡死，应回写默认参数且 ok=true）", c, "/dev/video13",
             true, false);
    mock_reset();
    mock_register("/dev/video13", c);
    r3d_ws::CamTuneConfig cfg;
    cfg.device = "/dev/video13";
    cfg.side = "mock";
    r3d_ws::CamTuneResult r = r3d_ws::cam_tune_one(cfg);
    CHECK(std::fabs(r.exp_ms - 7.8) < 0.01 && r.gain == 16,
          "卡死后参数=默认 7.8ms/16");
  }
  if (want("slowfps")) {
    MockCam c;
    c.scene = 60.0;
    c.fps = 3.0;
    run_case("slowfps（3fps，预算尽退出保留参数）", c, "/dev/video14", true,
             false);
  }
  if (want("noopen")) {
    std::printf("▶ 场景 noopen（设备不存在）\n");
    mock_reset();  // 不注册任何设备
    r3d_ws::CamTuneConfig cfg;
    cfg.device = "/dev/video99";
    cfg.side = "mock";
    r3d_ws::CamTuneResult r = r3d_ws::cam_tune_one(cfg);
    CHECK(!r.ok, "ok=false（打开失败）");
  }
  if (want("noexp")) {
    MockCam c;
    c.scene = 125.0;
    c.has_exp = false;
    std::printf("▶ 场景 noexp（无曝光控制项）\n");
    mock_reset();
    mock_register("/dev/video15", c);
    r3d_ws::CamTuneConfig cfg;
    cfg.device = "/dev/video15";
    cfg.side = "mock";
    r3d_ws::CamTuneResult r = r3d_ws::cam_tune_one(cfg);
    CHECK(!r.ok, "ok=false（无曝光控制项）");
  }
  if (want("pair")) case_pair();

  mock_reset();
  std::printf("\n%s（失败 %d 项）\n", g_fail ? "❌ 存在失败" : "✅ 全部通过", g_fail);
  return g_fail ? 1 : 0;
}
