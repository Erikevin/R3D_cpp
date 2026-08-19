#include "r3d_camodom/capture_node.hpp"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <sstream>
#include <utility>

#include <fcntl.h>
#include <signal.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include <filesystem>

namespace r3d_camodom {

namespace {

std::string timestamp_dir() {
  std::time_t t = std::time(nullptr);
  std::tm tm{};
  ::localtime_r(&t, &tm);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
  return std::string(buf);
}

std::string home_dir() {
  const char *h = std::getenv("HOME");
  return (h && h[0]) ? std::string(h) : std::string("/root");
}

// POSIX 循环写满，避免 ofstream 的额外缓冲层。
bool write_all(const std::string &path, const uint8_t *data, size_t n) {
  int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (fd < 0) {
    return false;
  }
  size_t off = 0;
  while (off < n) {
    ssize_t w = ::write(fd, data + off, n - off);
    if (w < 0) {
      if (errno == EINTR) {
        continue;
      }
      ::close(fd);
      return false;
    }
    off += static_cast<size_t>(w);
  }
  ::close(fd);
  return true;
}

}  // namespace

CaptureNode::CaptureNode() : rclcpp::Node("image_odom_recorder") {
  // ---- 参数 ----
  this->declare_parameter<std::string>("odom_topic", "/lio_sam/mapping/odometry");
  this->declare_parameter<std::string>("output_dir", "");
  this->declare_parameter<std::string>("camera_device0", "/dev/video0");
  this->declare_parameter<std::string>("camera_device2", "/dev/video2");
  this->declare_parameter<int>("image_width", 2592);
  this->declare_parameter<int>("image_height", 1944);
  this->declare_parameter<int>("fps", 10);        // 相机硬件抓图帧率（V4L2 driver）
  this->declare_parameter<int>("save_fps", 5);    // 目标保存帧率(Hz)；计数抽帧周期=cam_fps/save_fps
  this->declare_parameter<int>("jpeg_quality", 90);
  this->declare_parameter<bool>("auto_exposure", true);  // V4L2 自动曝光开关
  this->declare_parameter<bool>("auto_wb", true);        // V4L2 自动白平衡开关
  this->declare_parameter<int>("max_pending", 16);
  this->declare_parameter<bool>("record_bag", true);
  this->declare_parameter<bool>("save_odom", true);  // 是否落盘 odom.txt（false 时仅录 bag）
  this->declare_parameter<std::string>("bag_topics", "/livox/imu /livox/lidar");
  // ---- bag 存储调优（大场景防丢帧）----
  // --max-cache-size：rosbag2 默认缓存仅 100MiB（双缓冲），磁盘被图像/PCD 写入打满、
  // 写线程排空不及即溢出丢帧；加大以吸收 IO 突刺。0=不传该参数（用默认 100MiB）。
  this->declare_parameter<int>("bag_max_cache_mb", 256);
  // 单文件分包上限（MB），超限切新 .db3，防单库随录制时长无限膨胀。0=不分包。
  this->declare_parameter<int>("bag_max_file_mb", 2000);

  odom_topic_ = this->get_parameter("odom_topic").as_string();
  output_dir_ = this->get_parameter("output_dir").as_string();
  const std::string dev0 = this->get_parameter("camera_device0").as_string();
  const std::string dev2 = this->get_parameter("camera_device2").as_string();
  const int w = this->get_parameter("image_width").as_int();
  const int h = this->get_parameter("image_height").as_int();
  cam_fps_ = this->get_parameter("fps").as_int();
  save_fps_ = this->get_parameter("save_fps").as_int();
  const int q = this->get_parameter("jpeg_quality").as_int();
  const bool ae = this->get_parameter("auto_exposure").as_bool();
  const bool awb = this->get_parameter("auto_wb").as_bool();
  max_pending_ = static_cast<std::size_t>(this->get_parameter("max_pending").as_int());
  record_bag_ = this->get_parameter("record_bag").as_bool();
  const bool save_odom = this->get_parameter("save_odom").as_bool();
  bag_topics_ = this->get_parameter("bag_topics").as_string();
  bag_max_cache_mb_ = static_cast<int>(this->get_parameter("bag_max_cache_mb").as_int());
  bag_max_file_mb_ = static_cast<int>(this->get_parameter("bag_max_file_mb").as_int());

  cam0_cfg_ = CamConfig{dev0, w, h, cam_fps_, ae, awb, q};
  cam2_cfg_ = CamConfig{dev2, w, h, cam_fps_, ae, awb, q};

  // ---- 输出目录 ----
  // output_dir_ 作为存储根目录：默认 ~/Desktop/save_pcd，或由外部 output_dir 参数指定
  // （桥从 mapping_commands.json 的 data_dir 注入）。无论哪种，都在其下新建 <时间戳>/
  // 子目录，确保每次录制互不覆盖；images/camera{0,1}、odom.txt、rosbag2/ 全在此子目录下。
  if (output_dir_.empty()) {
    output_dir_ = home_dir() + "/Desktop/save_pcd";
  }
  output_dir_ += "/" + timestamp_dir();
  img_dir0_ = output_dir_ + "/images/camera0";
  img_dir1_ = output_dir_ + "/images/camera1";
  std::filesystem::create_directories(img_dir0_);
  std::filesystem::create_directories(img_dir1_);

  // ---- 相机 ----
  cam0_ = std::make_unique<V4L2Camera>(cam0_cfg_);
  cam2_ = std::make_unique<V4L2Camera>(cam2_cfg_);
  if (cam0_->open()) {
    RCLCPP_INFO(this->get_logger(), "✅ camera0 就绪: %s %dx%d MJPG", dev0.c_str(),
                cam0_->width(), cam0_->height());
  } else {
    RCLCPP_ERROR(this->get_logger(), "❌ camera0 打开失败: %s (sudo chmod 666 %s)", dev0.c_str(),
                 dev0.c_str());
  }
  if (cam2_->open()) {
    RCLCPP_INFO(this->get_logger(), "✅ camera1 就绪: %s %dx%d MJPG", dev2.c_str(),
                cam2_->width(), cam2_->height());
  } else {
    RCLCPP_ERROR(this->get_logger(), "❌ camera1 打开失败: %s (sudo chmod 666 %s)", dev2.c_str(),
                 dev2.c_str());
  }

  // 错误帧 / MJPG 标记异常告警走 ROS WARN（V4L2Camera 默认打 stderr）
  auto cam_warn = [this](const std::string &msg) {
    RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
  };
  cam0_->set_logger(cam_warn);
  cam2_->set_logger(cam_warn);

  // ---- Odometry ----
  // save_odom=false 时跳过 odom.txt 落盘（仅录 bag），减少不必要的 IO。
  std::string odom_path;  // save_odom=false 时保持空
  if (save_odom) {
    odom_path = output_dir_ + "/odom.txt";
    odom_ = std::make_unique<OdomWriter>(this, odom_topic_, odom_path);
    odom_->start();
  } else {
    RCLCPP_INFO(this->get_logger(), "📝 odom.txt 落盘已关闭 (save_odom=false)");
  }

  // ---- Livox 时间戳共享内存：文件名第二列读其第二个时间戳（系统 UTC ns）。
  // 构造时打开映射（get_sys_ns 不做懒 open，避免双采集线程并发 open 竞争）；
  // livox 未就绪/映射不足 32 字节时第二列为 0。
  if (ts_reader_.open()) {
    RCLCPP_INFO(this->get_logger(), "⏱️ Livox timeshare 已映射（第二列读其 UTC 时间戳）");
  } else {
    RCLCPP_WARN(this->get_logger(), "⚠️ Livox timeshare 未就绪，图片第二列时间戳将为 0");
  }

  // ---- ros2bag 子进程 ----
  if (record_bag_ && !bag_topics_.empty()) {
    bag_uri_ = output_dir_ + "/rosbag2";
    start_bag();
  } else if (!record_bag_) {
    RCLCPP_INFO(this->get_logger(), "📦 bag 录制已关闭 (record_bag=false)");
  }

  // ---- 线程：IO worker + 双相机事件驱动采集（各自 grab 等帧到达 + 曝光门限抽帧）----
  running_ = true;
  io_th_ = std::thread([this] { io_loop(); });
  cap0_th_ = std::thread([this] { capture_loop(*cam0_, img_dir0_, "camera0"); });
  cap2_th_ = std::thread([this] { capture_loop(*cam2_, img_dir1_, "camera1"); });

  const int save_period =
      (save_fps_ > 0 && save_fps_ <= cam_fps_) ? (cam_fps_ / save_fps_) : 1;
  RCLCPP_INFO(this->get_logger(),
              "✅ 双摄像头启动成功 | 事件驱动采集: 硬件 %dHz，计数抽帧保存 %dHz（每 %d 帧存 1）",
              cam_fps_, save_fps_, save_period);
  RCLCPP_INFO(this->get_logger(), "📷 camera0: %s → %s", dev0.c_str(), img_dir0_.c_str());
  RCLCPP_INFO(this->get_logger(), "📷 camera1: %s → %s", dev2.c_str(), img_dir1_.c_str());
  if (save_odom) {
    RCLCPP_INFO(this->get_logger(), "📝 odom: %s → %s", odom_topic_.c_str(), odom_path.c_str());
  }
}

CaptureNode::~CaptureNode() {
  running_ = false;
  // 唤醒所有阻塞在条件变量上的线程：IO 落盘线程、可能仍在起步栅栏等待的采集线程。
  // 其余采集线程阻塞在 grab() 的 poll()，running_=false 后最多 ~poll timeout(1s) 内自行退出。
  io_cv_.notify_all();
  start_gate_.cv.notify_all();
  if (cap0_th_.joinable()) {
    cap0_th_.join();
  }
  if (cap2_th_.joinable()) {
    cap2_th_.join();
  }
  if (io_th_.joinable()) {
    io_th_.join();
  }
  stop_bag();
  ts_reader_.close();
  if (odom_) {
    odom_->stop();
  }
  if (cam0_) {
    cam0_->release();
  }
  if (cam2_) {
    cam2_->release();
  }
  RCLCPP_INFO(this->get_logger(), "🛑 节点已关闭，资源已释放");
}

void CaptureNode::enqueue_save(const std::string &path, std::vector<uint8_t> data,
                               const char *label) {
  bool did_drop = false;
  std::size_t drop_delta = 0;
  std::size_t total_dropped = 0;
  {
    std::lock_guard<std::mutex> lk(io_mtx_);
    io_q_.push(SaveTask{path, std::move(data)});
    const std::size_t before = dropped_;
    while (io_q_.size() > max_pending_) {
      io_q_.pop();
      ++dropped_;
    }
    if (dropped_ != before) {
      did_drop = true;
      drop_delta = dropped_ - before;
      total_dropped = dropped_;
    }
  }
  if (did_drop) {
    RCLCPP_WARN(this->get_logger(), "[%s] 磁盘跟不上，丢弃 %zu 帧（累计 %zu）", label,
                drop_delta, total_dropped);
  }
  io_cv_.notify_one();
}

void CaptureNode::capture_loop(V4L2Camera &cam, const std::string &img_dir,
                               const char *label) {
  // 计数抽帧：save_fps_ 为目标保存帧率(Hz)，周期 period = cam_fps_/save_fps_（每 period 帧存 1）。
  // save_fps_<=0 或 >cam_fps_ 时退化为全帧率保存（period=1，并规避除零）。
  const int period = (save_fps_ > 0 && save_fps_ <= cam_fps_) ? (cam_fps_ / save_fps_) : 1;
  RCLCPP_INFO(this->get_logger(),
              "⌚ [%s] 事件驱动采集线程启动：grab 等帧到达 + 计数抽帧 %dHz（每 %d 帧存 1）", label,
              save_fps_, period);
  uint64_t frame_idx = 0;  // 本路相机已到达帧计数（含被抽帧丢弃的帧）
  std::size_t save_count = 0;

  // 首次步调对齐：等两路采集线程都就绪，再同步开始 grab 计数。两路 UVC 独立启动、
  // 第一帧曝光时刻可能相差很大；若不对齐，后续 period 计数抽帧配对的两路帧会严重错位。
  // 对齐后两路 frame_idx 步调一致，配对帧曝光时刻仅差两路曝光边沿（毫秒级）。
  {
    std::unique_lock<std::mutex> lk(start_gate_.mtx);
    ++start_gate_.started;
    start_gate_.cv.wait(lk, [&] { return start_gate_.started >= 2 || !running_; });
  }
  start_gate_.cv.notify_all();
  RCLCPP_INFO(this->get_logger(), "⌚ [%s] 两路已同步起步，开始计数抽帧", label);

  while (running_) {
    std::vector<uint8_t> buf;
    int64_t frame_ns = 0;
    // grab() 内部 poll() 阻塞等帧到达（USB 帧完成中断在用户态的投射），排空取最新帧；
    // 返回 false = 错误帧 / 超时 → 继续等下一帧（poll 会阻塞，非忙等）。
    if (!cam.grab(buf, &frame_ns)) {
      continue;
    }
    // 帧到达即刻采样 timeshare 第二个时间戳（系统 UTC ns），与本帧绑定：用户态无法
    // 在软中断里执行，这是离 v4l2 时间戳打点最近的采样位置（仅隔 DQBUF→返回的微秒
    // 级路径）。硬同步下这即"本帧曝光时刻雷达侧正在发布的时戳"。抽帧丢弃的帧同样
    // 采样但不用（一次 mmap 读，开销可忽略）；未就绪时为 0。
    const int64_t ts2_ns = ts_reader_.get_sys_ns();
    // 计数抽帧：仅每 period 帧保存一次，帧间均匀分布，不依赖时间戳时基/抖动。
    if (++frame_idx % period != 0) {
      continue;
    }

    // frame_ns 为 CLOCK_MONOTONIC 时基（开机起算，UVC 驱动曝光时刻）→ 转系统墙钟：
    // 同拍读两时钟得偏移再加回。墙钟可能被 settimeofday/NTP 调整，偏移不恒定，
    // 每帧现算（两次 clock_gettime 开销纳秒级），曝光到读钟仅隔 USB 传输（毫秒内）。
    struct timespec w{}, m{};
    ::clock_gettime(CLOCK_REALTIME, &w);
    ::clock_gettime(CLOCK_MONOTONIC, &m);
    const int64_t wall_now_ns =
        static_cast<int64_t>(w.tv_sec) * 1000000000LL + w.tv_nsec;
    const int64_t mono_now_ns =
        static_cast<int64_t>(m.tv_sec) * 1000000000LL + m.tv_nsec;
    const int64_t wall_ns = frame_ns + wall_now_ns - mono_now_ns;

    // 文件名第一段 = 驱动曝光时刻转墙钟纳秒（各相机独立）；第二段 = 本帧到达时刻采样
    // 的 timeshare UTC 纳秒（见上，左右各自读、无握手等待 → 差毫秒级）。
    char name[64];
    std::snprintf(name, sizeof(name), "%lld_%lld.jpg", static_cast<long long>(wall_ns),
                  static_cast<long long>(ts2_ns));
    enqueue_save(img_dir + "/" + name, std::move(buf), label);
    if (++save_count % 10 == 0) {
      std::size_t cur_dropped = 0;
      {
        std::lock_guard<std::mutex> lk(io_mtx_);
        cur_dropped = dropped_;
      }
      RCLCPP_INFO(this->get_logger(), "[%s] 已保存: %zu 帧 | 全局丢弃: %zu", label, save_count,
                  cur_dropped);
    }
  }
  RCLCPP_INFO(this->get_logger(), "[%s] 采集线程退出，共保存 %zu 帧", label, save_count);
}

void CaptureNode::start_bag() {
  // 按空白拆分话题列表
  std::vector<std::string> topics;
  {
    std::istringstream iss(bag_topics_);
    std::string t;
    while (iss >> t) {
      topics.push_back(t);
    }
  }
  if (topics.empty()) {
    return;
  }

  // argv: ros2 bag record -o <uri> [--max-cache-size N] [-b N] <topic...>
  // 大场景丢帧防护（勿加 PRAGMA 调优：rosbag2 的 sqlite 默认已是 journal_mode=MEMORY
  // + synchronous=OFF 最快档，改 WAL/NORMAL 反而更慢）：
  //   1) 缓存加大到 bag_max_cache_mb —— 默认 100MiB，图像/PCD 抢占磁盘导致写线程
  //      排空不及时即溢出丢帧（丢帧集中在磁盘繁忙时段）；
  //   2) 按 bag_max_file_mb 分包 —— 单 .db3 随录制时长无限膨胀，大场景长时间录制
  //      写入持续变慢，分包后每文件重置到轻量状态。
  std::vector<std::string> owned;
  owned.emplace_back("ros2");
  owned.emplace_back("bag");
  owned.emplace_back("record");
  owned.emplace_back("-o");
  owned.emplace_back(bag_uri_);
  if (bag_max_cache_mb_ > 0) {
    owned.emplace_back("--max-cache-size");
    owned.emplace_back(std::to_string(static_cast<long long>(bag_max_cache_mb_) * 1024LL * 1024LL));
  }
  if (bag_max_file_mb_ > 0) {
    owned.emplace_back("-b");
    owned.emplace_back(std::to_string(static_cast<long long>(bag_max_file_mb_) * 1024LL * 1024LL));
  }
  for (const auto &t : topics) {
    owned.emplace_back(t);
  }
  std::vector<char *> argv;
  argv.reserve(owned.size() + 1);
  for (auto &s : owned) {
    argv.push_back(s.data());
  }
  argv.push_back(nullptr);

  pid_t pid = fork();
  if (pid < 0) {
    RCLCPP_ERROR(this->get_logger(), "❌ fork bag 子进程失败: %s", std::strerror(errno));
    return;
  }
  if (pid == 0) {
    // 子进程：立即 exec，不触碰父进程任何锁/堆
    execvp("ros2", argv.data());
    _exit(127);  // 仅 exec 失败才到达
  }
  bag_pid_ = pid;
  RCLCPP_INFO(this->get_logger(),
              "🎬 bag 录制子进程已启动 pid=%d → %s (话题: %s | 缓存 %dMB%s)", pid,
              bag_uri_.c_str(), bag_topics_.c_str(),
              (bag_max_cache_mb_ > 0) ? bag_max_cache_mb_ : 100,
              (bag_max_file_mb_ > 0)
                  ? (std::string(", 每 ") + std::to_string(bag_max_file_mb_) + "MB 分包").c_str()
                  : ", 不分包");
}

void CaptureNode::stop_bag() {
  if (bag_pid_ <= 0) {
    return;
  }
  // 先 SIGINT，让 rosbag2 优雅 flush 落盘（关闭 bag 元数据）
  if (::kill(bag_pid_, SIGINT) == 0) {
    RCLCPP_INFO(this->get_logger(), "🛑 等待 bag 子进程 pid=%d 落盘...", bag_pid_);
  }
  // 轮询最多 ~9s：大场景长录制缓存积压多、且磁盘可能正被图像/PCD 写入占用，3s 内
  // flush 不完就被下面的 SIGKILL 打穿（rosbag2 默认 journal_mode=MEMORY，被杀时未
  // 落盘部分直接丢失）。上层 close_all_windows 阶段 1 已先停 livox 掐断数据源，
  // flush 量有限，放宽窗口不会明显拖慢停止流程。
  for (int i = 0; i < 900; ++i) {
    int status = 0;
    const pid_t r = ::waitpid(bag_pid_, &status, WNOHANG);
    if (r == bag_pid_) {
      bag_pid_ = -1;
      return;
    }
    if (r < 0) {  // ECHILD 等
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  // 兜底强杀，回收避免僵尸
  RCLCPP_WARN(this->get_logger(), "⚠️ bag 子进程未及时退出，SIGKILL 兜底 pid=%d", bag_pid_);
  ::kill(bag_pid_, SIGKILL);
  int status = 0;
  ::waitpid(bag_pid_, &status, 0);
  bag_pid_ = -1;
}

void CaptureNode::io_loop() {
  while (true) {
    SaveTask t;
    {
      std::unique_lock<std::mutex> lk(io_mtx_);
      io_cv_.wait(lk, [this] { return !running_ || !io_q_.empty(); });
      if (!running_ && io_q_.empty()) {
        return;
      }
      t = std::move(io_q_.front());
      io_q_.pop();
    }
    // 直接写原始 MJPG 字节，零编码开销。
    write_all(t.path, t.data.data(), t.data.size());
  }
}

}  // namespace r3d_camodom
