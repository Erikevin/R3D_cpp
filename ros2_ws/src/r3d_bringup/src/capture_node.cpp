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
  this->declare_parameter<int>("fps", 10);
  this->declare_parameter<int>("jpeg_quality", 90);
  this->declare_parameter<int>("max_pending", 16);
  this->declare_parameter<bool>("record_bag", true);
  this->declare_parameter<std::string>("bag_topics", "/livox/imu /livox/lidar");

  odom_topic_ = this->get_parameter("odom_topic").as_string();
  output_dir_ = this->get_parameter("output_dir").as_string();
  const std::string dev0 = this->get_parameter("camera_device0").as_string();
  const std::string dev2 = this->get_parameter("camera_device2").as_string();
  const int w = this->get_parameter("image_width").as_int();
  const int h = this->get_parameter("image_height").as_int();
  target_fps_ = this->get_parameter("fps").as_int();
  const int q = this->get_parameter("jpeg_quality").as_int();
  max_pending_ = static_cast<std::size_t>(this->get_parameter("max_pending").as_int());
  record_bag_ = this->get_parameter("record_bag").as_bool();
  bag_topics_ = this->get_parameter("bag_topics").as_string();

  cam0_cfg_ = CamConfig{dev0, w, h, target_fps_, true, true, q};
  cam2_cfg_ = CamConfig{dev2, w, h, target_fps_, true, true, q};

  // ---- 输出目录 ----
  if (output_dir_.empty()) {
    output_dir_ = home_dir() + "/Desktop/save_pcd/" + timestamp_dir();
  }
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

  // ---- Odometry ----
  const std::string odom_path = output_dir_ + "/odom.txt";
  odom_ = std::make_unique<OdomWriter>(this, odom_topic_, odom_path);
  odom_->start();

  // ---- Livox 时间戳共享内存 ----
  if (ts_reader_.open()) {
    RCLCPP_INFO(this->get_logger(), "⏱️ Livox timeshare 已映射，雷达时间戳触发就绪");
  } else {
    // 非致命：get_ns() 会在每次调用时懒重试 open()，雷达栈起来后自动恢复。
    RCLCPP_WARN(this->get_logger(),
                "⚠️ Livox timeshare 暂不可读（/home/$USER/timeshare 缺失？），等待雷达栈启动");
  }

  // ---- ros2bag 子进程 ----
  if (record_bag_ && !bag_topics_.empty()) {
    bag_uri_ = output_dir_ + "/rosbag2";
    start_bag();
  } else if (!record_bag_) {
    RCLCPP_INFO(this->get_logger(), "📦 bag 录制已关闭 (record_bag=false)");
  }

  // ---- 线程：IO/相机 worker 先就位，trigger 最后派发 tick ----
  running_ = true;
  io_th_ = std::thread([this] { io_loop(); });
  cap0_th_ = std::thread([this] { capture_loop(*cam0_, img_dir0_, "camera0"); });
  cap2_th_ = std::thread([this] { capture_loop(*cam2_, img_dir1_, "camera1"); });
  trig_th_ = std::thread([this] { trigger_loop(); });

  RCLCPP_INFO(this->get_logger(), "✅ 双摄像头启动成功 | 目标帧率: %dHz (V4L2 原生 MJPG)",
              target_fps_);
  RCLCPP_INFO(this->get_logger(), "📷 camera0: %s → %s", dev0.c_str(), img_dir0_.c_str());
  RCLCPP_INFO(this->get_logger(), "📷 camera1: %s → %s", dev2.c_str(), img_dir1_.c_str());
  RCLCPP_INFO(this->get_logger(), "📝 odom: %s → %s", odom_topic_.c_str(), odom_path.c_str());
}

CaptureNode::~CaptureNode() {
  running_ = false;
  // 唤醒所有阻塞在条件变量上的线程
  trig_.cv.notify_all();
  io_cv_.notify_all();
  if (trig_th_.joinable()) {
    trig_th_.join();
  }
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

void CaptureNode::trigger_loop() {
  RCLCPP_INFO(this->get_logger(), "⌚ 监听 Livox 时间戳，双相机同步触发...");
  const double min_interval = 1.0 / static_cast<double>(target_fps_);
  int64_t last_ns = 0;
  auto last_fire = std::chrono::steady_clock::now();
  bool warned_no_ts = false;
  std::size_t fire_count = 0;

  while (running_) {
    const int64_t cur_ns = ts_reader_.get_ns();
    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - last_fire).count();

    // 与 Python capture_loop 一致：时间戳变化 + 达到最小间隔才触发
    if (cur_ns > 0 && cur_ns != last_ns && elapsed >= min_interval) {
      last_ns = cur_ns;
      last_fire = now;
      {
        std::lock_guard<std::mutex> lk(trig_.mtx);
        trig_.ns = cur_ns;
        ++trig_.gen;
      }
      trig_.cv.notify_all();  // 唤醒两路相机线程
      if (++fire_count % 10 == 0) {
        RCLCPP_INFO(this->get_logger(), "✅ 已触发 %zu 个 tick | %dHz", fire_count, target_fps_);
      }
    } else if (cur_ns == 0 && !warned_no_ts) {
      warned_no_ts = true;
      RCLCPP_WARN(this->get_logger(),
                  "⚠️ Livox 时间戳未就绪（timeshare 缺失？），等待雷达栈启动...");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  RCLCPP_INFO(this->get_logger(), "⌚ trigger 线程退出，共触发 %zu 个 tick", fire_count);
}

void CaptureNode::capture_loop(V4L2Camera &cam, const std::string &img_dir,
                               const char *label) {
  RCLCPP_INFO(this->get_logger(), "⌚ [%s] 采集线程启动，等待雷达时间戳触发", label);
  uint64_t gen_seen = 0;
  std::size_t save_count = 0;

  while (running_) {
    // 等待下一个雷达 tick（同一 tick 唤醒两路 → 同名对齐）
    int64_t tick_ns = 0;
    {
      std::unique_lock<std::mutex> lk(trig_.mtx);
      trig_.cv.wait(lk, [this, &gen_seen] {
        return !running_ || trig_.gen != gen_seen;
      });
      if (!running_) {
        break;
      }
      gen_seen = trig_.gen;
      tick_ns = trig_.ns;
    }

    // 用雷达纳秒戳生成文件名（与 Python 一致：%.6f.jpg）
    char name[64];
    std::snprintf(name, sizeof(name), "%.6f.jpg", tick_ns / 1e9);
    const std::string path = img_dir + "/" + name;

    std::vector<uint8_t> buf;
    if (cam.grab(buf)) {
      enqueue_save(path, std::move(buf), label);
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

  // argv: ros2 bag record -o <uri> <topic...>
  std::vector<std::string> owned;
  owned.emplace_back("ros2");
  owned.emplace_back("bag");
  owned.emplace_back("record");
  owned.emplace_back("-o");
  owned.emplace_back(bag_uri_);
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
  RCLCPP_INFO(this->get_logger(), "🎬 bag 录制子进程已启动 pid=%d → %s (话题: %s)", pid,
              bag_uri_.c_str(), bag_topics_.c_str());
}

void CaptureNode::stop_bag() {
  if (bag_pid_ <= 0) {
    return;
  }
  // 先 SIGINT，让 rosbag2 优雅 flush 落盘（关闭 bag 元数据）
  if (::kill(bag_pid_, SIGINT) == 0) {
    RCLCPP_INFO(this->get_logger(), "🛑 等待 bag 子进程 pid=%d 落盘...", bag_pid_);
  }
  // 轮询最多 ~3s
  for (int i = 0; i < 300; ++i) {
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
