#pragma once

// 主节点：双相机 V4L2 原生 MJPG 高效保存 + 独立采集线程 + Odometry 记录。
//
// 时序对齐（移植自 camera_two_value.py）：
//   trigger_loop 作为“主时钟”监听 Livox 共享内存纳秒戳（TsReader），当时间戳变化且
//   距上次触发 >= 1/fps 时，把该雷达 ns 写入 trig_.ns 并 ++trig_.gen、notify_all()。
//
// 每个相机独立的 capture 线程：
//   在 trig_.cv 上等待同一个 tick，被唤醒后用该雷达 ns 生成文件名、grab() 最新帧、
//   推入共享 IO 队列。两路相机并发抓取、互不阻塞，但文件名同源 → 与雷达/odom 时间轴对齐。
//
// io_loop（落盘线程）：
//   消费 IO 队列，把 MJPG 字节直接 write 到磁盘（零编码）。有界队列 + 背压丢老帧。
//
// bag：节点内 fork `ros2 bag record` 子进程录制指定话题，析构时 SIGINT 优雅 flush。

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include <sys/types.h>  // pid_t

#include <rclcpp/rclcpp.hpp>

#include "r3d_camodom/odom_writer.hpp"
#include "r3d_camodom/ts_reader.hpp"
#include "r3d_camodom/v4l2_camera.hpp"

namespace r3d_camodom {

class CaptureNode : public rclcpp::Node {
public:
  CaptureNode();
  ~CaptureNode() override;

  CaptureNode(const CaptureNode &) = delete;
  CaptureNode &operator=(const CaptureNode &) = delete;

private:
  void trigger_loop();
  void capture_loop(V4L2Camera &cam, const std::string &img_dir, const char *label);
  void io_loop();
  // 入共享 IO 队列 + 背压（超 max_pending_ 丢老帧）。两路相机复用。
  void enqueue_save(const std::string &path, std::vector<uint8_t> data, const char *label);

  // ---- bag 子进程 ----
  void start_bag();
  void stop_bag();

  // ---- 参数 ----
  std::string odom_topic_;
  std::string output_dir_;
  std::string img_dir0_;
  std::string img_dir1_;
  CamConfig cam0_cfg_;
  CamConfig cam2_cfg_;
  int target_fps_ = 10;
  std::size_t max_pending_ = 16;
  bool record_bag_ = true;
  std::string bag_topics_;

  // ---- 组件 ----
  std::unique_ptr<V4L2Camera> cam0_;
  std::unique_ptr<V4L2Camera> cam2_;
  std::unique_ptr<OdomWriter> odom_;
  TsReader ts_reader_;

  // ---- 雷达 tick 共享触发状态 ----
  struct TriggerState {
    std::mutex mtx;
    std::condition_variable cv;
    int64_t ns = 0;       // 当前 tick 的 Livox 纳秒戳
    uint64_t gen = 0;     // 单调递增，每新 tick +1
  } trig_;

  // ---- 线程 ----
  std::atomic<bool> running_{false};
  std::thread trig_th_;
  std::thread cap0_th_;
  std::thread cap2_th_;

  // ---- bag 子进程句柄 ----
  std::string bag_uri_;
  pid_t bag_pid_ = -1;

  // ---- IO 队列 ----
  struct SaveTask {
    std::string path;
    std::vector<uint8_t> data;
  };
  std::queue<SaveTask> io_q_;
  std::mutex io_mtx_;
  std::condition_variable io_cv_;
  std::thread io_th_;
  std::size_t dropped_ = 0;
};

}  // namespace r3d_camodom
