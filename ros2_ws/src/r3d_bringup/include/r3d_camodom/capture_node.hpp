#pragma once

// 主节点：双相机 V4L2 原生 MJPG 高效保存 + 独立采集线程 + Odometry 记录。
//
// 采集模型（纯事件驱动）：
//   每个相机独立的 capture 线程死循环调用 grab()，grab() 内部 poll() 阻塞等“帧到达”
//   （USB 帧完成中断在用户态的投射），排空取最新帧；再以驱动曝光时刻(v4l2_buffer.timestamp)
//   做计数抽帧——save_fps 为目标保存帧率(Hz)，每 cam_fps/save_fps 帧保存 1 帧，
//   帧间均匀分布且不依赖时间戳时基；“曝光→取走”延迟约一次 USB 传输+DQBUF（毫秒级）。
//   文件名 = <墙钟微秒>_<timeshare UTC纳秒>：第一段 = 驱动曝光时刻转墙钟总微秒（各
//   相机独立）；第二段 = 帧到达时刻（grab 返回即采样）读 timeshare 第二个时间戳，
//   绑定本帧、无跨相机握手等待（左右第二段差毫秒级）。
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
  // 事件驱动采集循环：死循环 grab() 等帧到达，按曝光时刻门限抽帧落盘。
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
  int cam_fps_ = 10;    // V4L2 硬件抓图帧率（cfg_.fps）
  int save_fps_ = 2;    // 目标保存帧率(Hz)；计数抽帧周期 = cam_fps_/save_fps_（每 period 帧存 1）
  std::size_t max_pending_ = 16;
  bool record_bag_ = true;
  std::string bag_topics_;
  int bag_max_cache_mb_ = 256;  // --max-cache-size（MB）；0=用 rosbag2 默认 100MiB
  int bag_max_file_mb_ = 2000;  // 单文件分包上限（MB）；0=不分包

  // ---- 组件 ----
  std::unique_ptr<V4L2Camera> cam0_;
  std::unique_ptr<V4L2Camera> cam2_;
  std::unique_ptr<OdomWriter> odom_;
  TsReader ts_reader_;  // 读 timeshare 第二个时间戳（系统 UTC ns）作文件名第二列

  // ---- 线程 ----
  std::atomic<bool> running_{false};
  std::thread cap0_th_;
  std::thread cap2_th_;

  // ---- 首次步调对齐栅栏：等两路采集线程都就绪，再同步开始 grab 计数 ----
  struct StartGate {
    std::mutex mtx;
    std::condition_variable cv;
    int started = 0;  // 已就绪的采集线程数（1 或 2）
  } start_gate_;

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
