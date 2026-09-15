// WsBridgeNode implementation — ROS2 subscriptions, time-sync, command dispatch.

#include "r3d_ws/ws_bridge_node.hpp"

#include "r3d_ws/cam_tuner.hpp"
#include "r3d_ws/config.hpp"
#include "r3d_ws/logger.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <cmath>
#include <thread>
#include <utility>

namespace r3d_ws {

WsBridgeNode::WsBridgeNode() : rclcpp::Node("ws_sync_sender") {
  state_ = std::make_shared<SharedState>();
  mapping_ = std::make_shared<MappingControl>();
  server_ = std::make_unique<WsServer>(state_, config::WS_PORT);

  // BEST_EFFORT, keep_last(50) — matches web_a.py QoS.
  auto qos = rclcpp::QoS(rclcpp::KeepLast(50)).best_effort();

  img_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      "/camera/image_raw/compressed", qos,
      std::bind(&WsBridgeNode::on_image, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/lio_sam/mapping/odometry", qos,
      std::bind(&WsBridgeNode::on_odom, this, std::placeholders::_1));
  lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lio_sam/mapping/cloud_registered", qos,
      std::bind(&WsBridgeNode::on_lidar, this, std::placeholders::_1));

  sync_timer_ = create_wall_timer(std::chrono::milliseconds(50),
                                  std::bind(&WsBridgeNode::try_sync, this));

  // On connect: push device info (send_device_info_on_connect).
  server_->set_connect_handler(
      [](WsServer::ResponseFn respond) {
        respond({{"type", "deviceInfo"}, {"data", MappingControl::device_info()}});
        log::info("✅ 主动推送设备信息成功");
      });

  // Command dispatch (handle_command).
  server_->set_command_handler(
      [this](const nlohmann::json& msg, WsServer::ResponseFn respond) {
        handle_command(msg, std::move(respond));
      });

  server_->run();

  log::info("🚀 WebSocket 服务已启动（C++ 版，无历史数据残留）");
  log::info(std::string("📡 ws://0.0.0.0:") + std::to_string(config::WS_PORT) + "/data");
  log::info(std::string("📷 摄像头传输: ") +
            (config::ENABLE_CAMERA_STREAM ? "开启" : "关闭"));
}

WsBridgeNode::~WsBridgeNode() {
  if (server_) server_->stop();
}

// ---- subscriptions ---------------------------------------------------------
void WsBridgeNode::on_image(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
  if constexpr (!config::ENABLE_CAMERA_STREAM) {
    return;  // camera streaming disabled — leave latest_image empty
  }
  try {
    cv::Mat raw(1, static_cast<int>(msg->data.size()), CV_8U,
                const_cast<uchar*>(msg->data.data()));
    cv::Mat img = cv::imdecode(raw, cv::IMREAD_COLOR);
    if (img.empty()) return;
    cv::resize(img, img, cv::Size(config::SEND_WIDTH, config::SEND_HEIGHT), 0, 0, cv::INTER_LINEAR);
    std::vector<uchar> enc;
    cv::imencode(".jpg", img, enc);
    std::lock_guard<std::mutex> lk(state_->m);
    state_->latest_image = ImageData{base64_encode(enc)};
  } catch (const std::exception& e) {
    log::warn(std::string("图像处理错误: ") + e.what());
    std::lock_guard<std::mutex> lk(state_->m);
    state_->latest_image.reset();
  }
}

void WsBridgeNode::on_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
  OdomData o;
  o.t = stamp_to_sec(msg->header.stamp);
  o.x = msg->pose.pose.position.x;
  o.y = msg->pose.pose.position.y;
  o.z = msg->pose.pose.position.z;
  o.qw = msg->pose.pose.orientation.w;
  o.qx = msg->pose.pose.orientation.x;
  o.qy = msg->pose.pose.orientation.y;
  o.qz = msg->pose.pose.orientation.z;
  std::lock_guard<std::mutex> lk(state_->m);
  state_->odom_buffer.push_back(o);
  if (state_->odom_buffer.size() > config::ODOM_BUFFER_CAP) state_->odom_buffer.pop_front();
}

void WsBridgeNode::on_lidar(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  LidarData l = parse_pointcloud2(*msg, stamp_to_sec(msg->header.stamp));
  std::lock_guard<std::mutex> lk(state_->m);
  state_->lidar_buffer.push_back(std::move(l));
  if (state_->lidar_buffer.size() > config::LIDAR_BUFFER_CAP) state_->lidar_buffer.pop_front();
}

// ---- time sync (try_sync) --------------------------------------------------
void WsBridgeNode::try_sync() {
  const OdomData* best_odom = nullptr;
  const LidarData* best_lidar = nullptr;
  double best_diff = config::MAX_TIME_DIFF;  // only accept pairs under the threshold
  {
    std::lock_guard<std::mutex> lk(state_->m);
    if (state_->odom_buffer.empty() || state_->lidar_buffer.empty()) return;
    // Iterate NEWEST-first. odom & lidar usually share a stamp each tick, so
    // several pairs tie at diff≈0; iterating newest-first makes the newest
    // matched pair win (the strict `<` keeps it once set). The original
    // web_a.py iterated oldest-first, which locked onto the oldest pair in the
    // buffer and shipped ~buffer-depth (≈2 s) stale data.
    for (auto io = state_->odom_buffer.rbegin(); io != state_->odom_buffer.rend(); ++io) {
      for (auto il = state_->lidar_buffer.rbegin(); il != state_->lidar_buffer.rend(); ++il) {
        double diff = std::fabs(io->t - il->t);
        if (diff < best_diff) {
          best_diff = diff;
          best_odom = &*io;
          best_lidar = &*il;
        }
      }
    }
    if (best_odom && best_lidar) {
      state_->latest_odom = *best_odom;
      state_->latest_lidar = *best_lidar;
    }
  }
}

// ---- command dispatch (handle_command) -------------------------------------
void WsBridgeNode::handle_command(const nlohmann::json& msg,
                                  WsServer::ResponseFn respond) {
  const std::string cmd = msg.value("cmd", "");
  const std::string type = msg.value("type", "");

  if (cmd == "getStorage") {
    respond({{"type", "deviceInfo"}, {"data", MappingControl::device_info()}});
    log::info("✅ 响应 getStorage 成功");
    return;
  }

  if (cmd == "startRecord") {
    // Clear any stale data before starting.
    state_->clear_buffers();
    state_->is_recording.store(true);
    state_->mapping_ready.store(false);
    log::info("🟢 开始建图...");
    // Tune cameras FIRST (≤2 s), then launch the rest of the chain — off the io
    // thread. Tuning must finish before image_odom_recorder opens the cameras
    // (V4L2 devices are exclusive), and before anything else is started.
    auto state = state_;
    auto mapping = mapping_;
    std::thread([state, mapping] {
      // 1) 相机曝光/增益快速整定（策略对齐 adjust_cam_param，双机并行，≤2s/≤5轮）。
      //    全部成功 → 之后 camera 以手动曝光启动，沿用驱动里的整定值；
      //    任一失败 → 回退配置文件的 auto_exposure（AE 兜底）。
      const bool tuned = config::TUNE_ENABLE && cam_tune_pair();
      mapping->set_exposure_tuned(tuned);
      // 2) 整定完成后再开启其余进程：雷达 IPC → 5s 暂停 → livox/lio_sam/camera
      //    （镜像原 5 s pause + 后台启动链）。
      MappingControl::write_shared_cmd(1);
      std::this_thread::sleep_for(std::chrono::seconds(5));
      mapping->start_mapping_commands();
      state->mapping_ready.store(mapping->mapping_ready());
    }).detach();
    respond({{"type", "response"}, {"cmd", "startRecord"}, {"status", "ok"}});
    return;
  }

  if (cmd == "stopRecord") {
    state_->is_recording.store(false);
    state_->mapping_ready.store(false);
    state_->clear_buffers();
    auto mapping = mapping_;
    std::thread([mapping] {
      mapping->close_all_windows();
      MappingControl::write_shared_cmd(0);
    }).detach();
    log::info("🔴 已安全停止并关闭所有窗口，数据缓存已清空");
    respond({{"type", "response"}, {"cmd", "stopRecord"}, {"status", "ok"}});
    return;
  }

  if (type == "GET_FILE_LIST") {
    nlohmann::json files = MappingControl::pcd_file_list();
    respond({{"code", 200}, {"msg", "获取文件列表成功"}, {"data", std::move(files)}});
    log::info("✅ 已返回文件夹列表");
    return;
  }
}

} // namespace r3d_ws
