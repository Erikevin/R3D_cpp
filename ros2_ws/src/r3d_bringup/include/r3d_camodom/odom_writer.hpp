#pragma once

// Odometry 订阅 + 异步落盘（移植自 Python odom_cb / odom_loop）。
// 订阅回调只做轻量入队；独立线程消费队列、写 odom.txt 并 flush，避免阻塞执行器。

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace r3d_camodom {

class OdomWriter {
public:
  OdomWriter(rclcpp::Node *node, const std::string &topic, const std::string &path);
  ~OdomWriter();

  OdomWriter(const OdomWriter &) = delete;
  OdomWriter &operator=(const OdomWriter &) = delete;

  // 打开文件、启动消费线程、创建订阅。
  void start();
  // 停止消费线程并关闭文件。
  void stop();

private:
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void loop();

  struct Item {
    double ts, qw, qx, qy, qz, x, y, z;
  };

  rclcpp::Node *node_;
  std::string topic_;
  std::string path_;
  std::ofstream f_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;

  std::queue<Item> q_;
  std::mutex mtx_;
  std::condition_variable cv_;
  std::thread th_;
  std::atomic<bool> running_{false};
  std::atomic<uint64_t> id_{0};
};

}  // namespace r3d_camodom
