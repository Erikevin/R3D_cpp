#pragma once
// r3cpp node that bridges ROS2 topics -> WebSocket clients.
// The C++ counterpart of R3D/web_a.py's SyncSubscriber + handle_data/handle_command.

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>

#include "r3d_ws/mapping_control.hpp"
#include "r3d_ws/types.hpp"
#include "r3d_ws/ws_server.hpp"

namespace r3d_ws {

class WsBridgeNode : public rclcpp::Node {
 public:
  WsBridgeNode();
  ~WsBridgeNode() override;

 private:
  void on_image(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void on_lidar(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void try_sync();
  void handle_command(const nlohmann::json& msg, WsServer::ResponseFn respond);

  std::shared_ptr<SharedState> state_;
  std::shared_ptr<MappingControl> mapping_;
  std::unique_ptr<WsServer> server_;

  rclcpp::TimerBase::SharedPtr sync_timer_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr img_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
};

} // namespace r3d_ws
