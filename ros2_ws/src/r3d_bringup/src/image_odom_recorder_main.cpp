// image_odom_recorder entry point — the C++ counterpart of
// R3D/camera/camera_two_value.py. Runs the ROS2 spin loop until SIGINT/SIGTERM;
// the CaptureNode destructor stops the IO/capture/trigger threads and reaps the
// ros2 bag record subprocess (finally-style cleanup).

#include <csignal>

#include <rclcpp/rclcpp.hpp>

#include "r3d_camodom/capture_node.hpp"

static void install_signals() {
  auto handler = +[](int) { rclcpp::shutdown(); };
  std::signal(SIGINT, handler);
  std::signal(SIGTERM, handler);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  install_signals();
  rclcpp::spin(std::make_shared<r3d_camodom::CaptureNode>());
  rclcpp::shutdown();
  return 0;
}
