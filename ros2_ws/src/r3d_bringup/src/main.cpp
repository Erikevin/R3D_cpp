// r3d_ws_bridge entry point — the C++ counterpart of web_a.py's __main__.
// Runs the ROS2 spin loop until SIGINT/SIGTERM; the node destructor stops the
// WebSocket server (finally-style cleanup).

#include <csignal>
#include <exception>

#include <rclcpp/rclcpp.hpp>

#include "r3d_ws/logger.hpp"
#include "r3d_ws/ws_bridge_node.hpp"

static void install_signals() {
  auto handler = +[](int) { rclcpp::shutdown(); };
  std::signal(SIGINT, handler);
  std::signal(SIGTERM, handler);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  install_signals();

  try {
    // Node construction opens the listening socket; a bind failure (port in use)
    // throws — log and exit cleanly rather than aborting.
    rclcpp::spin(std::make_shared<r3d_ws::WsBridgeNode>());
  } catch (const std::exception& e) {
    // 启动异常（bind 失败 / 端口被占等）：非零退出，让 r3d_main 据此触发
    // r3d-main.service 重启（配合启动前杀掉 8765 占用者，端口冲突会在重启中被化解）。
    r3d_ws::log::error(std::string("启动失败: ") + e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
