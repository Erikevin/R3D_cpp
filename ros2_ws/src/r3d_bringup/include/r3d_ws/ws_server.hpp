#pragma once
// boost::beast async WebSocket server — the C++ counterpart of the
// asyncio `receive_loop` + `send_loop` in R3D/web_a.py.
//
// One io_context + acceptor on 0.0.0.0:8765. Each connection becomes a
// WsSession that runs a read loop (commands) and a 50 ms send tick
// (img/odom/pcd). All stream operations run on a per-session strand, and all
// writes go through a queue drained serially, so command responses and tick
// pushes never race (beast allows one read + one write concurrently).

#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/system/error_code.hpp>

#include <nlohmann/json.hpp>

#include <atomic>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "r3d_ws/types.hpp"

namespace r3d_ws {

// Thread-safe shared state: ROS callbacks (producer) append here, WS sessions
// (consumer) read here. Matches the Python module-level buffers + latest_*.
struct SharedState {
  std::mutex m;
  std::deque<OdomData> odom_buffer;
  std::deque<LidarData> lidar_buffer;
  std::optional<OdomData> latest_odom;
  std::optional<LidarData> latest_lidar;
  std::optional<ImageData> latest_image;
  std::atomic<bool> is_recording{false};
  std::atomic<bool> mapping_ready{false};

  void clear_buffers();  // wipe the deques + latest_* (used on start/stop/disconnect)
};

class WsServer {
 public:
  // Command handler: receives the parsed client JSON and a `respond` callback
  // that enqueues a fully-formed JSON message back to that client. The handler
  // runs on the session strand, so it must not block — it should spawn a thread
  // for long work and call `respond` for any immediate ack.
  using ResponseFn = std::function<void(nlohmann::json)>;
  using CommandHandler =
      std::function<void(const nlohmann::json& msg, ResponseFn respond)>;
  // Connect handler: called once per new session (mirrors
  // send_device_info_on_connect — the node pushes deviceInfo here).
  using ConnectHandler = std::function<void(ResponseFn respond)>;

  WsServer(std::shared_ptr<SharedState> state, unsigned short port);
  ~WsServer();

  WsServer(const WsServer&) = delete;
  WsServer& operator=(const WsServer&) = delete;

  void set_command_handler(CommandHandler h);
  void set_connect_handler(ConnectHandler h);
  void run();   // spawn the io worker thread + start accepting
  void stop();  // signal the io loop to stop and join the worker

 private:
  std::shared_ptr<SharedState> state_;
  boost::asio::io_context ioc_;
  boost::asio::ip::tcp::acceptor acceptor_;
  std::thread worker_;
  std::mutex handler_mutex_;
  CommandHandler cmd_handler_;
  ConnectHandler connect_handler_;

  void do_accept();
};

} // namespace r3d_ws
