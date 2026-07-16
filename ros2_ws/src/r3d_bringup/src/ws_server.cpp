// boost::beast async WebSocket server implementation.
//
// Path routing: only "/data" is accepted (matches web_a.py::router); any other
// target gets the socket closed. The send loop mirrors the Python send_loop:
// every 50 ms, when recording & mapping_ready (or R3D_FORCE_STREAM=1), it pushes
// img/odom and rate-limits pcd to PCD_SEND_INTERVAL (10 Hz).

#include "r3d_ws/ws_server.hpp"

#include "r3d_ws/logger.hpp"

#include <boost/asio/bind_executor.hpp>
#include <boost/asio/dispatch.hpp>
#include <boost/asio/post.hpp>
#include <boost/asio/socket_base.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/websocket.hpp>

#include <cstdlib>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;

namespace r3d_ws {

// ---- SharedState -----------------------------------------------------------
void SharedState::clear_buffers() {
  std::lock_guard<std::mutex> lk(m);
  odom_buffer.clear();
  lidar_buffer.clear();
  latest_odom.reset();
  latest_lidar.reset();
  latest_image.reset();
}

// ---- per-connection session ------------------------------------------------
class WsSession : public std::enable_shared_from_this<WsSession> {
 public:
  WsSession(tcp::socket socket, std::shared_ptr<SharedState> state,
            WsServer::CommandHandler handler, WsServer::ConnectHandler connect)
      : ws_(std::move(socket)),
        state_(std::move(state)),
        handler_(std::move(handler)),
        connect_(std::move(connect)),
        timer_(ws_.get_executor()) {
    // R3D_FORCE_STREAM lets the send loop run on a headless dev box without
    // launching terminals (debug only; inert when unset).
    const char* env = std::getenv("R3D_FORCE_STREAM");
    force_stream_ = (env != nullptr && env[0] == '1');
    // Send each message as a single frame (no auto-fragmentation). Each pcd
    // message is ~0.5 MB; a single frame is fine on this LAN link and simpler
    // for lightweight clients. (Browsers handle either form.)
    ws_.auto_fragment(false);
    ws_.read_message_max(64 * 1024 * 1024);  // allow large inbound frames too
  }

  void start() {
    // Read the HTTP upgrade request so we can inspect the target path.
    net::dispatch(ws_.get_executor(),
                  beast::bind_front_handler(&WsSession::do_read_http,
                                            shared_from_this()));
  }

  // Enqueue a fully-formed JSON message; thread-safe (posted to the strand).
  void send(nlohmann::json msg) {
    std::string s = msg.dump();
    net::post(ws_.get_executor(),
              [self = shared_from_this(), body = std::move(s)]() mutable {
                self->out_q_.push_back(std::move(body));
                self->do_write();
              });
  }

 private:
  void do_read_http() {
    req_.clear();
    http::async_read(
        ws_.next_layer(), read_buf_, req_,
        beast::bind_front_handler(&WsSession::on_read_http, shared_from_this()));
  }

  void on_read_http(beast::error_code ec, std::size_t) {
    if (ec) {
      log::warn(std::string("WS HTTP 读取失败: ") + ec.message());
      return;
    }
    if (req_.target() != "/data") {
      log::warn(std::string("WS 拒绝非 /data 路径: ") + std::string(req_.target()));
      beast::error_code ignore;
      ws_.next_layer().socket().shutdown(tcp::socket::shutdown_both, ignore);
      return;
    }
    // Accept the upgrade, reusing the already-parsed request (leftover bytes in
    // read_buf_ are consumed by the first ws read).
    ws_.set_option(websocket::stream_base::timeout::suggested(
        beast::role_type::server));
    ws_.async_accept(
        req_, beast::bind_front_handler(&WsSession::on_accept, shared_from_this()));
  }

  void on_accept(beast::error_code ec) {
    if (ec) {
      log::warn(std::string("WS 握手失败: ") + ec.message());
      return;
    }
    log::info("✅ 客户端已连接");
    // Mirrors send_device_info_on_connect().
    if (connect_) {
      auto respond = [self = shared_from_this()](nlohmann::json m) {
        self->send(std::move(m));
      };
      connect_(respond);
    }
    do_read();
    start_tick();
  }

  // ---- read loop (commands) ----
  void do_read() {
    ws_.async_read(read_buf_,
                   beast::bind_front_handler(&WsSession::on_read, shared_from_this()));
  }

  void on_read(beast::error_code ec, std::size_t bytes) {
    if (ec == websocket::error::closed || ec == net::error::eof ||
        ec == net::error::operation_aborted || ec == net::error::connection_reset ||
        ec == net::error::broken_pipe) {
      // Any of these = the peer is gone (clean close, EOF, RST, or aborted) —
      // treat as a normal disconnect, stop the send tick, drop buffered data.
      stop_session();
      log::info("🔌 客户端断开");
      state_->clear_buffers();
      return;
    }
    if (ec) {
      log::warn(std::string("WS 读取错误: ") + ec.message());
      return;
    }
    if (bytes > 0) {
      std::string text = beast::buffers_to_string(read_buf_.data());
      read_buf_.consume(read_buf_.size());
      handle_text(text);
    } else {
      read_buf_.consume(read_buf_.size());
    }
    do_read();
  }

  void handle_text(const std::string& text) {
    nlohmann::json msg;
    try {
      msg = nlohmann::json::parse(text);
    } catch (const std::exception& e) {
      log::warn(std::string("JSON 解析失败: ") + e.what());
      return;
    }
    std::string cmd = msg.value("cmd", "");
    std::string type = msg.value("type", "");
    log::info(std::string("📥 收到指令: ") + cmd + " / type: " + type);
    if (handler_) {
      auto respond = [self = shared_from_this()](nlohmann::json m) {
        self->send(std::move(m));
      };
      handler_(msg, std::move(respond));
    }
  }

  // ---- send tick (50 ms) ----
  void start_tick() {
    last_pcd_send_ = clock::now() - std::chrono::milliseconds(100);
    timer_.expires_after(std::chrono::milliseconds(50));
    timer_.async_wait(
        beast::bind_front_handler(&WsSession::on_tick, shared_from_this()));
  }

  // Stop the per-connection send loop (idempotent). Called when the read side
  // detects the peer disconnecting, or a write fails — otherwise the 50 ms send
  // tick keeps firing and logs a write error every tick.
  void stop_session() {
    if (!closed_.exchange(true)) {
      beast::error_code ignore;
      timer_.cancel(ignore);
    }
  }

  void on_tick(beast::error_code ec) {
    if (closed_ || ec == net::error::operation_aborted) return;
    if (ec) return;

    const bool gate = force_stream_ ||
                      (state_->is_recording.load() && state_->mapping_ready.load());
    if (gate) {
      // image (optional, off by default)
      if constexpr (config::ENABLE_CAMERA_STREAM) {
        std::optional<ImageData> img;
        {
          std::lock_guard<std::mutex> lk(state_->m);
          img = state_->latest_image;
        }
        if (img) send({{"type", "img"}, {"data", img->b64}});
      }
      // odometry
      std::optional<OdomData> odom;
      {
        std::lock_guard<std::mutex> lk(state_->m);
        odom = state_->latest_odom;
      }
      if (odom) send({{"type", "odom"}, {"data", odom_to_json(*odom)}});

      // point cloud (rate-limited to PCD_SEND_INTERVAL)
      auto now = clock::now();
      if (now - last_pcd_send_ >= std::chrono::duration<double>(config::PCD_SEND_INTERVAL)) {
        std::optional<LidarData> pcd;
        {
          std::lock_guard<std::mutex> lk(state_->m);
          pcd = state_->latest_lidar;
        }
        if (pcd) {
          send({{"type", "pcd"}, {"data", pcd_to_json(*pcd)}});
          last_pcd_send_ = now;
        }
      }
    }

    timer_.expires_after(std::chrono::milliseconds(50));
    timer_.async_wait(
        beast::bind_front_handler(&WsSession::on_tick, shared_from_this()));
  }

  // ---- serial write drain ----
  void do_write() {
    if (closed_ || writing_ || out_q_.empty()) return;
    writing_ = true;
    ws_.text(true);
    out_buf_ = std::move(out_q_.front());
    out_q_.pop_front();
    ws_.async_write(
        net::buffer(out_buf_),
        beast::bind_front_handler(&WsSession::on_write, shared_from_this()));
  }

  void on_write(beast::error_code ec, std::size_t) {
    writing_ = false;
    if (ec) {
      // Peer closed mid-write (operation_aborted / eof / reset) is the normal
      // disconnect path — stop the send tick quietly instead of logging every
      // 50 ms. Anything else is a real error.
      stop_session();
      if (ec != net::error::operation_aborted && ec != net::error::eof &&
          ec != net::error::connection_reset) {
        log::warn(std::string("WS 写入错误: ") + ec.message());
      }
      return;
    }
    do_write();  // drain the queue
  }

 private:
  using clock = std::chrono::steady_clock;

  websocket::stream<beast::tcp_stream> ws_;
  std::shared_ptr<SharedState> state_;
  WsServer::CommandHandler handler_;
  WsServer::ConnectHandler connect_;

  beast::flat_buffer read_buf_;
  http::request<http::string_body> req_;
  net::steady_timer timer_;

  std::deque<std::string> out_q_;
  std::string out_buf_;
  bool writing_{false};
  std::atomic<bool> closed_{false};
  bool force_stream_{false};
  clock::time_point last_pcd_send_{};
};

// ---- WsServer --------------------------------------------------------------
WsServer::WsServer(std::shared_ptr<SharedState> state, unsigned short port)
    : state_(std::move(state)),
      ioc_(),
      acceptor_(ioc_) {
  // open / reuse_address / bind / listen manually so a quick restart (e.g. right
  // after the previous process exits into TIME_WAIT) does not fail to bind.
  acceptor_.open(tcp::v4());
  acceptor_.set_option(tcp::acceptor::reuse_address(true));
  acceptor_.bind(tcp::endpoint(tcp::v4(), port));
  acceptor_.listen(net::socket_base::max_listen_connections);
}

WsServer::~WsServer() { stop(); }

void WsServer::set_command_handler(CommandHandler h) {
  std::lock_guard<std::mutex> lk(handler_mutex_);
  cmd_handler_ = std::move(h);
}

void WsServer::set_connect_handler(ConnectHandler h) {
  std::lock_guard<std::mutex> lk(handler_mutex_);
  connect_handler_ = std::move(h);
}

void WsServer::run() {
  do_accept();
  worker_ = std::thread([this] {
    try {
      ioc_.run();
    } catch (const std::exception& e) {
      log::error(std::string("io_context 异常: ") + e.what());
    }
  });
}

void WsServer::stop() {
  beast::error_code ignore;
  acceptor_.close(ignore);
  ioc_.stop();
  if (worker_.joinable() && worker_.get_id() != std::this_thread::get_id()) {
    worker_.join();
  }
}

void WsServer::do_accept() {
  acceptor_.async_accept(
      [this](beast::error_code ec, tcp::socket socket) {
        if (ec == net::error::operation_aborted) {
          return;  // shutting down — do not re-arm
        }
        if (ec) {
          log::warn(std::string("accept 错误: ") + ec.message());
        } else {
          CommandHandler handler;
          ConnectHandler connect;
          {
            std::lock_guard<std::mutex> lk(handler_mutex_);
            handler = cmd_handler_;
            connect = connect_handler_;
          }
          auto session = std::make_shared<WsSession>(
              std::move(socket), state_, std::move(handler), std::move(connect));
          session->start();
        }
        do_accept();  // keep accepting
      });
}

} // namespace r3d_ws
