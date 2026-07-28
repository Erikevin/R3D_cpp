#include "r3d_camodom/odom_writer.hpp"

#include <iomanip>
#include <sstream>
#include <utility>

namespace r3d_camodom {

OdomWriter::OdomWriter(rclcpp::Node *node, const std::string &topic, const std::string &path)
    : node_(node), topic_(topic), path_(path) {}

OdomWriter::~OdomWriter() { stop(); }

void OdomWriter::start() {
  f_.open(path_, std::ios::out | std::ios::trunc);
  if (f_) {
    f_ << "#id timestamp qw qx qy qz x y z\n";
  }
  running_ = true;
  th_ = std::thread([this] { loop(); });

  // LIO-SAM 原生 QoS：BEST_EFFORT + VOLATILE，KEEP_LAST(5)
  rclcpp::QoS qos(rclcpp::KeepLast(5));
  qos.best_effort().durability_volatile();
  sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      topic_, qos, std::bind(&OdomWriter::on_odom, this, std::placeholders::_1));
}

void OdomWriter::stop() {
  running_ = false;
  cv_.notify_all();
  if (th_.joinable()) {
    th_.join();
  }
  if (f_.is_open()) {
    f_.close();
  }
}

void OdomWriter::on_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
  const double ts = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  Item it;
  it.ts = ts;
  it.qw = msg->pose.pose.orientation.w;
  it.qx = msg->pose.pose.orientation.x;
  it.qy = msg->pose.pose.orientation.y;
  it.qz = msg->pose.pose.orientation.z;
  it.x = msg->pose.pose.position.x;
  it.y = msg->pose.pose.position.y;
  it.z = msg->pose.pose.position.z;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    q_.push(it);
  }
  cv_.notify_one();
}

void OdomWriter::loop() {
  while (running_) {
    Item it;
    {
      std::unique_lock<std::mutex> lk(mtx_);
      cv_.wait(lk, [this] { return !running_ || !q_.empty(); });
      if (!running_ && q_.empty()) {
        break;
      }
      it = q_.front();
      q_.pop();
    }
    const uint64_t id = ++id_;
    std::ostringstream os;
    os.setf(std::ios::fixed);
    os << std::setprecision(9);
    os << id << ' ' << it.ts << ' ' << it.qw << ' ' << it.qx << ' ' << it.qy << ' ' << it.qz << ' '
       << it.x << ' ' << it.y << ' ' << it.z << '\n';
    if (f_) {
      f_ << os.str();
      f_.flush();
    }
  }
}

}  // namespace r3d_camodom
