// mock_publisher — synthetic ROS2 data so r3d_ws_bridge can run end-to-end
// without a Livox/LIO-SAM stack. Publishes matched-stamp Odometry + PointCloud2
// (and optionally a CompressedImage with --camera) on the exact topics the
// bridge subscribes to, at ~10 Hz, BEST_EFFORT.

#include <opencv2/imgcodecs.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace {

constexpr std::size_t kPoints = 8000;   // matches the bridge's cap
constexpr int kWidth = 960;
constexpr int kHeight = 540;

// Build a PointCloud2 whose memory layout matches what Livox/LIO-SAM produces:
// point_step=32, x/y/z/intensity = float32 at offsets 0/4/8/12 (+ 16 pad bytes).
sensor_msgs::msg::PointCloud2 make_cloud(double theta, double t) {
  sensor_msgs::msg::PointCloud2 msg;
  auto set_field = [](sensor_msgs::msg::PointField& f, const std::string& n,
                      std::uint32_t off, std::uint8_t dt) {
    f.name = n; f.offset = off; f.datatype = dt; f.count = 1;
  };
  msg.fields.resize(4);
  set_field(msg.fields[0], "x", 0, sensor_msgs::msg::PointField::FLOAT32);
  set_field(msg.fields[1], "y", 4, sensor_msgs::msg::PointField::FLOAT32);
  set_field(msg.fields[2], "z", 8, sensor_msgs::msg::PointField::FLOAT32);
  set_field(msg.fields[3], "intensity", 12, sensor_msgs::msg::PointField::FLOAT32);
  msg.point_step = 32;
  msg.width = kPoints;
  msg.height = 1;
  msg.row_step = msg.point_step * msg.width;
  msg.is_bigendian = false;
  msg.is_dense = true;
  msg.data.assign(msg.row_step, 0);

  std::uint8_t* base = msg.data.data();
  for (std::size_t i = 0; i < kPoints; ++i) {
    // Distribute points on a torus-like ring that rotates over time.
    double u = (2.0 * M_PI * i) / kPoints;          // around the ring
    double r = 5.0 + 1.5 * std::sin(6.0 * u + t);   // radius wobble
    double ring = 0.6 * std::cos(8.0 * u);          // thickness
    double x0 = (r + ring * std::cos(6.0 * u)) * std::cos(u + theta);
    double y0 = (r + ring * std::cos(6.0 * u)) * std::sin(u + theta);
    double z0 = ring * std::sin(6.0 * u) + 0.4 * std::sin(3.0 * u + t);
    float xyz[4] = {static_cast<float>(x0), static_cast<float>(y0),
                    static_cast<float>(z0),
                    static_cast<float>(0.5f + 0.5f * std::sin(5.0 * u))};
    std::memcpy(base + i * 32, xyz, sizeof(xyz));
  }
  return msg;
}

std::vector<std::uint8_t> make_gradient_jpeg() {
  cv::Mat img(kHeight, kWidth, CV_8UC3);
  for (int y = 0; y < kHeight; ++y) {
    for (int x = 0; x < kWidth; ++x) {
      img.at<cv::Vec3b>(y, x)[0] = static_cast<uchar>(x * 255 / kWidth);
      img.at<cv::Vec3b>(y, x)[1] = static_cast<uchar>(y * 255 / kHeight);
      img.at<cv::Vec3b>(y, x)[2] = 128;
    }
  }
  std::vector<std::uint8_t> buf;
  cv::imencode(".jpg", img, buf);
  return buf;
}

} // namespace

int main(int argc, char** argv) {
  bool with_camera = false;
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--camera") with_camera = true;
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("mock_publisher");

  auto qos = rclcpp::QoS(rclcpp::KeepLast(50)).best_effort();
  auto odom_pub = node->create_publisher<nav_msgs::msg::Odometry>(
      "/lio_sam/mapping/odometry", qos);
  auto lidar_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/lio_sam/mapping/cloud_registered", qos);
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr img_pub;
  std::vector<std::uint8_t> jpeg;
  if (with_camera) {
    img_pub = node->create_publisher<sensor_msgs::msg::CompressedImage>(
        "/camera/image_raw/compressed", qos);
    jpeg = make_gradient_jpeg();
  }

  RCLCPP_INFO(node->get_logger(),
              "mock_publisher: odom + pcd (%zux32) @10Hz%s", kPoints,
              with_camera ? " + camera" : "");

  double theta = 0.0;
  // No callbacks here (publish-only), so pace with std::sleep and skip spin.
  // Using rclcpp::Rate / spin on a signal-triggered shutdown throws a guard-
  // condition error during teardown.
  while (rclcpp::ok()) {
    auto now = node->now();
    double t = now.seconds();

    // Odometry: march forward in +x with a gentle turn (so the trail curves).
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = 0.5 * t;
    odom.pose.pose.position.y = 1.0 * std::sin(0.2 * t);
    odom.pose.pose.position.z = 0.0;
    double yaw = std::cos(0.2 * t);  // ~heading from velocity
    odom.pose.pose.orientation.w = std::cos(0.5 * yaw);
    odom.pose.pose.orientation.z = std::sin(0.5 * yaw);
    odom_pub->publish(odom);

    // PointCloud2 with the same stamp (perfectly syncable).
    sensor_msgs::msg::PointCloud2 cloud = make_cloud(theta, t);
    cloud.header.stamp = now;
    cloud.header.frame_id = "odom";
    lidar_pub->publish(cloud);

    if (img_pub) {
      sensor_msgs::msg::CompressedImage img;
      img.header.stamp = now;
      img.format = "jpeg";
      img.data = jpeg;
      img_pub->publish(img);
    }

    theta += 0.02;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // The rclcpp signal handler already calls shutdown() on SIGINT/SIGTERM.
  return 0;
}
