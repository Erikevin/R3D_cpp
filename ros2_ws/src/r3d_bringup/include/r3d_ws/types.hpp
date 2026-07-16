#pragma once
// Core data types + helpers for the WebSocket bridge — C++ mirror of the data
// shapes used by R3D/web_a.py. Header-only (base64 is small enough to inline).

#include <array>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "r3d_ws/config.hpp"

namespace r3d_ws {

// ---- synced data payloads --------------------------------------------------
struct OdomData {
  double t;
  double x, y, z;
  double qw, qx, qy, qz;
};

struct LidarData {
  double t{0.0};
  std::vector<std::array<float, 3>> points;
};

struct ImageData {
  std::string b64;  // base64-encoded JPEG
};

// ROS stamp -> seconds (mirrors sec + nanosec*1e-9).
inline double stamp_to_sec(const builtin_interfaces::msg::Time& ts) {
  return static_cast<double>(ts.sec) + static_cast<double>(ts.nanosec) * 1e-9;
}

// Parse a PointCloud2 exactly like web_a.py::parse_pointcloud2: walk the data in
// steps of point_step, read x/y/z (first 3 floats of each point), cap the count.
inline LidarData parse_pointcloud2(const sensor_msgs::msg::PointCloud2& msg, double t) {
  LidarData out;
  out.t = t;
  const std::size_t step = msg.point_step > 0 ? static_cast<std::size_t>(msg.point_step) : 32;
  const auto& data = msg.data;
  const std::size_t n = data.size();
  const std::size_t reserve = std::min(config::PCD_MAX_POINTS, n / step + 1);
  out.points.reserve(reserve);
  for (std::size_t i = 0;
       i + sizeof(float) * 3 <= n && out.points.size() < config::PCD_MAX_POINTS;
       i += step) {
    float xyz[3];
    std::memcpy(xyz, data.data() + i, sizeof(xyz));
    out.points.push_back({xyz[0], xyz[1], xyz[2]});
  }
  return out;
}

// ---- base64 (used for the JPEG payload) ------------------------------------
inline std::string base64_encode(const std::uint8_t* data, std::size_t len) {
  static const char tbl[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve((len + 2) / 3 * 4);
  std::size_t i = 0;
  for (; i + 3 <= len; i += 3) {
    std::uint32_t v = (std::uint32_t(data[i]) << 16) |
                      (std::uint32_t(data[i + 1]) << 8) |
                      std::uint32_t(data[i + 2]);
    out.push_back(tbl[(v >> 18) & 0x3F]);
    out.push_back(tbl[(v >> 12) & 0x3F]);
    out.push_back(tbl[(v >> 6) & 0x3F]);
    out.push_back(tbl[v & 0x3F]);
  }
  std::size_t rem = len - i;
  if (rem == 1) {
    std::uint32_t v = std::uint32_t(data[i]) << 16;
    out.push_back(tbl[(v >> 18) & 0x3F]);
    out.push_back(tbl[(v >> 12) & 0x3F]);
    out.push_back('=');
    out.push_back('=');
  } else if (rem == 2) {
    std::uint32_t v = (std::uint32_t(data[i]) << 16) | (std::uint32_t(data[i + 1]) << 8);
    out.push_back(tbl[(v >> 18) & 0x3F]);
    out.push_back(tbl[(v >> 12) & 0x3F]);
    out.push_back(tbl[(v >> 6) & 0x3F]);
    out.push_back('=');
  }
  return out;
}

inline std::string base64_encode(const std::vector<std::uint8_t>& v) {
  return v.empty() ? std::string() : base64_encode(v.data(), v.size());
}

// ---- JSON serialization (inner "data" objects; shapes match web_a.py) ------
inline nlohmann::json odom_to_json(const OdomData& o) {
  return nlohmann::json{{"t", o.t}, {"x", o.x}, {"y", o.y}, {"z", o.z},
                        {"qw", o.qw}, {"qx", o.qx}, {"qy", o.qy}, {"qz", o.qz}};
}

// points -> [[x,y,z], ...]  (same nested-array shape the frontend expects)
inline nlohmann::json pcd_to_json(const LidarData& l) {
  nlohmann::json pts = nlohmann::json::array();
  for (const auto& p : l.points) {
    pts.push_back({p[0], p[1], p[2]});
  }
  return nlohmann::json{{"t", l.t}, {"points", std::move(pts)}};
}

} // namespace r3d_ws
