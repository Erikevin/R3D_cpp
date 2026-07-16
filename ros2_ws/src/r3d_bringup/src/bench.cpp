// r3d_bench — microbenchmarks for the bridge's hot paths, so the C++ impl can be
// compared apples-to-apples with tools/bench/bench_parse.py.
// Prints machine-readable `key=value` lines on stdout.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "r3d_ws/types.hpp"

namespace {

constexpr std::size_t kBenchPoints = 8000;
constexpr int kBenchIters = 200;

using clock = std::chrono::steady_clock;

// Build the same 32-byte/point layout the bridge parses.
sensor_msgs::msg::PointCloud2 make_bench_cloud() {
  sensor_msgs::msg::PointCloud2 msg;
  msg.fields.resize(4);
  const std::uint8_t f32 = sensor_msgs::msg::PointField::FLOAT32;
  auto sf = [&](int i, const char* n, std::uint32_t o) {
    msg.fields[i].name = n; msg.fields[i].offset = o;
    msg.fields[i].datatype = f32; msg.fields[i].count = 1;
  };
  sf(0, "x", 0); sf(1, "y", 4); sf(2, "z", 8); sf(3, "intensity", 12);
  msg.point_step = 32;
  msg.width = kBenchPoints;
  msg.height = 1;
  msg.row_step = msg.point_step * msg.width;
  msg.is_bigendian = false;
  msg.is_dense = true;
  msg.data.assign(msg.row_step, 0);
  for (std::size_t i = 0; i < kBenchPoints; ++i) {
    float v[4] = {static_cast<float>(i % 1000) * 0.01f,
                  static_cast<float>(i % 800) * 0.02f,
                  static_cast<float>(i % 600) * 0.03f,
                  static_cast<float>(i % 400) * 0.1f};
    std::memcpy(msg.data.data() + i * 32, v, sizeof(v));
  }
  return msg;
}

double secs(clock::duration d) {
  return std::chrono::duration<double, std::nano>(d).count() / 1e9;
}

} // namespace

int main() {
  auto cloud = make_bench_cloud();

  // warm up
  for (int i = 0; i < 10; ++i) r3d_ws::parse_pointcloud2(cloud, 0.0);

  // parse_pointcloud2
  double checksum = 0.0;
  {
    auto t0 = clock::now();
    for (int i = 0; i < kBenchIters; ++i) {
      auto l = r3d_ws::parse_pointcloud2(cloud, 0.0);
      if (!l.points.empty()) checksum += l.points[0][0];
    }
    auto t1 = clock::now();
    double per = secs(t1 - t0) / kBenchIters;
    std::printf("parse_us_per_frame=%.3f\n", per * 1e6);
    std::printf("parse_ns_per_point=%.2f\n", per * 1e9 / kBenchPoints);
  }
  asm volatile("" ::"g"(checksum) : "memory");  // keep checksum live

  auto lidar = r3d_ws::parse_pointcloud2(cloud, 0.0);

  // pcd json serialize + dump
  {
    std::string out;
    auto t0 = clock::now();
    for (int i = 0; i < kBenchIters; ++i) {
      auto j = r3d_ws::pcd_to_json(lidar);
      out = j.dump();
    }
    auto t1 = clock::now();
    double per = secs(t1 - t0) / kBenchIters;
    std::printf("pcd_json_us_per_frame=%.3f\n", per * 1e6);
    std::printf("pcd_json_bytes=%zu\n", out.size());
  }

  // odom json serialize + dump
  {
    r3d_ws::OdomData o{1.0, 0.5, 0.2, 0.0, 1.0, 0.0, 0.0, 0.0};
    std::string out;
    auto t0 = clock::now();
    for (int i = 0; i < 100000; ++i) {
      auto j = r3d_ws::odom_to_json(o);
      out = j.dump();
    }
    auto t1 = clock::now();
    double per = secs(t1 - t0) / 100000;
    std::printf("odom_json_us_per_frame=%.4f\n", per * 1e6);
    std::printf("odom_json_bytes=%zu\n", out.size());
  }

  // base64 of a JPEG-sized blob
  {
    std::vector<std::uint8_t> blob(100 * 1024, 0xAB);
    std::string out;
    auto t0 = clock::now();
    for (int i = 0; i < 500; ++i) out = r3d_ws::base64_encode(blob);
    auto t1 = clock::now();
    double per = secs(t1 - t0) / 500;
    std::printf("base64_us_per_100kb=%.3f\n", per * 1e6);
    std::printf("base64_bytes=%zu\n", out.size());
  }

  return 0;
}
