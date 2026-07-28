// Implementation of device/window/file-list control — faithful port of the
// device-management half of R3D/web_a.py.

#include "r3d_ws/mapping_control.hpp"

#include "r3d_ws/logger.hpp"

#include <sys/stat.h>
#include <sys/statvfs.h>
#include <sys/wait.h>
#include <dirent.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

namespace r3d_ws {

// Capture stdout of a shell command (used for `xdotool search`).
static std::string shell_capture(const std::string& cmd) {
  std::string out;
  FILE* p = popen(cmd.c_str(), "r");
  if (!p) return out;
  char buf[256];
  while (fgets(buf, sizeof(buf), p)) out += buf;
  pclose(p);
  return out;
}

// Run a quick synchronous shell command (xdotool windowactivate/key/close).
static void shell_run(const std::string& cmd) {
  int r = std::system(cmd.c_str());
  (void)r;
}

// Fire-and-forget a shell command fully detached (double-fork) so the caller is
// not blocked and no zombie is left behind. Used to launch gnome-terminal.
static void spawn_detached(const std::string& cmd) {
  pid_t pid = fork();
  if (pid < 0) return;
  if (pid == 0) {
    setsid();
    pid_t grand = fork();
    if (grand == 0) {
      execl("/bin/sh", "sh", "-c", cmd.c_str(), (char*)nullptr);
      _exit(127);
    }
    _exit(grand < 0 ? 1 : 0);
  }
  int st = 0;
  while (waitpid(pid, &st, 0) < 0 && errno == EINTR) {
  }
}

static std::vector<std::string> split_ws(const std::string& s) {
  std::vector<std::string> out;
  const char* p = s.c_str();
  while (*p) {
    while (*p && std::isspace((unsigned char)*p)) ++p;
    if (!*p) break;
    const char* b = p;
    while (*p && !std::isspace((unsigned char)*p)) ++p;
    out.emplace_back(b, static_cast<std::size_t>(p - b));
  }
  return out;
}

// ---- MappingControl: static helpers ----------------------------------------
std::string MappingControl::open_terminal(const std::string& cmd) {
  // Snapshot existing terminal window ids, spawn a new one, then diff.
  auto before = split_ws(
      shell_capture("xdotool search --class gnome-terminal 2>/dev/null"));
  // gnome-terminal -- bash -c "<cmd>"  (cmd has no single quotes)
  spawn_detached("gnome-terminal -- bash -c '" + cmd + "'");
  std::this_thread::sleep_for(std::chrono::milliseconds(600));
  auto after = split_ws(
      shell_capture("xdotool search --class gnome-terminal 2>/dev/null"));

  for (const auto& id : after) {
    if (std::find(before.begin(), before.end(), id) == before.end()) return id;
  }
  return "";
}

void MappingControl::stop_process_in_terminal(const std::string& xid) {
  if (xid.empty()) return;
  shell_run("xdotool windowactivate " + xid);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  shell_run("xdotool key ctrl+c");
  log::info(std::string("已发送停止指令(Ctrl+C)到窗口: ") + xid);
}

// ---- device info -----------------------------------------------------------
nlohmann::json MappingControl::device_info() {
  nlohmann::json info;
  info["version"]    = "R3D_V1_0_0";
  info["deviceCode"] = "B258A49D02";
  info["battery"]    = 78;
  info["wsStatus"]   = "已连接";

  struct statvfs vfs;
  if (statvfs("/", &vfs) == 0 && vfs.f_blocks > 0) {
    double total = static_cast<double>(vfs.f_blocks) * vfs.f_frsize;
    double freeb = static_cast<double>(vfs.f_bfree) * vfs.f_frsize;
    double used  = total - freeb;
    info["storageTotal"]  = std::round(total / 1073741824.0 * 100.0) / 100.0;
    info["storageUsed"]   = std::round(used / 1073741824.0 * 100.0) / 100.0;
    info["storagePercent"] = static_cast<int>(std::round(used / total * 100.0));
  } else {
    info["storageTotal"]  = 128;
    info["storageUsed"]   = 16;
    info["storagePercent"] = 12;
  }
  return info;
}

// ---- pcd file list ---------------------------------------------------------
nlohmann::json MappingControl::pcd_file_list() {
  nlohmann::json arr = nlohmann::json::array();
  DIR* d = opendir(config::SAVE_PCD_DIR);
  if (!d) return arr;  // mirrors Python: missing dir -> empty list

  struct Entry { std::string name, tstr; std::time_t ct; };
  std::vector<Entry> items;
  struct dirent* e;
  while ((e = readdir(d)) != nullptr) {
    std::string name = e->d_name;
    if (name == "." || name == "..") continue;
    std::string path = std::string(config::SAVE_PCD_DIR) + name;
    struct stat st;
    if (stat(path.c_str(), &st) != 0) continue;
    if (!S_ISDIR(st.st_mode)) continue;
    std::tm tm{};
    localtime_r(&st.st_ctim.tv_sec, &tm);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tm);
    items.push_back({name, std::string(buf), st.st_ctim.tv_sec});
  }
  closedir(d);

  std::sort(items.begin(), items.end(),
            [](const Entry& a, const Entry& b) { return a.ct > b.ct; });

  for (const auto& it : items) {
    arr.push_back({{"name", it.name}, {"createTime", it.tstr}, {"size", "文件夹"}});
  }
  return arr;
}

// ---- shared command file ---------------------------------------------------
bool MappingControl::write_shared_cmd(int value) {
  std::string tmp = std::string(config::SHARE_FILE) + ".tmp";
  {
    std::ofstream f(tmp, std::ios::trunc);
    if (!f) {
      log::error("[IPC] 写入共享文件失败: " + tmp + " — " + std::strerror(errno));
      return false;
    }
    f << "{\"cmd\": " << value << "}";
  }
  if (std::rename(tmp.c_str(), config::SHARE_FILE) != 0) {
    log::error("[IPC] rename 失败: " + std::string(std::strerror(errno)));
    return false;
  }
  log::info("[IPC] 已写入跨进程指令: " + std::to_string(value) + " -> " + config::SHARE_FILE);
  return true;
}

// ---- terminal lifecycle ----------------------------------------------------
// Exact bash source/launch strings from web_a.py (jetson deploy target).
// start_mapping_commands() 在 gnome-terminal 里执行；终端继承 r3d_ws_bridge 的 cwd，
// 故工作区用相对路径 "ros2_ws/install/setup.bash" —— 桥必须从 ros2_ws 的父目录启动。
// Livox / LIO-SAM 需其各自包已装入该 ros2_ws，否则对应终端会报 "package not found"。
static const char* kCmdLivox =
    "source /opt/ros/humble/setup.bash && source ~/.bashrc && "
    "source ros2_ws/install/setup.bash && "
    "ros2 launch livox_ros_driver2 msg_MID360_launch.py; exec bash";
static const char* kCmdLioSam =
    "source /opt/ros/humble/setup.bash && source ~/.bashrc && "
    "source ros2_ws/install/setup.bash && source /root/my_robot_x/install/local_setup.bash"
    "ros2 launch lio_sam run.launch.py; exec bash";
static const char* kCmdCamera =
    "source /opt/ros/humble/setup.bash && source ~/.bashrc && "
    "source ros2_ws/install/setup.bash && "
    "ros2 run r3d_bringup image_odom_recorder; exec bash";

void MappingControl::start_mapping_commands() {
  std::vector<std::string> local;
  try {
    log::info("🔹 启动 Livox 驱动窗口");
    std::string xid = open_terminal(kCmdLivox);
    if (!xid.empty()) local.push_back(xid);
    std::this_thread::sleep_for(std::chrono::seconds(5));

    log::info("🔹 启动 LIO-SAM 窗口");
    xid = open_terminal(kCmdLioSam);
    if (!xid.empty()) local.push_back(xid);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    log::info("🔹 启动保存脚本窗口");
    xid = open_terminal(kCmdCamera);
    if (!xid.empty()) local.push_back(xid);

    {
      std::lock_guard<std::mutex> lk(m_);
      xids_ = std::move(local);
      mapping_ready_ = true;
    }
    log::info("✅ 建图启动完成，开始发送数据");
  } catch (const std::exception& e) {
    log::error(std::string("启动失败: ") + e.what());
    std::lock_guard<std::mutex> lk(m_);
    mapping_ready_ = false;
  }
}

void MappingControl::close_all_windows() {
  std::vector<std::string> local;
  {
    std::lock_guard<std::mutex> lk(m_);
    local = xids_;
  }
  log::info("🧹 正在安全停止并关闭所有窗口...");

  for (const auto& xid : local) {
    stop_process_in_terminal(xid);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  std::this_thread::sleep_for(std::chrono::seconds(2));

  for (const auto& xid : local) {
    shell_run("xdotool windowclose " + xid);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }

  {
    std::lock_guard<std::mutex> lk(m_);
    xids_.clear();
    mapping_ready_ = false;
  }
  log::info("✅ 所有窗口已安全关闭");
}

bool MappingControl::mapping_ready() const {
  std::lock_guard<std::mutex> lk(m_);
  return mapping_ready_;
}

} // namespace r3d_ws
