#pragma once
// Device / process / file-list control — C++ mirror of the device-management
// half of R3D/web_a.py (get_device_info, start_mapping_commands,
// close_all_windows, get_pcd_file_list, write_shared_cmd).
//
// start_mapping_commands() launches Livox / LIO-SAM / recorder as background
// processes (setsid) with stdout/stderr to /tmp/r3d_mapping_<name>.log — no
// gnome-terminal / xdotool, so it works on headless hosts (e.g. OrangePi 3B).
//
// close_all_windows() stops them in TWO phases:
//  Phase 1 — lio_sam + livox (the CPU hogs): these are ros2 launch trees whose
//    nodes are setsid'd into their own session by launch, so kill(-pgid) can't
//    reach them. We pkill -f by command line (hits launch proc + every node)
//    and reap the launch PIDs concurrently. Stopping them first also cuts the
//    bag/odom data sources (bag records /livox/*, odom subs /lio_sam/*).
//  Phase 2 — camera + bag (5s): image_odom_recorder is its own group leader and
//    its forked ros2 bag record shares the group, so kill(-camera_pid) reaches
//    both. On timeout we SIGKILL only the leader (never the group — that would
//    punch through a flushing rosbag2) and separately retire any orphaned bag.

#include <nlohmann/json.hpp>

#include <mutex>
#include <optional>
#include <string>
#include <sys/types.h>
#include <vector>

#include "r3d_ws/config.hpp"

namespace r3d_ws {

class MappingControl {
 public:
  // {"version","deviceCode","battery","storageTotal","storageUsed",
  //  "storagePercent","wsStatus"} — mirrors get_device_info().
  static nlohmann::json device_info();

  // Subfolders of SAVE_PCD_DIR, newest-first. Each entry:
  // {"name","createTime":"%Y-%m-%d %H:%M:%S","size":"文件夹"}.
  static nlohmann::json pcd_file_list();

  // Atomically write {"cmd": value} to SHARE_FILE (same format R3D_cpp uses).
  static bool write_shared_cmd(int value);

  // Background-process lifecycle: launch via setsid (logs to
  // /tmp/r3d_mapping_<name>.log); stop via kill(-pid, SIGINT) on the whole
  // process group. No display / terminal required.
  void start_mapping_commands();
  void close_all_windows();

  // startRecord 前置整定的曝光模式覆盖：tuned=true → camera 以 auto_exposure:=false
  // 启动（V4L2Camera 切 Manual 时保留驱动里的整定值）；tuned=false → 用配置文件的
  // auto_exposure（AE 兜底）。每次 startRecord 整定后重设。
  void set_exposure_tuned(bool tuned);

  bool mapping_ready() const;

  // Loads the three command strings from JSON once (see config.hpp:
  // MAPPING_CMD_FILE_DEFAULT, overridable via $R3D_MAPPING_CMD_FILE); on any
  // load failure falls back to built-in defaults. Never throws.
  MappingControl();
  // Safety net: if mapping_ready_, stop all child processes so they don't
  // survive the bridge as orphans holding Livox/camera/bag resources.
  ~MappingControl();

 private:
  // fork once -> child setsid -> redirect std fds -> execl bash -c cmd.
  // Returns child PID (== PGID == SID after setsid) or -1 on any failure.
  // Child uses only async-signal-safe calls (bridge is multi-threaded).
  static pid_t spawn_background(const std::string& cmd, const std::string& logfile);
  // Spawn cmd logged to /tmp/r3d_mapping_<name>.log; returns pid or -1.
  static pid_t launch_background(const std::string& cmd, const char* name);
  // fork + execvp pkill/pgrep — async-signal-safe child (stack argv, snprintf,
  // execvp; no heap alloc / mutex, same fork-safety rationale as spawn_background).
  //   is_kill=true  -> pkill -<sig> -f pattern  (deliver signal)
  //   is_kill=false -> pgrep -f pattern         (detect only)
  // Returns whether any process matched (pkill/pgrep exit 0 = match, 1 = none).
  static bool run_pk(const char* pattern, int sig, bool is_kill);
  // Concurrently poll waitpid(WNOHANG) on several direct children for up to
  // grace_ms. Returns true if all were reaped (or were never our child, ECHILD).
  static bool reap_all_within(const std::vector<pid_t>& pids, int grace_ms);

  // Three command strings, loaded once from JSON (or built-in defaults on any
  // failure). Immutable after construction — read by start_mapping_commands().
  // data_dir 是相机图片/odom/rosbag 的存储根目录（非命令串）：启动 camera 时桥自动
  // 追加 `-p output_dir:=<data_dir>` 注入 image_odom_recorder；留空则用节点默认。
  struct LoadedCommands {
    std::string livox;
    std::string lio_sam;
    std::string camera;
    std::string data_dir;
    bool auto_exposure = true;  // JSON "auto_exposure"（默认 true），注入 camera 参数
    bool auto_wb = true;        // JSON "auto_wb"（默认 true），注入 camera 参数
    bool save_odom = true;      // JSON "save_odom"（默认 true），注入 camera 参数
  };
  // Reads $R3D_MAPPING_CMD_FILE (else MAPPING_CMD_FILE_DEFAULT rel. to cwd); on
  // any error the affected field — or all fields — falls back to the built-in
  // default. Never throws.
  static LoadedCommands load_commands_from_file();

  mutable std::mutex m_;
  // Launch PIDs (== PGIDs after setsid) of the three mapping procs. livox/lio_sam
  // are ros2 launch trees; camera is exec ros2 run (group leader, bag shares group).
  pid_t livox_pid_{-1};
  pid_t lio_sam_pid_{-1};
  pid_t camera_pid_{-1};
  bool mapping_ready_{false};
  // startRecord 整定结果覆盖（m_ 保护）：engaged 时 start_mapping_commands 用
  // *value 替代 cmds_.auto_exposure 注入 camera。nullopt=未整定/整定失败。
  std::optional<bool> exp_override_;
  const LoadedCommands cmds_;  // write-once in ctor; read-only thereafter
};

} // namespace r3d_ws
