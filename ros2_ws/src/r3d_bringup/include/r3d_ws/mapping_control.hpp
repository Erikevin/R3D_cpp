#pragma once
// Device / process / file-list control — C++ mirror of the device-management
// half of R3D/web_a.py (get_device_info, start_mapping_commands,
// close_all_windows, get_pcd_file_list, write_shared_cmd).
//
// start_mapping_commands() launches Livox / LIO-SAM / recorder as background
// processes (setsid) with stdout/stderr to /tmp/r3d_mapping_<name>.log, and
// stops them via kill(-pid) on the whole process group — no gnome-terminal /
// xdotool, so it works on headless hosts (e.g. OrangePi 3B).

#include <nlohmann/json.hpp>

#include <mutex>
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
  // kill(-pid, SIGINT) the whole group, poll up to 5s, then SIGKILL + reap.
  static void stop_background(pid_t pid);

  // Three command strings, loaded once from JSON (or built-in defaults on any
  // failure). Immutable after construction — read by start_mapping_commands().
  struct LoadedCommands {
    std::string livox;
    std::string lio_sam;
    std::string camera;
  };
  // Reads $R3D_MAPPING_CMD_FILE (else MAPPING_CMD_FILE_DEFAULT rel. to cwd); on
  // any error the affected field — or all fields — falls back to the built-in
  // default. Never throws.
  static LoadedCommands load_commands_from_file();

  mutable std::mutex m_;
  std::vector<pid_t> pids_;      // child PIDs (== PGIDs) of running mapping procs
  bool mapping_ready_{false};
  const LoadedCommands cmds_;  // write-once in ctor; read-only thereafter
};

} // namespace r3d_ws
