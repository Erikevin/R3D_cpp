#pragma once
// Device / window / file-list control — C++ mirror of the device-management
// half of R3D/web_a.py (get_device_info, open_terminal, start_mapping_commands,
// stop_process_in_terminal, close_all_windows, get_pcd_file_list, write_shared_cmd).

#include <nlohmann/json.hpp>

#include <mutex>
#include <string>
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

  // Terminal / window lifecycle. These call gnome-terminal + xdotool exactly as
  // the Python does; on a headless host (no xdotool / no DISPLAY) they log and
  // return without throwing.
  void start_mapping_commands();
  void close_all_windows();

  bool mapping_ready() const;

 private:
  // Open a gnome-terminal running `cmd`; return the new xdotool window id or "".
  static std::string open_terminal(const std::string& cmd);
  // Send Ctrl+C into the window with the given id.
  static void stop_process_in_terminal(const std::string& xid);

  mutable std::mutex m_;
  std::vector<std::string> xids_;
  bool mapping_ready_{false};
};

} // namespace r3d_ws
