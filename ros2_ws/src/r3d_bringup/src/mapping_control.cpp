// Implementation of device/window/file-list control — faithful port of the
// device-management half of R3D/web_a.py.

#include "r3d_ws/mapping_control.hpp"

#include "r3d_ws/logger.hpp"

#include <sys/stat.h>
#include <sys/statvfs.h>
#include <sys/wait.h>
#include <dirent.h>
#include <fcntl.h>
#include <signal.h>
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
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

namespace r3d_ws {

// ---- MappingControl: background-process spawn/stop -------------------------
// spawn_background: fork once -> child setsid() -> redirect std fds ->
// execl("/bin/bash","-c",cmd). Returns child PID (>0, and PID==PGID==SID after
// setsid) on success, or -1 on fork failure / child died before exec.
//
// A close-on-exec pipe reports the fork->exec outcome:
//   - exec succeeds -> kernel auto-closes write end -> parent read() gets EOF.
//   - exec/setsid/open fails in child -> child writes errno then _exit.
// The child uses ONLY async-signal-safe calls (no log::, no std::strerror, no
// std::string alloc) because the bridge is multi-threaded and a mutex could be
// locked-at-fork -> deadlock. Inherited non-CLOEXEC fds (WsServer socket, ROS2
// transports, OpenCV) are closed in the child before exec.
pid_t MappingControl::spawn_background(const std::string& cmd,
                                       const std::string& logfile) {
  int pipefd[2];
  if (pipe2(pipefd, O_CLOEXEC) < 0) {
    log::error(std::string("[mapping] pipe2 失败: ") + std::strerror(errno));
    return -1;
  }

  pid_t pid = fork();
  if (pid < 0) {
    log::error(std::string("[mapping] fork 失败: ") + std::strerror(errno));
    close(pipefd[0]);
    close(pipefd[1]);
    return -1;
  }

  if (pid == 0) {
    // ---------- child: async-signal-safe only ----------
    close(pipefd[0]);  // close read end; write end stays O_CLOEXEC

    if (setsid() < 0) {
      int e = errno; ssize_t w = ::write(pipefd[1], &e, sizeof(e)); (void)w; _exit(126);
    }

    int devnull = ::open("/dev/null", O_RDONLY);
    int logfd   = ::open(logfile.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (devnull < 0 || logfd < 0) {
      int e = errno; ssize_t w = ::write(pipefd[1], &e, sizeof(e)); (void)w; _exit(126);
    }

    // Close inherited non-CLOEXEC fds (keep 0/1/2 and the pipe write end).
    DIR* d = ::opendir("/proc/self/fd");
    if (d) {
      struct dirent* de;
      while ((de = ::readdir(d)) != nullptr) {
        int fdi = std::atoi(de->d_name);
        if (fdi > STDERR_FILENO && fdi != pipefd[1]) ::close(fdi);
      }
      ::closedir(d);
    }

    ::dup2(devnull, STDIN_FILENO);
    ::dup2(logfd,   STDOUT_FILENO);
    ::dup2(logfd,   STDERR_FILENO);
    if (devnull > STDERR_FILENO) ::close(devnull);
    if (logfd   > STDERR_FILENO) ::close(logfd);

    ::execl("/bin/bash", "bash", "-c", cmd.c_str(), (char*)nullptr);
    // execl returns only on failure:
    int e = errno; ssize_t w = ::write(pipefd[1], &e, sizeof(e)); (void)w; _exit(127);
  }

  // ---------- parent ----------
  close(pipefd[1]);
  int ec = 0;
  ssize_t n = ::read(pipefd[0], &ec, sizeof(ec));
  close(pipefd[0]);

  if (n == 0) return pid;  // CLOEXEC auto-close -> exec succeeded

  log::error(std::string("[mapping] 子进程启动失败(setsid/open/execl): ")
             + std::strerror(n == sizeof(ec) ? ec : errno));
  int st = 0;
  while (waitpid(pid, &st, 0) < 0 && errno == EINTR) {
  }
  return -1;
}

// Launch a mapping command logged to /tmp/r3d_mapping_<name>.log. Returns the
// child pid (>0) or -1. Logs pid + logfile for the operator.
pid_t MappingControl::launch_background(const std::string& cmd, const char* name) {
  std::string logfile = std::string("/tmp/r3d_mapping_") + name + ".log";
  pid_t pid = spawn_background(cmd, logfile);
  if (pid < 0) {
    log::error(std::string("[mapping] 启动失败: ") + name);
    return -1;
  }
  log::info(std::string("[mapping] 已启动 ") + name
            + "  pid=" + std::to_string(pid)
            + "  log=" + logfile);
  return pid;
}

// fork + execvp pkill/pgrep. The child builds argv from stack arrays + snprintf
// and calls only async-signal-safe functions — same fork-at-locked-mutex hazard
// as spawn_background (bridge is multi-threaded), so no heap alloc / no mutex.
//   is_kill=true  -> execvp("pkill", {"pkill","-<sig>","-f",pattern})
//   is_kill=false -> execvp("pgrep",  {"pgrep","-f",pattern})
// pkill/pgrep exit 0 when >=1 process matched, 1 when none matched. We map that
// to a bool so callers can tell "delivered / still present" from "nothing there".
bool MappingControl::run_pk(const char* pattern, int sig, bool is_kill) {
  pid_t pid = ::fork();
  if (pid < 0) {
    log::error(std::string("[mapping] run_pk fork 失败: ") + std::strerror(errno));
    return false;
  }
  if (pid == 0) {
    // child: async-signal-safe only. 绝对路径优先 —— 桥由 systemd 拉起时 PATH 可能
    // 精简，execvp("pkill") 可能找不到；execv("/usr/bin/pkill") 不依赖 PATH。
    if (is_kill) {
      char sigstr[16];
      ::snprintf(sigstr, sizeof(sigstr), "-%d", sig);
      char* argv[] = {const_cast<char*>("pkill"), sigstr,
                      const_cast<char*>("-f"), const_cast<char*>(pattern), nullptr};
      ::execv("/usr/bin/pkill", argv);
      ::execvp("pkill", argv);  // fallback：非标准安装路径再按 PATH 找一次
    } else {
      char* argv[] = {const_cast<char*>("pgrep"), const_cast<char*>("-f"),
                      const_cast<char*>(pattern), nullptr};
      ::execv("/usr/bin/pgrep", argv);
      ::execvp("pgrep", argv);
    }
    _exit(127);  // execv/execvp 均失败
  }
  int st = 0;
  while (::waitpid(pid, &st, 0) < 0 && errno == EINTR) {
  }
  // 退出码语义（procps）：0=匹配到并已发信号/已列出，1=无匹配，127=exec 失败。
  const int rc = WIFEXITED(st) ? WEXITSTATUS(st) : -1;
  log::info(std::string("[mapping] run_pk ") + (is_kill ? "pkill" : "pgrep") + " -f '" +
            pattern + "' rc=" + std::to_string(rc) +
            (rc == 0 ? "(命中)" : rc == 1 ? "(无匹配)" : rc == 127 ? "(exec失败)" : "(异常)"));
  return rc == 0;
}

// Concurrently poll waitpid(WNOHANG) across several direct children for up to
// grace_ms (100ms steps). Returns true once all are reaped (or were never our
// child — ECHILD). Used to give a shared graceful window to several launches
// without serializing the wait (lio_sam + livox stop in parallel).
bool MappingControl::reap_all_within(const std::vector<pid_t>& pids, int grace_ms) {
  std::vector<pid_t> pending;
  for (pid_t p : pids) {
    if (p > 0) pending.push_back(p);
  }
  const int step_ms = 100;
  for (int i = 0; i < grace_ms / step_ms && !pending.empty(); ++i) {
    for (auto it = pending.begin(); it != pending.end();) {
      int st = 0;
      pid_t r = ::waitpid(*it, &st, WNOHANG);
      if (r == *it || (r < 0 && errno == ECHILD)) {
        it = pending.erase(it);
      } else {
        ++it;
      }
    }
    if (!pending.empty()) std::this_thread::sleep_for(std::chrono::milliseconds(step_ms));
  }
  return pending.empty();
}

// ---- device info -----------------------------------------------------------
nlohmann::json MappingControl::device_info() {
  nlohmann::json info;
  info["version"]    = "R3D_V1_0_0";
  info["deviceCode"] = "B258A49D02";
  // battery: read the real level published by r3d_main to the shared telemetry
  // file; fall back to 78 when r3d_main hasn't published yet (file missing /
  // unreadable / malformed). /tmp is tmpfs, so this read is memory-only and
  // never contends with rosbag2/PCD disk writes even under periodic getStorage.
  int battery = 78;
  {
    std::ifstream f(config::TELEMETRY_FILE);
    if (f) {
      std::stringstream ss;
      ss << f.rdbuf();
      try {
        auto j = nlohmann::json::parse(ss.str());
        if (j.contains("battery") && j["battery"].is_number_integer()) {
          battery = j["battery"].get<int>();
        }
      } catch (const std::exception&) {
        // stale/partial write (atomic rename should prevent this) → keep default
      }
    }
  }
  info["battery"]    = battery;
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
// start_mapping_commands() 把 Livox / LIO-SAM / 相机录制作为后台进程启动（setsid），输出重定向到
// /tmp/r3d_mapping_<name>.log；继承 r3d_ws_bridge 的 cwd，故工作区用相对路径
// "ros2_ws/install/setup.bash" —— 桥必须从 ros2_ws 的父目录启动。Livox / LIO-SAM 需其各自包已装入
// 该 ros2_ws，否则对应进程会报 "package not found"。
//
// 三个命令串外部化到 JSON（见 config.hpp: MAPPING_CMD_FILE_DEFAULT，可用 $R3D_MAPPING_CMD_FILE
// 覆盖），改文件 + 重启 r3d_ws_bridge 即生效，无需重新编译。任何加载失败均回退到下面的 kDef*
// 内置默认并告警，绝不中断建图。命令串经 execl argv 传给 bash（不经 shell 拼接），故可含任意
// bash 语法（引号、$() 等）。
MappingControl::MappingControl() : cmds_(load_commands_from_file()) {}

MappingControl::~MappingControl() {
  // 桥退出时若建图仍在运行，主动停掉子进程，避免它们成为孤儿继续占用 Livox/相机/bag。
  if (mapping_ready_) close_all_windows();
}

MappingControl::LoadedCommands MappingControl::load_commands_from_file() {
  // 内置默认串（即所有回退的来源）。lio_sam 已修复：原代码相邻字面量拼接处缺 && / 空格，
  // 实际产生 "...local_setup.bashros2 launch..."。
  // 末尾不用 "; exec bash"（那是为保持交互终端窗口）；改用 "exec ros2 ..."，让 bash 直接替换成
  // workload，PID 精确指向 ros2 launch/run，便于 kill(-pid) 与 waitpid。
  static const std::string kDefLivox =
      "source /opt/ros/humble/setup.bash && source ~/.bashrc && "
      "source ros2_ws/install/setup.bash && "
      "exec ros2 launch livox_ros_driver2 msg_MID360_launch.py";
  static const std::string kDefLioSam =
      "source /opt/ros/humble/setup.bash && source ~/.bashrc && "
      "source ros2_ws/install/setup.bash && "
      "source /root/my_robot_x/install/local_setup.bash && "
      "exec ros2 launch lio_sam run.launch.py";
  static const std::string kDefCamera =
      "source /opt/ros/humble/setup.bash && source ~/.bashrc && "
      "source ros2_ws/install/setup.bash && "
      "exec ros2 run r3d_bringup image_odom_recorder";

  LoadedCommands out{kDefLivox, kDefLioSam, kDefCamera, ""};  // data_dir 默认空=用节点默认

  // 1) 解析路径：优先 $R3D_MAPPING_CMD_FILE，否则默认相对 cwd 路径。
  const char* env = std::getenv(config::MAPPING_CMD_FILE_ENV);
  const std::string path = (env && *env) ? env : config::MAPPING_CMD_FILE_DEFAULT;

  // 2) 读取整个文件（惯用法同 ipc.cpp 的 read_cmd）。
  std::ifstream f(path);
  if (!f) {
    log::warn(std::string("建图命令配置文件无法打开，使用内置默认: ") + path);
    return out;  // 全默认 —— 安全
  }
  std::stringstream ss;
  ss << f.rdbuf();

  // 3) 解析 JSON。
  nlohmann::json j;
  try {
    j = nlohmann::json::parse(ss.str());
  } catch (const std::exception& e) {
    log::warn(std::string("建图命令配置 JSON 解析失败，使用内置默认: ") + e.what());
    return out;  // 全默认 —— 安全
  }
  if (!j.is_object()) {
    log::warn("建图命令配置 JSON 顶层非对象，使用内置默认");
    return out;  // 全默认 —— 安全
  }

  // 4) 逐字段：类型检查 -> 空值守卫 -> 接受。
  auto apply = [&](const char* key, std::string& dst) {
    if (!j.contains(key)) {
      log::warn(std::string("配置项缺失，该项使用默认: ") + key);
      return;  // dst 已是默认
    }
    const auto& v = j.at(key);  // key 已确认存在，不插入
    if (!v.is_string()) {
      log::warn(std::string("配置项非字符串，该项使用默认: ") + key);
      return;
    }
    std::string s = v.get<std::string>();
    if (s.empty()) {  // 单引号约束已随 gnome-terminal 移除：execl argv 形式下命令串含引号合法
      log::warn(std::string("配置项为空，该项使用默认: ") + key);
      return;
    }
    dst = std::move(s);
  };
  apply("livox", out.livox);
  apply("lio_sam", out.lio_sam);
  apply("camera", out.camera);

  // data_dir 是可选配置（非命令串）：相机图片 / odom / rosbag 的存储根目录，启动
  // camera 时桥注入 `-p output_dir:=<data_dir>`。缺失或留空都静默接受为 ""，
  // 此时节点用默认 ~/Desktop/save_pcd/<时间戳>。不走 apply()——它对缺失/空会告警，
  // 对可选项是无意义噪音。
  if (j.contains("data_dir") && j.at("data_dir").is_string()) {
    out.data_dir = j.at("data_dir").get<std::string>();
  }

  log::info(std::string("已加载建图命令配置: ") + path);
  return out;
}

void MappingControl::start_mapping_commands() {
  try {
    log::info("🔹 启动 Livox 驱动");
    if (pid_t pid = launch_background(cmds_.livox, "livox"); pid > 0) {
      std::lock_guard<std::mutex> lk(m_);
      livox_pid_ = pid;
    }
    std::this_thread::sleep_for(std::chrono::seconds(5));

    log::info("🔹 启动 LIO-SAM");
    if (pid_t pid = launch_background(cmds_.lio_sam, "lio_sam"); pid > 0) {
      std::lock_guard<std::mutex> lk(m_);
      lio_sam_pid_ = pid;
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));

    log::info("🔹 启动相机录制");
    // data_dir 非空时注入 -p output_dir:=<data_dir>：相机图片/odom/rosbag 的存储根。
    // 单引号包裹路径，兼容含空格/特殊字符的 data_dir（命令串经 bash -c 执行，会正确去引号）。
    // 节点在 <data_dir>/<时间戳>/ 下落盘（每次录制新建子目录，不覆盖）。
    std::string cam_cmd = cmds_.camera;
    if (!cmds_.data_dir.empty()) {
      cam_cmd += " -p output_dir:='" + cmds_.data_dir + "'";
      log::info(std::string("[mapping] 数据存储根目录: ") + cmds_.data_dir + "/<时间戳>");
    }
    if (pid_t pid = launch_background(cam_cmd, "camera"); pid > 0) {
      std::lock_guard<std::mutex> lk(m_);
      camera_pid_ = pid;
    }

    {
      std::lock_guard<std::mutex> lk(m_);
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
  // 锁内拷贝 pid 快照，并立即清空 —— 防止 stopRecord 线程与析构并发时二次停止。
  pid_t livox, liosam, camera;
  {
    std::lock_guard<std::mutex> lk(m_);
    livox = livox_pid_;
    liosam = lio_sam_pid_;
    camera = camera_pid_;
    livox_pid_ = lio_sam_pid_ = camera_pid_ = -1;
    mapping_ready_ = false;
  }
  log::info("🧹 正在安全停止所有建图进程...");
  log::info(std::string("[mapping] close_all_windows pid 快照 livox=") + std::to_string(livox) +
            " lio_sam=" + std::to_string(liosam) + " camera=" + std::to_string(camera) +
            " (-1=未启动/已清空)");

  // ============ 阶段 1：并发停 CPU 大头（lio_sam + livox）============
  // 先停它们两个收益：① 释放 CPU/IO；② bag 录的正是 /livox/imu /livox/lidar、
  // odom 订阅 /lio_sam/mapping/odometry —— 停了即掐断数据源，bag 缓冲不再增长，
  // 阶段 2 的 flush 量固定且小。camera 触发用 steady_clock 自驱，不依赖 livox。
  //
  // 这两个是 ros2 launch 拉起的树：launch 给每个节点 setsid → 节点脱离 launch 主
  // 进程的进程组，kill(-launch_pid) 够不到节点（这正是旧 stop_background 偶尔漏杀、
  // 节点残留的根因）。改用 pkill -f 按命令行点名 —— 同时命中 ros2 launch 主进程与
  // 各节点进程，绕开进程组。
  log::info("🔹 [阶段1] 停止 lio_sam + livox（释放 CPU / 断 bag·odom 数据源）");
  run_pk("lio_sam", SIGINT, true);
  run_pk("livox_ros_driver2", SIGINT, true);

  std::vector<pid_t> stage1;
  if (liosam > 0) stage1.push_back(liosam);
  if (livox > 0) stage1.push_back(livox);
  // lio_sam 存 PCD / GTSAM 因子图清理耗时，给 8s 优雅窗口；并发 reap 两个 launch 主进程。
  if (!reap_all_within(stage1, 8000)) {
    log::warn("[mapping] 阶段1 超时，SIGKILL 强杀残留 lio_sam/livox");
    run_pk("lio_sam", SIGKILL, true);
    run_pk("livox_ros_driver2", SIGKILL, true);
    for (pid_t p : stage1) {  // 阻塞回收未退出的 launch 主进程，避免僵尸
      int st = 0;
      while (::waitpid(p, &st, 0) < 0 && errno == EINTR) {
      }
    }
  }

  // ============ 阶段 2：停 camera + bag（CPU 已空闲、数据源已断）============
  // image_odom_recorder 由 exec ros2 run 拉起、自任组长；其 start_bag() fork 的
  // ros2 bag record 未 setsid → 与节点同组。故 kill(-camera_pid) 组播 SIGINT 能同时
  // 命中节点与 bag，节点析构 ~CaptureNode→stop_bag 再给 bag SIGINT 并轮询 3s flush。
  // 维持 5s 优雅窗口：阶段 1 完成后 bag flush 量小，5s 足够。
  log::info("🔹 [阶段2] 停止 camera + bag");
  if (camera > 0) {
    if (::kill(-camera, SIGINT) < 0 && errno != ESRCH) {
      log::warn(std::string("[mapping] kill(-camera,SIGINT): ") + std::strerror(errno));
    }
    if (!reap_all_within({camera}, 5000)) {
      // 超时：只 SIGKILL 节点组长，【绝不组播 -camera SIGKILL】—— 会打穿正在 flush 的
      // rosbag2（丢缓冲 / 损坏 bag）。bag 子进程的善后单独处理。
      log::warn("[mapping] 阶段2 超时，SIGKILL camera 节点（不组播，保护 bag）");
      ::kill(camera, SIGKILL);
      int st = 0;
      while (::waitpid(camera, &st, 0) < 0 && errno == EINTR) {
      }
      // 兜底：节点被强杀可能没走到析构的 stop_bag，bag 沦为孤儿。单独给它 SIGINT + 3s
      // flush 窗口（CPU 已空闲，flush 很快），仍未退才 SIGKILL（可能丢末尾缓冲）。
      if (run_pk("ros2 bag record", 0, false)) {
        log::warn("[mapping] bag 残留，单独 SIGINT + 3s flush 窗口");
        run_pk("ros2 bag record", SIGINT, true);
        for (int i = 0; i < 30; ++i) {  // 3s
          if (!run_pk("ros2 bag record", 0, false)) break;
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (run_pk("ros2 bag record", 0, false)) {
          log::warn("[mapping] bag 仍未退出，SIGKILL（可能丢末尾缓冲）");
          run_pk("ros2 bag record", SIGKILL, true);
        }
      }
    }
  }

  log::info("✅ 所有建图进程已停止");
}

bool MappingControl::mapping_ready() const {
  std::lock_guard<std::mutex> lk(m_);
  return mapping_ready_;
}

} // namespace r3d_ws
