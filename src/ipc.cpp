// File-based cross-process IPC. Minimal hand parser for {"cmd": <int>}.

#include "r3d/ipc.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>

#include <fstream>
#include <sstream>
#include <string>

#include "r3d/logger.hpp"

namespace r3d::ipc {

bool read_cmd(const std::string& path, int& cmd) {
    cmd = -1;
    std::ifstream f(path);
    if (!f) return false;
    std::stringstream ss; ss << f.rdbuf();
    std::string s = ss.str();

    // Find "cmd" key, then the first integer after the colon.
    std::size_t k = s.find("cmd");
    if (k == std::string::npos) return true;
    std::size_t colon = s.find(':', k);
    if (colon == std::string::npos) return true;
    try {
        cmd = std::stoi(s.substr(colon + 1));
    } catch (...) {
        cmd = -1;
    }
    return true;
}

bool write_cmd(const std::string& path, int value) {
    std::string tmp = path + ".tmp";
    {
        std::ofstream f(tmp, std::ios::trunc);
        if (!f) {
            log::error("[IPC] 写入共享文件失败: " + tmp + " — " + std::strerror(errno));
            return false;
        }
        f << "{\"cmd\": " << value << "}";
    }
    if (std::rename(tmp.c_str(), path.c_str()) != 0) {
        log::error("[IPC] rename 失败: " + std::string(std::strerror(errno)));
        return false;
    }
    log::info("[IPC] 已写入跨进程指令: " + std::to_string(value) + " -> " + path);
    return true;
}

} // namespace r3d::ipc
