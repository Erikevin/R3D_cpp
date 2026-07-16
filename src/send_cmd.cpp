// r3d_send_cmd — CLI writer for /tmp/shared_cmd.txt. Mirrors test_process.py.
//
//   r3d_send_cmd 1   # open radar / start mapping
//   r3d_send_cmd 0   # close radar / stop mapping

#include <cstdlib>
#include <iostream>

#include "r3d/config.hpp"
#include "r3d/ipc.hpp"

int main(int argc, char** argv) {
    int value = (argc >= 2) ? std::atoi(argv[1]) : 0;
    std::cout << "测试进程启动\n";

    if (r3d::ipc::write_cmd(r3d::config::SHARE_FILE, value)) {
        std::cout << "已发送指令：" << value << "\n";
        return 0;
    }
    return 1;
}
