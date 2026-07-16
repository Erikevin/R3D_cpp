#pragma once
// Cross-process command file IPC — C++ mirror of the /tmp/shared_cmd.txt logic
// shared between main.py and test_process.py / web_a.py.
//
// The file holds a tiny fixed-shape JSON object: {"cmd": <int>}.

#include <string>

namespace r3d::ipc {

// Read the "cmd" field from the share file. Returns true on success.
// Returns cmd=-1 (and true) if the value is absent/malformed, mirroring Python.
bool read_cmd(const std::string& path, int& cmd);

// Write {"cmd": value} atomically (temp file + rename).
bool write_cmd(const std::string& path, int value);

} // namespace r3d::ipc
