#!/usr/bin/env python3
"""Runner that launches the ORIGINAL R3D/web_a.py for the Tier-2 benchmark, with
its send loop force-enabled (is_recording & mapping_ready = True) so data flows
without launching terminals.

Requires the web_a.py dependencies: `pip install websockets opencv-python`,
plus rclpy (ROS 2). On this sandbox `websockets`/pip is absent, so the Python
end-to-end numbers must be gathered on a machine that has it — see BENCHMARK.md.
"""
import asyncio
import sys
import threading

sys.path.insert(0, "/home/bjw/code/R3D")  # location of web_a.py
import web_a  # noqa: E402  (imports rclpy / cv2 / websockets)

# Force the send loop on (mirrors what R3D_FORCE_STREAM does in the C++ port).
web_a.is_recording = True
web_a.mapping_ready = True

# Start the ROS2 subscriber exactly like web_a.py::__main__.
threading.Thread(target=web_a.run_ros2, daemon=True).start()

asyncio.run(web_a.main())
