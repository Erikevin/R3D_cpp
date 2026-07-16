cmake_minimum_required(VERSION 3.8)
project(r3d_bringup CXX)

# ---- language / warnings ---------------------------------------------------
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
# Static lib is linked into each executable, so compile objects position-independent
# (Ubuntu builds PIE executables by default).
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# ---- dependencies ----------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
# boost::beast / boost::asio are header-only; no link line needed.

# ---- core library (static — linked into each executable) ------------------
# NOTE: ament_target_dependencies() uses the *plain* signature of
# target_link_libraries, so every target_link_libraries() in this file must
# also use the plain signature (no PRIVATE/PUBLIC keywords) to stay consistent.
add_library(${PROJECT_NAME}_lib STATIC
  src/ws_bridge_node.cpp
  src/ws_server.cpp
  src/mapping_control.cpp
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_lib rclcpp sensor_msgs nav_msgs)
target_link_libraries(${PROJECT_NAME}_lib ${OpenCV_LIBS} Threads::Threads)

# ---- r3d_ws_bridge: the WebSocket bridge node (= web_a.py) ------------------
add_executable(r3d_ws_bridge src/main.cpp)
target_link_libraries(r3d_ws_bridge ${PROJECT_NAME}_lib)
ament_target_dependencies(r3d_ws_bridge rclcpp)

# ---- mock_publisher: synthetic ROS topics for headless testing -------------
add_executable(mock_publisher src/mock_publisher.cpp)
ament_target_dependencies(mock_publisher rclcpp sensor_msgs nav_msgs)
target_link_libraries(mock_publisher ${OpenCV_LIBS})

# ---- r3d_bench: microbenchmark (parse / json / base64) ---------------------
add_executable(r3d_bench src/bench.cpp)
target_link_libraries(r3d_bench ${PROJECT_NAME}_lib)
ament_target_dependencies(r3d_bench sensor_msgs)

# ---- install ---------------------------------------------------------------
install(TARGETS
    r3d_ws_bridge
    mock_publisher
    r3d_bench
  DESTINATION lib/${PROJECT_NAME})

ament_package()
