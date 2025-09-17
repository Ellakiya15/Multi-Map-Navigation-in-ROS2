cmake_minimum_required(VERSION 3.8)
project(wormhole_navigator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(wormhole_interfaces REQUIRED)

# --- SQLite3 (system dependency) ---
find_path(SQLITE3_INCLUDE_DIR sqlite3.h)
find_library(SQLITE3_LIBRARY sqlite3)
if(NOT SQLITE3_INCLUDE_DIR OR NOT SQLITE3_LIBRARY)
  message(FATAL_ERROR "Could not find SQLite3. Install libsqlite3-dev")
endif()

# --- Include directories ---
include_directories(
  include
  ${SQLITE3_INCLUDE_DIR}
)

# --- Sources ---
set(SOURCES
  src/wormhole_navigator.cpp
)

# --- Executable ---
add_executable(wormhole_navigator_node ${SOURCES})

ament_target_dependencies(wormhole_navigator_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  nav2_msgs
  wormhole_interfaces
)

target_link_libraries(wormhole_navigator_node
  ${SQLITE3_LIBRARY}
)

# --- Install target ---
install(TARGETS
  wormhole_navigator_node
  DESTINATION lib/${PROJECT_NAME}
)

# --- Install headers ---
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()

