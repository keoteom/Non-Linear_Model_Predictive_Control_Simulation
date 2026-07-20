cmake_minimum_required(VERSION 3.8)
project(nmpc_uav_avoidance)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================================
#  Find dependencies
# ============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(px4_msgs REQUIRED)

# ============================================================================
#  OpEn solver (C bindings)
# ============================================================================
# Set this to the path where generate_solver.py created the solver
# Default: same directory as CMakeLists.txt
set(OPEN_SOLVER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/nmpc_solver/nmpc_uav_obstacle"
    CACHE PATH "Path to OpEn generated solver")

# ============================================================================
#  Executable
# ============================================================================
add_executable(nmpc_node
  src/main.cpp
  src/nmpc_node.cpp
)

target_include_directories(nmpc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  # OpEn generated C header
  ${OPEN_SOLVER_DIR}
)

# Link OpEn static library (compiled Rust)
target_link_libraries(nmpc_node
  ${OPEN_SOLVER_DIR}/target/release/libnmpc_uav_obstacle.a
  pthread
  dl
  m
)

ament_target_dependencies(nmpc_node
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  px4_msgs
)

# ============================================================================
#  Odom publisher (for testing)
# ============================================================================
add_executable(odom_publisher
  src/odom_publisher.cpp
)

ament_target_dependencies(odom_publisher
  rclcpp
  nav_msgs
  px4_msgs
)


# ============================================================================
#  Testing
# ============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS nmpc_node odom_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()