cmake_minimum_required(VERSION 3.8)
project(scan_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(path_searching REQUIRED)
find_package(plan_env REQUIRED)
find_package(rclcpp REQUIRED)
find_package(scan_planner_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(traj_utils REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-msgs8 REQUIRED)

set(scan_planner_dependencies
  ament_index_cpp bspline_opt geometry_msgs map_msgs nav_msgs path_searching plan_env
  rclcpp scan_planner_msgs sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros
  trajectory_msgs traj_utils visualization_msgs)

add_executable(scan_planner_node
  src/scan_planner_node.cpp
  src/scan_replan_fsm.cpp
  src/planner_manager.cpp)
target_include_directories(scan_planner_node PRIVATE include)
ament_target_dependencies(scan_planner_node ${scan_planner_dependencies})
target_link_libraries(scan_planner_node Eigen3::Eigen)

foreach(target open_loop_controller closed_loop_controller go2_kinematic_sim go2_gait_publisher)
  add_executable(${target} src/${target}.cpp)
  target_include_directories(${target} PRIVATE include)
  ament_target_dependencies(${target}
    bspline_opt geometry_msgs nav_msgs rclcpp scan_planner_msgs sensor_msgs
    std_msgs tf2 tf2_geometry_msgs tf2_ros)
  target_link_libraries(${target} Eigen3::Eigen)
endforeach()

# Planar cmd_vel adapter node for Gazebo ApplyLinkWrench
add_executable(planar_cmd_vel_adapter src/planar_cmd_vel_adapter.cpp)
target_include_directories(planar_cmd_vel_adapter PRIVATE include)
ament_target_dependencies(planar_cmd_vel_adapter
  geometry_msgs nav_msgs rclcpp)
target_link_libraries(planar_cmd_vel_adapter
  ignition-transport11::ignition-transport11
  ignition-msgs8::ignition-msgs8)

install(TARGETS
  scan_planner_node open_loop_controller closed_loop_controller
  go2_kinematic_sim go2_gait_publisher planar_cmd_vel_adapter
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config maps DESTINATION share/${PROJECT_NAME} OPTIONAL)
install(FILES launch/default.rviz DESTINATION share/${PROJECT_NAME}/rviz OPTIONAL)
install(PROGRAMS
  scripts/keypoint_recorder.py
  scripts/global_astar_planner.py
  scripts/frontier_explorer.py
  scripts/global_representation_node.py
  scripts/sdf_map_publisher.py
  scripts/sdf_lidar_simulator.py
  scripts/go2_tf_bridge.py
  scripts/go2_policy_node.py
  scripts/go2_locomotion_supervisor.py
  scripts/go2_locomotion_logic.py
  DESTINATION lib/${PROJECT_NAME} OPTIONAL)
install(DIRECTORY scripts/explorer_core
  DESTINATION lib/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_pytest_test(test_global_astar test/test_global_astar.py)
  ament_add_pytest_test(test_frontier_explorer test/test_frontier_explorer.py)
  ament_add_pytest_test(test_sparse_routing test/test_sparse_routing.py)
  ament_add_pytest_test(test_region_commitment test/test_region_commitment.py)
  ament_add_pytest_test(test_topology test/test_topology.py)
  ament_add_pytest_test(test_safe_region_graph test/test_safe_region_graph.py)
  ament_add_pytest_test(test_portal_connectivity test/test_portal_connectivity.py)
  ament_add_pytest_test(test_tunnel_grid_third_slice test/test_tunnel_grid_third_slice.py)
  ament_add_pytest_test(test_sdf_world_geometry test/test_sdf_world_geometry.py)
  # The locomotion supervisor regression test uses unittest instead of pytest:
  # pytest cannot start in this workspace (broken anyio plugin), so a pytest
  # target here would silently never run.
  find_package(PythonInterp REQUIRED)
  add_test(NAME test_locomotion_supervisor
    COMMAND ${PYTHON_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/test/test_locomotion_supervisor.py)
  ament_add_gtest(test_replan_fsm_utils test/test_replan_fsm_utils.cpp)
  if(TARGET test_replan_fsm_utils)
    target_include_directories(test_replan_fsm_utils PRIVATE include)
    target_link_libraries(test_replan_fsm_utils Eigen3::Eigen)
  endif()
  add_launch_test(test/test_planner_startup.py TIMEOUT 30)
  add_launch_test(test/test_waypoint_parameters.py TIMEOUT 30)
endif()

ament_package()
