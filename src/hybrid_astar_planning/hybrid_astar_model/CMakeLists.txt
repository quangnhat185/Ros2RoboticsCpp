cmake_minimum_required(VERSION 3.8)
project(hybrid_astar_model)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
add_executable(hybrid_astar_run src/main.cpp src/reeds_shepp.cpp src/car.cpp src/hybrid_astar_model.cpp)
ament_target_dependencies(hybrid_astar_run rclcpp visualization_msgs tf2_ros geometry_msgs Eigen3)

install (TARGETS hybrid_astar_run
  DESTINATION lib/${PROJECT_NAME}
)

install (DIRECTORY
  launch
  rviz
  meshes
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# Add the test executable

if(BUILD_TESTING)
find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_car test/test_car.cpp src/car.cpp)
  ament_target_dependencies(test_car rclcpp Eigen3)

  ament_add_gtest(test_reedsShepp test/test_reeds_shepp.cpp src/reeds_shepp.cpp)
  ament_target_dependencies(test_reedsShepp rclcpp Eigen3)

  ament_add_gtest(test_hybird_astar_model test/test_hybrid_astar_model.cpp src/reeds_shepp.cpp src/hybrid_astar_model.cpp src/car.cpp)
endif()

ament_package()
