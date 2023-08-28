cmake_minimum_required(VERSION 3.8)
project(ros_natnet)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

#Add NatNet
set (NatNet_SDK_PATH CACHE PATH "full path to NatNet sdk root folder")
find_library(NatNet_LIBRARY NAMES libNatNet.so PATHS ${NatNet_SDK_PATH}/lib/)
set (EXTRA_LIBS ${EXTRA_LIBS} ${NatNet_LIBRARY})

add_executable(broadcast src/mocap_broadcaster.cpp src/NatNetTfBroadcaster.cpp)
ament_target_dependencies(
	broadcast
	rclcpp
	std_msgs
	geometry_msgs
	tf2
	tf2_ros
	)
target_link_libraries (broadcast ${EXTRA_LIBS})

target_include_directories(broadcast PUBLIC
	$<BUILD_INTERFACE:${NatNet_SDK_PATH}/include>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
  )
target_compile_features(broadcast PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS broadcast
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
