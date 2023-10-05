cmake_minimum_required(VERSION 3.8)
project(ntrip_ros)

set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++2a)
endif()
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(irt_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(novatel_oem7_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(Flatbuffers REQUIRED)

include_directories(
    include
    include/${PROJECT_NAME}
)


rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/NtripCaster.msg"
        "srv/UpdateReferenceStation.srv"
        DEPENDENCIES std_msgs)

ament_export_dependencies(rosidl_default_runtime)

add_executable(ntrip_ros_node
        src/ntrip_ros_node.cpp
        src/rtcm/rtcm_parser.cpp
        src/ntripcom.cpp
        src/udp_service.cpp include/ntrip_ros/ntrip_ros_node.h)
target_include_directories(ntrip_ros_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

#rosidl_get_typesupport_target(cpp_typesupport_target
 #       ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(ntrip_ros_node
        Boost::boost
        ${Boost_LIBRARIES}
        "${cpp_typesupport_target}"
        )
rosidl_target_interfaces(ntrip_ros_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_compile_features(ntrip_ros_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  ntrip_ros_node
  "rclcpp"
  "std_msgs"
  "irt_msgs"
  "sensor_msgs"
  "novatel_oem7_msgs"
)

install(TARGETS ntrip_ros_node
  DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(reference_station_tracker)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME} FILES_MATCHING PATTERN "*.yaml")
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME} FILES_MATCHING PATTERN "*.py")
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME} FILES_MATCHING PATTERN "*.launch")

install(PROGRAMS
        scripts/ref_station_tracker_node.py
        DESTINATION lib/${PROJECT_NAME}
        )

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
