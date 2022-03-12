cmake_minimum_required(VERSION 3.8)
project(latency)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

include_directories(include)

set(msg_files
  "msg/BinaryData.msg"
  "msg/LatencySample.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

ament_export_dependencies(rosidl_default_runtime)

# add_executable(binary_source src/binary_source.cpp)
# target_include_directories(binary_source PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(binary_source PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# 
# install(TARGETS binary_source
#   DESTINATION lib/${PROJECT_NAME})

add_library(binary_source SHARED
  src/binary_source.cpp)
target_compile_definitions(binary_source
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(binary_source
  "rclcpp"
  "rclcpp_components"
)
rosidl_target_interfaces(binary_source
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rclcpp_components_register_nodes(binary_source "latency::BinarySource")
set(node_plugins "${node_plugins}latency::BinarySource;$<TARGET_FILE:binary_source>\n")

add_library(receiver SHARED
  src/receiver.cpp)
target_compile_definitions(receiver
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(receiver
  "rclcpp"
  "rclcpp_components"
)
rosidl_target_interfaces(receiver
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rclcpp_components_register_nodes(receiver "latency::Receiver")
set(node_plugins "${node_plugins}latency::Receiver;$<TARGET_FILE:receiver>\n")

add_library(csv_sink SHARED
  src/csv_sink.cpp)
target_compile_definitions(csv_sink
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(csv_sink
  "rclcpp"
  "rclcpp_components"
)
rosidl_target_interfaces(csv_sink
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rclcpp_components_register_nodes(csv_sink "latency::CSVSink")
set(node_plugins "${node_plugins}latency::CSVSink;$<TARGET_FILE:csv_sink>\n")

add_library(map SHARED
  src/map.cpp)
target_compile_definitions(map
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(map
  "rclcpp"
  "rclcpp_components"
)
rosidl_target_interfaces(map
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rclcpp_components_register_nodes(map "latency::Map")
set(node_plugins "${node_plugins}latency::Map;$<TARGET_FILE:map>\n")

add_library(join SHARED
  src/join.cpp)
target_compile_definitions(join
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(join
  "rclcpp"
  "rclcpp_components"
)
rosidl_target_interfaces(join
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rclcpp_components_register_nodes(map "latency::Join")
set(node_plugins "${node_plugins}latency::Join;$<TARGET_FILE:join>\n")

add_executable(broadcast
  src/broadcast.cpp)
target_link_libraries(broadcast
  binary_source
  receiver
  csv_sink)
ament_target_dependencies(broadcast
  "rclcpp")

add_executable(binary_source_driver
  src/binary_source_driver.cpp)
target_link_libraries(binary_source_driver
  binary_source)
ament_target_dependencies(binary_source_driver
  "rclcpp")

add_executable(receiver_driver
  src/receiver_driver.cpp)
target_link_libraries(receiver_driver
  binary_source
  receiver
  csv_sink)
ament_target_dependencies(receiver_driver
  "rclcpp")

add_executable(csv_sink_driver
  src/csv_sink_driver.cpp)
target_link_libraries(csv_sink_driver
  binary_source
  receiver
  csv_sink)
ament_target_dependencies(csv_sink_driver
  "rclcpp")

add_executable(map_driver
  src/map_driver.cpp)
target_link_libraries(map_driver
  map)
ament_target_dependencies(map_driver
  "rclcpp")

add_executable(join_two_driver
  src/join_two_driver.cpp)
target_link_libraries(join_two_driver
  join)
ament_target_dependencies(join_two_driver
  "rclcpp")

add_executable(join_three_driver
  src/join_three_driver.cpp)
target_link_libraries(join_three_driver
  join)
ament_target_dependencies(join_three_driver
  "rclcpp")

install(TARGETS
  binary_source
  receiver
  csv_sink
  map
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  broadcast
  binary_source_driver
  receiver_driver
  csv_sink_driver
  map_driver
  join_two_driver
  join_three_driver
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
