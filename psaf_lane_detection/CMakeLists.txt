cmake_minimum_required(VERSION 3.5)
project(psaf_lane_detection)

set(NODE_NAME "lane_detection")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(libpsaf REQUIRED)
find_package(libpsaf_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(psaf_configuration REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(${NODE_NAME}
        src/${NODE_NAME}.cpp
        src/${NODE_NAME}_node.cpp)

target_include_directories(${NODE_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${OpenCV_INCLUDE_DIRS})

target_link_libraries(${NODE_NAME} ${OpenCV_LIBS})

ament_target_dependencies(${NODE_NAME} rclcpp std_msgs libpsaf OpenCV psaf_configuration)

install(TARGETS
        ${NODE_NAME}
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
        launch
        config
        DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(rosbag2_cpp REQUIRED)



  # ament_add_gtest(lane_detection_unit_tests test/unit_tests.cpp src/lane_detection_node.cpp)
  ament_add_gmock(lane_detection_unit_tests test/unit_tests.cpp src/lane_detection_node.cpp)
  target_include_directories(lane_detection_unit_tests PRIVATE "include")
  ament_target_dependencies(lane_detection_unit_tests OpenCV rclcpp libpsaf libpsaf_msgs psaf_configuration)
  target_link_libraries(lane_detection_unit_tests ${OpenCV_LIBS})

  ament_add_gmock(lane_detection_integration_tests test/integration_tests.cpp src/lane_detection_node.cpp)
  target_include_directories(lane_detection_integration_tests PRIVATE "include")
  ament_target_dependencies(lane_detection_integration_tests OpenCV cv_bridge rclcpp libpsaf std_msgs
          sensor_msgs libpsaf_msgs psaf_configuration)
  target_link_libraries(lane_detection_integration_tests ${OpenCV_LIBS})

  ament_add_gmock(lane_detection_simulation_tests test/simulation_tests.cpp src/lane_detection_node.cpp)
  target_include_directories(lane_detection_simulation_tests PRIVATE "include")
  ament_target_dependencies(lane_detection_simulation_tests OpenCV cv_bridge rclcpp rosbag2_cpp
          libpsaf std_msgs sensor_msgs libpsaf_msgs psaf_configuration)
  target_link_libraries(lane_detection_simulation_tests ${OpenCV_LIBS})


  set_tests_properties(lane_detection_unit_tests PROPERTIES
          ENVIRONMENT
          "TEST_DATA_DIR=${CMAKE_CURRENT_SOURCE_DIR}/test/resources")

  set_tests_properties(lane_detection_integration_tests PROPERTIES
          ENVIRONMENT
          "TEST_DATA_DIR=${CMAKE_CURRENT_SOURCE_DIR}/test/resources")

  set_tests_properties(lane_detection_integration_tests PROPERTIES TIMEOUT 600)

  set_tests_properties(lane_detection_simulation_tests PROPERTIES
          ENVIRONMENT
          "TEST_DATA_DIR=${CMAKE_CURRENT_SOURCE_DIR}/test/resources")

  set_tests_properties(lane_detection_simulation_tests PROPERTIES TIMEOUT 600)


  install(TARGETS
          lane_detection_unit_tests
          lane_detection_integration_tests
          lane_detection_simulation_tests
          DESTINATION lib/${PROJECT_NAME})

endif()

ament_package()
