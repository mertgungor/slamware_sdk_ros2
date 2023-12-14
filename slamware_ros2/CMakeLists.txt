cmake_minimum_required(VERSION 3.5)

project(slamware_ros2)

find_path(slamware_sdk_INCLUDE_DIR rpos/rpos.h ${PROJECT_SOURCE_DIR}/../slamware_sdk/include)
find_path(slamware_sdk_INCLUDE_DIR rpos/rpos.h ${PROJECT_SOURCE_DIR}/../slamware_sdk/include)
find_path(slamware_sdk_LIBRARY librpos_framework.a ${PROJECT_SOURCE_DIR}/../slamware_sdk/lib)

set(SLTC_SDK_INC_DIR "${slamware_sdk_INCLUDE_DIR}")
set(SLTC_SDK_LIB_DIR "${slamware_sdk_LIBRARY}")

# set(SLTC_SDK_ROOT "/home/mert/Downloads/slamware_sdk_linux-x86_64-gcc9")
# set(SLTC_SDK_DIR "${SLTC_SDK_ROOT}/linux-x86_64-release")
# set(SLTC_SDK_INC_DIR "${SLTC_SDK_DIR}/include")
# set(SLTC_SDK_LIB_DIR "${SLTC_SDK_DIR}/lib")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -std=gnu++11")

find_package(nav_msgs    REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs    REQUIRED)
find_package(tf2         REQUIRED)
find_package(rclcpp      REQUIRED)
find_package(tf2_ros     REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(Threads)
find_package(rosidl_default_generators REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)



if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


message(${SLTC_SDK_LIB_DIR})

add_executable(slamware_node src/slamware_node.cpp src/server_map_holder.cpp include/slamware_ros2/server_map_holder.hpp)
ament_target_dependencies(slamware_node rclcpp std_msgs nav_msgs sensor_msgs tf2 tf2_ros)
target_include_directories(slamware_node PRIVATE ${SLTC_SDK_INC_DIR})
target_compile_options(slamware_node PRIVATE -Wno-deprecated-declarations)
target_link_libraries(slamware_node                                    
                                   
  ${SLTC_SDK_LIB_DIR}/librpos_robotplatforms_rpslamware.a
  ${SLTC_SDK_LIB_DIR}/librpos_framework.a
  ${SLTC_SDK_LIB_DIR}/libbase64.a
  ${SLTC_SDK_LIB_DIR}/librlelib.a
  ${SLTC_SDK_LIB_DIR}/libjsoncpp.a
  ${SLTC_SDK_LIB_DIR}/libcurl.a
  ${SLTC_SDK_LIB_DIR}/libcares.a
  ${SLTC_SDK_LIB_DIR}/libssl.a
  ${SLTC_SDK_LIB_DIR}/libcrypto.a
  ${SLTC_SDK_LIB_DIR}/libboost_atomic.a
  ${SLTC_SDK_LIB_DIR}/libboost_chrono.a
  ${SLTC_SDK_LIB_DIR}/libboost_date_time.a
  ${SLTC_SDK_LIB_DIR}/libboost_regex.a 
  ${SLTC_SDK_LIB_DIR}/libboost_filesystem.a
  ${SLTC_SDK_LIB_DIR}/libboost_system.a
  ${SLTC_SDK_LIB_DIR}/libboost_thread.a
  ${SLTC_SDK_LIB_DIR}/libboost_random.a
  ${SLTC_SDK_LIB_DIR}/libz.a                   
  ${catkin_LIBRARIES}
  pthread
  dl
  rt
  )

  install(TARGETS
  slamware_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
