cmake_minimum_required(VERSION 3.8)
project(lidar_perception_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 의존성 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED) # Eigen3 찾기

# 실행 파일 생성
add_executable(iterative_ransac src/iterative_ransac.cpp)

# 헤더 및 라이브러리 연결
target_include_directories(iterative_ransac PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(iterative_ransac
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
)

# c++ 실행파일설치 규칙
install(TARGETS
  iterative_ransac
  DESTINATION lib/${PROJECT_NAME}
)
# 파이썬 노드 설치 규칙                                     TO DO: src로 옮기기, Lib에 있는 모듈 파일까지
install(PROGRAMS 
  src/lidar_obstacle_detector_node.py
  src/lidar_processor_node.py
  src/lidar_publisher_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()