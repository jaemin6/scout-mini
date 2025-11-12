cmake_minimum_required(VERSION 3.8)
project(scout_robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
# -------------------------------------------------------------
# 액션 인터페이스 정의 및 빌드
# -------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavigateRoom.action"
  "action/RotateRobot.action"
  DEPENDENCIES std_msgs action_msgs
)

# -------------------------------------------------------------
# 설치 설정 (Install)
# -------------------------------------------------------------
install(
  DIRECTORY action
  DESTINATION share/${PROJECT_NAME}
)

# -------------------------------------------------------------
# 패키지 내보내기 (Export)
# -------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
