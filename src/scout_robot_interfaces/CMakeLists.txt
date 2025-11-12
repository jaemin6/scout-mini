cmake_minimum_required(VERSION 3.8)
project(scout_robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# 🌟🌟🌟 액션 메시지 빌드에 필요한 패키지 추가 🌟🌟🌟
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_adapter REQUIRED)

# -------------------------------------------------------------
# 🌟🌟🌟 액션 인터페이스 정의 및 빌드 🌟🌟🌟
# -------------------------------------------------------------
rosidl_generate_action_interfaces(
  # 여기에 모든 .action 파일을 나열합니다.
  action/NavigateRoom.action
  action/RotateRobot.action
  # DEPENDENCIES는 action 파일 내에서 사용하는 메시지 타입입니다.
  DEPENDENCIES std_msgs action_msgs
)

# -------------------------------------------------------------
# 🌟🌟🌟 설치 설정 🌟🌟🌟
# -------------------------------------------------------------

# 빌드된 액션 인터페이스를 ROS 환경에 설치합니다.
install(
  PROGRAMS
    # 빌드된 파이썬 모듈이 있다면 여기에 추가
  DESTINATION lib/${PROJECT_NAME}
)

# .action 파일이 담긴 'action' 폴더를 설치합니다. (다른 패키지에서 참조 가능하게)
install(
  DIRECTORY action
  DESTINATION share/${PROJECT_NAME}
)

# 기존 설치 설정 유지
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY src/
  DESTINATION lib/${PROJECT_NAME}
)

# -------------------------------------------------------------
# 🌟🌟🌟 패키지 내보내기 🌟🌟🌟
# -------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# 🌟🌟🌟 rosidl_default_generators의 런타임 종속성을 내보냅니다. 🌟🌟🌟
# 이 부분이 있어야 파이썬에서 액션을 import 할 수 있습니다.
ament_export_dependencies(rosidl_default_generators)