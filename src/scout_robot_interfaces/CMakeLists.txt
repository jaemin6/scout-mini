cmake_minimum_required(VERSION 3.8)
project(scout_robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# 🌟 액션 빌드 도구 찾기 (이 명령어들이 rosidl_generate_action_interfaces를 정의합니다)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_adapter REQUIRED)

# -------------------------------------------------------------
# 액션 인터페이스 정의 및 빌드
# -------------------------------------------------------------
rosidl_generate_action_interfaces(
  action/NavigateRoom.action
  action/RotateRobot.action
  # 액션 파일 내에서 사용된 기본 메시지 종속성
  DEPENDENCIES std_msgs action_msgs
)

# -------------------------------------------------------------
# 설치 설정 (Install)
# -------------------------------------------------------------

# .action 파일이 담긴 'action' 폴더를 설치합니다.
install(
  DIRECTORY action
  DESTINATION share/${PROJECT_NAME}
)

# -------------------------------------------------------------
# 패키지 내보내기 (Export)
# -------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # ... (테스트 관련 코드)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 🌟🌟🌟 AMENT_PACKAGE() 호출 전에 종속성을 내보냅니다. (이전 오류 수정 부분) 🌟🌟🌟
ament_export_dependencies(rosidl_default_generators)

# 🌟 모든 설정 완료 후 AMENT_PACKAGE() 호출
ament_package()