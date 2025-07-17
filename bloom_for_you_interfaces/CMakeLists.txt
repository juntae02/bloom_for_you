cmake_minimum_required(VERSION 3.8)
project(bloom_for_you_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# rosidl_default_generators : 코드 생성기( .msg, .srv, .action을 각 언어별로 코드로 자동 생성해주는 빌드 도구)
#                             .msg, .srv, .action 파일을 읽어서 C++, PYTHON 등 언어 타입지원 코드를 생성시키고,
#                             colcon build 시에 msg 파일을 코드로 변환하는 역할을 하며 자동 실행됨
# builtin_interfaces : ros2 인터페이스 메시지에서 공통으로 사용되는 기본타입을 정의한 패키지
#                      Time, Duration, String, Bool, Int32, Float54 등등

set(msg_files
  "msg/Command.msg"
  "msg/FlowerInfo.msg"
  "msg/GripTarget.msg"
)

set(srv_files
  "srv/CardSrv.srv"
  "srv/SrvDepthPosition.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces
)

ament_package()
