cmake_minimum_required(VERSION 3.16)
project(drone_companion CXX)

set(DC_PROJECT_CONFIG
  "${CMAKE_CURRENT_SOURCE_DIR}/config/project.cmake"
  CACHE FILEPATH "Project build-profile file")
if(NOT EXISTS "${DC_PROJECT_CONFIG}")
  message(FATAL_ERROR "Project build profile not found: ${DC_PROJECT_CONFIG}")
endif()
include("${DC_PROJECT_CONFIG}")

set(_DC_HAILORT_MODES AUTO ON OFF)
if(NOT DC_HAILORT_MODE IN_LIST _DC_HAILORT_MODES)
  message(FATAL_ERROR "DC_HAILORT_MODE must be AUTO, ON, or OFF")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Threads REQUIRED)
if(DC_BUILD_VIDEO OR DC_BUILD_MQTT_TRANSPORT)
find_package(PkgConfig REQUIRED)
endif()
if(DC_BUILD_VIDEO)
pkg_check_modules(GST REQUIRED IMPORTED_TARGET
  gstreamer-1.0
  gstreamer-app-1.0
  gstreamer-video-1.0)
pkg_check_modules(CAIRO REQUIRED IMPORTED_TARGET cairo)

# HailoRT 는 선택 의존성이다. Hailo 미장착 기체에서도 빌드되어야 하므로,
# 못 찾으면 검출 없는(통과 전용) 실행 파일을 만든다.
# pkg-config 파일을 설치하지 않으므로 deb 가 놓는 경로를 직접 찾는다.
if(NOT DC_HAILORT_MODE STREQUAL "OFF")
find_path(HAILORT_INCLUDE_DIR hailo/hailort.hpp
  HINTS /usr/include /usr/local/include)
find_library(HAILORT_LIBRARY NAMES hailort
  HINTS /usr/lib /usr/local/lib /usr/lib/aarch64-linux-gnu)
endif()

set(DC_SOURCES
  src/main.cpp
  src/config.cpp
  src/detection.cpp
  src/hailo_probe.cpp
  src/pipeline.cpp
  src/video_pipeline.cpp)

if(HAILORT_INCLUDE_DIR AND HAILORT_LIBRARY)
  message(STATUS "HailoRT 발견: ${HAILORT_LIBRARY} — 검출 기능 포함")
  list(APPEND DC_SOURCES src/hailo_inference.cpp)
  set(DC_WITH_HAILO ON)
elseif(DC_HAILORT_MODE STREQUAL "ON")
  message(FATAL_ERROR "DC_HAILORT_MODE=ON but HailoRT was not found")
else()
  if(DC_HAILORT_MODE STREQUAL "OFF")
    message(STATUS "HailoRT compile support disabled by project profile")
  else()
    message(WARNING
      "libhailort 를 찾지 못했습니다 — 검출 없이 영상 송출만 하는 바이너리를 만듭니다. "
      "hailort deb 설치 후 재빌드하면 검출이 켜집니다.")
  endif()
  list(APPEND DC_SOURCES src/hailo_inference_stub.cpp)
  set(DC_WITH_HAILO OFF)
endif()

add_executable(drone_companion ${DC_SOURCES})

target_include_directories(drone_companion PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(drone_companion PRIVATE
  PkgConfig::GST
  PkgConfig::CAIRO
  Threads::Threads)

if(DC_WITH_HAILO)
  target_include_directories(drone_companion PRIVATE ${HAILORT_INCLUDE_DIR})
  target_link_libraries(drone_companion PRIVATE ${HAILORT_LIBRARY})
  target_compile_definitions(drone_companion PRIVATE DC_WITH_HAILO=1)
endif()

target_compile_options(drone_companion PRIVATE -Wall -Wextra -Wno-unused-parameter)

install(TARGETS drone_companion RUNTIME DESTINATION bin)
endif()

# ---------------------------------------------------------------------------
# 비행 서브시스템 (Riposte) — PX4/MAVSDK 기반 시험비행·미션 실행
#
# 영상과 마찬가지로 MAVSDK 도 선택 의존성이다. MAVSDK 가 없는 장비(개발 PC,
# 영상 전용 기체)에서도 저장소 전체가 빌드되어야 한다.
#
# 기하·판정 로직은 헤더 온리이고 MAVSDK 의존이 없다. 그래서 단위시험은
# MAVSDK 유무와 무관하게 항상 빌드·실행한다 — 하드웨어 없이 회귀를 잡는 유일한 수단이다.
# ---------------------------------------------------------------------------
set(RIPOSTE_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/include/riposte
  ${CMAKE_CURRENT_SOURCE_DIR}/tests/riposte)

# 원본 프로젝트의 엄격 경고 정책을 그대로 유지한다. 비행 코드의 암시적 형 변환은
# 실제로 위험하므로 완화하지 않는다.
set(RIPOSTE_WARNING_FLAGS -Wall -Wextra -Wpedantic -Wconversion -Werror)

if(DC_BUILD_FLIGHT_CORE)
  add_library(dc_flight_core INTERFACE)
  target_compile_features(dc_flight_core INTERFACE cxx_std_17)
  target_include_directories(dc_flight_core INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/riposte)

  add_library(dc_remote_mission STATIC
    src/flight/remote_mission_coordinator.cpp)
  target_compile_options(dc_remote_mission PRIVATE ${RIPOSTE_WARNING_FLAGS})
  target_link_libraries(dc_remote_mission PUBLIC dc_flight_core)

  add_library(dc_preemptive_executor STATIC
    src/flight/preemptive_mission_executor.cpp)
  target_compile_options(dc_preemptive_executor PRIVATE ${RIPOSTE_WARNING_FLAGS})
  target_link_libraries(dc_preemptive_executor PUBLIC
    dc_flight_core Threads::Threads)
endif()

if((DC_BUILD_RIPOSTE_AGENT OR DC_BUILD_RIPOSTE_GCS
    OR DC_BUILD_FLIGHT_CORE_TESTS OR DC_BUILD_REMOTE_COMMAND_TESTS
    OR DC_BUILD_RIPOSTE_TESTS OR DC_BUILD_MQTT_CONNECTOR
    OR DC_BUILD_MQTT_CONNECTOR_TESTS OR DC_BUILD_MQTT_PERSISTENCE
    OR DC_BUILD_MQTT_PERSISTENCE_TESTS OR DC_BUILD_MQTT_TRANSPORT
    OR DC_BUILD_MQTT_PX4_AGENT)
   AND NOT DC_BUILD_FLIGHT_CORE)
  message(FATAL_ERROR "A flight target is enabled while DC_BUILD_FLIGHT_CORE=OFF")
endif()

if(DC_BUILD_MQTT_CONNECTOR)
  find_package(nlohmann_json 3.10 REQUIRED)

  add_library(dc_mqtt_codec STATIC
    src/mqtt/mission_codec.cpp)
  target_compile_options(dc_mqtt_codec PRIVATE ${RIPOSTE_WARNING_FLAGS})
  target_link_libraries(dc_mqtt_codec PUBLIC
    dc_flight_core nlohmann_json::nlohmann_json)

  # aegisq_state_codec.cpp belongs here rather than in dc_mqtt_codec: it needs
  # valid_vehicle_id() from this target, and folding it downward would invert
  # the documented codec -> command-source direction.
  add_library(dc_mqtt_command_source STATIC
    src/mqtt/mqtt_command_source.cpp
    src/mqtt/aegisq_state_codec.cpp)
  target_compile_options(dc_mqtt_command_source PRIVATE ${RIPOSTE_WARNING_FLAGS})
  target_link_libraries(dc_mqtt_command_source PUBLIC
    dc_mqtt_codec Threads::Threads)
endif()

if(DC_BUILD_MQTT_PERSISTENCE)
  if(NOT DC_BUILD_MQTT_CONNECTOR)
    message(FATAL_ERROR "DC_BUILD_MQTT_PERSISTENCE requires DC_BUILD_MQTT_CONNECTOR=ON")
  endif()
  find_package(SQLite3 REQUIRED)

  add_library(dc_mqtt_persistence STATIC
    src/mqtt/sqlite_command_ledger.cpp)
  target_compile_options(dc_mqtt_persistence PRIVATE ${RIPOSTE_WARNING_FLAGS})
  target_link_libraries(dc_mqtt_persistence PUBLIC
    dc_mqtt_command_source SQLite::SQLite3 Threads::Threads)
endif()

if(DC_BUILD_MQTT_TRANSPORT)
  if(NOT DC_BUILD_MQTT_CONNECTOR)
    message(FATAL_ERROR "DC_BUILD_MQTT_TRANSPORT requires DC_BUILD_MQTT_CONNECTOR=ON")
  endif()
  if(NOT DC_BUILD_MQTT_PERSISTENCE)
    message(FATAL_ERROR "DC_BUILD_MQTT_TRANSPORT requires DC_BUILD_MQTT_PERSISTENCE=ON")
  endif()
  if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(FATAL_ERROR "The WireGuard/libmosquitto adapter is Linux-only")
  endif()
  pkg_check_modules(MOSQUITTO REQUIRED IMPORTED_TARGET libmosquitto)

  add_library(dc_mqtt_transport STATIC
    src/mqtt/mosquitto_session.cpp
    src/mqtt/agent_config.cpp)
  target_compile_options(dc_mqtt_transport PRIVATE ${RIPOSTE_WARNING_FLAGS})
  target_link_libraries(dc_mqtt_transport PUBLIC
    dc_mqtt_persistence PkgConfig::MOSQUITTO Threads::Threads)
endif()

# The agent executable is defined further down, after the MAVSDK block it needs.
# Durable inbox/outbox/UUID deduplication, priority STOP/RTL/E-STOP, terminal
# status reporting and the shared MAVSDK executor adapter now exist, so the
# build no longer refuses outright. SITL and vehicle verification are NOT done:
# the profile keeps DC_BUILD_MQTT_PX4_AGENT=OFF, REMOTE_FLIGHT_EXECUTE stays 0,
# and the service is neither installed nor enabled. Building this target is not
# a statement that the vehicle may fly it.
if(DC_BUILD_MQTT_PX4_AGENT AND NOT DC_BUILD_MQTT_TRANSPORT)
  message(FATAL_ERROR
    "DC_BUILD_MQTT_PX4_AGENT requires DC_BUILD_MQTT_TRANSPORT=ON")
endif()

# 3.17.1 은 고정 버전이 아니라 최소 버전이다 — find_package 는 "이 버전 이상"을 요구한다.
# 이 값이 바닥인 이유는 v3 에서 API 의미가 바뀐 것들에 코드가 의존하기 때문이다
# (remaining_percent 가 0~100 퍼센트, CompanionComputer 기본 컴포넌트 ID 195 등,
#  docs/riposte/GCS_USAGE.md §9 참고). 그 아래로는 조용히 오동작한다.
# 더 높은 3.x 는 그대로 쓰면 된다. 메이저가 올라가면(4.x) 위 의미들을 다시 확인할 것.
set(RIPOSTE_APPS)
if(DC_BUILD_RIPOSTE_AGENT)
  list(APPEND RIPOSTE_APPS riposte_test)
endif()
if(DC_BUILD_RIPOSTE_GCS)
  list(APPEND RIPOSTE_APPS riposte_gcs)
endif()

if(RIPOSTE_APPS OR DC_BUILD_MQTT_PX4_AGENT)
  find_package(MAVSDK 3.17.1 REQUIRED)
  message(STATUS "MAVSDK ${MAVSDK_VERSION} 발견 — 비행 앱 포함")
endif()

# MAVSDK 를 실제로 호출하는 비행 역학은 여기 한 곳에만 있다. riposte_test 와
# MQTT 에이전트가 같은 코드를 쓰게 하려는 것이고, 이 라이브러리가 없으면
# test_main.cpp 는 MavsdkOffboardControl 멤버 때문에 링크되지 않는다.
if(DC_BUILD_RIPOSTE_AGENT OR DC_BUILD_MQTT_PX4_AGENT)
  add_library(dc_mavsdk_flight STATIC
    src/riposte/mavsdk_rc_ownership_monitor.cpp
    src/riposte/mavsdk_offboard_control.cpp
    src/riposte/mavsdk_mission_backend.cpp)
  target_include_directories(dc_mavsdk_flight PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/riposte)
  target_compile_options(dc_mavsdk_flight PRIVATE ${RIPOSTE_WARNING_FLAGS})
  target_link_libraries(dc_mavsdk_flight PUBLIC
    dc_flight_core MAVSDK::mavsdk Threads::Threads)
  foreach(plugin MAVSDK::mavsdk_action MAVSDK::mavsdk_offboard
                 MAVSDK::mavsdk_telemetry MAVSDK::mavsdk_mavlink_passthrough)
    if(TARGET ${plugin})
      target_link_libraries(dc_mavsdk_flight PUBLIC ${plugin})
    endif()
  endforeach()
endif()

if(RIPOSTE_APPS)
  foreach(app IN LISTS RIPOSTE_APPS)
    string(REPLACE "riposte_" "" src_name ${app})
    add_executable(${app} src/riposte/${src_name}_main.cpp)
    target_include_directories(${app} PRIVATE ${RIPOSTE_INCLUDE_DIRS})
    target_compile_options(${app} PRIVATE ${RIPOSTE_WARNING_FLAGS})
    target_link_libraries(${app} PRIVATE
      dc_flight_core MAVSDK::mavsdk Threads::Threads)
    # MAVSDK 2/3 일부 패키지는 플러그인별 imported target 을 따로 제공한다.
    foreach(plugin MAVSDK::mavsdk_action MAVSDK::mavsdk_offboard
                   MAVSDK::mavsdk_telemetry MAVSDK::mavsdk_mavlink_passthrough)
      if(TARGET ${plugin})
        target_link_libraries(${app} PRIVATE ${plugin})
      endif()
    endforeach()
    install(TARGETS ${app} RUNTIME DESTINATION bin)
  endforeach()
endif()

if(DC_BUILD_RIPOSTE_AGENT)
  target_link_libraries(riposte_test PRIVATE dc_mavsdk_flight)
endif()

if(DC_BUILD_MQTT_PX4_AGENT)
  add_executable(mqtt_px4_control src/mqtt/mqtt_px4_control_main.cpp)
  target_compile_options(mqtt_px4_control PRIVATE ${RIPOSTE_WARNING_FLAGS})
  # include/ 와 include/riposte/ 는 dc_flight_core, src/riposte/ 는
  # dc_mavsdk_flight 가 PUBLIC 으로 넘겨준다. tests/riposte 는 넣지 않는다.
  target_link_libraries(mqtt_px4_control PRIVATE
    dc_mqtt_transport
    dc_remote_mission
    dc_preemptive_executor
    dc_mavsdk_flight
    Threads::Threads)
  install(TARGETS mqtt_px4_control RUNTIME DESTINATION bin)
endif()

set(FLIGHT_TEST_SUITES)
if(DC_BUILD_FLIGHT_CORE_TESTS)
  list(APPEND FLIGHT_TEST_SUITES bullet_path geo test_trajectory)
endif()
if(DC_BUILD_RIPOSTE_TESTS)
  list(APPEND FLIGHT_TEST_SUITES gcs_protocol verdict)
endif()

if(FLIGHT_TEST_SUITES OR DC_BUILD_VIDEO_TESTS OR DC_BUILD_REMOTE_COMMAND_TESTS
   OR DC_BUILD_MQTT_CONNECTOR_TESTS OR DC_BUILD_MQTT_PERSISTENCE_TESTS)
  enable_testing()
  foreach(suite IN LISTS FLIGHT_TEST_SUITES)
    add_executable(${suite}_tests tests/riposte/test_${suite}.cpp)
    target_include_directories(${suite}_tests PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/riposte)
    target_compile_options(${suite}_tests PRIVATE ${RIPOSTE_WARNING_FLAGS})
    target_link_libraries(${suite}_tests PRIVATE dc_flight_core)
    add_test(NAME ${suite}_tests COMMAND ${suite}_tests)
  endforeach()

  if(DC_BUILD_VIDEO_TESTS)
    add_executable(video_pipeline_tests
      tests/video/test_video_pipeline.cpp
      src/config.cpp
      src/video_pipeline.cpp)
    target_include_directories(video_pipeline_tests PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/riposte)
    target_compile_options(video_pipeline_tests PRIVATE
      -Wall -Wextra -Wpedantic -Werror)
    add_test(NAME video_pipeline_tests COMMAND video_pipeline_tests)
  endif()

  if(DC_BUILD_REMOTE_COMMAND_TESTS)
    add_executable(rc_ownership_tests tests/flight/test_rc_ownership.cpp)
    target_include_directories(rc_ownership_tests PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/riposte)
    target_compile_options(rc_ownership_tests PRIVATE ${RIPOSTE_WARNING_FLAGS})
    target_link_libraries(rc_ownership_tests PRIVATE dc_flight_core)
    add_test(NAME rc_ownership_tests COMMAND rc_ownership_tests)

    add_executable(remote_command_tests tests/flight/test_remote_command.cpp)
    target_include_directories(remote_command_tests PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/riposte)
    target_compile_options(remote_command_tests PRIVATE ${RIPOSTE_WARNING_FLAGS})
    target_link_libraries(remote_command_tests PRIVATE dc_flight_core)
    add_test(NAME remote_command_tests COMMAND remote_command_tests)

    add_executable(remote_mission_coordinator_tests
      tests/flight/test_remote_mission_coordinator.cpp)
    target_include_directories(remote_mission_coordinator_tests PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/riposte)
    target_compile_options(remote_mission_coordinator_tests PRIVATE
      ${RIPOSTE_WARNING_FLAGS})
    target_link_libraries(remote_mission_coordinator_tests PRIVATE dc_remote_mission)
    add_test(NAME remote_mission_coordinator_tests
      COMMAND remote_mission_coordinator_tests)

    add_executable(preemptive_mission_executor_tests
      tests/flight/test_preemptive_mission_executor.cpp)
    target_include_directories(preemptive_mission_executor_tests PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/riposte)
    target_compile_options(preemptive_mission_executor_tests PRIVATE
      ${RIPOSTE_WARNING_FLAGS})
    target_link_libraries(preemptive_mission_executor_tests PRIVATE
      dc_preemptive_executor)
    add_test(NAME preemptive_mission_executor_tests
      COMMAND preemptive_mission_executor_tests)
  endif()

  if(DC_BUILD_MQTT_CONNECTOR_TESTS)
    if(NOT DC_BUILD_MQTT_CONNECTOR)
      message(FATAL_ERROR
        "DC_BUILD_MQTT_CONNECTOR_TESTS requires DC_BUILD_MQTT_CONNECTOR=ON")
    endif()

    add_executable(mqtt_mission_codec_tests
      tests/mqtt/test_mission_codec.cpp)
    target_include_directories(mqtt_mission_codec_tests PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/mqtt
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/riposte)
    target_compile_options(mqtt_mission_codec_tests PRIVATE
      ${RIPOSTE_WARNING_FLAGS})
    target_link_libraries(mqtt_mission_codec_tests PRIVATE dc_mqtt_codec)
    add_test(NAME mqtt_mission_codec_tests COMMAND mqtt_mission_codec_tests)

    add_executable(mqtt_command_source_tests
      tests/mqtt/test_mqtt_command_source.cpp)
    target_include_directories(mqtt_command_source_tests PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/mqtt
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/riposte)
    target_compile_options(mqtt_command_source_tests PRIVATE
      ${RIPOSTE_WARNING_FLAGS})
    target_link_libraries(mqtt_command_source_tests PRIVATE
      dc_mqtt_command_source)
    add_test(NAME mqtt_command_source_tests COMMAND mqtt_command_source_tests)

    add_executable(mqtt_state_codec_tests
      tests/mqtt/test_aegisq_state_codec.cpp)
    target_include_directories(mqtt_state_codec_tests PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/mqtt
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/riposte)
    target_compile_options(mqtt_state_codec_tests PRIVATE
      ${RIPOSTE_WARNING_FLAGS})
    target_link_libraries(mqtt_state_codec_tests PRIVATE
      dc_mqtt_command_source)
    add_test(NAME mqtt_state_codec_tests COMMAND mqtt_state_codec_tests)

    # The agent configuration lives in dc_mqtt_transport, which is Linux-only.
    if(DC_BUILD_MQTT_TRANSPORT)
      add_executable(mqtt_agent_config_tests tests/mqtt/test_agent_config.cpp)
      target_include_directories(mqtt_agent_config_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/mqtt
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/riposte)
      target_compile_options(mqtt_agent_config_tests PRIVATE
        ${RIPOSTE_WARNING_FLAGS})
      target_link_libraries(mqtt_agent_config_tests PRIVATE dc_mqtt_transport)
      add_test(NAME mqtt_agent_config_tests COMMAND mqtt_agent_config_tests)
    endif()
  endif()

  if(DC_BUILD_MQTT_PERSISTENCE_TESTS)
    if(NOT DC_BUILD_MQTT_PERSISTENCE)
      message(FATAL_ERROR
        "DC_BUILD_MQTT_PERSISTENCE_TESTS requires DC_BUILD_MQTT_PERSISTENCE=ON")
    endif()

    add_executable(mqtt_command_ledger_tests
      tests/mqtt/test_sqlite_command_ledger.cpp)
    target_include_directories(mqtt_command_ledger_tests PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/mqtt
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/riposte)
    target_compile_options(mqtt_command_ledger_tests PRIVATE
      ${RIPOSTE_WARNING_FLAGS})
    target_link_libraries(mqtt_command_ledger_tests PRIVATE
      dc_mqtt_persistence)
    add_test(NAME mqtt_command_ledger_tests COMMAND mqtt_command_ledger_tests)
  endif()
endif()

message(STATUS "Project profile: ${DC_PROJECT_VARIANT}")
message(STATUS
  "Targets: video=${DC_BUILD_VIDEO}, video-tests=${DC_BUILD_VIDEO_TESTS}, "
  "flight-core=${DC_BUILD_FLIGHT_CORE}, "
  "riposte-agent=${DC_BUILD_RIPOSTE_AGENT}, riposte-gcs=${DC_BUILD_RIPOSTE_GCS}, "
  "mqtt-connector=${DC_BUILD_MQTT_CONNECTOR}, "
  "mqtt-persistence=${DC_BUILD_MQTT_PERSISTENCE}, "
  "mqtt-transport=${DC_BUILD_MQTT_TRANSPORT}, "
  "mqtt-px4-agent=${DC_BUILD_MQTT_PX4_AGENT}")
