cmake_minimum_required(VERSION 3.16)
project(refuel_demo_skeleton LANGUAGES CXX)

# 这个工程是“接口骨架”，用于把流程图里的每个环节拆成独立模块。
# 你只需要在 src/stages/*.cpp 中把 TODO 部分替换为你的具体实现即可。

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(REFUEL_BUILD_TESTS "Build unit/integration tests" ON)

add_executable(refuel_demo
    app/main.cpp
    src/pipeline/pipeline.cpp
    src/io/demo_io.cpp
    src/io/output_writer.cpp
    src/stages/coordinate_transform_stage.cpp
    src/stages/preprocess_stage.cpp
    src/stages/safe_zone_select_stage.cpp
    src/stages/racetrack_build_stage.cpp
    src/stages/sequence_stage.cpp
    src/stages/tanker_racetrack_decision_stage.cpp
    src/stages/rendezvous_stage.cpp
    src/stages/path_planning_stage.cpp
    src/stages/cost_stage.cpp
)

target_include_directories(refuel_demo PRIVATE include)

if(REFUEL_BUILD_TESTS)
  enable_testing()

  add_executable(refuel_mode0_tests
      tests/mode0_tests.cpp
  )
  target_include_directories(refuel_mode0_tests PRIVATE include .)

  add_test(NAME refuel_mode0_tests COMMAND refuel_mode0_tests)
endif()
