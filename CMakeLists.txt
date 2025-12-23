cmake_minimum_required(VERSION 3.16)
project(situation_awareness_uam_demo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(uam_sa_demo
    src/main.cpp
    src/sim_sensor.cpp
    src/tracker.cpp
    src/risk.cpp
    src/udp_pub.cpp
)

target_include_directories(uam_sa_demo PRIVATE include)

if (MSVC)
  target_compile_options(uam_sa_demo PRIVATE /W4)
else()
  target_compile_options(uam_sa_demo PRIVATE -Wall -Wextra -Wpedantic -O2)
endif()
