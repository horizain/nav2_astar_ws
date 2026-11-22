cmake_minimum_required(VERSION 3.28)

# 默认使用 g++-14（支持 C++23），可在配置时用 -DCMAKE_CXX_COMPILER=... 覆盖
if(NOT DEFINED CMAKE_CXX_COMPILER AND NOT DEFINED ENV{CXX})
    find_program(GXX14 g++-14)
    if(GXX14)
        set(CMAKE_CXX_COMPILER ${GXX14})
    endif()
endif()

project(icp LANGUAGES CXX VERSION 0.2.0)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra -Wpedantic -march=native)

# ── 调试工具 ──
add_executable(dbg
    src/dbg.cpp
    src/fast_icp.cpp
    src/icp.cpp
)
target_include_directories(dbg PRIVATE src)

# ── demo ──
add_executable(icp
    src/main.cpp
    src/icp.cpp
)
target_include_directories(icp PRIVATE src)

# ── bench：FastICP vs mini_slam CorrelativeMatcher ──
# 迁入 nav2_astar_ws 后默认指向同工作区的 mini_slam core；
# 独立使用（本包被拷出去单独跑）时可用 -DMINI_SLAM_CORE=/path/to/mini_slam/core 覆盖。
set(MINI_SLAM_CORE "${CMAKE_CURRENT_SOURCE_DIR}/../mini_slam/core"
    CACHE PATH "mini_slam core 源码路径（纯标准库，无 ROS/ceres 依赖）")

add_executable(bench
    src/bench.cpp
    src/fast_icp.cpp
    src/icp.cpp
    ${MINI_SLAM_CORE}/src/likelihood_field.cpp
    ${MINI_SLAM_CORE}/src/correlative_matcher.cpp
)
target_include_directories(bench PRIVATE src ${MINI_SLAM_CORE}/include)

# ── 安装（colcon/cmake build type 需要；独立构建时无副作用）──
install(TARGETS icp dbg bench RUNTIME DESTINATION bin)
