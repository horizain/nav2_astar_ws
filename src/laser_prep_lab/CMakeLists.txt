cmake_minimum_required(VERSION 3.22)

# 与 icp 包同款：默认 g++-14（C++23），可覆盖
if(NOT DEFINED CMAKE_CXX_COMPILER AND NOT DEFINED ENV{CXX})
    find_program(GXX14 g++-14)
    if(GXX14)
        set(CMAKE_CXX_COMPILER ${GXX14})
    endif()
endif()

project(laser_prep_lab LANGUAGES CXX VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

# ── icp 包（配准后端：位姿链用它）── 纯源码引用，与 icp 包引用
# mini_slam 的方式一致，不引入安装/导出耦合。
set(ICP_SRC "${CMAKE_CURRENT_SOURCE_DIR}/../icp/src"
    CACHE PATH "icp 包源码路径（types/align 所在目录）")

enable_testing()
foreach(t IN ITEMS test_filters test_deskew test_cluster
                    test_dynamic test_accumulate)
    add_executable(${t} src/tests/${t}.cpp)
    target_include_directories(${t} PRIVATE src ${ICP_SRC})
    add_test(NAME ${t} COMMAND ${t})
endforeach()

# 数据集入口：全链清洗 + ICP 位姿链 + 可视化
add_executable(laser_prep_lab
    src/main.cpp
    ${ICP_SRC}/icp.cpp
)
target_include_directories(laser_prep_lab PRIVATE src ${ICP_SRC})

# 效果评估：病灶注入 + 下游消融（评估方法本身也是可测试的交付物）
add_executable(laser_prep_eval
    src/eval.cpp
    ${ICP_SRC}/icp.cpp
)
target_include_directories(laser_prep_eval PRIVATE src ${ICP_SRC})

install(TARGETS laser_prep_lab laser_prep_eval RUNTIME DESTINATION bin)
