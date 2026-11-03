cmake_minimum_required(VERSION 3.16)

project(ArmorPnP LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(OpenCV REQUIRED)
message(STATUS "OpenCV ${OpenCV_VERSION}")

add_executable(armor_demo
    main.cpp
    video_reader.cpp
    video_reader.hpp
    armor_process.cpp
    armor_process.hpp
    pnp.cpp
    pnp.hpp
)

target_include_directories(armor_demo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(armor_demo PRIVATE ${OpenCV_LIBS})

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(armor_demo PRIVATE -Wall -Wextra)
endif()

# 把测试视频拷到可执行文件旁边，这样在 build 目录里也能直接运行
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/raw.mp4)
    add_custom_command(TARGET armor_demo POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${CMAKE_CURRENT_SOURCE_DIR}/raw.mp4
                $<TARGET_FILE_DIR:armor_demo>/raw.mp4
        COMMENT "Copying raw.mp4 to build directory"
    )
endif()
