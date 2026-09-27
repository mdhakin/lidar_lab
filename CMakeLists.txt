cmake_minimum_required(VERSION 3.16)

project(lidar_lab LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(IMGUI_DIR
    ${CMAKE_SOURCE_DIR}/external/imgui)

set(RPLIDAR_DIR
    ${CMAKE_SOURCE_DIR}/external/rplidar_sdk)

set(RPLIDAR_LIB
    ${RPLIDAR_DIR}/output/Linux/Release/libsl_lidar_sdk.a)

find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

add_executable(lidar_lab
    src/main.cpp
    src/lidar.cpp

    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)

target_include_directories(lidar_lab PRIVATE
    src

    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends

    ${RPLIDAR_DIR}/sdk/include
    ${RPLIDAR_DIR}/sdk/src
)

target_link_libraries(lidar_lab PRIVATE
    ${RPLIDAR_LIB}
    glfw
    OpenGL::GL
    pthread
)