cmake_minimum_required(VERSION 3.10)
project(plotTM_imgui)

set(CMAKE_CXX_STANDARD 17)

# This example expects you to provide ImGui, ImPlot, SDL2 and OpenGL3
# Either link them via your package manager or add them as subprojects.

find_package(SDL2 REQUIRED)
# You need to find or provide ImGui and ImPlot targets: ImGui::ImGui ImPlot::ImPlot
# For a quick local build, add imgui and implot sources and create targets manually.

add_executable(plotTM_imgui main.cpp)
# Link libraries here, e.g.:
# target_link_libraries(plotTM_imgui PRIVATE SDL2::SDL2 ImGui::ImGui ImPlot::ImPlot OpenGL::GL)

message(STATUS "Note: you must provide ImGui/ImPlot/SDL/OpenGL linking in CMakeLists")
