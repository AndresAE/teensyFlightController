Plotter ImGui + ImPlot example

Files:
- main.cpp : example program that plays back `test/tm/telemetry_log.txt` and renders two-column strip charts.
- CMakeLists.txt : minimal CMake; you must link ImGui/ImPlot/SDL2/OpenGL yourself.

Build (example using your own ImGui/ImPlot setup):

mkdir build
cd build
cmake ..
make

Run:

./plotTM_imgui [path/to/telemetry_log.txt]

Notes:
- This is a minimal example. It uses SDL2 + OpenGL3 backend and assumes you have the ImGui + ImPlot backends (imgui_impl_sdl.cpp/imgui_impl_opengl3.cpp) compiled in your project.
- If you want, I can convert this to a self-contained CMake that fetches ImGui/ImPlot via add_subdirectory and compiles everything automatically. Request that and I'll add it.