cmake_minimum_required(VERSION 3.10)
project(NesEmulator)
SET(CXX_STANDARD 20)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(X11 REQUIRED)

include_directories(include)

add_executable(NES_Emulator src/olcNes_Video1_6502.cpp src/PZ6502.cpp src/Bus.cpp)
target_link_libraries(NES_Emulator
                      ${OPENGL_LIBRARIES}
                      ${GLUT_LIBRARIES}
                      ${X11_LIBRARIES}
                      pthread
                      png
                      stdc++fs)
