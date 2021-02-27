cmake_minimum_required(VERSION 3.19.4)

project(NesEmulator)

SET(CXX_STANDARD 17)

include_directories(include)

add_executable(NES_Emulator src/olcNes_Video1_6502.cpp)
