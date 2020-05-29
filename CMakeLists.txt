cmake_minimum_required(VERSION 3.14)

project(Synthesizer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)


message("win sdk: ${WinSDK}")

add_executable(synthesizer src/main.cpp)
target_include_directories(synthesizer PRIVATE include)
target_link_libraries(synthesizer Winmm.lib)

# add_executable(synthesizer2 src/main2.cpp)
# target_include_directories(synthesizer2 PRIVATE include)
# target_link_libraries(synthesizer2 Winmm.lib)
