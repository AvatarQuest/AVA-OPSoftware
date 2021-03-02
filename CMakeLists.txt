cmake_minimum_required (VERSION 3.8)

project ("AVAOPSoftware")
# Add the cmake folder so the FindSphinx module is found
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

add_subdirectory ("docs")
add_subdirectory ("src/c++")
