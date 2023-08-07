cmake_minimum_required (VERSION 3.12)

project ("GAMES_101")

set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/install)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/target_debug)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/target_release)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    # 解决MSVC C4819警告
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W3 /Zc:__cplusplus /utf-8 /EHsc")
endif()

find_package(OpenCV REQUIRED)

if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV not found")
else()
    message("------------------------------------------------------")
    message("OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
    message("    OpenCV_LIB_PATH: ${OpenCV_LIB_PATH}")
    message("   _OpenCV_LIB_PATH: ${_OpenCV_LIB_PATH}")
    message("        OpenCV_LIBS: ${OpenCV_LIBS}")
    message("------------------------------------------------------")
endif()

find_package(Eigen3 REQUIRED)

if(NOT Eigen3_FOUND)
    message(FATAL_ERROR "Eigen3 not found")
else()
    message("------------------------------------------------------")
    message("EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")
    message("------------------------------------------------------")
endif()

include(${PROJECT_SOURCE_DIR}/cmake/build_target.cmake)
BuildTarget(${CMAKE_CURRENT_SOURCE_DIR}/sources)
