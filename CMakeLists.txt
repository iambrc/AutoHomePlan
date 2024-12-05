cmake_minimum_required(VERSION 3.15)
project(
    "AutoHomePlan"
    VERSION 1.0
    LANGUAGES C CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
message(STATUS "Started CMake for ${PROJECT_NAME} v${PROJECT_VERSION}...\n")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug")
endif()

# set vcpkg path
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/vcpkg/scripts/buildsystems/vcpkg.cmake")

# find packages
find_package(OpenGL REQUIRED)
set(CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/vcpkg_installed/x64-windows")
find_package(assimp CONFIG REQUIRED PATHS CMAKE_PREFIX_PATH)
find_package(boost_graph REQUIRED CONFIG PATHS CMAKE_PREFIX_PATH)
find_package(nlohmann_json CONFIG REQUIRED PATHS CMAKE_PREFIX_PATH)
find_package(glfw3 CONFIG REQUIRED PATHS CMAKE_PREFIX_PATH)
find_package(imgui CONFIG REQUIRED PATHS CMAKE_PREFIX_PATH)
find_package(glm CONFIG REQUIRED PATHS CMAKE_PREFIX_PATH)

# set include
include_directories(${CMAKE_SOURCE_DIR}/include)

# set externals
set(EXTERNAL_DIR ${CMAKE_SOURCE_DIR}/externals)

include_directories(${EXTERNAL_DIR}/glad/include)
set(GLAD_SRC ${EXTERNAL_DIR}/glad/src/glad.c)
add_library(glad ${GLAD_SRC})

include_directories(${EXTERNAL_DIR}/ImGuiFileDialog)
add_library(ImGuiFileDialog STATIC
    ${EXTERNAL_DIR}/ImGuiFileDialog/ImGuiFileDialog.cpp
    ${EXTERNAL_DIR}/ImGuiFileDialog/ImGuiFileDialog.h
    ${EXTERNAL_DIR}/ImGuiFileDialog/ImGuiFileDialogConfig.h
)
target_include_directories(ImGuiFileDialog PRIVATE ${VCPKG_INCLUDE_DIRS}/imgui)
target_link_libraries(ImGuiFileDialog PRIVATE imgui::imgui)

# set Gurobi
include(FindGUROBI.cmake)
include_directories(${GUROBI_INCLUDE_DIRS})

# add executable
file(GLOB_RECURSE SOLVER_SOURCES src/Solver/*.cpp)
file(GLOB_RECURSE GUI_SOURCES src/GUI/*.cpp)
add_executable(AutoHomePlan ${GUI_SOURCES} ${SOLVER_SOURCES})

# link libraries
target_link_libraries(AutoHomePlan PRIVATE ${catkin_LIBRARIES} ${GUROBI_LIBRARIES})
target_link_libraries(AutoHomePlan PRIVATE assimp::assimp Boost::graph nlohmann_json::nlohmann_json imgui::imgui glm::glm glad glfw OpenGL::GL ImGuiFileDialog)
