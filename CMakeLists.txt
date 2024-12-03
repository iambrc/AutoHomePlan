cmake_minimum_required(VERSION 3.15)
project(
    "AutoHomePlan"
    VERSION 1.0
    LANGUAGES C CXX
)

# 设置C++标准
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
message(STATUS "Started CMake for ${PROJECT_NAME} v${PROJECT_VERSION}...\n")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug")
endif()

# 设置vcpkg路径
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE STRING "Vcpkg toolchain file")

# 查找所需的库
set(CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/vcpkg_installed/x64-windows")
find_package(assimp CONFIG REQUIRED PATHS CMAKE_PREFIX_PATH)
find_package(boost_graph REQUIRED CONFIG PATHS CMAKE_PREFIX_PATH)
find_package(imgui CONFIG REQUIRED PATHS CMAKE_PREFIX_PATH)
find_package(nlohmann_json CONFIG REQUIRED PATHS CMAKE_PREFIX_PATH)

# 添加源文件和头文件
include_directories(${CMAKE_SOURCE_DIR}/include)

# 设置Gurobi库路径
include(FindGUROBI.cmake)
include_directories(${GUROBI_INCLUDE_DIRS})

# 添加可执行文件
add_executable(AutoHomePlan src/Solver/main.cpp)

# 链接库
target_link_libraries(AutoHomePlan PRIVATE ${catkin_LIBRARIES} ${GUROBI_LIBRARIES})
target_link_libraries(AutoHomePlan PRIVATE assimp::assimp imgui::imgui Boost::graph nlohmann_json::nlohmann_json)