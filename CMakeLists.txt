cmake_minimum_required(VERSION 3.5.0)
project(path-planning VERSION 0.1.0 LANGUAGES C CXX)

add_subdirectory(external/toml11)

include_directories(external/toml11)

add_executable(path-planning src/main.cpp src/ConfigLoader.cpp src/utils.cpp src/conePath.cpp src/coneBST.cpp src/cone.cpp src/TrackLoader.cpp src/GridPlotter.cpp)

include(CTest)
enable_testing()

# Find the Cairo library
find_package(PkgConfig REQUIRED)
pkg_check_modules(CAIRO REQUIRED cairo)

# Include the Cairo include directories
include_directories(${CAIRO_INCLUDE_DIRS})

target_link_libraries(path-planning PRIVATE toml11 ${CAIRO_LIBRARIES})

file(COPY ${CMAKE_SOURCE_DIR}/src/config.toml DESTINATION ${CMAKE_BINARY_DIR})

file(COPY ${CMAKE_SOURCE_DIR}/cone_csv DESTINATION ${CMAKE_BINARY_DIR})

# Optionally, define the config path for use in code
add_definitions(-DCONFIG_PATH="${CMAKE_BINARY_DIR}/config.toml")
add_definitions(-DTRACK_PATH="${CMAKE_BINARY_DIR}/cone_csv")

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
