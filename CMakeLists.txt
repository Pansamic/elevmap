cmake_minimum_required(VERSION 3.10)

project(MobilePlanner)

option(BUILD_TESTS "Build test programs" OFF)
option(BUILD_SHARED_LIBS "Build shared library" OFF)
option(MOBILE_PLANNER_LOG_MAP OFF)

enable_language(C CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if ( NOT CMAKE_BUILD_TYPE )
    set(CMAKE_BUILD_TYPE Release CACHE STRING "cmake build type")
endif()

if ( CMAKE_BUILD_TYPE STREQUAL "Release" )
    add_compile_options(-O3)
    add_compile_options(-DNDEBUG)
endif()

if ( CMAKE_BUILD_TYPE STREQUAL "Release" )
    add_compile_options(-O3)
    add_compile_options(-DNDEBUG)
endif()

include(cmake/dependencies.cmake)

# Source files
set(MOBILE_PLANNER_SOURCES
    src/elevation_map.cpp
    src/grid_map.cpp
    src/mobile_planner.cpp
)

# Header files
set(MOBILE_PLANNER_HEADERS
    include/mobile_planner/elevation_map.h
    include/mobile_planner/grid_map.h
    include/mobile_planner/mobile_planner.h
)

# Create library based on option
if(MOBILE_PLANNER_BUILD_SHARED)
    add_library(${PROJECT_NAME} SHARED ${MOBILE_PLANNER_SOURCES})
else()
    add_library(${PROJECT_NAME} STATIC ${MOBILE_PLANNER_SOURCES})
endif()

add_dependencies(${PROJECT_NAME} eigen pcl yaml-cpp boost)

# Set include directories and link libraries for target
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/_deps/install/include>
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/_deps/install/include/eigen3>
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/_deps/install/include/pcl-1.15>
    $<INSTALL_INTERFACE:include>
    $<INSTALL_INTERFACE:include/eigen3>
    $<INSTALL_INTERFACE:include/pcl-1.15>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    ${CMAKE_BINARY_DIR}/_deps/install/lib/libpcl_common.so
    ${CMAKE_BINARY_DIR}/_deps/install/lib/libpcl_io.so
    ${CMAKE_BINARY_DIR}/_deps/install/lib/libyaml-cpp.a
)

# Install targets with export
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Install headers
install(FILES ${MOBILE_PLANNER_HEADERS} 
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mobile_planner
)

# Install third-party headers and libraries
install(DIRECTORY ${CMAKE_BINARY_DIR}/_deps/install/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(DIRECTORY ${CMAKE_BINARY_DIR}/_deps/install/lib/
    DESTINATION ${CMAKE_INSTALL_LIBDIR}
    FILES_MATCHING 
    PATTERN "*.so*"
    PATTERN "*.a"
    PATTERN "*.dylib*"
)

# Export targets
install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

# Generate the version file for the config file
write_basic_package_version_file(
    "${CMAKE_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    VERSION 1.0.0
    COMPATIBILITY SameMajorVersion
)

# Configure the config file
configure_package_config_file(
    "cmake/${PROJECT_NAME}Config.cmake.in"
    "${CMAKE_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

# Install config files
install(FILES
    "${CMAKE_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    "${CMAKE_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

# Add test executable if tests are enabled
if(MOBILE_PLANNER_BUILD_TESTS)
    add_executable(test_map_plan_static test/test_map_plan_static.cpp)
    target_link_libraries(test_map_plan_static PRIVATE MobilePlanner::MobilePlanner)
endif()