cmake_minimum_required(VERSION 3.10)

project(MobilePlanner
    VERSION 1.0
    LANGUAGES C CXX
)

option(BUILD_TESTS "Build test programs" OFF)
option(BUILD_SHARED_LIBS "Build shared library" OFF)

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

# Check for AVX support
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-mavx2" COMPILER_SUPPORTS_AVX2)
check_cxx_compiler_flag("-mavx" COMPILER_SUPPORTS_AVX)
if(COMPILER_SUPPORTS_AVX2)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mavx2")
endif()
if(NOT COMPILER_SUPPORTS_AVX2 AND COMPILER_SUPPORTS_AVX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mavx")
endif()

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

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

add_library(${PROJECT_NAME}::MobilePlanner ALIAS MobilePlanner)

# Link with dependencies (either system installed or built from source)
target_link_libraries(${PROJECT_NAME} PUBLIC
    Eigen3::Eigen
    Boost::boost
    ${PCL_LIBRARIES}
)

# Set include directories
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${EIGEN_INCLUDE_DIRS}>
    $<BUILD_INTERFACE:${PCL_INCLUDE_DIRS}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_options(${PROJECT_NAME} PUBLIC -fopenmp)

# Add test executable if tests are enabled
if(BUILD_TESTS)
    find_package(yaml-cpp REQUIRED)
    add_executable(test_map_plan_static test/test_map_plan_static.cpp)
    target_link_libraries(test_map_plan_static PRIVATE
        MobilePlanner
        yaml-cpp::yaml-cpp)
endif()

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
