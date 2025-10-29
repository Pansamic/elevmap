cmake_minimum_required(VERSION 3.10)

project(MobilePlanner
    VERSION 1.0
    LANGUAGES C CXX
)

option(BUILD_TESTS "Build test programs" OFF)
option(BUILD_SHARED_LIBS "Build shared library" OFF)
option(MOBILE_PLANNER_LOG_MAP OFF)

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

# Set third party directory for potential source builds
set(THIRD_PARTY_PREFIX "${CMAKE_BINARY_DIR}/_deps/")

# Find packages or include build configurations
# find_package(Eigen3 QUIET)
# if(NOT Eigen3_FOUND)
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/dependency/eigen.cmake)
# endif()

# find_package(Boost QUIET)
# if(NOT Boost_FOUND)
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/dependency/boost.cmake)
# endif()

# find_package(PCL QUIET)
# if(NOT PCL_FOUND)
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/dependency/pcl.cmake)
# endif()

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/dependency/yaml-cpp.cmake)

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

# Link with dependencies (either system installed or built from source)
target_link_libraries(${PROJECT_NAME} PUBLIC
    Eigen3::Eigen
    yaml-cpp::yaml-cpp
    Boost::boost
    ${PCL_LIBRARIES}
)

# Set include directories
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${PCL_INCLUDE_DIRS}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
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