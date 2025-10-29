include(FetchContent)
include(ExternalProject)

ExternalProject_Add(eigen
    URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
    DOWNLOAD_NAME eigen-3.4.0.tar.gz
    DOWNLOAD_DIR   "${THIRD_PARTY_PREFIX}/downloads"
    TMP_DIR        "${THIRD_PARTY_PREFIX}/tmp"
    SOURCE_DIR     "${THIRD_PARTY_PREFIX}/src/eigen"
    BINARY_DIR     "${CMAKE_BINARY_DIR}/_deps/eigen-build"
    CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/_deps/install
        -DCMAKE_BUILD_TYPE=Release
        -DCMAKE_POLICY_VERSION_MINIMUM=3.10
    INSTALL_COMMAND $(MAKE) install
    TEST_COMMAND ""
)

ExternalProject_Get_Property(eigen INSTALL_DIR)

file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/_deps/install/include/eigen3)

if(NOT TARGET Eigen3::Eigen)
    add_library(Eigen3::Eigen INTERFACE IMPORTED GLOBAL)
    set_target_properties(Eigen3::Eigen PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_BINARY_DIR}/_deps/install/include/eigen3
    )
    add_dependencies(Eigen3::Eigen eigen)
endif()
