include(FetchContent)
include(ExternalProject)

ExternalProject_Add(boost
    URL https://github.com/boostorg/boost/releases/download/boost-1.89.0/boost-1.89.0-cmake.tar.gz
    DOWNLOAD_NAME boost-1.89.0.tar.gz
    DOWNLOAD_DIR   "${THIRD_PARTY_PREFIX}/downloads"
    TMP_DIR        "${THIRD_PARTY_PREFIX}/tmp"
    SOURCE_DIR     "${THIRD_PARTY_PREFIX}/src/boost"
    BINARY_DIR     "${CMAKE_BINARY_DIR}/_deps/boost-build"
    CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/_deps/install
        -DCMAKE_BUILD_TYPE=Release
        -DBUILD_SHARED_LIBS=ON
        -DBOOST_CXX_FLAGS=-fPIC
    INSTALL_COMMAND $(MAKE) install
    TEST_COMMAND ""
)

ExternalProject_Get_Property(boost INSTALL_DIR)

file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/_deps/install/include)
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/_deps/install/lib)

if(NOT TARGET Boost::boost)
    set(Boost_INCLUDE_DIRS ${CMAKE_BINARY_DIR}/_deps/install/include)
    set(Boost_LIBRARY_DIRS ${CMAKE_BINARY_DIR}/_deps/install/lib)
    add_library(Boost::boost INTERFACE IMPORTED)
    add_dependencies(Boost::boost boost)
    set_target_properties(Boost::boost PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES ${Boost_INCLUDE_DIRS}
        INTERFACE_LINK_DIRECTORIES ${Boost_LIBRARY_DIRS}
    )
endif()
