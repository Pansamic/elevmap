include(FetchContent)
include(ExternalProject)

ExternalProject_Add(pcl
    URL https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-1.15.1.zip
    URL_HASH SHA256=e40fff11625b45beee1fe247a74c79e02bb6a88f38d1b247b4335bda51c84f95
    DOWNLOAD_NAME pcl-1.15.1.zip
    DOWNLOAD_DIR   "${THIRD_PARTY_PREFIX}/downloads"
    TMP_DIR        "${THIRD_PARTY_PREFIX}/tmp"
    SOURCE_DIR     "${THIRD_PARTY_PREFIX}/src/pcl"
    BINARY_DIR     "${CMAKE_BINARY_DIR}/_deps/pcl-build"
    CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/_deps/install
        -DCMAKE_BUILD_TYPE=Release
        -DCMAKE_MODULE_PATH=${CMAKE_MODULE_PATH}
        -DCMAKE_PREFIX_PATH=${CMAKE_BINARY_DIR}/_deps/install
        -DBoost_USE_STATIC_LIBS=OFF
        -DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}
        -DEIGEN_MAX_ALIGN_BYTES=32
    INSTALL_COMMAND $(MAKE) install
    TEST_COMMAND ""
    DEPENDS boost eigen
)

ExternalProject_Get_Property(boost INSTALL_DIR)
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/_deps/install/include/pcl-1.15)
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/_deps/install/lib)

set(PCL_LIBRARIES
    libpcl_common.so
    libpcl_io_ply.so
    libpcl_io.so
    libpcl_ml.so
    libpcl_octree.so
    libpcl_stereo.so
)
if(NOT TARGET PCL::PCL)
    # Create imported target
    add_library(PCL::PCL INTERFACE IMPORTED)
    add_dependencies(PCL::PCL pcl)
    set_target_properties(PCL::PCL PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_BINARY_DIR}/_deps/install/include/pcl-1.15"
        INTERFACE_LINK_DIRECTORIES "${CMAKE_BINARY_DIR}/_deps/install/lib"
        INTERFACE_LINK_LIBRARIES "${PCL_LIBRARIES}"
    )
endif()
