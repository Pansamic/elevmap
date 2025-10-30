include(FetchContent)
include(ExternalProject)

ExternalProject_Add(yaml-cpp
    URL https://github.com/jbeder/yaml-cpp/archive/refs/tags/0.8.0.tar.gz
    URL_HASH SHA256=fbe74bbdcee21d656715688706da3c8becfd946d92cd44705cc6098bb23b3a16
    DOWNLOAD_NAME yaml-cpp-0.8.0.tar.gz
    DOWNLOAD_DIR   "${THIRD_PARTY_PREFIX}/downloads"
    TMP_DIR        "${THIRD_PARTY_PREFIX}/tmp"
    SOURCE_DIR     "${THIRD_PARTY_PREFIX}/src/yaml-cpp"
    BINARY_DIR     "${CMAKE_BINARY_DIR}/_deps/yaml-cpp-build"
    CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/_deps/install
        -DCMAKE_BUILD_TYPE=Release
        -DCMAKE_POLICY_VERSION_MINIMUM=3.10
        -DYAML_CPP_BUILD_TOOLS=OFF
        -DYAML_BUILD_SHARED_LIBS=OFF
    INSTALL_COMMAND $(MAKE) install
    TEST_COMMAND ""
)

ExternalProject_Get_Property(yaml-cpp INSTALL_DIR)
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/_deps/install/include)
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/_deps/install/lib)

set(YAML_CPP_INCLUDE_DIR ${CMAKE_BINARY_DIR}/_deps/install/include)
set(YAML_CPP_LIBRARIES ${CMAKE_BINARY_DIR}/_deps/install/lib/libyaml-cpp.a)
add_library(yaml-cpp::yaml-cpp STATIC IMPORTED)
add_dependencies(yaml-cpp::yaml-cpp yaml-cpp)
set_target_properties(yaml-cpp::yaml-cpp PROPERTIES
    IMPORTED_LOCATION ${YAML_CPP_LIBRARIES}
    INTERFACE_INCLUDE_DIRECTORIES ${YAML_CPP_INCLUDE_DIR}
)