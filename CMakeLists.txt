cmake_minimum_required(VERSION 3.16)

option(WITH_TESTS "Build Tests" ON)
option(WITH_COVERAGE "" ON)

set(CMAKE_CXX_STANDARD 17)

# public header
set(LIB_HEADERS include/traact/point_cloud.h include/traact/traact_point_cloud_export.h)

set(PLUGIN_HEADERS

        )


# rest of source code
set(LIB_SOURCES src/point_cloud.cpp)

set(PLUGIN_SOURCES src/PointCloudPlugin.cpp
        src/component/PCLVisualizerWindow.h
        src/component/PCLVisualizerWindow.cpp
        src/component/PCLVisualizerBodyInput.h
        src/component/StandAloneDebugViewer.h
        src/component/StandAloneDebugViewer.cpp)


# conan is required hard, library name, version and description are configured from conan
if (EXISTS ${CMAKE_CURRENT_BINARY_DIR}/conanbuildinfo.cmake)
    include(${CMAKE_CURRENT_BINARY_DIR}/conanbuildinfo.cmake)
else ()
    message(FATAL_ERROR "The file conanbuildinfo.cmake doesn't exist, you have to run conan install first")
endif ()

project(${CONAN_PACKAGE_NAME}_PROJECT VERSION ${CONAN_PACKAGE_VERSION})

conan_basic_setup(TARGETS)

if (WITH_COVERAGE)
    #add_compile_options(-g -O0 -Wall -W -Wshadow -Wunused-variable -Wunused-parameter -Wunused-function -Wunused -Wno-system-headers -Wno-deprecated -Woverloaded-virtual -Wwrite-strings -fprofile-arcs -ftest-coverage)
    add_link_options(-fprofile-arcs -ftest-coverage)
endif ()

if (CMAKE_BUILD_TYPE EQUAL "DEBUG")
    add_definitions(-DTBB_USE_DEBUG=1)
endif ()

add_definitions(-DTBB_PREVIEW_FLOW_GRAPH_FEATURES=1)
add_definitions(-DTBB_PREVIEW_FLOW_GRAPH_NODES=1)

# conan workspace and cmake don't work that well together, did not manage to get the build dir into the includes: 
# force generated file into source include dir 
# only one export file could be generated
# use files and defines instead
#set(CMAKE_CXX_VISIBILITY_PRESET hidden)
#set(CMAKE_VISIBILITY_INLINES_HIDDEN 1)
#generate_export_header(${CONAN_PACKAGE_NAME} EXPORT_FILE_NAME "../../include/traact/${CONAN_PACKAGE_NAME}_export.h")

if(UNIX)
    if(APPLE)
        MESSAGE(STATUS "Building for Macos.")
        add_definitions(-DTRAACT_TARGET_APPLE=1)
    endif()
    MESSAGE(STATUS "Building for Unix.")
    add_definitions(-DTRAACT_TARGET_UNIX=1)
elseif(WIN32)
    MESSAGE(STATUS "Building for Windows.")
    add_definitions(-DTRAACT_TARGET_WINDOWS=1)
endif()

if (MSVC)
    # per default disable extended aligned storage for now on msvc
    add_definitions(-D_DISABLE_EXTENDED_ALIGNED_STORAGE -DHAVE_SNPRINTF)
endif()

add_link_options( -fno-gnu-unique )

add_library(${CONAN_PACKAGE_NAME} SHARED
        ${LIB_HEADERS}
        ${LIB_SOURCES}
        )

conan_target_link_libraries(${CONAN_PACKAGE_NAME})

set_target_properties(${CONAN_PACKAGE_NAME} PROPERTIES VERSION ${CONAN_PACKAGE_VERSION})
#set_target_properties(${CONAN_PACKAGE_NAME} PROPERTIES PUBLIC_HEADER ${LIB_HEADERS})

target_include_directories(${CONAN_PACKAGE_NAME} PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_include_directories(${CONAN_PACKAGE_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        )

IF (NOT WIN32)
    MESSAGE(STATUS "Set boost::ublas alignment to 16")
    set_target_properties(${CONAN_PACKAGE_NAME} PROPERTIES COMPILE_DEFINITIONS "BOOST_UBLAS_BOUNDED_ARRAY_ALIGN=__attribute__ ((aligned (16)))")
ENDIF (NOT WIN32)

install(TARGETS ${CONAN_PACKAGE_NAME}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})


# Plugin -----------------------
add_library(traact_point_cloud_plugin SHARED
        ${PLUGIN_HEADERS}
        ${PLUGIN_SOURCES}
        )

target_link_libraries(traact_point_cloud_plugin
        ${CONAN_PACKAGE_NAME})

#set_target_properties(traact_spatial_plugin PROPERTIES VERSION ${CONAN_PACKAGE_VERSION})
#set_target_properties(${CONAN_PACKAGE_NAME} PROPERTIES PUBLIC_HEADER ${LIB_HEADERS})

if (MSVC)
    # per default disable extended aligned storage for now on msvc
    add_definitions(-D_DISABLE_EXTENDED_ALIGNED_STORAGE -DHAVE_SNPRINTF)
endif()

target_include_directories(traact_point_cloud_plugin PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_include_directories(traact_point_cloud_plugin PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        )

IF (NOT WIN32)
    MESSAGE(STATUS "Set boost::ublas alignment to 16")
    set_target_properties(traact_point_cloud_plugin PROPERTIES COMPILE_DEFINITIONS "BOOST_UBLAS_BOUNDED_ARRAY_ALIGN=__attribute__ ((aligned (16)))")
ENDIF (NOT WIN32)

install(TARGETS traact_point_cloud_plugin
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

#-----------------------------


if (WITH_TESTS)
    add_subdirectory(tests)
endif ()

