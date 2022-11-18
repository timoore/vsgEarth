cmake_minimum_required(VERSION 3.7)

project(vsgearth
    VERSION 0.0.1
    DESCRIPTION "osgEarth terrain with VSG backend"
    LANGUAGES CXX C
)
set(VSGEARTH_SOVERSION 0)
SET(VSGEARTH_RELEASE_CANDIDATE 0)

# We have some custom .cmake scripts not in the official distribution.
SET(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${vsgearth_SOURCE_DIR}/CMakeModules")

# set the use of C++17 globally as all examples require it
set(CMAKE_CXX_STANDARD 17)


# Find Vulkan and the VSG
if (VULKAN_SDK)
    set(ENV{VULKAN_SDK} ${VULKAN_SDK})
endif()

set(VSG_MIN_VERSION 1.0.0)
find_package(vsg ${VSG_MIN_VERSION} REQUIRED)

# find osg2vsg package for loading image/nodes using OpenSceneGraph
find_package(osg2vsg REQUIRED)
set(EXTRA_LIBRARIES ${EXTRA_LIBRARIES} osg2vsg::osg2vsg)

find_package(vsgXchange REQUIRED)

# for generated cmake support files
set(FIND_DEPENDENCY ${FIND_DEPENDENCY} "find_dependency(vsg ${VSG_MIN_VERSION} REQUIRED)")

find_package(osgEarth REQUIRED)

include(vsgearthMacroUtils)

vsg_setup_build_vars()
# This does include(GNUInstallDirs), which sets up all the correct install directories.
vsg_setup_dir_vars()

set(GENERATED_HEADERS_DIR "${PROJECT_BINARY_DIR}/include")

set(VSGEARTH_DATA_DIR "${CMAKE_INSTALL_DATADIR}/vsgearth")
set(VSGEARTH_FULL_DATA_DIR "${CMAKE_INSTALL_FULL_DATADIR}/vsgearth")

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/src/vsgEarth/Config.h.in"
  "${GENERATED_HEADERS_DIR}/vsgEarth/Config.h")

vsg_add_target_clang_format(
  FILES
  ${CMAKE_SOURCE_DIR}/include/*/*.h
  ${CMAKE_SOURCE_DIR}/src/*/*.h
  ${CMAKE_SOURCE_DIR}/src/*/*.cpp
  ${CMAKE_SOURCE_DIR}/applications/*/*.h
  ${CMAKE_SOURCE_DIR}/applications/*/*.cpp
  )
      
vsg_add_target_clobber()
vsg_add_target_docs(
    FILES
        ${CMAKE_SOURCE_DIR}/include
)
vsg_add_target_uninstall()

vsg_add_option_maintainer(
    PREFIX ${PROJECT_NAME}
    RCLEVEL ${VSGEARTH_RELEASE_CANDIDATE}
)

# Make the headers visible to everything. This is not OSG / VSG style,
# but we prefer that include files live next to their source files.
include_directories(${vsgearth_SOURCE_DIR}/src ${GENERATED_HEADERS_DIR})

# source directory for main vsgXchange library
add_subdirectory(src)

vsg_add_feature_summary()
