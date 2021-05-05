set(SOURCES
    TileReaderVOE.h
    TileReaderVOE.cpp
    vsgpagedlod.cpp
)

SET(TARGET_SRC vsgpagedlod.cpp TileReaderVOE.cpp TileReaderVOE.h)

INCLUDE_DIRECTORIES(${OSG_INCLUDE_DIRS} ${Vulkan_INCLUDE_DIR})

add_executable(voe_globe ${SOURCES})

#SET(TARGET_LIBRARIES_VARS OSG_LIBRARY OSGDB_LIBRARY OSGUTIL_LIBRARY OSGVIEWER_LIBRARY OPENTHREADS_LIBRARY)

#SETUP_APPLICATION(voe_globe)

message("osg libraries:" ${OpenSceneGraph_LIBRARIES})

target_link_libraries(voe_globe vsg::vsg osgEarth ${OpenSceneGraph_LIBRARIES})

target_compile_definitions(voe_globe PRIVATE vsgXchange_FOUND)
target_link_libraries(voe_globe vsgXchange::vsgXchange)

