set(SOURCES
    vsgpagedlod.cpp
)

SET(TARGET_SRC ${SOURCES})

INCLUDE_DIRECTORIES(${OSG_INCLUDE_DIRS} ${Vulkan_INCLUDE_DIR})

add_executable(voe_globe ${SOURCES})

target_link_libraries(voe_globe vsgEarth vsg::vsg osgEarth ${OpenSceneGraph_LIBRARIES})

target_link_libraries(voe_globe vsgXchange::vsgXchange)

