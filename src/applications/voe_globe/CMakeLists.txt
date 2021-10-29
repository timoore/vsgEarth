set(SOURCES
    TileReaderVOE.h
    TileReaderVOE.cpp
    vsgpagedlod.cpp
    VoeLOD.cpp
    VoeLOD.h
    VoeSim.cpp
    VoeSim.h
    VoeImageUtils.cpp
    VoeImageUtils.h
)

SET(TARGET_SRC ${SOURCES})

INCLUDE_DIRECTORIES(${OSG_INCLUDE_DIRS} ${Vulkan_INCLUDE_DIR})

add_executable(voe_globe ${SOURCES})

#SET(TARGET_LIBRARIES_VARS OSG_LIBRARY OSGDB_LIBRARY OSGUTIL_LIBRARY OSGVIEWER_LIBRARY OPENTHREADS_LIBRARY)

#SETUP_APPLICATION(voe_globe)

# add_custom_target()

SET(CONCAT_COMMAND ${CMAKE_SOURCE_DIR}/CMakeModules/ConcatFiles.cmake)
SET(GLSL_FILES voeSDK.glsl)
list_transform_prepend(GLSL_FILES "${CMAKE_CURRENT_SOURCE_DIR}/")

message("GLSL_FILES: ${GLSL_FILES}")

set(GLSLC $ENV{VULKAN_SDK}/bin/glslc)

macro(make_shader SHADER_SOURCE SHADER_OUTPUT)
  add_custom_command(OUTPUT ${SHADER_OUTPUT}
  COMMAND "${CMAKE_COMMAND}" -P ${CONCAT_COMMAND} ${SHADER_SOURCE} ${GLSL_FILES} ${SHADER_OUTPUT}
  DEPENDS ${SHADER_SOURCE} ${GLSL_FILES})

  add_custom_command(OUTPUT ${SHADER_OUTPUT}.spv
  COMMAND ${GLSLC} ${SHADER_OUTPUT} -o ${SHADER_OUTPUT}.spv
  DEPENDS ${SHADER_OUTPUT} ${CMAKE_CURRENT_SOURCE_DIR}/VoeSDK.h)
endmacro(make_shader)

#add_custom_command(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/elevation.vert
#  COMMAND "${CMAKE_COMMAND}" -P ${CONCAT_COMMAND} ${GLSL_FILES} "${CMAKE_CURRENT_SOURCE_DIR}/elevation.vert"
#  DEPENDS ${GLSL_FILES})

#add_custom_command(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/elevation.vert.spv
#  COMMAND ${GLSLC} ${CMAKE_CURRENT_SOURCE_DIR}/elevation.vert -o ${CMAKE_CURRENT_SOURCE_DIR}/elevation.vert.spv
#  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/elevation.vert)

make_shader(${CMAKE_CURRENT_SOURCE_DIR}/reverse_depth.vert ${CMAKE_CURRENT_SOURCE_DIR}/elevation.vert)
make_shader(${CMAKE_CURRENT_SOURCE_DIR}/elevation_main.frag ${CMAKE_CURRENT_SOURCE_DIR}/elevation.frag)

add_custom_target(voe_shaders ALL DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/elevation.vert.spv ${CMAKE_CURRENT_SOURCE_DIR}/elevation.frag.spv)
target_link_libraries(voe_globe vsg::vsg osgEarth ${OpenSceneGraph_LIBRARIES})

target_compile_definitions(voe_globe PRIVATE vsgXchange_FOUND)
target_link_libraries(voe_globe vsgXchange::vsgXchange)

