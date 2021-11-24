IF(OSGEARTH_BUILD_SHARED_LIBS)
    ADD_DEFINITIONS(-DVSGEARTH_LIBRARY)
ELSE()
    ADD_DEFINITIONS(-DVSGEARTH_LIBRARY_STATIC)
ENDIF()

set(SOURCES
    TerrainEngineVOE.h
    TerrainEngineVOE.cpp
    TileReaderVOE.h
    TileReaderVOE.cpp
    VoeLOD.cpp
    VoeLOD.h
    VoeSim.cpp
    VoeSim.h
    VoeImageUtils.cpp
    VoeImageUtils.h
)

set(LIB_NAME vsgEarth)

set(TARGET_SRC ${SOURCES})

set(LIB_PUBLIC_HEADERS
  TerrainEngineVOE.h
  VoeSim.h)

set(TARGET_H ${LIB_PUBLIC_HEADERS})

ADD_LIBRARY(
    ${LIB_NAME}
    ${OSGEARTH_DYNAMIC_OR_STATIC}
    ${LIB_PUBLIC_HEADERS}
    ${TARGET_SRC}
)

target_link_libraries(${LIB_NAME} vsg::vsg osgEarth ${OpenSceneGraph_LIBRARIES})

target_compile_definitions(${LIB_NAME} PRIVATE vsgXchange_FOUND)
target_link_libraries(${LIB_NAME} vsgXchange::vsgXchange)

SET(CONCAT_COMMAND ${CMAKE_SOURCE_DIR}/CMakeModules/ConcatFiles.cmake)
SET(GLSL_FILES voeSDK.glsl)
list_transform_prepend(GLSL_FILES "${CMAKE_CURRENT_SOURCE_DIR}/")

set(GLSLC $ENV{VULKAN_SDK}/bin/glslc)

macro(make_shader SHADER_SOURCE SHADER_OUTPUT)
  add_custom_command(OUTPUT ${SHADER_OUTPUT}
  COMMAND "${CMAKE_COMMAND}" -P ${CONCAT_COMMAND} ${SHADER_SOURCE} ${GLSL_FILES} ${SHADER_OUTPUT}
  DEPENDS ${SHADER_SOURCE} ${GLSL_FILES})

  add_custom_command(OUTPUT ${SHADER_OUTPUT}.spv
  COMMAND ${GLSLC} ${SHADER_OUTPUT} -o ${SHADER_OUTPUT}.spv
  DEPENDS ${SHADER_OUTPUT} ${CMAKE_CURRENT_SOURCE_DIR}/VoeSDK.h)
endmacro(make_shader)

make_shader(${CMAKE_CURRENT_SOURCE_DIR}/reverse_depth.vert ${CMAKE_CURRENT_SOURCE_DIR}/elevation.vert)
make_shader(${CMAKE_CURRENT_SOURCE_DIR}/elevation_main.frag ${CMAKE_CURRENT_SOURCE_DIR}/elevation.frag)

add_custom_target(voe_shaders ALL DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/elevation.vert.spv ${CMAKE_CURRENT_SOURCE_DIR}/elevation.frag.spv)
