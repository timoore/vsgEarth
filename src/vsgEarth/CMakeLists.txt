IF(VSGEARTH_BUILD_SHARED_LIBS)
    ADD_DEFINITIONS(-DVSGEARTH_LIBRARY)
ELSE()
    ADD_DEFINITIONS(-DVSGEARTH_LIBRARY_STATIC)
ENDIF()

set(LIB_NAME vsgEarth)

set(LIB_PUBLIC_HEADERS
  Export.h
  TerrainEngineVOE.h
  TileReaderVOE.h
  VoeSim.h)

set(SOURCES
  ${LIB_PUBLIC_HEADERS}
  TerrainEngineVOE.cpp
  TileReaderVOE.cpp
  VoeLOD.cpp
  VoeLOD.h
  VoeSim.cpp
  VoeImageUtils.cpp
  VoeImageUtils.h
  )

ADD_LIBRARY(
    ${LIB_NAME}
    ${LIB_PUBLIC_HEADERS}
    ${SOURCES}
)

set_target_properties(${LIB_NAME} PROPERTIES
  PUBLIC_HEADER "${LIB_PUBLIC_HEADERS}"
)

target_link_libraries(${LIB_NAME} vsg::vsg osgEarth ${OpenSceneGraph_LIBRARIES})

target_compile_definitions(${LIB_NAME} PRIVATE vsgXchange_FOUND)
target_link_libraries(${LIB_NAME} vsgXchange::vsgXchange)

SET(CONCAT_COMMAND ${CMAKE_SOURCE_DIR}/CMakeModules/ConcatFiles.cmake)
# Our GLSL "library" of functions that are made available in vertex and fragment shaders
SET(GLSL_FILES voeSDK.glsl)
list_transform_prepend(GLSL_FILES "${CMAKE_CURRENT_SOURCE_DIR}/")

set(GLSLC $ENV{VULKAN_SDK}/bin/glslc)

macro(make_shader SHADER_SOURCE SHADER_OUTPUT)
  add_custom_command(OUTPUT ${SHADER_OUTPUT}
  COMMAND "${CMAKE_COMMAND}" -P ${CONCAT_COMMAND} ${SHADER_SOURCE} ${GLSL_FILES} ${SHADER_OUTPUT}
  DEPENDS ${SHADER_SOURCE} ${GLSL_FILES})

  add_custom_command(OUTPUT ${SHADER_OUTPUT}.spv
  COMMAND ${GLSLC} -I${CMAKE_CURRENT_SOURCE_DIR} -o ${SHADER_OUTPUT}.spv  ${SHADER_OUTPUT}
  DEPENDS ${SHADER_OUTPUT} ${CMAKE_CURRENT_SOURCE_DIR}/VoeSDK.h)
endmacro(make_shader)

make_shader(${CMAKE_CURRENT_SOURCE_DIR}/reverse_depth.vert ${CMAKE_CURRENT_BINARY_DIR}/elevation.vert)
make_shader(${CMAKE_CURRENT_SOURCE_DIR}/elevation_main.frag ${CMAKE_CURRENT_BINARY_DIR}/elevation.frag)

set(TARGET_SPIRV
  ${CMAKE_CURRENT_BINARY_DIR}/elevation.vert.spv
  ${CMAKE_CURRENT_BINARY_DIR}/elevation.frag.spv
  )

add_custom_target(voe_shaders ALL DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/elevation.vert.spv ${CMAKE_CURRENT_BINARY_DIR}/elevation.frag.spv)

INCLUDE(ModuleInstall OPTIONAL)

install(FILES ${TARGET_SPIRV}
  DESTINATION ${VSGEARTH_DATA_DIR})

install(TARGETS vsgEarth EXPORT vsgEarthTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${LIB_NAME}"
  )
