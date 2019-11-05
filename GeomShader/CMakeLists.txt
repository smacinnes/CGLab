get_filename_component(FOLDERNAME ${CMAKE_CURRENT_LIST_DIR} NAME)

file(GLOB_RECURSE SOURCES "*.cpp")
file(GLOB_RECURSE SHADERS "*.glsl")

add_executable(${FOLDERNAME} ${SOURCES})

target_link_libraries(${FOLDERNAME} ${COMMON_LIBS})

configure_file(vert.glsl ${CMAKE_CURRENT_BINARY_DIR} COPYONLY)
configure_file(frag.glsl ${CMAKE_CURRENT_BINARY_DIR} COPYONLY)
configure_file(geom.glsl ${CMAKE_CURRENT_BINARY_DIR} COPYONLY)
