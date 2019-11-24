cmake_minimum_required(VERSION 2.8)
project(CGLab)

#--- Load the common configuration
include(common/config.cmake)
include(common/Eigen.cmake)
include(common/OpenGP.cmake)

#--- OpenGL configuration
include(common/OpenGL.cmake)
include(common/GLEW.cmake)
include(common/GLFW.cmake)

#--- Subprojects
#add_subdirectory(vector)
#add_subdirectory(raytrace)
#add_subdirectory(rasterize)
add_subdirectory(glsl)
#add_subdirectory(antialiasing)
add_subdirectory(transform)
add_subdirectory(glsltexture)
add_subdirectory(GeomShader)
add_subdirectory(framebuffer2d)
add_subdirectory(mouse)
add_subdirectory(mesh3D)
add_subdirectory(terrains)
add_subdirectory(virtuallandscape)
add_subdirectory(PerlinNoise)


