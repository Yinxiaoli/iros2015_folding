cmake_minimum_required(VERSION 3.8)

set(CMAKE_C_COMPILER "gcc")
set(CMAKE_CXX_COMPILER "g++")

set(CMAKE_CXX_STANDARD 11)

project(aerialeye)

SET(CMAKE_CXX_FLAGS "-lglut -lGL -lGLU -lGLEW")
SET(CMAKE_EXE_LINKER_FLAGS  "-lglut -lGL -lGLU -lGLEW")

set(SOURCE_FILES
        glbutton.cpp
        glbutton.h
        gldrawobject.cpp
        gldrawobject.h
        glframework.cpp
        glframework.h
        glinit.h
        glpanel.cpp
        glpanel.h
        glprogressbar.cpp
        glprogressbar.h)

set(dir ${CMAKE_CURRENT_SOURCE_DIR})
#SET(LIBRARY_OUTPUT_PATH ${dir}/../../build/lib)

find_package(OpenGL REQUIRED)
if(NOT OPENGL_FOUND)
    message(ERROR " OPENGL not found!")
endif(NOT OPENGL_FOUND)


find_package(GLUT REQUIRED)
if(NOT GLUT_FOUND)
    message(ERROR " GLUT not found!")
endif(NOT GLUT_FOUND)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../ ${OPENGL_INCLUDE_DIR} ${GLUT_INCLUDE_DIR})

add_library(aerialeye STATIC
        ${SOURCE_FILES}
        )

target_link_libraries(aerialeye ${CMAKE_CURRENT_SOURCE_DIR}/../blackbill/libblackbill ${OPENGL_INCLUDE_DIR} ${GLUT_INCLUDE_DIR} ${OPENGL_LIBRARY} ${GLUT_glut_LIBRARY})

