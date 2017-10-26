cmake_minimum_required(VERSION 3.8)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_C_COMPILER "gcc")
set(CMAKE_CXX_COMPILER "g++")

project(2dregistration_v2)

SET(CMAKE_CXX_FLAGS "-lglut -lGL -lGLU")
SET(CMAKE_EXE_LINKER_FLAGS  "-lglut -lGL -lGLU")


set(SOURCE_FILES
        datatypes.cpp
        datatypes.h
        distancefield.cpp
        distancefield.h
        FoldPlanner.cpp
        FoldPlanner.h
        garment_template.cpp
        garment_template.h
        ImagePreprocessor.cpp
        ImagePreprocessor.h
        ImageUtil.cpp
        ImageUtil.h
        main.cpp
        mesh2d.cpp
        mesh2d.h
        ObjectSegmenter.cpp
        ObjectSegmenter.h
        registration.cpp
        registration.h)

find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

add_executable(2dregistration_v2 ${SOURCE_FILES})

set(dir ${CMAKE_CURRENT_SOURCE_DIR})
# SET(LIBRARY_OUTPUT_PATH ${dir}/../lib)


#set abs path
set(AERIALEYE /home/yinxiao/folding/lib/aerialeye)
set(BLACKBILL /home/yinxiao/folding/lib/blackbill)
set(BLUETAIL /home/yinxiao/folding/lib/bluetail)

#link_directories(${AERIALEYE} ${BLACKBILL} ${BLUETAIL})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../lib/ 
        ${AERIALEYE} ${BLACKBILL} ${BLUETAIL}
        ${OPENGL_INCLUDE_DIR} ${GLUT_INCLUDE_DIR})


target_link_libraries(2dregistration_v2 
        ${BLUETAIL}/libbluetail.a ${BLACKBILL}/libblackbill.a ${AERIALEYE}/libaerialeye.a
        ${OpenCV_LIBS} ${OPENGL_LIBRARY} ${GLUT_INCLUDE_DIR} ${GLUT_glut_LIBRARY}
        )
