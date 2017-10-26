cmake_minimum_required(VERSION 3.8)
project(bluetail)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        camera/camera.h
        image/image.h
        image/imageiobmp.h
        image/imageiohdr.h
        image/imageioutil.h
        loader/lwoloader.h
        loader/objectloader.h
        material/material.h
        objectmodel/triangleset.h
        ray/ray.h)

set(dir ${CMAKE_CURRENT_SOURCE_DIR})
#SET(LIBRARY_OUTPUT_PATH ${dir}/../../build/lib)
add_library(bluetail STATIC ${SOURCE_FILES})

set_target_properties(bluetail PROPERTIES LINKER_LANGUAGE CXX)


