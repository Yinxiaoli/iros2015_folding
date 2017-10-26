cmake_minimum_required(VERSION 3.8)
project(blackbill)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        delegator/delegator.h
        delegator/delegatorchain.h
        delegator/eventhandler2.h
        delegator/property.h
        filepath/filepath.cpp
        filepath/filepath.h
        loader/rawlfsloader.cpp
        loader/rawlfsloader.h
        mathbase/mathbase.h
        random/random.c
        random/random.h
        result/result.h
        thread/criticalsection.cpp
        thread/criticalsection.h
        thread/criticalsectionimpl.cpp
        thread/criticalsectionimpl.h
        thread/thread.cpp
        thread/thread.h
        thread/threadimpl.cpp
        thread/threadimpl.h
        util/endianutil.h)

set(dir ${CMAKE_CURRENT_SOURCE_DIR})
#SET(LIBRARY_OUTPUT_PATH ${dir}/../../build/lib)

add_library(blackbill STATIC ${SOURCE_FILES})
