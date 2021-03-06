cmake_minimum_required(VERSION 3.15.0)

set(CMAKE_CXX_STANDARD 11)

set(TARGET EDCircle)

add_executable(${TARGET})

if(NOT DEFINED OPENCV_DIR)
    message(FATAL_ERROR "Set the OPENCV_DIR variable.")
endif()

if(${CMAKE_HOST_UNIX})
    target_include_directories(${TARGET} PRIVATE "${OPENCV_DIR}/include")
    target_link_directories(${TARGET} PRIVATE ${OPENCV_DIR}/lib)
    target_link_libraries(${TARGET} PRIVATE opencv_core opencv_imgproc)
elseif(${CMAKE_HOST_WIN32})
    if(NOT DEFINED OPENCV_VERSION)
        message(FATAL_ERROR "Set the OPENCV_VERSION variable. For example, if you want to use opencv-3.4.7, set OPENCV_VERSION as 347.")
    endif()

    if(${CMAKE_GENERATOR_PLATFORM})
        if(${CMAKE_GENERATOR_PLATFORM} STREQUAL "Win32")
            set(OPENCV_ARCHITECTURE "x86")
        elseif(${CMAKE_GENERATOR_PLATFORM} STREQUAL "x64")
            set(OPENCV_ARCHITECTURE "x64")
        endif()
    else()
        if(${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "^[Xx]86$")
            set(OPENCV_ARCHITECTURE "x86")
        elseif(${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "^[Aa][Mm][Dd]64$")
            set(OPENCV_ARCHITECTURE "x64")
        endif()        
    endif()

    string(REGEX REPLACE "0$" "" OPENCV_TOOLSET "${MSVC_TOOLSET_VERSION}")
    string(SUBSTRING "${OPENCV_TOOLSET}" 0 2 OPENCV_TOOLSET)

    target_include_directories(${TARGET} PRIVATE "${OPENCV_DIR}/include")

    target_link_directories(${TARGET} PRIVATE ${OPENCV_DIR}/${OPENCV_ARCHITECTURE}/vc${OPENCV_TOOLSET}/lib)

    target_link_libraries(${TARGET} PRIVATE debug opencv_world${OPENCV_VERSION}d)
    target_link_libraries(${TARGET} PRIVATE optimized opencv_world${OPENCV_VERSION})
    target_link_libraries(${TARGET} PRIVATE general opencv_world${OPENCV_VERSION})
endif()

target_sources(${TARGET}
    PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/main.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/types.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/edge_drawing.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/edge_drawing.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/edpf.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/edpf.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/ed_line.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/ed_line.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/ed_circle.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/ed_circle.h"	
    "${CMAKE_CURRENT_SOURCE_DIR}/util.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/util.h"	
    "${CMAKE_CURRENT_SOURCE_DIR}/primitives/line.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/primitives/line.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/primitives/edge_segment.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/primitives/edge_segment.cc"	
    "${CMAKE_CURRENT_SOURCE_DIR}/primitives/circle.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/primitives/circle.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/primitives/ellipse.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/primitives/ellipse.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/primitives/arc.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/primitives/arc.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/image/image.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/image/filter.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/image/filter.h"
)

