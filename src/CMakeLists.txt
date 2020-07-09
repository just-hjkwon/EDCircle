cmake_minimum_required(VERSION 3.15.0)

set(CMAKE_CXX_STANDARD 11)

set(TARGET EDCircle)

add_executable(${TARGET})

target_sources(${TARGET}
    PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/main.cc"
)