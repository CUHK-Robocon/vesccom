cmake_minimum_required(VERSION 3.25)

project(vesccom)

if(POLICY CMP0167)
    # Use upstream BoostConfig instead of deprecated FindBoost.
    cmake_policy(SET CMP0167 NEW)
endif()
find_package(Boost REQUIRED)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(vesccom STATIC
    src/serial.cpp
    src/socketcan.cpp
    src/vesc.cpp
)
target_include_directories(vesccom
    PUBLIC
        include
    PRIVATE
        src
)
target_link_libraries(vesccom
    PUBLIC
        Boost::boost
)

add_executable(vesccom_test
    test/main.cpp
)
target_link_libraries(vesccom_test vesccom)
