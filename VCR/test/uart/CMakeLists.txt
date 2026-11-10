cmake_minimum_required(VERSION 3.20)
project(vcr_uart_decoder_tests LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include(FetchContent)
# Match platformio.ini's nanopb and HT_proto dependencies exactly.
FetchContent_Declare(nanopb
    URL https://github.com/nanopb/nanopb/archive/refs/tags/0.4.9.1.tar.gz
    URL_HASH SHA256=4575944a468718ef25f05eb01d994364650b581563089a9841986bb1e460eac3
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_Declare(vcr_proto
    URL https://github.com/hytech-racing/HT_proto/releases/download/2026-06-14T06_30_48/hytech_msgs_pb_lib.tar.gz
    URL_HASH SHA256=2c7675c2c6b61012527c96d189d32438077e7a790e53cd6ff20b22a8164f5ed3
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
set(nanopb_BUILD_GENERATOR OFF CACHE BOOL "" FORCE)
set(nanopb_BUILD_RUNTIME ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(nanopb vcr_proto)
add_library(command_decoder
    ../../lib/uart/src/UARTCommandDecoder.cpp
    ${vcr_proto_SOURCE_DIR}/base_msgs.pb.c
    ${vcr_proto_SOURCE_DIR}/hytech_msgs.pb.c)
target_include_directories(command_decoder PUBLIC ../../lib/uart/include ${vcr_proto_SOURCE_DIR})
target_link_libraries(command_decoder PUBLIC protobuf-nanopb-static)
target_compile_options(command_decoder PRIVATE -Wall -Wextra -Wpedantic)
add_executable(uart_decoder_test uart_decoder_test.cpp)
target_include_directories(uart_decoder_test PRIVATE mocks)
target_link_libraries(uart_decoder_test PRIVATE command_decoder)
target_compile_definitions(uart_decoder_test PRIVATE FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures")
enable_testing()
add_test(NAME uart_decoder COMMAND uart_decoder_test)
