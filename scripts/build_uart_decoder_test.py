# The bare-Teensy example uses only UART, without compiling the rest of VCR's
# interfaces library (which depends on CAN, Ethernet, and board peripherals).
Import("env")

# BuildSources creates its objects before PlatformIO adds discovered library
# include paths. These are the pinned dependencies declared by vcr_uart_test.
env.Append(
    CPPPATH=[
        "$PROJECT_LIBDEPS_DIR/$PIOENV/hytech_msgs_pb_lib",
        "$PROJECT_LIBDEPS_DIR/$PIOENV/Nanopb",
    ]
)

env.BuildSources(
    "$BUILD_DIR/uart_decoder",
    "$PROJECT_DIR/VCR/lib/interfaces/src",
    src_filter=["-<*>", "+<UARTCommandDecoder.cpp>"],
)
