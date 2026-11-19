# The bare-Teensy example uses only UART, without compiling the rest of VCR's
# interfaces library (which depends on CAN, Ethernet, and board peripherals).
Import("env")

# The tester setup generates matching nanopb messages here before compilation.
from pathlib import Path

generated = Path(env.subst("$PROJECT_DIR/.pio/uart_bench_generated"))
if not (generated / "uart_bench.pb.h").exists():
    raise RuntimeError("Run drivebrain/tests/hil/uart/tester/uart_bench.py setup (or generate_messages.py --nanopb) first")
env.Append(CPPPATH=[str(generated), "$PROJECT_LIBDEPS_DIR/$PIOENV/Nanopb"])
env.BuildSources(
    "$BUILD_DIR/uart_decoder",
    "$PROJECT_DIR/VCR/lib/interfaces/src",
    src_filter=["-<*>", "+<UARTCommandDecoder.cpp>"],
)
env.BuildSources("$BUILD_DIR/uart_bench_messages", str(generated), src_filter=["+<*.pb.c>"])
env.BuildSources("$BUILD_DIR/uart_bench_firmware", "$PROJECT_DIR/VCR/test/uart_bench/firmware", src_filter=["+<*.cpp>"])
