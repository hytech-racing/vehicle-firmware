#!/bin/bash
# Generate compile_commands.json for IntelliSense
# Usage: ./scripts/getcompilecommands.sh <subsystem> [test]
# Examples:
#   ./scripts/getcompilecommands.sh ACU        ← main build
#   ./scripts/getcompilecommands.sh ACU test   ← test build

SUBSYSTEM=${1:-ACU}
MODE=${2:-""}

if [ "$MODE" = "test" ]; then
    ENV="${SUBSYSTEM}_test_systems"
else
    ENV="$SUBSYSTEM"
fi

echo "Generating compile commands for $ENV..."
pio run -e $ENV
pio run -t compiledb -e $ENV
echo "Done. compile_commands.json at .pio/build/$ENV/compile_commands.json"