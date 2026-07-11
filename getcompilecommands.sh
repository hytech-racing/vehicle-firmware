#!/bin/bash
set -o pipefail
pio run -e teensy41
pio run -t compiledb -e teensy41