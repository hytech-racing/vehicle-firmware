#!/bin/bash
set -o pipefail
pio run -e test_systems_env
pio run -t compiledb -e test_systems_env