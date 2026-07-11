#!/bin/bash
set -o pipefail
pio run -e teensy41
pio test -e test_systems_env
pio check -e teensy41 --fail-on-defect high