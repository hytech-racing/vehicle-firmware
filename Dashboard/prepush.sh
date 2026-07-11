#!/bin/bash
set -o pipefail
pio run -e H750_dfu
pio check -e H750_dfu --fail-on-defect high