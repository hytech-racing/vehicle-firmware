#!/bin/bash

# Build every production board. Run this after editing a shared library
# to confirm each board still compiles.
#
# Usage: ./scripts/checkall.sh

cd "$(dirname "$0")/.." || exit 1

ENVS=(
    acu-prod
    ccu-prod
    vcf-prod
    vcr-prod
    rdc-prod
    dash-dfu-prod
)

FAILED=()

for ENV in "${ENVS[@]}"; do
    echo ""
    echo "════════════════════════════════════════"
    echo " Building $ENV"
    echo "════════════════════════════════════════"
    if ! pio run -e "$ENV"; then
        FAILED+=("$ENV")
    fi
done

echo ""
echo "════════════════════════════════════════"
if [ ${#FAILED[@]} -eq 0 ]; then
    echo " ✅ All boards built"
else
    echo " ❌ Failed builds:"
    for FAIL in "${FAILED[@]}"; do
        echo "    - $FAIL"
    done
    exit 1
fi
