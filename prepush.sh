#!/bin/bash
# HyTech Racing Monorepo — pre-push check script
# Run this before pushing to catch issues before CI does
# Usage: ./scripts/prepush.sh [subsystem]
# Examples:
#   ./scripts/prepush.sh          ← runs all subsystems
#   ./scripts/prepush.sh ACU      ← runs ACU only
#   ./scripts/prepush.sh CCU VCF  ← runs CCU and VCF

set -o pipefail

SUBSYSTEMS=${@:-"ACU CCU VCF VCR Dashboard_H750_dfu"}
FAILED=()

for SUBSYSTEM in $SUBSYSTEMS; do
    echo ""
    echo "════════════════════════════════════════"
    echo " Checking $SUBSYSTEM"
    echo "════════════════════════════════════════"

    echo "→ Building $SUBSYSTEM..."
    if ! pio run -e $SUBSYSTEM; then
        FAILED+=("$SUBSYSTEM build")
        continue
    fi

    # Skip tests for Dashboard — no native test environment
    if [ "$SUBSYSTEM" != "Dashboard_H750_dfu" ]; then
        echo "→ Testing ${SUBSYSTEM}..."
        if ! pio test -e ${SUBSYSTEM}_test_systems; then
            FAILED+=("$SUBSYSTEM test")
        fi
    fi

    echo "→ Linting $SUBSYSTEM..."
    if ! pio check -e $SUBSYSTEM --fail-on-defect high; then
        FAILED+=("$SUBSYSTEM lint")
    fi
done

echo ""
echo "════════════════════════════════════════"
if [ ${#FAILED[@]} -eq 0 ]; then
    echo " ✅ All checks passed — safe to push"
else
    echo " ❌ Failed checks:"
    for FAIL in "${FAILED[@]}"; do
        echo "    - $FAIL"
    done
    exit 1
fi