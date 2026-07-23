#!/bin/bash

# This is a local convenience script that you can run before pushing.
# Its purpose is to let you catch build/test/lint failures on your own machine before CI does.
#
# Usage: ./scripts/prepush.sh [subsystem ...]
# Examples:
#   ./scripts/prepush.sh          ← runs all subsystems
#   ./scripts/prepush.sh acu      ← runs ACU only
#   ./scripts/prepush.sh ccu vcf  ← runs CCU and VCF

# Maps a friendly subsystem name to its actual PlatformIO "prod" build
# environment. Explicit lookup rather than string-concatenation, since
# prod env names don't follow one consistent pattern
get_prod_env() {
    case "$1" in
    # "$1" refers to the first argument passed to this function when it's called.
    # "case" works like a switch statement
        acu)       echo "acu-prod" ;;
        ccu)       echo "ccu-prod" ;;
        vcf)       echo "vcf-prod" ;;
        vcr)       echo "vcr-prod" ;;
        dash) echo "dash-dfu-prod" ;;
    esac
}

# Same idea for the native "test_systems" environments. Dashboard has no
# entry since no native test environment exists for it.
get_test_env() {
    case "$1" in
        acu) echo "acu_test_systems" ;;
        ccu) echo "ccu_test_systems" ;;
        vcf) echo "vcf_test_systems" ;;
        vcr) echo "vcr_test_systems" ;;
    esac
}

# ${@:-default}: use script arguments if given, otherwise run every subsystem.
SUBSYSTEMS=${@:-"acu ccu vcf vcr dash"}

FAILED=()

for SUBSYSTEM in $SUBSYSTEMS; do
    echo ""
    echo "════════════════════════════════════════"
    echo " Checking $SUBSYSTEM"
    echo "════════════════════════════════════════"

    PROD="$(get_prod_env "$SUBSYSTEM")"

    # --- Build check ---
    echo "→ Building $SUBSYSTEM ($PROD)..."
    if ! pio run -e "$PROD"; then
        FAILED+=("$SUBSYSTEM build")
        continue
    fi

    # --- Test check ---
    # Skipped for Dashboard (no native test environment). A test failure
    # does NOT `continue` — lint still runs afterward regardless.
    if [ "$SUBSYSTEM" != "dash" ]; then
        TEST="$(get_test_env "$SUBSYSTEM")"
        echo "→ Testing $SUBSYSTEM ($TEST)..."
        if ! pio test -e "$TEST"; then
            FAILED+=("$SUBSYSTEM test")
        fi
    fi

    # --- Lint check ---
    echo "→ Linting $SUBSYSTEM ($PROD)..."
    if ! pio check -e "$PROD" --fail-on-defect high; then
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