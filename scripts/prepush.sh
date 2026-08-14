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
# prod env names don't follow one consistent pattern.
get_prod_env() {
    case "$1" in
        acu)  echo "acu-prod" ;;
        ccu)  echo "ccu-prod" ;;
        vcf)  echo "vcf-prod" ;;
        vcr)  echo "vcr-prod" ;;
        dash) echo "dash-dfu-prod" ;;
    esac
}

# Maps a friendly subsystem name to its native *_unit_tests environment(s).
# Space-separated — a subsystem can have more than one (e.g. a systems
# suite and an interfaces suite). Not every subsystem has to have both;
# just omit whichever one doesn't exist yet. Dashboard has no entry
# since no native test environment exists for it.
get_test_envs() {
    case "$1" in
        acu) echo "acu_unit_tests" ;;
        ccu) echo "ccu_unit_tests" ;;
        vcf) echo "vcf_systems_unit_tests" ;;   # add vcf_interfaces_unit_tests here once it exists
        vcr) echo "vcr_unit_tests" ;;
    esac
}

# Maps a friendly subsystem name to the pio test --filter value matching
# each of its test environments, in the same order as get_test_envs.
# This must line up 1:1 with get_test_envs's output — used only for the
# informational --filter passed alongside pio run's build.
get_test_filters() {
    case "$1" in
        acu) echo "acu/test_systems" ;;
        ccu) echo "ccu/test_systems" ;;
        vcf) echo "vcf/test_systems" ;;
        vcr) echo "vcr/test_systems" ;;
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
    #
    # We build + run the compiled test binary directly rather than using
    # `pio test`, because pio test's discovery pass runs before
    # scripts/set_directory.py redirects PROJECT_TEST_DIR, which makes
    # it unreliable in this monorepo's layout. `pio run` + executing the
    # binary directly gives the same result without that discovery step.
    # See docs/unit-testing-structure.md for the full explanation.
    if [ "$SUBSYSTEM" != "dash" ]; then
        TEST_ENVS="$(get_test_envs "$SUBSYSTEM")"
        for TEST_ENV in $TEST_ENVS; do
            echo "→ Building tests for $SUBSYSTEM ($TEST_ENV)..."
            if ! pio run -e "$TEST_ENV"; then
                FAILED+=("$SUBSYSTEM test build ($TEST_ENV)")
                continue
            fi

            echo "→ Running tests for $SUBSYSTEM ($TEST_ENV)..."
            if ! "./.pio/build/$TEST_ENV/program"; then
                FAILED+=("$SUBSYSTEM test ($TEST_ENV)")
            fi
        done
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