#!/bin/bash
# HyTech Racing Monorepo — flash script
# Usage: ./scripts/flash.sh <subsystem>
# Examples:
#   ./scripts/flash.sh ACU
#   ./scripts/flash.sh CCU
#   ./scripts/flash.sh Dashboard_H750_dfu

SUBSYSTEM=$1

if [ -z "$SUBSYSTEM" ]; then
    echo "Usage: $0 <subsystem>"
    echo "Available: ACU CCU VCF VCR Dashboard_H750_dfu Dashboard_H750_stlink"
    exit 1
fi

echo "→ Building and flashing $SUBSYSTEM..."
pio run -e $SUBSYSTEM --target upload