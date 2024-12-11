#!/bin/bash

CURSOR_DIR="/home/zawl/Applications"
LATEST_CURSOR=$(ls -t "$CURSOR_DIR"/cursor-*.AppImage | head -n1)

if [ -n "$LATEST_CURSOR" ]; then
    if [ $# -eq 0 ]; then
        # No arguments, open in current directory
        exec "$LATEST_CURSOR"
    else
        # Open in specified directory
        exec "$LATEST_CURSOR" "$1"
    fi
else
    echo "No Cursor AppImage found in $CURSOR_DIR"
    exit 1
fi

