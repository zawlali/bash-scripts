#!/bin/bash

# Installation script for Cursor command

# Define the paths
INSTALL_DIR="/home/zawl/Applications"
LAUNCHER_SCRIPT="$INSTALL_DIR/launch-cursor.v2.sh"
COMMAND_NAME="cursor"
SYSTEM_BIN="/usr/local/bin/$COMMAND_NAME"

# Create launcher script
cat > "$LAUNCHER_SCRIPT" << 'EOL'
#!/bin/bash

CURSOR_DIR="/home/zawl/Applications"
LATEST_CURSOR=$(ls -t "$CURSOR_DIR"/cursor-*.AppImage | head -n1)

if [ -n "$LATEST_CURSOR" ]; then
    if [ $# -eq 0 ]; then
        # No arguments, open in current directory
        exec "$LATEST_CURSOR" "$(pwd)"
    else
        # Open in specified directory
        exec "$LATEST_CURSOR" "$1"
    fi
else
    echo "No Cursor AppImage found in $CURSOR_DIR"
    exit 1
fi
EOL

# Make launcher script executable
chmod +x "$LAUNCHER_SCRIPT"

# Create system-wide command
if [ -L "$SYSTEM_BIN" ]; then
    sudo rm "$SYSTEM_BIN"
fi
sudo ln -s "$LAUNCHER_SCRIPT" "$SYSTEM_BIN"

echo "Installation complete! You can now use 'cursor' or 'cursor .' command anywhere in the system."
