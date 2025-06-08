#!/bin/bash

# Microsoft Edge Wayland Pacman Hook Setup Script

set -e

echo "Setting up Microsoft Edge Wayland pacman hook..."

# Create hooks directory if it doesn't exist
echo "Creating hooks directory..."
sudo mkdir -p /etc/pacman.d/hooks

# Create the pacman hook
echo "Creating pacman hook..."
sudo tee /etc/pacman.d/hooks/microsoft-edge-wayland.hook > /dev/null << 'EOF'
[Trigger]
Operation = Install
Operation = Upgrade
Type = Package
Target = microsoft-edge-stable-bin
Target = microsoft-edge-stable
Target = microsoft-edge-dev-bin
Target = microsoft-edge-beta-bin

[Action]
Description = Adding Wayland support to Microsoft Edge desktop files
When = PostTransaction
Exec = /usr/local/bin/fix-edge-wayland.sh
EOF

# Create the fix script
echo "Creating fix script..."
sudo tee /usr/local/bin/fix-edge-wayland.sh > /dev/null << 'EOF'
#!/bin/bash

# Fix Microsoft Edge desktop files for Wayland
DESKTOP_FILES=(
    "/usr/share/applications/microsoft-edge.desktop"
    "/usr/share/applications/com.microsoft.Edge.desktop"
)

for file in "${DESKTOP_FILES[@]}"; do
    if [[ -f "$file" ]]; then
        echo "Fixing $file for Wayland..."
        sed -i 's|/usr/bin/microsoft-edge-stable\([^-]\)|/usr/bin/microsoft-edge-stable --ozone-platform=wayland\1|g' "$file"
        echo "Fixed $file"
    fi
done

echo "Microsoft Edge Wayland fix applied successfully"
EOF

# Make the script executable
echo "Making script executable..."
sudo chmod +x /usr/local/bin/fix-edge-wayland.sh

# Apply the fix immediately
echo "Applying Wayland fix to current desktop files..."
sudo /usr/local/bin/fix-edge-wayland.sh

echo ""
echo "✅ Setup complete!"
echo "The pacman hook will now automatically apply Wayland flags to Microsoft Edge after updates."
echo ""
echo "Files created:"
echo "  - /etc/pacman.d/hooks/microsoft-edge-wayland.hook"
echo "  - /usr/local/bin/fix-edge-wayland.sh"
