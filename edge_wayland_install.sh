#!/bin/bash

# Function to log messages with timestamps
log_message() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1"
}

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to backup a file with timestamp
backup_file() {
    local file="$1"
    if [ -f "$file" ]; then
        local backup="${file}.backup.$(date '+%Y%m%d_%H%M%S')"
        if cp "$file" "$backup"; then
            log_message "Created backup: $backup"
            return 0
        else
            log_message "ERROR: Failed to create backup of $file"
            return 1
        fi
    fi
    return 0
}

# Function to check if we're running as root
check_root() {
    if [ "$EUID" -eq 0 ]; then
        log_message "ERROR: This script should not be run as root"
        exit 1
    fi
}

# Function to verify Edge installation
check_edge_installation() {
    if ! command_exists microsoft-edge-stable; then
        log_message "ERROR: Microsoft Edge is not installed"
        log_message "Please install Microsoft Edge first"
        exit 1
    fi
}

# Function to create directory safely
create_directory_safe() {
    local dir="$1"
    if [ ! -d "$dir" ]; then
        if mkdir -p "$dir"; then
            log_message "Created directory: $dir"
        else
            log_message "ERROR: Failed to create directory: $dir"
            return 1
        fi
    fi
    return 0
}

# Function to modify desktop entry actions
modify_desktop_actions() {
    local file="$1"
    local flags="$2"
    
    # Update or add Actions section
    sed -i '/\[Desktop Action new-window\]/,$d' "$file"
    cat >> "$file" << EOL

[Desktop Action new-window]
Name=New Window
Exec=/usr/bin/microsoft-edge-stable $flags

[Desktop Action new-private-window]
Name=New InPrivate Window
Exec=/usr/bin/microsoft-edge-stable $flags --inprivate
EOL
}

# Main setup function
setup_edge_wayland() {
    local success=true
    local desktop_file="/usr/share/applications/microsoft-edge.desktop"
    local user_desktop_dir="$HOME/.local/share/applications"
    local user_desktop_file="$user_desktop_dir/microsoft-edge.desktop"
    local flags="--enable-features=UseOzonePlatform --ozone-platform=wayland"
    
    # 1. Create user-specific desktop entry
    log_message "Setting up user-specific desktop entry..."
    if ! create_directory_safe "$user_desktop_dir"; then
        success=false
    else
        # Copy system desktop file if it exists
        if [ -f "$desktop_file" ]; then
            if ! cp "$desktop_file" "$user_desktop_file"; then
                log_message "ERROR: Failed to copy desktop file"
                success=false
            else
                # Modify the Exec line and ensure visibility
                if ! sed -i "s|^Exec=.*|Exec=/usr/bin/microsoft-edge-stable $flags %U|" "$user_desktop_file"; then
                    log_message "ERROR: Failed to modify Exec line"
                    success=false
                elif ! grep -q "^NoDisplay=" "$user_desktop_file"; then
                    echo "NoDisplay=false" >> "$user_desktop_file"
                fi
                
                # Update Actions
                if ! modify_desktop_actions "$user_desktop_file" "$flags"; then
                    log_message "ERROR: Failed to modify desktop actions"
                    success=false
                else
                    # Update desktop database to ensure visibility
                    update-desktop-database "$user_desktop_dir" 2>/dev/null
                    log_message "Successfully modified desktop entry"
                fi
            fi
        else
            # Create new desktop file if system file doesn't exist
            cat > "$user_desktop_file" << EOL
[Desktop Entry]
Version=1.0
Name=Microsoft Edge (Wayland)
Comment=Access the Internet
GenericName=Web Browser
Exec=/usr/bin/microsoft-edge-stable $flags %U
StartupNotify=true
Terminal=false
Icon=microsoft-edge
Type=Application
Categories=Network;WebBrowser;
MimeType=application/pdf;application/rdf+xml;application/rss+xml;application/xhtml+xml;application/xhtml_xml;application/xml;image/gif;image/jpeg;image/png;image/webp;text/html;text/xml;x-scheme-handler/http;x-scheme-handler/https;x-scheme-handler/mailto;
Actions=new-window;new-private-window;
NoDisplay=false

[Desktop Action new-window]
Name=New Window
Exec=/usr/bin/microsoft-edge-stable $flags

[Desktop Action new-private-window]
Name=New InPrivate Window
Exec=/usr/bin/microsoft-edge-stable $flags --inprivate
EOL
            if [ ! -f "$user_desktop_file" ]; then
                log_message "ERROR: Failed to create new desktop file"
                success=false
            else
                # Update desktop database to ensure visibility
                update-desktop-database "$user_desktop_dir" 2>/dev/null
                log_message "Successfully created new desktop entry"
            fi
        fi
    fi

    # 2. Create custom launcher script
    log_message "Setting up custom launcher script..."
    local script_dir="$HOME/.local/bin"
    local script_path="$script_dir/edge-wayland"
    
    if create_directory_safe "$script_dir"; then
        cat > "$script_path" << EOL
#!/bin/bash
# Custom Microsoft Edge Wayland launcher
# Created: $(date '+%Y-%m-%d %H:%M:%S')

if command -v microsoft-edge-stable >/dev/null 2>&1; then
    exec microsoft-edge-stable $flags "\$@"
else
    echo "ERROR: Microsoft Edge is not installed"
    exit 1
fi
EOL

        if [ -f "$script_path" ]; then
            chmod +x "$script_path"
            log_message "Created custom launcher script: $script_path"
        else
            log_message "ERROR: Failed to create launcher script"
            success=false
        fi
    else
        success=false
    fi

    # 3. Create command-line flags configuration
    log_message "Setting up command-line flags..."
    local flags_dir="$HOME/.config/microsoft-edge-flags"
    local flags_file="$flags_dir/command-line-flags"
    
    if create_directory_safe "$flags_dir"; then
        if echo "$flags" > "$flags_file"; then
            log_message "Created flags file: $flags_file"
        else
            log_message "ERROR: Failed to create flags file"
            success=false
        fi
    else
        success=false
    fi

    # Add PATH update to shell RC file if needed
    local rc_file="$HOME/.$(basename "$SHELL")rc"
    if [ -f "$rc_file" ]; then
        if ! grep -q "$HOME/.local/bin" "$rc_file"; then
            echo "export PATH=\"\$HOME/.local/bin:\$PATH\"" >> "$rc_file"
            log_message "Updated PATH in $rc_file"
        fi
    fi

    # Final status
    if [ "$success" = true ]; then
        log_message "Setup completed successfully!"
        log_message "Please log out and log back in for PATH changes to take effect"
        log_message "You can now run Edge with Wayland support using: edge-wayland"
    else
        log_message "Setup completed with some errors. Please check the log messages above."
        return 1
    fi
}

# Main execution
main() {
    log_message "Starting Microsoft Edge Wayland setup..."
    
    # Initial checks
    check_root
    check_edge_installation

    # Create backup of existing configurations
    backup_file "$HOME/.local/share/applications/microsoft-edge.desktop"
    backup_file "$HOME/.local/bin/edge-wayland"
    backup_file "$HOME/.config/microsoft-edge-flags/command-line-flags"

    # Run setup
    setup_edge_wayland
}

# Execute main with error handling
if main; then
    exit 0
else
    log_message "Script execution failed"
    exit 1
fi