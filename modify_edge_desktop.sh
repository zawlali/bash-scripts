#!/bin/bash

# Find and modify .desktop files for Microsoft Edge
desktop_files=$(find /usr/share/applications ~/.local/share/applications -name "msedge*.desktop")

for file in $desktop_files; do
    if [[ "$file" == "/usr/share/applications/"* ]]; then
        sudo sed -i '/^Exec/s/$/ --enable-features=UseOzonePlatform --ozone-platform=wayland/' "$file"
    else
        sed -i '/^Exec/s/$/ --enable-features=UseOzonePlatform --ozone-platform=wayland/' "$file"
    fi
done

echo "Modification complete for all detected Microsoft Edge .desktop files."
