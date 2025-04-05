#!/bin/bash

SOURCE_DIR="$(pwd)"
DEST_DIR="/media/tv_shows"
yes_to_all=false

echo "Scanning: $SOURCE_DIR"
echo "Destination: $DEST_DIR"
echo

# Extensions to consider as video
video_exts="mkv|mp4|avi|mov|flv|wmv"

# Loop through subdirectories in current directory
find "$SOURCE_DIR" -mindepth 1 -maxdepth 1 -type d | while read -r dir; do
    # Find the first media file in this folder
    media_file=$(find "$dir" -type f -iregex ".*\.($video_exts)" | head -n 1)
    [ -z "$media_file" ] && continue

    filename=$(basename "$media_file")

    # Parse show name, season, episode using regex
    if [[ "$filename" =~ ^([^.]+(\.[^.]+)*)\.S([0-9]{2})E([0-9]{2}) ]]; then
        raw_show_name="${BASH_REMATCH[1]}"
        season="${BASH_REMATCH[3]}"
        episode="${BASH_REMATCH[4]}"
        extension="${filename##*.}"

        # Convert dots to spaces in show name
        show_name=$(echo "$raw_show_name" | tr '.' ' ')
        season_folder="Season $season"
        new_filename="$show_name - S${season}E${episode}.$extension"
        dest_path="$DEST_DIR/$show_name/$season_folder"

        mkdir -p "$dest_path"
        full_dest="$dest_path/$new_filename"

        if [ -f "$full_dest" ]; then
            if ! $yes_to_all; then
                echo "File already exists: $full_dest"
                echo -n "Overwrite [o], Skip [s], Cancel [c], Yes to all [y]? "
                read -r choice

                case "$choice" in
                    o|O) echo "Overwriting..." ;;
                    s|S) echo "Skipping..."; continue ;;
                    c|C) echo "Cancelled."; exit 1 ;;
                    y|Y) yes_to_all=true; echo "Overwriting all existing files from now." ;;
                    *) echo "Invalid choice. Skipping."; continue ;;
                esac
            fi
        fi

        mv "$media_file" "$full_dest"
        echo "Moved: $filename -> $full_dest"
    else
        echo "Skipping: $filename (pattern not matched)"
    fi
done
