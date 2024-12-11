#!/bin/bash

# Function to extract subscribers from node info
get_subscribers() {
    local node_name=$1
    local in_subscribers=0
    
    # Get node info and process it line by line
    ros2 node info $node_name | while IFS= read -r line; do
        # Check if we're entering the Subscribers section
        if [[ "$line" == "  Subscribers:" ]]; then
            in_subscribers=1
            continue
        fi
        
        # Check if we're entering a different section (Publishers, Service Servers, etc.)
        if [[ "$line" =~ ^[[:space:]]*[A-Za-z]+: && "$line" != "  Subscribers:" ]]; then
            in_subscribers=0
            continue
        fi
        
        # If we're in the Subscribers section and line contains a topic
        if [[ $in_subscribers -eq 1 && "$line" =~ ^[[:space:]]+/ ]]; then
            # Extract just the topic name (remove message type)
            topic=$(echo "$line" | awk -F: '{print $1}' | tr -d ' ')
            echo "$topic"
        fi
    done
}

# Array of nodes to check
nodes=("/rviz" "/foxglove_bridge")

# Print header
echo "Unique Subscribed Topics:"
echo "-----------------------"

# Create a temporary file to store all topics
temp_file=$(mktemp)

# Get subscribers for each node and save to temp file
for node in "${nodes[@]}"; do
    if ros2 node list | grep -q "$node"; then
        get_subscribers "$node" >> "$temp_file"
    else
        echo "Node $node not found!"
    fi
done

# Get unique topics and store them in an array
mapfile -t unique_topics < <(sort "$temp_file" | uniq)

# Clean up temp file
rm "$temp_file"

# Print the unique topics
printf '%s\n' "${unique_topics[@]}"

echo -e "\nRecording bag file..."

# Create the ros2 bag record command with all unique topics
record_cmd="ros2 bag record -o $HOME/rosbags/$(date +%Y-%m-%d_%H-%M-%S) -s mcap"
for topic in "${unique_topics[@]}"; do
    record_cmd+=" $topic"
done

# Execute the record command
echo "Executing: $record_cmd"
$record_cmd