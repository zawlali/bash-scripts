#!/bin/bash

# Check if jq is installed
if ! command -v jq &> /dev/null; then
    echo "Error: jq is not installed. Please install it first:"
    echo "Ubuntu/Debian: sudo apt-get install jq"
    echo "macOS: brew install jq"
    exit 1
fi

# Initialize variables
command_mode=false
message=""

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -c|--command)
            command_mode=true
            shift
            ;;
        *)
            if [ -z "$message" ]; then
                message="$1"
            else
                message="$message $1"
            fi
            shift
            ;;
    esac
done

# Check if a message was provided
if [ -z "$message" ]; then
    echo "Usage: xai [-c] <message>"
    echo "Options:"
    echo "  -c, --command    Output only the command(s)"
    exit 1
fi

# Set system content based on mode
if [ "$command_mode" = true ]; then
    system_content="output only the command(s) requested"
else
    system_content="You are a helpful assistant."
fi

# Your xAI API key - you should store this more securely in practice
API_KEY="xai-t9lvZ2RNzzi5Y98UmbG3qEJztMbUz2tWJJeKGNiBDCJea5BPSwcZkt7iUhcN3XohsFXb2HaoQSBGdKxs"

# Make the API call and store the response
response=$(curl -s https://api.x.ai/v1/chat/completions \
    -H "Content-Type: application/json" \
    -H "Authorization: Bearer $API_KEY" \
    -d '{
    "messages": [
        {
            "role": "system",
            "content": "'"$system_content"'"
        },
        {
            "role": "user",
            "content": "'"$message"'"
        }
    ],
    "model": "grok-beta",
    "stream": false,
    "temperature": 0.7
}')

# Get the AI response
ai_response=$(echo "$response" | jq -r '.choices[0].message.content')

# If in command mode, process the response
if [ "$command_mode" = true ]; then
    # Remove ```bash, ```sh, and ``` markers and trim whitespace
    cleaned_response=$(echo "$ai_response" | sed -E 's/^```(bash|sh)?\s*//; s/^```\s*//; s/```\s*$//' | sed '/^sh$/d')
    
    # Print the cleaned response
    echo "$cleaned_response"
    
    # Count number of non-empty lines
    line_count=$(echo "$cleaned_response" | grep -v '^[[:space:]]*$' | wc -l)
    
    # Check if it's a single line and doesn't start with ```
    if [ "$line_count" -eq 1 ] && [[ "$cleaned_response" != \`\`\`* ]]; then
        command_to_run=$(echo "$cleaned_response" | tr -d '\r' | awk 'NF')
        # Check if it looks like a bash command (no markdown formatting)
        if [[ ! "$command_to_run" =~ ^\# ]] && [[ ! "$command_to_run" =~ ^- ]] && [[ ! "$command_to_run" =~ ^\* ]]; then
            echo -e "\nDo you want to run this command? (Press Enter to proceed, Ctrl+C to cancel)"
            read -r
            eval "$command_to_run"
        fi
    fi
else
    # In normal mode, just print the response
    echo "$ai_response"
fi
