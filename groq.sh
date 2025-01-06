#!/bin/bash

# Check if jq is installed
if ! command -v jq &>/dev/null; then
  echo "Error: jq is not installed. Please install it first:"
  echo "Ubuntu/Debian: sudo apt-get install jq"
  echo "macOS: brew install jq"
  exit 1
fi

# Check for API key
if [ -z "$GROQ_API_KEY" ]; then
  GROQ_API_KEY="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"
fi

# Initialize variables
command_mode=false
message=""

# Parse arguments
while [[ $# -gt 0 ]]; do
  case $1 in
  -c | --command)
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
  echo "Usage: groq [-c] <message>"
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

# Make the API call and store the response
response=$(curl -s "https://api.groq.com/openai/v1/chat/completions" \
  -H "Authorization: Bearer $GROQ_API_KEY" \
  -H "Content-Type: application/json" \
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
    "model": "llama-3.1-70b-versatile",
    "temperature": 0.7
}')

# Check for API errors
if echo "$response" | jq -e 'has("error")' >/dev/null; then
  error_message=$(echo "$response" | jq -r '.error.message')
  echo "Error: $error_message"
  exit 1
fi

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

