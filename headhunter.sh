#!/bin/bash

headhunter() {
    if [ $# -eq 0 ]; then
        echo "Usage: headhunter <search_term>"
        return 1
    fi

    search_term="$1"
    pids=$(ps aux | grep "$search_term" | grep -v grep | awk '{print $2}')

    if [ -z "$pids" ]; then
        echo "No processes found matching '$search_term'"
    else
        echo "Killing processes matching '$search_term':"
        echo "$pids" | xargs echo
        echo "$pids" | xargs -r kill -9
        echo "Processes killed."
    fi
}
