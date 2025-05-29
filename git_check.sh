#!/bin/bash

# Git Repository Checker and Manager
# Searches for git repositories belonging to 'zawlali' and provides management interface

# Default search directory
SEARCH_DIR="${1:-$HOME}"
REPOS=()
CHECKED_REPOS=()
REPO_STATUSES=()

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to find all git repositories
find_git_repos() {
    echo "Searching for git repositories in $SEARCH_DIR..."
    REPOS=()
    REPO_STATUSES=()
    
    local found_repos_temp=()
    while IFS= read -r -d '' repo; do
        local repo_dir=$(dirname "$repo")
        # Check if remote contains 'zawlali'
        # Ensure we are in a known directory before relative cd
        local current_dir_find_loop=$(pwd)
        cd "$repo_dir" 2>/dev/null || continue
        local remote_url=$(git remote get-url origin 2>/dev/null || echo "")
        if [[ "$remote_url" == *"zawlali"* ]]; then
            found_repos_temp+=("$repo_dir")
            # echo "Found zawlali repo: $repo_dir" # Made quieter for refresh
        fi
        cd "$current_dir_find_loop" # Go back to original directory
    done < <(find "$SEARCH_DIR" -name ".git" -type d -print0 2>/dev/null)

    # Sort repositories by name for consistent order
    IFS=$'\n' REPOS=($(sort <<<"${found_repos_temp[*]}"))
    unset IFS
    
    echo "Found ${#REPOS[@]} repositories belonging to zawlali."

    if [[ ${#REPOS[@]} -gt 0 ]]; then
        echo "Fetching statuses (this may take a moment)..."
        for i in "${!REPOS[@]}"; do
            echo -n "." # Progress indicator
            REPO_STATUSES[$i]=$(get_repo_status "${REPOS[$i]}")
        done
        echo " Status fetch complete."
    fi
}

# Function to get repository status
get_repo_status() {
    local repo_dir="$1"
    cd "$repo_dir" || return 1
    
    # Fetch latest changes
    git fetch origin 2>/dev/null || true
    
    local status=""
    local ahead=$(git rev-list --count @{u}..HEAD 2>/dev/null || echo "0")
    local behind=$(git rev-list --count HEAD..@{u} 2>/dev/null || echo "0")
    local uncommitted=$(git status --porcelain 2>/dev/null | wc -l)
    
    if [[ $uncommitted -gt 0 ]]; then
        status="${status}${RED}UNCOMMITTED${NC} "
    fi
    if [[ $ahead -gt 0 ]]; then
        status="${status}${YELLOW}PUSH($ahead)${NC} "
    fi
    if [[ $behind -gt 0 ]]; then
        status="${status}${BLUE}PULL($behind)${NC} "
    fi
    if [[ -z "$status" ]]; then
        status="${GREEN}UP-TO-DATE${NC}"
    fi
    
    echo -e "$status"
}

# Function to display repository list with checkboxes
display_repos() {
    clear
    echo "=== Git Repository Manager ==="
    echo "Search directory: $SEARCH_DIR"
    echo ""
    
    if [[ ${#REPOS[@]} -eq 0 ]]; then
        echo "No repositories found. Press 'r' to refresh or 'q' to quit."
        return
    fi
    
    for i in "${!REPOS[@]}"; do
        local repo="${REPOS[$i]}"
        local checked=" "
        if [[ " ${CHECKED_REPOS[*]} " =~ " ${i} " ]]; then
            checked="x"
        fi
        
        # Use cached status
        local status="${REPO_STATUSES[$i]}" 
        # Fallback if status somehow empty (should not happen if find_git_repos ran)
        if [[ -z "$status" ]]; then
            status="Fetching..." 
        fi

        printf "[%s] %2d. %-50s %s\n" "$checked" "$((i+1))" "$(basename "$repo")" "$status"
        printf "     %s\n" "$repo"
    done
    
    echo ""
    echo "Commands:"
    echo "  [number] - Toggle repository selection"
    echo "  a - Select all    | n - Select none"
    echo "  s - Show status   | r - Refresh repos"
    echo "  p - Pull selected | P - Push selected"
    echo "  c - Commit selected | q - Quit"
    echo "  U - Select Uncommitted | H - Select Needs Push"
    echo "  B - Select for Rebase (Uncommitted & Pull)"
    echo "  E - Rebase selected"
}

# Function to toggle repository selection
toggle_repo() {
    local index="$1"
    if [[ " ${CHECKED_REPOS[*]} " =~ " ${index} " ]]; then
        # Remove from checked
        CHECKED_REPOS=(${CHECKED_REPOS[@]/$index})
    else
        # Add to checked
        CHECKED_REPOS+=("$index")
    fi
}

# Function to select all repositories
select_all() {
    CHECKED_REPOS=()
    for i in "${!REPOS[@]}"; do
        CHECKED_REPOS+=("$i")
    done
}

# Function to deselect all repositories
select_none() {
    CHECKED_REPOS=()
}

# Function to select repositories with uncommitted changes
select_uncommitted() {
    CHECKED_REPOS=()
    local selected_count=0
    # The actual text part we are looking for in the status string
    local uncommitted_text_pattern="UNCOMMITTED"

    for i in "${!REPOS[@]}"; do
        local current_status="${REPO_STATUSES[$i]}"
        # Check if the plain text "UNCOMMITTED" is part of the status string
        if [[ "$current_status" == *"$uncommitted_text_pattern"* ]]; then
            CHECKED_REPOS+=("$i")
            selected_count=$((selected_count + 1))
        fi
    done

    if [[ $selected_count -gt 0 ]]; then
        echo "Selected $selected_count repositories with uncommitted changes."
    else
        echo "No repositories found with uncommitted changes to select."
    fi
    # Optional: Short pause to see the message if not many repos are listed
    # echo "Press any key to continue..."
    # read -s -n 1
}

# Function to select repositories needing a push
select_needs_push() {
    CHECKED_REPOS=()
    local selected_count=0
    local needs_push_text_pattern="PUSH(" # Match the start of PUSH(count)

    for i in "${!REPOS[@]}"; do
        if [[ "${REPO_STATUSES[$i]}" == *"$needs_push_text_pattern"* ]]; then
            CHECKED_REPOS+=("$i")
            selected_count=$((selected_count + 1))
        fi
    done

    if [[ $selected_count -gt 0 ]]; then
        echo "Selected $selected_count repositories needing a push."
    else
        echo "No repositories found needing a push to select."
    fi
}

# Function to select repositories suitable for rebase (uncommitted and needs pull)
select_for_rebase() {
    CHECKED_REPOS=()
    local selected_count=0
    local uncommitted_text_pattern="UNCOMMITTED"
    local needs_pull_text_pattern="PULL("

    for i in "${!REPOS[@]}"; do
        # Check for both UNCOMMITTED text and PULL( text
        if [[ "${REPO_STATUSES[$i]}" == *"$uncommitted_text_pattern"* && \
              "${REPO_STATUSES[$i]}" == *"$needs_pull_text_pattern"* ]]; then
            CHECKED_REPOS+=("$i")
            selected_count=$((selected_count + 1))
        fi
    done
    
    if [[ $selected_count -gt 0 ]]; then
        echo "Selected $selected_count repositories for rebase (uncommitted & needs pull)."
    else
        echo "No repositories found meeting rebase criteria to select."
    fi
}

# Function to pull selected repositories
pull_selected() {
    echo "Pulling selected repositories..."
    if [[ ${#CHECKED_REPOS[@]} -eq 0 ]]; then
        echo "No repositories selected. Press any key to continue..."
        read -s -n 1
        return
    fi
    for index in "${CHECKED_REPOS[@]}"; do
        local repo="${REPOS[$index]}"
        echo "Pulling $(basename "$repo")..."
        cd "$repo" && git pull origin $(git branch --show-current) # Removed 2>&1 to see output/errors
    done
    echo "Pull completed. Refreshing statuses..."
    find_git_repos # Refresh statuses
    echo "Press any key to continue..."
    read -s -n 1
}

# Function to push selected repositories
push_selected() {
    echo "Pushing selected repositories..."
    if [[ ${#CHECKED_REPOS[@]} -eq 0 ]]; then
        echo "No repositories selected. Press any key to continue..."
        read -s -n 1
        return
    fi
    for index in "${CHECKED_REPOS[@]}"; do
        local repo="${REPOS[$index]}"
        echo "Pushing $(basename "$repo")..."
        cd "$repo" && git push origin $(git branch --show-current) # Removed 2>&1
    done
    echo "Push completed. Refreshing statuses..."
    find_git_repos # Refresh statuses
    echo "Press any key to continue..."
    read -s -n 1
}

# Function to commit selected repositories
commit_selected() {
    if [[ ${#CHECKED_REPOS[@]} -eq 0 ]]; then
        echo "No repositories selected. Press any key to continue..."
        read -s -n 1
        return
    fi
    local changes_made=0
    for index in "${CHECKED_REPOS[@]}"; do
        local repo="${REPOS[$index]}"
        cd "$repo" || continue
        
        # Check if there are changes to commit
        if [[ $(git status --porcelain | wc -l) -eq 0 ]]; then
            echo "No changes in $(basename "$repo")"
            continue
        fi
        
        echo "=== Changes in $(basename "$repo") ==="
        git status --short
        echo ""
        git diff --stat
        echo ""
        
        read -p "Enter commit message for $(basename "$repo") (or 'skip' to skip): " commit_msg
        if [[ "$commit_msg" != "skip" && -n "$commit_msg" ]]; then
            git add .
            git commit -m "$commit_msg"
            echo "Committed changes in $(basename "$repo")"
            changes_made=1
        else
            echo "Skipped $(basename "$repo")"
        fi
        echo ""
    done
    echo "Commit process completed."
    if [[ $changes_made -eq 1 ]]; then
        echo "Refreshing statuses..."
        find_git_repos # Refresh statuses if any commit was made
    fi
    echo "Press any key to continue..."
    read -s -n 1
}

# Function to rebase selected repositories
rebase_selected() {
    echo "Rebasing selected repositories..."
    if [[ ${#CHECKED_REPOS[@]} -eq 0 ]]; then
        echo "No repositories selected for rebase. Press any key to continue..."
        read -s -n 1
        return
    fi

    local rebase_summary=()
    local initial_pwd=$(pwd) # Save PWD at the start of the function

    for index in "${CHECKED_REPOS[@]}"; do
        local repo_path="${REPOS[$index]}"
        local repo_name=$(basename "$repo_path")
        echo -e "\n--- Rebasing $repo_name ---"
        
        cd "$repo_path" || { 
            echo -e "${RED}Failed to cd to $repo_path. Skipping.${NC}"; 
            rebase_summary+=("$repo_name: Failed (cd error)"); 
            cd "$initial_pwd"; # Go back before continuing
            continue; 
        }

        local current_branch=$(git branch --show-current 2>/dev/null)
        if [[ -z "$current_branch" ]]; then
            echo -e "${YELLOW}Could not determine current branch for $repo_name. Skipping.${NC}"
            rebase_summary+=("$repo_name: Skipped (no branch)")
            cd "$initial_pwd" # Go back
            continue
        fi

        # Check if remote tracking branch is set
        local remote_tracking_branch=$(git rev-parse --abbrev-ref --symbolic-full-name @{u} 2>/dev/null)
        if [[ -z "$remote_tracking_branch" ]]; then
            echo -e "${YELLOW}No remote tracking branch set for $current_branch in $repo_name. Skipping rebase.${NC}"
            rebase_summary+=("$repo_name: Skipped (no upstream)")
            cd "$initial_pwd" # Go back
            continue
        fi

        local stashed=0
        # Check for uncommitted changes (both staged and unstaged)
        if ! git diff-index --quiet HEAD -- || ! git diff-index --quiet --cached HEAD --; then
            echo "Stashing uncommitted changes in $repo_name..."
            # Stash including untracked files, keeping staged files staged if possible (though rebase might change that)
            if git stash push --include-untracked -m "git_check_rebase_stash_$$"; then
                stashed=1
                echo "Changes stashed."
            else
                echo -e "${RED}Failed to stash changes in $repo_name. Skipping rebase for this repo.${NC}"
                rebase_summary+=("$repo_name: Failed (stash error)")
                cd "$initial_pwd" # Go back
                continue
            fi
        fi

        echo "Fetching origin for $repo_name..."
        git fetch origin 2>/dev/null || echo -e "${YELLOW}Warning: git fetch for $repo_name encountered issues.${NC}"


        echo "Attempting pull --rebase origin/$current_branch for $repo_name..."
        if git pull --rebase origin "$current_branch"; then
            echo -e "${GREEN}Pull --rebase successful for $repo_name.${NC}"
            if [[ $stashed -eq 1 ]]; then
                echo "Attempting to apply stashed changes for $repo_name..."
                if git stash pop; then
                    echo -e "${GREEN}Stashed changes applied successfully for $repo_name.${NC}"
                    rebase_summary+=("$repo_name: Rebased & Stash Popped")
                else
                    echo -e "${YELLOW}Failed to auto-apply stashed changes for $repo_name. Please resolve conflicts manually (git status; git stash list).${NC}"
                    rebase_summary+=("$repo_name: Rebased, Stash Pop CONFLICT")
                fi
            else
                rebase_summary+=("$repo_name: Rebased (no stash needed)")
            fi
        else
            echo -e "${RED}Pull --rebase failed for $repo_name. Please resolve issues manually (e.g., git rebase --abort, resolve conflicts, then try again).${NC}"
            rebase_summary+=("$repo_name: FAILED REBASE")
            if [[ $stashed -eq 1 ]]; then
                echo -e "${YELLOW}Changes were stashed. You might need to 'git stash apply' or 'git stash pop' manually after resolving rebase issues (git stash list).${NC}"
            fi
        fi
        cd "$initial_pwd" # Go back to initial PWD before next iteration
    done

    echo -e "\n--- Rebase Summary ---"
    if [[ ${#rebase_summary[@]} -gt 0 ]]; then
        for summary_item in "${rebase_summary[@]}"; do
            echo "$summary_item"
        done
    else
        echo "No repositories were processed for rebase (e.g., none selected or all skipped)."
    fi
    echo "----------------------"
    
    echo "Rebase process completed. Refreshing statuses..."
    find_git_repos # Refresh statuses
    echo "Press any key to continue..."
    read -s -n 1
}

# Function to show detailed status
show_status() {
    clear
    echo "=== Detailed Repository Status ==="
    for index in "${CHECKED_REPOS[@]}"; do
        local repo="${REPOS[$index]}"
        echo "Repository: $(basename "$repo")"
        echo "Path: $repo"
        cd "$repo" || continue
        git status
        echo "------------------------"
    done
    echo "Press any key to continue..."
    read -n 1
}

# Main menu loop
main_menu() {
    local input_buffer # For multi-digit numeric input
    while true; do
        display_repos
        input_buffer="" 

        # Read first character for command or start of number
        read -s -n 1 -p "Enter command: " char1
        echo # Move to next line for command output

        if [[ "$char1" =~ ^[0-9]$ ]]; then
            input_buffer="$char1"
            # Loop to read subsequent digits with a short timeout
            while true; do
                # Try reading next char with a timeout. -t expects a decimal.
                read -s -n 1 -t 0.3 char_next 
                local read_status=$?

                if [[ $read_status -ne 0 ]]; then # Timeout (Bash typically >128) or read error
                    break # End of number input
                fi

                if [[ "$char_next" =~ ^[0-9]$ ]]; then
                    input_buffer="${input_buffer}${char_next}"
                    if [[ ${#input_buffer} -ge 3 ]]; then # Max 3 digits (e.g., up to 999 repos)
                        break
                    fi
                else
                    # Non-digit character read, means end of number.
                    # This char_next is currently "lost" for this command cycle.
                    # A more complex input system could "push it back" onto an input queue.
                    break
                fi
            done

            if [[ -n "$input_buffer" ]]; then
                local num_choice
                # Ensure it's a valid number and interpret as decimal
                if [[ "$input_buffer" =~ ^[0-9]+$ && "$input_buffer" -ge 0 ]]; then # Allow "0" if needed, though 1-based is typical
                    num_choice=$((10#$input_buffer)) # Force decimal interpretation

                    # Convert 1-based user input to 0-based index
                    local index_to_toggle=$((num_choice - 1))

                    if [[ $num_choice -gt 0 && $index_to_toggle -ge 0 && $index_to_toggle -lt ${#REPOS[@]} ]]; then
                        toggle_repo "$index_to_toggle"
                    else
                        echo -e "${RED}Invalid repository number: '$input_buffer' (parsed as $num_choice). Valid: 1-${#REPOS[@]}.${NC} Press any key."
                        read -s -n 1
                    fi
                else
                    echo -e "${RED}Invalid numeric input: '$input_buffer'.${NC} Press any key."
                    read -s -n 1
                fi
                continue # Restart main loop to display updated selection
            fi
            # If input_buffer is empty or invalid, char1 might be a non-numeric command
            # or a single digit that didn't form a valid multi-digit number.
            # The flow will naturally pass char1 to the case statement below.
        fi
        
        local cmd_char="$char1" # Use the first char read as the command if not handled as multi-digit number

        case "$cmd_char" in
            # Numeric cases [0-9] are now primarily handled by the multi-digit input block above.
            # If a single digit is pressed and the timeout occurs immediately, 
            # it will be processed by that block.
            a|A) select_all ;;
            n|N) select_none ;;
            s|S) show_status ;;
            r|R) 
                find_git_repos 
                # No explicit "press any key" here, display_repos will show new state
                ;; 
            p) pull_selected ;;
            P) push_selected ;;
            c|C) commit_selected ;;
            u|U) select_uncommitted ;;
            h|H) select_needs_push ;;
            b|B) select_for_rebase ;;
            e|E) rebase_selected ;;
            q|Q) 
                echo "Exiting..."
                break 
                ;;
            "") 
                # Empty input (e.g., if read was interrupted, or only Enter was pressed if not using -s -n 1)
                ;; 
            *) 
                # This will also catch single digits if they weren't processed by the numeric block
                # (e.g., if user types '0' and it's not a valid repo number by itself)
                # However, the numeric block tries to handle all valid number inputs.
                echo -e "${RED}Invalid command: '$cmd_char'.${NC} Press any key."
                read -s -n 1 
                ;;
        esac
    done
}

# Cleanup function
cleanup() {
    echo "Cleaned up." # Optional: message if any cleanup actions were performed
}

# Set up trap for cleanup
trap cleanup EXIT

# Main execution
echo "Git Repository Checker and Manager"
echo "Searching for repositories belonging to 'zawlali'..."
find_git_repos
main_menu

echo "Goodbye!"