#!/bin/bash
set -e # Exit immediately if a command exits with a non-zero status.

# --- Configuration ---
# Define the ROS distribution and its setup file path
# You might want to source a different setup if you manage multiple ROS versions
ROS_DISTRO="jazzy"
ROS_SETUP_PATH="/opt/ros/$ROS_DISTRO/setup.bash"

# --- Helper Function ---
# Displays the usage information for the script
show_help() {
    echo "Usage: $0 [OPTIONS] [--skip | --select] [package_name1 package_name2 ...]"
    echo ""
    echo "Builds a ROS 2 workspace using colcon."
    echo ""
    echo "Modes (choose one):"
    echo "  --all             Builds all packages (default if no mode specified)."
    echo "  --skip PKGS...    Builds all packages except those listed."
    echo "  --select PKGS...  Builds only the packages listed."
    echo ""
    echo "Options:"
    echo "  --direct-output   Use console_direct+ event handler for immediate output."
    echo "  --no-direct-output  Use the default buffered output (default)."
    echo "  --clean           Remove build, install, and log directories before building."
    echo "  -h, --help        Show this help message and exit."
    echo ""
    echo "Note: --skip and --select flags must be the last options before listing package names."
    echo "      Package names with spaces should be quoted (e.g., \"my package\")."
}

# --- Initialize Variables ---
mode="all" # Default mode: build all packages
packages_list="" # List of packages for skip/select modes
clean_before_build=false # Flag to indicate if cleaning is needed
# *** CHANGE HERE: Set default to false for buffered output ***
use_direct_output=false # Default: use default buffered output

# Colcon base arguments (always applied unless overridden by options)
colcon_base_args="--symlink-install"

# --- Argument Parsing ---
# Process flags first. --skip and --select are expected to be followed by package names
# and should be the last flags before package names.
while [[ "$#" -gt 0 ]]; do
    case "$1" in
        --skip)
            mode="skip"
            shift # Consume --skip
            break # The rest of the arguments are package names for --skip
            ;;
        --select)
            mode="select"
            shift # Consume --select
            break # The rest of the arguments are package names for --select
            ;;
        --all)
            mode="all"
            shift # Consume --all
            ;; # Continue processing other potential flags
        --direct-output)
            # *** CHANGE HERE: Set to true when --direct-output is used ***
            use_direct_output=true
            shift # Consume --direct-output
            ;;
        --no-direct-output)
             # *** CHANGE HERE: Explicitly set to false, even though it's default ***
             # This allows overriding --direct-output if both were somehow given.
            use_direct_output=false
            shift # Consume --no-direct-output
            ;;
        --clean)
            clean_before_build=true
            shift # Consume --clean
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            # Handle unknown options or misplaced arguments before mode flags
            echo "Error: Unknown option or misplaced argument: $1" >&2
            echo "If using --skip or --select, they must come before package names." >&2
            show_help >&2
            exit 1
            ;;
    esac
done

# If the mode is skip or select, the remaining arguments are the package list
if [[ "$mode" == "skip" || "$mode" == "select" ]]; then
    packages_list="$@" # Capture all remaining arguments as the package list
    if [[ -z "$packages_list" ]]; then
        echo "Error: --$mode mode requires package names." >&2
        show_help >&2
        exit 1
    fi
fi

# --- Script Execution ---

echo "============================================"
echo "ROS 2 Workspace Build Script"
echo "============================================"

# Navigate to the workspace root
SCRIPT_DIR="$(dirname "$0")"
# Get the absolute path of the workspace root
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Basic validation: Check if the 'src' directory exists in the assumed workspace root
if [ ! -d "$WORKSPACE_ROOT/src" ]; then
    echo "Error: Could not find workspace root containing 'src' directory at $WORKSPACE_ROOT." >&2
    echo "Please ensure the script is located in a directory like 'ros2_ws/scripts/'." >&2
    exit 1
fi

echo "Navigating to workspace root: $WORKSPACE_ROOT"
cd "$WORKSPACE_ROOT"

# Source ROS environment
if [ -f "$ROS_SETUP_PATH" ]; then
    echo "Sourcing ROS 2 $ROS_DISTRO environment: $ROS_SETUP_PATH"
    # Use '|| exit 1' to ensure sourcing failure causes script exit
    source "$ROS_SETUP_PATH" || { echo "Failed to source ROS environment." >&2; exit 1; }
else
    echo "Error: ROS 2 setup file not found: $ROS_SETUP_PATH" >&2
    echo "Please ensure ROS 2 $ROS_DISTRO is installed or update the ROS_SETUP_PATH in the script." >&2
    exit 1
fi

# Optional: Clean before build
if [ "$clean_before_build" = true ]; then
    echo "--------------------------------------------"
    echo "Cleaning build, install, and log directories..."
    # Use find or direct rm with globs. Globs are simpler if directories are at the root.
    CLEAN_DIRS=("build" "install" "log")
    CLEANED_ANY=false
    for dir in "${CLEAN_DIRS[@]}"; do
        if [ -d "$WORKSPACE_ROOT/$dir" ]; then
            echo "Removing $dir..."
            rm -rf "$WORKSPACE_ROOT/$dir"
            CLEANED_ANY=true
        else
            echo "$dir directory not found, skipping clean."
        fi # Fixed syntax error here
    done
    if [ "$CLEANED_ANY" = true ]; then
        echo "Clean finished."
        echo "--------------------------------------------"
    else
        echo "No build directories found to clean."
        echo "--------------------------------------------"
    fi
fi


# Construct the full colcon command
COLCON_COMMAND="colcon build $colcon_base_args"

# Add direct output argument if requested (and not overridden)
if [ "$use_direct_output" = true ]; then
    COLCON_COMMAND+=" --event-handlers console_direct+"
    echo "Output mode: Direct (console_direct+)"
else
    echo "Output mode: Buffered (default)"
fi


# Add mode-specific arguments and messages
case "$mode" in
    skip)
        COLCON_COMMAND+=" --packages-skip $packages_list"
        echo "Mode: Building all packages EXCEPT those listed below."
        echo "Packages to skip: $packages_list"
        ;;
    select)
        COLCON_COMMAND+=" --packages-select $packages_list"
        echo "Mode: Building ONLY the packages listed below."
        echo "Packages to build: $packages_list"
        ;;
    all)
        echo "Mode: Building ALL packages in the workspace."
        ;;
esac

echo "Running colcon command:"
echo "$COLCON_COMMAND"
echo "--------------------------------------------"

# Execute the colcon command
# Note: Using 'eval' can be risky with untrusted input, but here the input
# is command line args for package names, which colcon expects. It handles
# spaces in names if they were quoted when passed to the script.
# Direct execution without eval is often preferred if possible.
# $COLCON_COMMAND # Direct execution (safer) - relies on shell word splitting
eval $COLCON_COMMAND # Using eval to handle potential complex quoting from $packages_list

# --- Finish ---
EXIT_STATUS=$? # Capture exit status of the last command (colcon build)

echo "============================================"
if [ $EXIT_STATUS -eq 0 ]; then
    echo "Build finished successfully."
else
    echo "Build finished with errors. Exit status: $EXIT_STATUS" >&2
fi
echo "============================================"

exit $EXIT_STATUS # Exit with the same status as colcon build