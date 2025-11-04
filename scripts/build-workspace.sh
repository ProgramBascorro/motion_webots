#!/bin/bash

# Build script for OP3 workspace
# Can be used inside or outside Docker container

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if ROS is sourced
if [ -z "$ROS_DISTRO" ]; then
    print_info "Sourcing ROS 2 Humble..."
    source /opt/ros/humble/setup.bash
fi

# Navigate to workspace root
if [ ! -f "src/CMakeLists.txt" ] && [ ! -d "src/ROBOTIS-OP3" ]; then
    print_error "Not in workspace root! Please run from workspace directory."
    exit 1
fi

# Create COLCON_IGNORE for ROS1 packages if not exists
print_info "Ignoring ROS1 packages..."
touch src/ROBOTIS-Utility/ros_madplay_player/COLCON_IGNORE 2>/dev/null || true
touch src/ROBOTIS-Utility/ros_mpg321_player/COLCON_IGNORE 2>/dev/null || true

# Parse arguments
CLEAN=false
PACKAGES=""
PARALLEL=4

while [[ $# -gt 0 ]]; do
    case $1 in
        --clean)
            CLEAN=true
            shift
            ;;
        --packages)
            PACKAGES="$2"
            shift 2
            ;;
        --parallel)
            PARALLEL="$2"
            shift 2
            ;;
        --help|-h)
            cat << EOF
Usage: $0 [OPTIONS]

Options:
    --clean             Clean build artifacts before building
    --packages PKG      Build only specific packages (space-separated)
    --parallel N        Number of parallel jobs (default: 4)
    --help, -h          Show this help message

Examples:
    $0                                  # Build all packages
    $0 --clean                          # Clean and build all
    $0 --packages "op3_manager op3_walking_module"  # Build specific packages
    $0 --parallel 8                     # Use 8 parallel jobs

EOF
            exit 0
            ;;
        *)
            print_error "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Clean if requested
if [ "$CLEAN" = true ]; then
    print_warn "Cleaning build artifacts..."
    rm -rf build/ install/ log/
fi

# Build command
BUILD_CMD="colcon build --symlink-install --parallel-workers $PARALLEL"

# Add specific packages if requested
if [ -n "$PACKAGES" ]; then
    BUILD_CMD="$BUILD_CMD --packages-select $PACKAGES"
else
    # Skip problematic packages
    BUILD_CMD="$BUILD_CMD --packages-skip ros_madplay_player ros_mpg321_player"
fi

# Build
print_info "Building workspace..."
print_info "Command: $BUILD_CMD"
echo ""

if $BUILD_CMD; then
    print_info "Build completed successfully!"
    echo ""
    print_info "To use the workspace, run:"
    echo "  source install/setup.bash"
    exit 0
else
    print_error "Build failed!"
    print_warn "Trying with --continue-on-error flag..."

    if $BUILD_CMD --continue-on-error; then
        print_warn "Build completed with some errors. Check logs above."
        echo ""
        print_info "To use the workspace, run:"
        echo "  source install/setup.bash"
        exit 0
    else
        print_error "Build failed completely!"
        exit 1
    fi
fi
