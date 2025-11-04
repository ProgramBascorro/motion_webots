#!/bin/bash

# Docker run script for OP3 Webots Simulation
# This script provides easy access to run the OP3 robot simulation

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Configuration
IMAGE_NAME="op3_webots:humble"
CONTAINER_NAME="op3_webots_sim"

# Function to print colored messages
print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to check if X11 is available
check_x11() {
    if [ -z "$DISPLAY" ]; then
        print_error "DISPLAY environment variable is not set!"
        print_info "Please set DISPLAY (e.g., export DISPLAY=:0)"
        exit 1
    fi

    if ! xhost > /dev/null 2>&1; then
        print_warn "xhost not available, X11 forwarding may not work"
    fi
}

# Function to allow X11 connections
setup_x11() {
    print_info "Setting up X11 permissions..."
    xhost +local:docker > /dev/null 2>&1 || print_warn "Could not update xhost permissions"
}

# Function to build the Docker image
build_image() {
    local dockerfile="${1:-Dockerfile}"
    local tag="${2:-$IMAGE_NAME}"

    print_info "Building Docker image: $tag"
    print_info "Using Dockerfile: $dockerfile"

    if docker build -f "$dockerfile" -t "$tag" .; then
        print_info "Build complete!"
        return 0
    else
        print_error "Build failed!"
        return 1
    fi
}

# Function to build development image (no workspace build)
build_dev_image() {
    print_info "Building development Docker image (no workspace build)..."
    if build_image "Dockerfile.dev" "op3_webots:dev"; then
        print_info "Development image built successfully!"
        print_info "Use './docker-run.sh shell-dev' to start the container"
        print_info "Then run './scripts/build-workspace.sh' inside the container"
    fi
}

# Function to run interactive shell
run_shell() {
    local image="${1:-$IMAGE_NAME}"
    print_info "Starting interactive shell in container..."
    docker run -it --rm \
        --name "$CONTAINER_NAME" \
        --env DISPLAY="$DISPLAY" \
        --env QT_X11_NO_MITSHM=1 \
        --env LIBGL_ALWAYS_SOFTWARE=0 \
        --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
        --volume "$(pwd)/src:/ws/src:rw" \
        --device /dev/dri:/dev/dri \
        --network host \
        --ipc host \
        "$image" \
        bash
}

# Function to run dev shell
run_shell_dev() {
    print_info "Starting development shell..."
    run_shell "op3_webots:dev"
}

# Function to run Webots simulation
run_webots() {
    print_info "Launching Webots simulation..."
    docker run -it --rm \
        --name "$CONTAINER_NAME" \
        --env DISPLAY="$DISPLAY" \
        --env QT_X11_NO_MITSHM=1 \
        --env LIBGL_ALWAYS_SOFTWARE=0 \
        --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
        --volume "$(pwd)/src:/ws/src:rw" \
        --device /dev/dri:/dev/dri \
        --network host \
        --ipc host \
        "$IMAGE_NAME" \
        ros2 launch op3_webots_ros2 robot_launch.py
}

# Function to run op3_manager in simulation mode
run_manager() {
    print_info "Launching OP3 manager in simulation mode..."
    docker run -it --rm \
        --name "${CONTAINER_NAME}_manager" \
        --env DISPLAY="$DISPLAY" \
        --network host \
        --ipc host \
        "$IMAGE_NAME" \
        ros2 launch op3_manager op3_simulation.launch.py
}

# Function to run custom command
run_custom() {
    print_info "Running custom command: $*"
    docker run -it --rm \
        --name "$CONTAINER_NAME" \
        --env DISPLAY="$DISPLAY" \
        --env QT_X11_NO_MITSHM=1 \
        --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
        --volume "$(pwd)/src:/ws/src:rw" \
        --device /dev/dri:/dev/dri \
        --network host \
        --ipc host \
        "$IMAGE_NAME" \
        "$@"
}

# Function to stop running containers
stop_containers() {
    print_info "Stopping running OP3 containers..."
    docker ps -a --filter "name=op3_webots" --format "{{.Names}}" | while read -r container; do
        print_info "Stopping $container"
        docker stop "$container" 2>/dev/null || true
        docker rm "$container" 2>/dev/null || true
    done
    print_info "Containers stopped!"
}

# Function to show usage
usage() {
    cat << EOF
Usage: $0 [COMMAND]

Commands:
    build           Build the Docker image (with workspace build)
    build-dev       Build development image (no workspace build - faster)
    shell           Run interactive shell in container (default)
    shell-dev       Run development shell (requires build-dev first)
    webots          Launch Webots simulation
    manager         Launch OP3 manager in simulation mode
    stop            Stop all running OP3 containers
    exec <cmd>      Execute custom command in container
    help            Show this help message

Examples:
    # Option 1: Full build (slower, ready to use)
    $0 build                              # Build with workspace
    $0 webots                             # Run simulation

    # Option 2: Dev build (faster, manual workspace build)
    $0 build-dev                          # Build without workspace
    $0 shell-dev                          # Start shell
    # Inside container: ./scripts/build-workspace.sh

    # Other commands
    $0 manager                            # Run OP3 manager
    $0 exec ros2 topic list               # List ROS2 topics
    $0 stop                               # Stop all containers

EOF
}

# Main script logic
main() {
    # Check X11 availability for GUI commands
    if [[ "$1" != "build" && "$1" != "stop" && "$1" != "help" ]]; then
        check_x11
        setup_x11
    fi

    case "${1:-shell}" in
        build)
            build_image
            ;;
        build-dev)
            build_dev_image
            ;;
        shell)
            run_shell
            ;;
        shell-dev)
            run_shell_dev
            ;;
        webots)
            run_webots
            ;;
        manager)
            run_manager
            ;;
        exec)
            shift
            run_custom "$@"
            ;;
        stop)
            stop_containers
            ;;
        help|--help|-h)
            usage
            ;;
        *)
            print_error "Unknown command: $1"
            usage
            exit 1
            ;;
    esac
}

main "$@"
