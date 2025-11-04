# Makefile for OP3 Webots Project
# Provides convenient shortcuts for common tasks

.PHONY: help build run shell webots manager stop clean clean-all dev-build dev-up dev-down

# Default target
help:
	@echo "OP3 Webots Project - Available Commands"
	@echo "========================================"
	@echo ""
	@echo "Docker Commands:"
	@echo "  make build        - Build the Docker image"
	@echo "  make run          - Run interactive shell in container"
	@echo "  make shell        - Alias for 'make run'"
	@echo "  make webots       - Launch Webots simulation"
	@echo "  make manager      - Launch OP3 manager in simulation mode"
	@echo "  make stop         - Stop all running containers"
	@echo ""
	@echo "Development Commands:"
	@echo "  make dev-build    - Build development image"
	@echo "  make dev-up       - Start development container"
	@echo "  make dev-down     - Stop development container"
	@echo "  make dev-shell    - Attach to development container"
	@echo ""
	@echo "Native Build Commands:"
	@echo "  make build-ros    - Build ROS2 workspace (native)"
	@echo "  make clean-ros    - Clean ROS2 build artifacts (native)"
	@echo ""
	@echo "Cleanup Commands:"
	@echo "  make clean        - Remove Docker containers and images"
	@echo "  make clean-all    - Remove everything including volumes"
	@echo ""
	@echo "Usage Examples:"
	@echo "  make build && make webots    # Build and run simulation"
	@echo "  make dev-up && make dev-shell # Start dev environment"
	@echo ""

# Docker production commands
build:
	@echo "Building Docker image..."
	./docker-run.sh build

run: shell

shell:
	@echo "Starting interactive shell..."
	./docker-run.sh shell

webots:
	@echo "Launching Webots simulation..."
	./docker-run.sh webots

manager:
	@echo "Launching OP3 manager..."
	./docker-run.sh manager

stop:
	@echo "Stopping containers..."
	./docker-run.sh stop

# Development commands
dev-build:
	@echo "Building development image..."
	docker-compose -f docker-compose-dev.yml build

dev-up:
	@echo "Starting development container..."
	docker-compose -f docker-compose-dev.yml up -d op3_dev
	@echo "Container started. Use 'make dev-shell' to attach."

dev-down:
	@echo "Stopping development container..."
	docker-compose -f docker-compose-dev.yml down

dev-shell:
	@echo "Attaching to development container..."
	docker-compose -f docker-compose-dev.yml exec op3_dev bash

# Native ROS2 build commands
build-ros:
	@echo "Building ROS2 workspace..."
	@bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

clean-ros:
	@echo "Cleaning ROS2 build artifacts..."
	rm -rf build/ install/ log/

# Cleanup commands
clean:
	@echo "Removing Docker containers and images..."
	docker-compose down
	docker-compose -f docker-compose-dev.yml down
	docker rmi op3_webots:humble op3_webots:dev 2>/dev/null || true

clean-all: clean
	@echo "Removing volumes..."
	docker-compose down -v
	docker-compose -f docker-compose-dev.yml down -v
	@echo "Cleaning Docker system..."
	docker system prune -f

# X11 setup (for Linux)
x11-setup:
	@echo "Setting up X11 permissions..."
	xhost +local:docker

# Check dependencies
check-deps:
	@echo "Checking dependencies..."
	@which docker > /dev/null || (echo "Docker not found! Please install Docker." && exit 1)
	@which docker-compose > /dev/null || (echo "Docker Compose not found! Please install Docker Compose." && exit 1)
	@echo "All dependencies found!"

# Quick start (build and run)
quickstart: check-deps x11-setup build webots

# Status information
status:
	@echo "Docker Containers:"
	@docker ps -a --filter "name=op3" --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}"
	@echo ""
	@echo "Docker Images:"
	@docker images | grep -E "^(REPOSITORY|op3_webots)"
	@echo ""
	@echo "Docker Volumes:"
	@docker volume ls | grep -E "^(DRIVER|op3)" || echo "No OP3 volumes found"
