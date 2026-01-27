#!/bin/bash
# Script to check if soccer_localization is properly set up

echo "=========================================="
echo "  Checking Soccer Localization Setup"
echo "=========================================="
echo ""

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check workspace
echo "1. Checking workspace structure..."
if [ -d "$HOME/localization_ws/src/soccer_localization" ]; then
    echo -e "${GREEN}✓${NC} soccer_localization found in localization_ws"
    WS_PATH="$HOME/localization_ws"
elif [ -d "$HOME/basbot/src/soccer_localization" ]; then
    echo -e "${GREEN}✓${NC} soccer_localization found in basbot"
    WS_PATH="$HOME/basbot"
else
    echo -e "${RED}✗${NC} soccer_localization not found!"
    exit 1
fi

# Source workspace
echo ""
echo "2. Sourcing workspace..."
source "$WS_PATH/install/setup.bash" 2>/dev/null
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓${NC} Workspace sourced successfully"
else
    echo -e "${YELLOW}⚠${NC} Workspace not built yet, need to build first"
fi

# Check if package is available
echo ""
echo "3. Checking if package is in ROS2..."
ros2 pkg list | grep soccer_localization > /dev/null
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓${NC} soccer_localization package available"
else
    echo -e "${RED}✗${NC} soccer_localization package NOT available"
    echo "   Run: cd $WS_PATH && colcon build --packages-select soccer_localization"
    exit 1
fi

# Check executables
echo ""
echo "4. Checking available executables..."
EXECUTABLES=$(ros2 pkg executables soccer_localization 2>/dev/null)
if [ -z "$EXECUTABLES" ]; then
    echo -e "${RED}✗${NC} No executables found"
    echo "   Check setup.py entry_points"
else
    echo -e "${GREEN}✓${NC} Found executables:"
    echo "$EXECUTABLES" | while read line; do
        echo "   - $line"
    done
fi

# Check dependencies
echo ""
echo "5. Checking dependencies..."

# soccer_msgs
ros2 pkg list | grep soccer_msgs > /dev/null
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓${NC} soccer_msgs available"
else
    echo -e "${RED}✗${NC} soccer_msgs missing"
fi

# soccer_common
ros2 pkg list | grep soccer_common > /dev/null
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓${NC} soccer_common available"
else
    echo -e "${RED}✗${NC} soccer_common missing"
fi

# Check Python files
echo ""
echo "6. Checking Python files..."
MAIN_PY="$WS_PATH/src/soccer_localization/src/soccer_localization/main.py"
if [ -f "$MAIN_PY" ]; then
    echo -e "${GREEN}✓${NC} main.py found"
else
    echo -e "${RED}✗${NC} main.py not found"
fi

UKF_ROS_PY="$WS_PATH/src/soccer_localization/src/soccer_localization/field_lines_ukf_ros.py"
if [ -f "$UKF_ROS_PY" ]; then
    echo -e "${GREEN}✓${NC} field_lines_ukf_ros.py found"
else
    echo -e "${RED}✗${NC} field_lines_ukf_ros.py not found"
fi

# Check setup.py
echo ""
echo "7. Checking setup.py..."
SETUP_PY="$WS_PATH/src/soccer_localization/setup.py"
if [ -f "$SETUP_PY" ]; then
    echo -e "${GREEN}✓${NC} setup.py found"
    echo ""
    echo "   Entry points:"
    grep -A 5 "entry_points" "$SETUP_PY" | grep "=" | sed 's/^/   /'
else
    echo -e "${RED}✗${NC} setup.py not found"
fi

echo ""
echo "=========================================="
echo "  Summary"
echo "=========================================="
echo ""
echo "Workspace: $WS_PATH"
echo ""
echo "To build all packages:"
echo "  cd $WS_PATH"
echo "  colcon build --packages-select soccer_msgs soccer_common soccer_localization"
echo "  source install/setup.bash"
echo ""
echo "To test localization:"
echo "  ros2 run soccer_localization main"
echo ""
echo "To launch full system:"
echo "  ros2 launch op3_utra_bridge localization_odometry_only.launch.py use_sim:=true"
echo ""