# Bascorro Studio - Enable Head Module Button

## What Was Added

A new **"Enable Head"** button was added to the Bascorro Studio dashboard that sends the command to enable the head control module.

## Location

**Dashboard → Safety & Quick Actions** section

The button appears alongside the Torque ON/OFF buttons:
```
[Init Pose] [Soft Stop]          [Torque OFF] [Torque ON] [Enable Head]
```

## How It Works

When clicked, the button:
1. Publishes to `/robotis/enable_ctrl_module` topic
2. Sends the message: `"head_control_module"`
3. Shows status: "Head Module Enabled"

## Technical Details

### ROS Topic
- **Topic:** `/robotis/enable_ctrl_module`
- **Type:** `std_msgs/String`
- **Message:** `"head_control_module"`

### Code Changes
- Added `headModulePubRef` to track the publisher
- Created `handleEnableHeadModule()` function
- Added green button in dashboard UI

## When to Use

Use this button when:
- You need to manually enable head control
- After running actions that disable the head module
- When switching from other control modes to head tracking
- Before using ball tracker or vision-based head movement

## Styling

- **Color:** Green (`bg-green-500`)
- **Hover:** Darker green (`hover:bg-green-600`)
- **Position:** Next to Torque buttons
- **Responsive:** Adapts to mobile/desktop layout

## Launch and Access

```bash
./script.sh --studio
# Or
ros2 launch bascorro_studio bascorro_studio.launch.py
```

Then open: **http://localhost:5173**

The button is now available on the main Dashboard page!
